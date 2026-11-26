struct _iofunc_attr;
#define RESMGR_HANDLE_T struct _iofunc_attr
struct _iofunc_ocb;
#define RESMGR_OCB_T struct _iofunc_ocb

#include <errno.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <time.h>
#include <unistd.h>

#include "config_loader.h"
#include "safety_logic.h"
#include "safety_protocol.h"

// All arbitration decisions live in safety_logic.h (unit tested on a host).
// This file only adapts them to the QNX resource-manager / timer-pulse APIs.

static workcell_config_t cfg;
static gate_t gate;

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static const char *state_str(gate_state_t s) {
    switch (s) {
        case STATE_SAFE_RUN:       return "SAFE_RUN";
        case STATE_INTRUSION_HALT: return "INTRUSION_HALT";
        case STATE_WATCHDOG_FAULT: return "WATCHDOG_FAULT";
        default:                   return "UNKNOWN";
    }
}

static const char *reason_str(int r) {
    switch (r) {
        case FAULT_NONE:           return "none";
        case FAULT_POWER_UP:       return "power-up interlock";
        case FAULT_INTRUSION:      return "intrusion";
        case FAULT_HEARTBEAT_LOST: return "heartbeat lost";
        case FAULT_STALE_FRAME:    return "stale frame";
        case FAULT_BAD_REPORT:     return "invalid vision report";
        case FAULT_INVALID_ZONE:   return "invalid zone polygon";
        case FAULT_ZONE_MISMATCH:  return "vision/gate zone mismatch";
        default:                   return "unknown";
    }
}

static const char *reset_str(int r) {
    switch (r) {
        case RESET_OK:                   return "accepted";
        case RESET_REFUSED_NO_HEARTBEAT: return "refused: no fresh vision heartbeat";
        case RESET_REFUSED_HAZARD:       return "refused: hazard still present";
        case RESET_REFUSED_SETTLING:     return "refused: scene not clear long enough / baseline settling";
        case RESET_REFUSED_ZONE:         return "refused: zone polygon invalid";
        default:                         return "refused";
    }
}

static uint32_t heartbeat_age_ms(uint64_t now) {
    if (!gate.seen_report) return UINT32_MAX;
    uint64_t ms = gate_heartbeat_age_ns(&gate, now) / 1000000ULL;
    return ms >= UINT32_MAX ? UINT32_MAX - 1U : (uint32_t)ms;
}

static void log_state(const char *what) {
    printf("[%s] %s | reason=%s | OSSD %s | trips=%u\n", what, state_str(gate.state), reason_str(gate.reason),
           gate_ossd_mv(&gate) ? "24V_ARMED" : "0V_CUTOFF", gate.trip_count);
    fflush(stdout);
}

static int watchdog_pulse_cb(message_context_t *ctp, int code, unsigned flags, void *handle) {
    (void)ctp; (void)code; (void)flags; (void)handle;
    if (gate_on_tick(&gate, now_ns())) log_state("WATCHDOG TRIP");
    return 0;
}

static int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb) {
    int status = iofunc_write_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;

    size_t nbytes = _IO_WRITE_GET_NBYTES(msg);
    if (nbytes != sizeof(vision_report_t)) {
        // A malformed report is a fault, not something to ignore.
        gate_enter_fault(&gate, STATE_WATCHDOG_FAULT, FAULT_BAD_REPORT, 0);
        log_state("BAD REPORT");
        return EINVAL;
    }

    vision_report_t report;
    if (resmgr_msgget(ctp, &report, sizeof(report), sizeof(msg->i)) == -1) return errno;

    if (gate_on_report(&gate, &report, now_ns())) {
        printf("Frame #%u | latency %llu us | dist %.1fpx\n", report.frame_id,
               (unsigned long long)gate.last_latency_us, report.min_zone_distance_px);
        log_state("STATE CHANGE");
    }

    _IO_SET_WRITE_NBYTES(ctp, nbytes);
    return _RESMGR_NPARTS(0);
}

static int io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb) {
    static char reply_buf[320];
    int status = iofunc_read_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;
    if (ocb->offset > 0) return _RESMGR_NPARTS(0);

    int len = snprintf(reply_buf, sizeof(reply_buf),
                       "state=%s|reason=%s|ossd_mv=%u|latched=%d|trips=%u|frame=%u|latency_us=%llu|dist_px=%.1f\n",
                       state_str(gate.state), reason_str(gate.reason), gate_ossd_mv(&gate), gate.latched,
                       gate.trip_count, gate.last_report.frame_id, (unsigned long long)gate.last_latency_us,
                       gate.last_report.min_zone_distance_px);
    if (len < 0) return EIO;
    if ((size_t)len >= sizeof(reply_buf)) len = (int)sizeof(reply_buf) - 1;

    size_t nbytes = _IO_READ_GET_NBYTES(msg);
    if ((size_t)len > nbytes) len = (int)nbytes;

    SETIOV(ctp->iov, reply_buf, len);
    _IO_SET_READ_NBYTES(ctp, len);
    ocb->offset += len;
    return _RESMGR_NPARTS(1);
}

static int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb) {
    int status = iofunc_devctl_default(ctp, msg, ocb);
    if (status != _RESMGR_DEFAULT) return status;

    switch (msg->i.dcmd) {
        case DCMD_SAFETY_GET_STATUS: {
            uint64_t now = now_ns();
            safety_status_t *st = (safety_status_t *)_DEVCTL_DATA(msg->o);
            memset(st, 0, sizeof(*st));
            st->state                = (int32_t)gate.state;
            st->latched              = gate.latched;
            st->latch_enabled        = gate.latch_enabled;
            st->trip_count           = gate.trip_count;
            st->ossd_mv              = gate_ossd_mv(&gate);
            st->frame_id             = gate.last_report.frame_id;
            st->latency_us           = gate.last_latency_us;
            st->last_trip_us         = gate.last_trip_us;
            st->min_zone_distance_px = gate.last_report.min_zone_distance_px;
            st->bbox_x               = gate.last_report.bbox_x;
            st->bbox_y               = gate.last_report.bbox_y;
            st->bbox_w               = gate.last_report.bbox_w;
            st->bbox_h               = gate.last_report.bbox_h;
            st->frame_w              = gate.last_report.frame_w;
            st->frame_h              = gate.last_report.frame_h;
            st->zone                 = gate.zone;
            st->fault_reason         = (int32_t)gate.reason;
            st->reset_ready          = gate_reset_check(&gate, now) == RESET_OK;
            st->last_reset_result    = gate.last_reset_result;
            st->reset_refusals       = gate.reset_refusals;
            st->clear_streak         = gate.clear_streak;
            st->stream_epoch         = gate.last_report.stream_epoch;
            st->heartbeat_age_ms     = heartbeat_age_ms(now);
            st->test_hooks           = WORKCELL_HOOKS_ENABLED;
            msg->o.ret_val = 0;
            return _RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) + sizeof(safety_status_t));
        }
        case DCMD_SAFETY_SET_ZONE: {
            if (msg->i.nbytes < sizeof(safety_zone_t)) return EINVAL;
            safety_zone_t z;
            memcpy(&z, _DEVCTL_DATA(msg->i), sizeof(z));
            int e = gate_set_zone(&gate, &z);
            if (e != 0) {
                // An invalid polygon is rejected and the machine is stopped, never guessed around.
                log_state("INVALID ZONE REJECTED");
                return e;
            }
            printf("[DEVCTL] Polygon zone updated (%u vertices)\n", z.num_pts);
            fflush(stdout);
            msg->o.ret_val = 0;
            return _RESMGR_PTR(ctp, &msg->o, sizeof(msg->o));
        }
        case DCMD_SAFETY_RESET: {
            int r = gate_request_reset(&gate, now_ns());
            printf("[DEVCTL] Reset %s\n", reset_str(r));
            fflush(stdout);
            if (r != RESET_OK) return EBUSY;
            log_state("RESET");
            msg->o.ret_val = 0;
            return _RESMGR_PTR(ctp, &msg->o, sizeof(msg->o));
        }
        case DCMD_SAFETY_SET_LATCH: {
            if (msg->i.nbytes < sizeof(int32_t)) return EINVAL;
            int32_t val;
            memcpy(&val, _DEVCTL_DATA(msg->i), sizeof(val));
            int e = gate_set_latch(&gate, val != 0);
            if (e != 0) {
                printf("[DEVCTL] Latch disable refused (restart interlock cannot be switched off)\n");
                fflush(stdout);
                return e;
            }
            printf("[DEVCTL] ISO 13849-1 Latch Mode -> %d\n", gate.latch_enabled);
            fflush(stdout);
            msg->o.ret_val = 0;
            return _RESMGR_PTR(ctp, &msg->o, sizeof(msg->o));
        }
        default:
            return ENOSYS;
    }
}

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
        else return EXIT_FAILURE;
    }
    if (config_load(cfg_path, &cfg) != 0) return EXIT_FAILURE;

    // Priority isolation is a safety requirement of this process: no RT priority, no service.
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_gate;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("sched_setscheduler(SCHED_FIFO) - refusing to run without real-time priority");
        return EXIT_FAILURE;
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) perror("mlockall (continuing)");

    gate_params_t gp = {
        .heartbeat_timeout_ms = (uint32_t)cfg.heartbeat_timeout_ms,
        .max_frame_age_ms     = (uint32_t)cfg.max_frame_age_ms,
        .reset_clear_frames   = (uint32_t)cfg.reset_clear_frames,
        .allow_latch_disable  = WORKCELL_HOOKS_ENABLED
    };
    if (gate_init(&gate, &gp, &cfg.zone, now_ns()) != 0) {
        fprintf(stderr, "configured zone is invalid; gate starts faulted\n");
    }
    if (WORKCELL_HOOKS_ENABLED && cfg.latch_faults == 0) gate_set_latch(&gate, 0);

    dispatch_t *dpp = dispatch_create();
    if (!dpp) { perror("dispatch_create"); return EXIT_FAILURE; }

    resmgr_attr_t resmgr_attr;
    memset(&resmgr_attr, 0, sizeof(resmgr_attr));
    resmgr_attr.nparts_max = 1;
    resmgr_attr.msg_max_size = 2048;

    resmgr_connect_funcs_t connect_funcs;
    resmgr_io_funcs_t io_funcs;
    iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, _RESMGR_IO_NFUNCS, &io_funcs);
    io_funcs.write  = io_write;
    io_funcs.read   = io_read;
    io_funcs.devctl = io_devctl;

    // Owner-only: reports, resets and zone changes are privileged operations.
    iofunc_attr_t attr;
    iofunc_attr_init(&attr, S_IFNAM | 0600, NULL, NULL);

    if (resmgr_attach(dpp, &resmgr_attr, cfg.device_path, _FTYPE_ANY, 0,
                      &connect_funcs, &io_funcs, &attr) == -1) {
        perror("resmgr_attach");
        return EXIT_FAILURE;
    }

    int pulse_code = pulse_attach(dpp, MSG_FLAG_ALLOC_PULSE, 0, watchdog_pulse_cb, NULL);
    if (pulse_code == -1) { perror("pulse_attach"); return EXIT_FAILURE; }
    int coid = message_connect(dpp, MSG_FLAG_SIDE_CHANNEL);
    if (coid == -1) { perror("message_connect"); return EXIT_FAILURE; }

    struct sigevent sev;
    SIGEV_PULSE_INIT(&sev, coid, cfg.prio_gate, pulse_code, 0);

    timer_t timer_id;
    if (timer_create(CLOCK_MONOTONIC, &sev, &timer_id) == -1) {
        perror("timer_create");
        return EXIT_FAILURE;
    }

    long tick_ns = (long)cfg.watchdog_tick_ms * 1000000L;
    struct itimerspec its = {
        .it_value    = { .tv_sec = tick_ns / 1000000000L, .tv_nsec = tick_ns % 1000000000L },
        .it_interval = { .tv_sec = tick_ns / 1000000000L, .tv_nsec = tick_ns % 1000000000L }
    };
    if (timer_settime(timer_id, 0, &its, NULL) == -1) {
        perror("timer_settime");
        return EXIT_FAILURE;
    }

    printf("[%s] Online | Prio %d | Tick %dms | Heartbeat %dms | MaxFrameAge %dms | ResetClear %d | Latch=%d | hooks=%d\n",
           cfg.device_path, cfg.prio_gate, cfg.watchdog_tick_ms, cfg.heartbeat_timeout_ms,
           cfg.max_frame_age_ms, cfg.reset_clear_frames, gate.latch_enabled, WORKCELL_HOOKS_ENABLED);
    log_state("BOOT");

    dispatch_context_t *ctp = dispatch_context_alloc(dpp);
    if (!ctp) { perror("dispatch_context_alloc"); return EXIT_FAILURE; }
    while (1) {
        ctp = dispatch_block(ctp);
        if (!ctp) { perror("dispatch_block"); return EXIT_FAILURE; }
        dispatch_handler(ctp);
    }
    return EXIT_SUCCESS;
}
