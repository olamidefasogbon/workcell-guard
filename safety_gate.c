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
#include <sys/neutrino.h>
#include <time.h>
#include <unistd.h>

#include "config_loader.h"
#include "safety_protocol.h"

static workcell_config_t cfg;
static gate_state_t current_state = STATE_SAFE_RUN;
static int fault_latched = 0;
static uint32_t trip_count = 0;
static uint64_t last_heartbeat_ns = 0;
static uint64_t last_latency_us = 0;
static uint64_t last_trip_us = 0;
static vision_report_t last_report;
static int seen_first_frame = 0;

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

static int watchdog_pulse_cb(message_context_t *ctp, int code, unsigned flags, void *handle) {
    (void)ctp; (void)code; (void)flags; (void)handle;
    if (!seen_first_frame) return 0;

    uint64_t timeout_ns = (uint64_t)cfg.heartbeat_timeout_ms * 1000000ULL;
    uint64_t elapsed = now_ns() - last_heartbeat_ns;

    if (elapsed > timeout_ns && current_state != STATE_WATCHDOG_FAULT) {
        current_state = STATE_WATCHDOG_FAULT;
        trip_count++;
        last_trip_us = elapsed / 1000ULL;
        if (cfg.latch_faults) fault_latched = 1;
        printf("\n[WATCHDOG TRIP] Heartbeat lost for %.1f ms -> OSSD 24V->0V (%s, latched=%d)\n",
               elapsed / 1e6, state_str(current_state), fault_latched);
        fflush(stdout);
    }
    return 0;
}

static int io_write(resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb) {
    int status = iofunc_write_verify(ctp, msg, ocb, NULL);
    if (status != EOK) return status;
    if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) return ENOSYS;

    size_t nbytes = _IO_WRITE_GET_NBYTES(msg);
    if (nbytes < sizeof(vision_report_t)) return EINVAL;

    vision_report_t report;
    if (resmgr_msgget(ctp, &report, sizeof(report), sizeof(msg->i)) == -1) {
        return errno;
    }

    uint64_t arrival = now_ns();
    last_heartbeat_ns = arrival;
    seen_first_frame = 1;
    last_report = report;
    last_latency_us = (arrival > report.capture_timestamp_ns)
                          ? (arrival - report.capture_timestamp_ns) / 1000ULL
                          : 0;

    if (!fault_latched) {
        gate_state_t next_state = report.intrusion_detected ? STATE_INTRUSION_HALT : STATE_SAFE_RUN;
        if (next_state != current_state) {
            current_state = next_state;
            if (current_state != STATE_SAFE_RUN) {
                trip_count++;
                last_trip_us = last_latency_us;
                if (cfg.latch_faults) fault_latched = 1;
            }
            printf("\n[STATE CHANGE] Frame #%u | Latency: %llu us | Dist: %.1fpx -> OSSD %s (%s)\n",
                   report.frame_id, (unsigned long long)last_latency_us,
                   report.min_zone_distance_px,
                   current_state == STATE_SAFE_RUN ? "24V_ARMED" : "0V_CUTOFF",
                   state_str(current_state));
            fflush(stdout);
        }
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
                       "state=%s|ossd_mv=%u|latched=%d|trips=%u|frame=%u|latency_us=%llu|dist_px=%.1f\n",
                       state_str(current_state),
                       current_state == STATE_SAFE_RUN ? 24000U : 0U,
                       fault_latched, trip_count, last_report.frame_id,
                       (unsigned long long)last_latency_us, last_report.min_zone_distance_px);

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
            safety_status_t *st = (safety_status_t *)_DEVCTL_DATA(msg->o);
            memset(st, 0, sizeof(*st));
            st->state                = (int32_t)current_state;
            st->latched              = fault_latched;
            st->latch_enabled        = cfg.latch_faults;
            st->trip_count           = trip_count;
            st->ossd_mv              = (current_state == STATE_SAFE_RUN) ? 24000U : 0U;
            st->frame_id             = last_report.frame_id;
            st->latency_us           = last_latency_us;
            st->last_trip_us         = last_trip_us;
            st->min_zone_distance_px = last_report.min_zone_distance_px;
            st->bbox_x               = last_report.bbox_x;
            st->bbox_y               = last_report.bbox_y;
            st->bbox_w               = last_report.bbox_w;
            st->bbox_h               = last_report.bbox_h;
            st->frame_w              = last_report.frame_w;
            st->frame_h              = last_report.frame_h;
            st->zone                 = cfg.zone;
            msg->o.ret_val = 0;
            return _RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) + sizeof(safety_status_t));
        }
        case DCMD_SAFETY_SET_ZONE: {
            safety_zone_t *z = (safety_zone_t *)_DEVCTL_DATA(msg->i);
            if (z->num_pts < 3 || z->num_pts > MAX_ZONE_VERTICES) return EINVAL;
            for (uint32_t i = 0; i < z->num_pts; i++) {
                if (z->vx[i] < 0.0f || z->vx[i] > 1.0f || z->vy[i] < 0.0f || z->vy[i] > 1.0f) {
                    return EINVAL;
                }
            }
            cfg.zone = *z;
            printf("[DEVCTL] Polygon zone updated (%u vertices)\n", z->num_pts);
            fflush(stdout);
            msg->o.ret_val = 0;
            return _RESMGR_PTR(ctp, &msg->o, sizeof(msg->o));
        }
        case DCMD_SAFETY_RESET: {
            fault_latched = 0;
            current_state = STATE_SAFE_RUN;
            seen_first_frame = 0;
            printf("[DEVCTL] Safety interlock reset -> OSSD 24V ARMED (SAFE_RUN)\n");
            fflush(stdout);
            msg->o.ret_val = 0;
            return _RESMGR_PTR(ctp, &msg->o, sizeof(msg->o));
        }
        case DCMD_SAFETY_SET_LATCH: {
            int32_t *val = (int32_t *)_DEVCTL_DATA(msg->i);
            cfg.latch_faults = (*val != 0) ? 1 : 0;
            if (!cfg.latch_faults) fault_latched = 0;
            printf("[DEVCTL] ISO 13849-1 Latch Mode -> %d\n", cfg.latch_faults);
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
    }
    config_load(cfg_path, &cfg);

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_gate;
    sched_setscheduler(0, SCHED_FIFO, &sp);

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

    iofunc_attr_t attr;
    iofunc_attr_init(&attr, S_IFNAM | 0666, NULL, NULL);

    if (resmgr_attach(dpp, &resmgr_attr, cfg.device_path, _FTYPE_ANY, 0,
                      &connect_funcs, &io_funcs, &attr) == -1) {
        perror("resmgr_attach");
        return EXIT_FAILURE;
    }

    int pulse_code = pulse_attach(dpp, MSG_FLAG_ALLOC_PULSE, 0, watchdog_pulse_cb, NULL);
    int coid = message_connect(dpp, MSG_FLAG_SIDE_CHANNEL);

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
    timer_settime(timer_id, 0, &its, NULL);

    printf("[%s] Online | Prio %d | Tick %dms | Timeout %dms | Latch=%d\n",
           cfg.device_path, cfg.prio_gate, cfg.watchdog_tick_ms, cfg.heartbeat_timeout_ms, cfg.latch_faults);
    fflush(stdout);

    dispatch_context_t *ctp = dispatch_context_alloc(dpp);
    while (1) {
        ctp = dispatch_block(ctp);
        if (!ctp) { perror("dispatch_block"); return EXIT_FAILURE; }
        dispatch_handler(ctp);
    }
    return EXIT_SUCCESS;
}
