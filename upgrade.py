from pathlib import Path

FILES = {}

FILES["safety_protocol.h"] = r"""#ifndef SAFETY_PROTOCOL_H
#define SAFETY_PROTOCOL_H

#include <devctl.h>
#include <stdint.h>

#define MAX_FRAME_PIXELS (1280 * 720)
#define MAX_ZONE_VERTICES 8

typedef enum {
    STATE_SAFE_RUN       = 0,
    STATE_INTRUSION_HALT = 1,
    STATE_WATCHDOG_FAULT = 2
} gate_state_t;

// Dynamic N-point normalized danger-zone polygon (0.0 to 1.0)
typedef struct {
    uint32_t num_pts;
    float    vx[MAX_ZONE_VERTICES];
    float    vy[MAX_ZONE_VERTICES];
} safety_zone_t;

// Telemetry packet sent from vision_worker -> /dev/safety_gate
typedef struct {
    uint64_t capture_timestamp_ns;
    uint32_t frame_id;
    uint8_t  intrusion_detected;
    float    min_zone_distance_px;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t frame_w;
    uint16_t frame_h;
    safety_zone_t active_zone;
} vision_report_t;

// Snapshot returned via devctl(DCMD_SAFETY_GET_STATUS)
typedef struct {
    int32_t  state;
    int32_t  latched;
    int32_t  latch_enabled;
    uint32_t trip_count;
    uint32_t ossd_mv;       // 24000 mV (24V DC) when SAFE_RUN, 0 mV on HALT/FAULT
    uint32_t frame_id;
    uint64_t latency_us;
    uint64_t last_trip_us;
    float    min_zone_distance_px;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t frame_w;
    uint16_t frame_h;
    safety_zone_t zone;
} safety_status_t;

// POSIX Shared Memory layout for decoupled camera frames (/dev/shmem/workcell_cam)
typedef struct {
    volatile uint32_t write_seq;
    uint32_t width;
    uint32_t height;
    uint32_t frame_id;
    uint64_t capture_timestamp_ns;
    uint8_t  pixels[MAX_FRAME_PIXELS];
} shm_camera_t;

// Native QNX devctl commands for /dev/safety_gate
#define DCMD_SAFETY_GET_STATUS __DIOF(_DCMD_MISC, 1, safety_status_t)
#define DCMD_SAFETY_SET_ZONE   __DIOT(_DCMD_MISC, 2, safety_zone_t)
#define DCMD_SAFETY_RESET      __DION(_DCMD_MISC, 3)
#define DCMD_SAFETY_SET_LATCH  __DIOT(_DCMD_MISC, 4, int32_t)

#endif
"""

FILES["config_loader.h"] = r"""#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "safety_protocol.h"

typedef struct {
    char device_path[64];
    char shm_name[64];
    int prio_gate;
    int prio_supervisor;
    int prio_camera;
    int prio_vision;
    int prio_telemetry;
    int watchdog_tick_ms;
    int heartbeat_timeout_ms;
    int respawn_delay_ms;
    int target_fps;
    uint32_t sim_width;
    uint32_t sim_height;
    uint32_t bg_warmup_frames;
    int diff_threshold;
    float bg_alpha;
    float min_blob_ratio;
    int latch_faults;
    int telemetry_port;
    int cam_ingest_port;
    int frame_stream_port;
    safety_zone_t zone;
} workcell_config_t;

static inline void config_set_defaults(workcell_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    strcpy(cfg->device_path, "/dev/safety_gate");
    strcpy(cfg->shm_name, "/workcell_cam");
    cfg->prio_gate            = 25;
    cfg->prio_supervisor      = 20;
    cfg->prio_camera          = 15;
    cfg->prio_vision          = 10;
    cfg->prio_telemetry       = 5;
    cfg->watchdog_tick_ms     = 5;
    cfg->heartbeat_timeout_ms = 150;
    cfg->respawn_delay_ms     = 250;
    cfg->target_fps           = 30;
    cfg->sim_width            = 320;
    cfg->sim_height           = 240;
    cfg->bg_warmup_frames     = 30;
    cfg->diff_threshold       = 35;
    cfg->bg_alpha             = 0.02f;
    cfg->min_blob_ratio       = 0.005f;
    cfg->latch_faults         = 1;
    cfg->telemetry_port       = 9876;
    cfg->cam_ingest_port      = 9877;
    cfg->frame_stream_port    = 9878;

    cfg->zone.num_pts = 4;
    cfg->zone.vx[0] = 0.50f; cfg->zone.vy[0] = 0.20f;
    cfg->zone.vx[1] = 0.92f; cfg->zone.vy[1] = 0.20f;
    cfg->zone.vx[2] = 0.92f; cfg->zone.vy[2] = 0.85f;
    cfg->zone.vx[3] = 0.50f; cfg->zone.vy[3] = 0.85f;
}

static inline int config_load(const char *path, workcell_config_t *cfg) {
    config_set_defaults(cfg);
    FILE *f = fopen(path, "r");
    if (!f) return -1;

    float zx1 = 0.50f, zy1 = 0.20f, zx2 = 0.92f, zy2 = 0.85f;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = '\0';
        char *k = p;
        char *v = eq + 1;
        char *end = v + strlen(v);
        while (end > v && (end[-1] == '\n' || end[-1] == '\r' || end[-1] == ' ' || end[-1] == '\t')) *--end = '\0';

        if      (strcmp(k, "device_path") == 0)          strncpy(cfg->device_path, v, sizeof(cfg->device_path) - 1);
        else if (strcmp(k, "shm_name") == 0)             strncpy(cfg->shm_name, v, sizeof(cfg->shm_name) - 1);
        else if (strcmp(k, "prio_gate") == 0)            cfg->prio_gate = atoi(v);
        else if (strcmp(k, "prio_supervisor") == 0)      cfg->prio_supervisor = atoi(v);
        else if (strcmp(k, "prio_camera") == 0)          cfg->prio_camera = atoi(v);
        else if (strcmp(k, "prio_vision") == 0)          cfg->prio_vision = atoi(v);
        else if (strcmp(k, "prio_telemetry") == 0)       cfg->prio_telemetry = atoi(v);
        else if (strcmp(k, "watchdog_tick_ms") == 0)     cfg->watchdog_tick_ms = atoi(v);
        else if (strcmp(k, "heartbeat_timeout_ms") == 0) cfg->heartbeat_timeout_ms = atoi(v);
        else if (strcmp(k, "respawn_delay_ms") == 0)     cfg->respawn_delay_ms = atoi(v);
        else if (strcmp(k, "target_fps") == 0)           cfg->target_fps = atoi(v);
        else if (strcmp(k, "sim_width") == 0)            cfg->sim_width = (uint32_t)atoi(v);
        else if (strcmp(k, "sim_height") == 0)           cfg->sim_height = (uint32_t)atoi(v);
        else if (strcmp(k, "bg_warmup_frames") == 0)     cfg->bg_warmup_frames = (uint32_t)atoi(v);
        else if (strcmp(k, "diff_threshold") == 0)       cfg->diff_threshold = atoi(v);
        else if (strcmp(k, "bg_alpha") == 0)             cfg->bg_alpha = strtof(v, NULL);
        else if (strcmp(k, "min_blob_ratio") == 0)       cfg->min_blob_ratio = strtof(v, NULL);
        else if (strcmp(k, "telemetry_port") == 0)       cfg->telemetry_port = atoi(v);
        else if (strcmp(k, "cam_ingest_port") == 0)      cfg->cam_ingest_port = atoi(v);
        else if (strcmp(k, "frame_stream_port") == 0)    cfg->frame_stream_port = atoi(v);
        else if (strcmp(k, "zone_x1") == 0)              zx1 = strtof(v, NULL);
        else if (strcmp(k, "zone_y1") == 0)              zy1 = strtof(v, NULL);
        else if (strcmp(k, "zone_x2") == 0)              zx2 = strtof(v, NULL);
        else if (strcmp(k, "zone_y2") == 0)              zy2 = strtof(v, NULL);
    }
    fclose(f);

    cfg->zone.num_pts = 4;
    cfg->zone.vx[0] = zx1; cfg->zone.vy[0] = zy1;
    cfg->zone.vx[1] = zx2; cfg->zone.vy[1] = zy1;
    cfg->zone.vx[2] = zx2; cfg->zone.vy[2] = zy2;
    cfg->zone.vx[3] = zx1; cfg->zone.vy[3] = zy2;
    return 0;
}

#endif
"""

FILES["safety_gate.c"] = r"""struct _iofunc_attr;
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
"""

FILES["vision_worker.c"] = r"""#include <devctl.h>
#include <fcntl.h>
#include <math.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "config_loader.h"
#include "safety_protocol.h"

static int point_in_poly(float px, float py, const float *vx, const float *vy, uint32_t n) {
    int inside = 0;
    for (uint32_t i = 0, j = n - 1; i < n; j = i++) {
        if (((vy[i] > py) != (vy[j] > py)) &&
            (px < (vx[j] - vx[i]) * (py - vy[i]) / (vy[j] - vy[i] + 1e-6f) + vx[i])) {
            inside = !inside;
        }
    }
    return inside;
}

static float orient2d(float ax, float ay, float bx, float by, float cx, float cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static int on_segment(float ax, float ay, float bx, float by, float px, float py) {
    return px >= fminf(ax, bx) - 1e-3f && px <= fmaxf(ax, bx) + 1e-3f &&
           py >= fminf(ay, by) - 1e-3f && py <= fmaxf(ay, by) + 1e-3f;
}

static int segments_intersect(float ax, float ay, float bx, float by,
                              float cx, float cy, float dx, float dy) {
    float o1 = orient2d(ax, ay, bx, by, cx, cy);
    float o2 = orient2d(ax, ay, bx, by, dx, dy);
    float o3 = orient2d(cx, cy, dx, dy, ax, ay);
    float o4 = orient2d(cx, cy, dx, dy, bx, by);

    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) &&
        ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return 1;
    if (fabsf(o1) < 1e-3f && on_segment(ax, ay, bx, by, cx, cy)) return 1;
    if (fabsf(o2) < 1e-3f && on_segment(ax, ay, bx, by, dx, dy)) return 1;
    if (fabsf(o3) < 1e-3f && on_segment(cx, cy, dx, dy, ax, ay)) return 1;
    if (fabsf(o4) < 1e-3f && on_segment(cx, cy, dx, dy, bx, by)) return 1;
    return 0;
}

static float dist_pt_to_seg(float px, float py, float ax, float ay, float bx, float by) {
    float vx = bx - ax, vy = by - ay;
    float len2 = vx * vx + vy * vy;
    float t = (len2 > 1e-6f) ? ((px - ax) * vx + (py - ay) * vy) / len2 : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float cx = ax + t * vx, cy = ay + t * vy;
    return hypotf(px - cx, py - cy);
}

static float dist_pt_to_rect(float px, float py, float rx1, float ry1, float rx2, float ry2) {
    float dx = 0.0f;
    if (px < rx1) dx = rx1 - px;
    else if (px > rx2) dx = px - rx2;
    float dy = 0.0f;
    if (py < ry1) dy = ry1 - py;
    else if (py > ry2) dy = py - ry2;
    return hypotf(dx, dy);
}

static void eval_poly_rect(const safety_zone_t *z, float w, float h,
                           float rx1, float ry1, float rx2, float ry2,
                           uint8_t *out_intrusion, float *out_dist) {
    uint32_t n = z->num_pts;
    if (n < 3 || n > MAX_ZONE_VERTICES) {
        *out_intrusion = 0;
        *out_dist = 999.0f;
        return;
    }

    float px[MAX_ZONE_VERTICES], py[MAX_ZONE_VERTICES];
    for (uint32_t i = 0; i < n; i++) {
        px[i] = z->vx[i] * w;
        py[i] = z->vy[i] * h;
        if (px[i] >= rx1 && px[i] <= rx2 && py[i] >= ry1 && py[i] <= ry2) {
            *out_intrusion = 1;
            *out_dist = 0.0f;
            return;
        }
    }

    float rcx[4] = { rx1, rx2, rx2, rx1 };
    float rcy[4] = { ry1, ry1, ry2, ry2 };
    for (int c = 0; c < 4; c++) {
        if (point_in_poly(rcx[c], rcy[c], px, py, n)) {
            *out_intrusion = 1;
            *out_dist = 0.0f;
            return;
        }
    }

    for (uint32_t i = 0; i < n; i++) {
        uint32_t j = (i + 1) % n;
        for (int e = 0; e < 4; e++) {
            int en = (e + 1) % 4;
            if (segments_intersect(px[i], py[i], px[j], py[j], rcx[e], rcy[e], rcx[en], rcy[en])) {
                *out_intrusion = 1;
                *out_dist = 0.0f;
                return;
            }
        }
    }

    float min_d = 9999.0f;
    for (int c = 0; c < 4; c++) {
        for (uint32_t i = 0; i < n; i++) {
            uint32_t j = (i + 1) % n;
            float d = dist_pt_to_seg(rcx[c], rcy[c], px[i], py[i], px[j], py[j]);
            if (d < min_d) min_d = d;
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        float d = dist_pt_to_rect(px[i], py[i], rx1, ry1, rx2, ry2);
        if (d < min_d) min_d = d;
    }

    *out_intrusion = 0;
    *out_dist = min_d;
}

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
    }

    workcell_config_t cfg;
    config_load(cfg_path, &cfg);

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_vision;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    int gate_fd = open(cfg.device_path, O_RDWR);
    if (gate_fd < 0) { perror("open safety_gate"); return EXIT_FAILURE; }

    int shm_fd = shm_open(cfg.shm_name, O_RDONLY, 0666);
    if (shm_fd < 0) { perror("shm_open camera"); return EXIT_FAILURE; }

    shm_camera_t *cam = (shm_camera_t *)mmap(NULL, sizeof(shm_camera_t),
                                             PROT_READ, MAP_SHARED, shm_fd, 0);
    if (cam == MAP_FAILED) { perror("mmap"); return EXIT_FAILURE; }

    uint32_t cur_w = 0, cur_h = 0, last_seen_seq = 0, last_fid = 0;
    uint8_t *frame_copy = NULL;
    uint8_t *smooth_buf = NULL;
    float *bg_model = NULL;
    safety_zone_t live_zone = cfg.zone;

    printf("[vision_worker] Online at Prio %d | Polygon CV -> %s\n",
           cfg.prio_vision, cfg.device_path);
    fflush(stdout);

    while (1) {
        uint32_t seq1 = cam->write_seq;
        if ((seq1 & 1U) || seq1 == 0 || seq1 == last_seen_seq) {
            usleep(1000);
            continue;
        }

        uint32_t w = cam->width;
        uint32_t h = cam->height;
        uint32_t fid = cam->frame_id;
        uint64_t t_cap = cam->capture_timestamp_ns;

        if (w < 16 || h < 16 || (size_t)w * h > MAX_FRAME_PIXELS) {
            usleep(1000);
            continue;
        }

        int need_reseed = 0;
        if (w != cur_w || h != cur_h) {
            cur_w = w;
            cur_h = h;
            frame_copy = (uint8_t *)realloc(frame_copy, (size_t)w * h);
            smooth_buf = (uint8_t *)realloc(smooth_buf, (size_t)w * h);
            bg_model   = (float *)realloc(bg_model, (size_t)w * h * sizeof(float));
            need_reseed = 1;
        } else if (fid < last_fid) {
            need_reseed = 1;
        }
        last_fid = fid;

        memcpy(frame_copy, cam->pixels, (size_t)w * h);
        __sync_synchronize();
        uint32_t seq2 = cam->write_seq;
        if (seq1 != seq2) continue;
        last_seen_seq = seq2;

        safety_status_t st;
        if (devctl(gate_fd, DCMD_SAFETY_GET_STATUS, &st, sizeof(st), NULL) == 0 &&
            st.zone.num_pts >= 3 && st.zone.num_pts <= MAX_ZONE_VERTICES) {
            live_zone = st.zone;
        }

        memcpy(smooth_buf, frame_copy, (size_t)w * h);
        for (uint32_t y = 1; y + 1 < h; y++) {
            for (uint32_t x = 1; x + 1 < w; x++) {
                uint32_t sum =
                    (uint32_t)frame_copy[(y - 1) * w + (x - 1)] + frame_copy[(y - 1) * w + x] + frame_copy[(y - 1) * w + (x + 1)] +
                    (uint32_t)frame_copy[y * w + (x - 1)]       + frame_copy[y * w + x]       + frame_copy[y * w + (x + 1)] +
                    (uint32_t)frame_copy[(y + 1) * w + (x - 1)] + frame_copy[(y + 1) * w + x] + frame_copy[(y + 1) * w + (x + 1)];
                smooth_buf[y * w + x] = (uint8_t)(sum / 9U);
            }
        }

        size_t total_px = (size_t)w * h;
        if (need_reseed) {
            for (size_t i = 0; i < total_px; i++) bg_model[i] = (float)smooth_buf[i];
        }

        float thresh = fmaxf((float)cfg.diff_threshold, 38.0f);
        const uint32_t CELL = 8;
        uint32_t cols = w / CELL;
        uint32_t rows = h / CELL;
        uint16_t min_x = (uint16_t)w, min_y = (uint16_t)h, max_x = 0, max_y = 0;
        uint32_t valid_cells = 0;

        for (uint32_t cy = 0; cy < rows; cy++) {
            for (uint32_t cx = 0; cx < cols; cx++) {
                uint32_t x0 = cx * CELL;
                uint32_t y0 = cy * CELL;
                if (x0 < 6 || y0 < 6 || x0 + CELL + 6 > w || y0 + CELL + 6 > h) continue;

                uint32_t hits = 0;
                for (uint32_t dy = 0; dy < CELL; dy++) {
                    for (uint32_t dx = 0; dx < CELL; dx++) {
                        size_t idx = (size_t)(y0 + dy) * w + (x0 + dx);
                        if (fabsf((float)smooth_buf[idx] - bg_model[idx]) > thresh) hits++;
                    }
                }
                if (hits >= 18U) {
                    valid_cells++;
                    if ((uint16_t)x0 < min_x) min_x = (uint16_t)x0;
                    if ((uint16_t)(x0 + CELL - 1) > max_x) max_x = (uint16_t)(x0 + CELL - 1);
                    if ((uint16_t)y0 < min_y) min_y = (uint16_t)y0;
                    if ((uint16_t)(y0 + CELL - 1) > max_y) max_y = (uint16_t)(y0 + CELL - 1);
                }
            }
        }

        if (valid_cells == 0) {
            for (size_t i = 0; i < total_px; i++) {
                bg_model[i] = (1.0f - cfg.bg_alpha) * bg_model[i] + cfg.bg_alpha * (float)smooth_buf[i];
            }
        }

        uint8_t intrusion = 0;
        float dist_px = 999.0f;
        uint16_t bx = 0, by = 0, bw = 0, bh = 0;

        if (valid_cells >= 4U) {
            bx = min_x;
            by = min_y;
            bw = (uint16_t)(max_x - min_x + 1);
            bh = (uint16_t)(max_y - min_y + 1);
            eval_poly_rect(&live_zone, (float)w, (float)h,
                           (float)min_x, (float)min_y, (float)max_x, (float)max_y,
                           &intrusion, &dist_px);
        }

        vision_report_t report = {
            .capture_timestamp_ns = t_cap,
            .frame_id             = fid,
            .intrusion_detected   = intrusion,
            .min_zone_distance_px = dist_px,
            .bbox_x               = bx,
            .bbox_y               = by,
            .bbox_w               = bw,
            .bbox_h               = bh,
            .frame_w              = (uint16_t)w,
            .frame_h              = (uint16_t)h,
            .active_zone          = live_zone
        };

        if (write(gate_fd, &report, sizeof(report)) != sizeof(report)) {
            perror("write safety_gate");
            break;
        }
    }

    free(frame_copy);
    free(smooth_buf);
    free(bg_model);
    close(gate_fd);
    return 0;
}
"""

FILES["telemetry_bridge.c"] = r"""#include <arpa/inet.h>
#include <devctl.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include "config_loader.h"
#include "safety_protocol.h"

#define TELEMETRY_MAGIC 0x514E5838U // "QNX8"
#define FRAME_MAGIC     0x46524D38U // "FRM8"

typedef struct __attribute__((packed)) {
    uint32_t magic;
    int32_t  state;
    int32_t  latched;
    int32_t  latch_enabled;
    uint32_t trip_count;
    uint32_t ossd_mv;
    uint32_t frame_id;
    uint64_t latency_us;
    uint64_t last_trip_us;
    float    min_zone_distance_px;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t frame_w;
    uint16_t frame_h;
    uint32_t num_pts;
    float    vx[MAX_ZONE_VERTICES];
    float    vy[MAX_ZONE_VERTICES];
} udp_telemetry_pkt_t;

static workcell_config_t cfg;
static shm_camera_t *cam = NULL;

static int send_exact(int fd, const void *buf, size_t len) {
    size_t sent = 0;
    const uint8_t *p = (const uint8_t *)buf;
    while (sent < len) {
        ssize_t n = send(fd, p + sent, len - sent, 0);
        if (n <= 0) return -1;
        sent += (size_t)n;
    }
    return 0;
}

static void *frame_stream_thread(void *arg) {
    (void)arg;
    int port = (cfg.frame_stream_port > 0) ? cfg.frame_stream_port : 9878;

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("frame stream socket"); return NULL; }

    int yes = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(srv, 4) < 0) {
        perror("frame stream bind/listen");
        close(srv);
        return NULL;
    }

    printf("[telemetry_bridge] Full-Res TCP video stream active on port %d\n", port);
    fflush(stdout);

    uint8_t *snap = (uint8_t *)malloc(MAX_FRAME_PIXELS);
    if (!snap) return NULL;

    while (1) {
        int cli = accept(srv, NULL, NULL);
        if (cli < 0) continue;
        setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        char req[16];
        while (recv(cli, req, 4, 0) == 4) {
            uint32_t w = cam->width;
            uint32_t h = cam->height;
            uint32_t fid = cam->frame_id;
            if (w == 0 || h == 0 || (size_t)w * h > MAX_FRAME_PIXELS) break;

            size_t bytes = (size_t)w * h;
            memcpy(snap, cam->pixels, bytes);
            uint32_t hdr[4] = { FRAME_MAGIC, w, h, fid };
            if (send_exact(cli, hdr, sizeof(hdr)) < 0 || send_exact(cli, snap, bytes) < 0) {
                break;
            }
        }
        close(cli);
    }
    free(snap);
    return NULL;
}

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
    }

    config_load(cfg_path, &cfg);
    if (cfg.prio_telemetry <= 0) cfg.prio_telemetry = 5;
    if (cfg.telemetry_port <= 0) cfg.telemetry_port = 9876;

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_telemetry;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    int gate_fd = open(cfg.device_path, O_RDWR);
    if (gate_fd < 0) { perror("open safety_gate"); return EXIT_FAILURE; }

    int shm_fd = shm_open(cfg.shm_name, O_RDONLY, 0666);
    if (shm_fd < 0) { perror("shm_open camera"); return EXIT_FAILURE; }

    cam = (shm_camera_t *)mmap(NULL, sizeof(shm_camera_t),
                               PROT_READ, MAP_SHARED, shm_fd, 0);
    if (cam == MAP_FAILED) { perror("mmap"); return EXIT_FAILURE; }

    pthread_t ftid;
    pthread_create(&ftid, NULL, frame_stream_thread, NULL);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return EXIT_FAILURE; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)cfg.telemetry_port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind UDP telemetry port");
        return EXIT_FAILURE;
    }

    printf("[telemetry_bridge] Online at Prio %d | UDP port %d -> %s\n",
           cfg.prio_telemetry, cfg.telemetry_port, cfg.device_path);
    fflush(stdout);

    char cmd_buf[512];
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        ssize_t n = recvfrom(sock, cmd_buf, sizeof(cmd_buf) - 1, 0,
                             (struct sockaddr *)&client_addr, &client_len);
        if (n <= 0) continue;
        cmd_buf[n] = '\0';

        if (strncmp(cmd_buf, "ZONE ", 5) == 0) {
            safety_zone_t z;
            memset(&z, 0, sizeof(z));
            char *tok = strtok(cmd_buf + 5, " \t\r\n");
            if (tok) {
                z.num_pts = (uint32_t)atoi(tok);
                if (z.num_pts >= 3 && z.num_pts <= MAX_ZONE_VERTICES) {
                    uint32_t ok = 1;
                    for (uint32_t i = 0; i < z.num_pts; i++) {
                        char *sx = strtok(NULL, " \t\r\n");
                        char *sy = strtok(NULL, " \t\r\n");
                        if (!sx || !sy) { ok = 0; break; }
                        z.vx[i] = strtof(sx, NULL);
                        z.vy[i] = strtof(sy, NULL);
                    }
                    if (ok) devctl(gate_fd, DCMD_SAFETY_SET_ZONE, &z, sizeof(z), NULL);
                }
            }
            continue;
        }

        if (strncmp(cmd_buf, "RESET", 5) == 0) {
            devctl(gate_fd, DCMD_SAFETY_RESET, NULL, 0, NULL);
            continue;
        }

        if (strncmp(cmd_buf, "LATCH ", 6) == 0) {
            int32_t val = atoi(cmd_buf + 6);
            devctl(gate_fd, DCMD_SAFETY_SET_LATCH, &val, sizeof(val), NULL);
            continue;
        }

        if (strncmp(cmd_buf, "CHAOS", 5) == 0) {
            int rc = system("slay vision_worker >/dev/null 2>&1");
            (void)rc;
            continue;
        }

        safety_status_t st;
        memset(&st, 0, sizeof(st));
        if (devctl(gate_fd, DCMD_SAFETY_GET_STATUS, &st, sizeof(st), NULL) != 0) {
            continue;
        }

        udp_telemetry_pkt_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.magic                = TELEMETRY_MAGIC;
        pkt.state                = st.state;
        pkt.latched              = st.latched;
        pkt.latch_enabled        = st.latch_enabled;
        pkt.trip_count           = st.trip_count;
        pkt.ossd_mv              = st.ossd_mv;
        pkt.frame_id             = st.frame_id;
        pkt.latency_us           = st.latency_us;
        pkt.last_trip_us         = st.last_trip_us;
        pkt.min_zone_distance_px = st.min_zone_distance_px;
        pkt.bbox_x               = st.bbox_x;
        pkt.bbox_y               = st.bbox_y;
        pkt.bbox_w               = st.bbox_w;
        pkt.bbox_h               = st.bbox_h;
        pkt.frame_w              = st.frame_w;
        pkt.frame_h              = st.frame_h;
        pkt.num_pts              = st.zone.num_pts;
        memcpy(pkt.vx, st.zone.vx, sizeof(pkt.vx));
        memcpy(pkt.vy, st.zone.vy, sizeof(pkt.vy));

        sendto(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&client_addr, client_len);
    }

    close(sock);
    close(gate_fd);
    return 0;
}
"""

FILES["dashboard.py"] = r'''import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import shutil
import socket
import struct
import subprocess
import tempfile
import threading
from urllib.parse import unquote

PKT_FMT = "<IiiiIIIQQfHHHHHH I 8f 8f"
PKT_SIZE = struct.calcsize(PKT_FMT)
TELEMETRY_MAGIC = 0x514E5838
FRAME_MAGIC = 0x46524D38
CAM_INGEST_MAGIC = 0x43414D38

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Workcell Guard — QNX 8.0 Type-4 Optical Safety Controller</title>
<style>
  :root {
    --bg: #09090b;
    --surface: #111113;
    --surface-2: #18181b;
    --border: #27272a;
    --border-strong: #3f3f46;
    --text: #fafafa;
    --text-muted: #a1a1aa;
    --text-dim: #71717a;
    --ok: #10b981;
    --ok-bg: rgba(16, 185, 129, 0.12);
    --warn: #f59e0b;
    --warn-bg: rgba(245, 158, 11, 0.12);
    --danger: #ef4444;
    --danger-bg: rgba(239, 68, 68, 0.14);
    --mono: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace;
    --sans: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  }

  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--text);
    font-family: var(--sans); font-size: 13px; line-height: 1.5;
    -webkit-font-smoothing: antialiased;
  }

  .topbar {
    position: sticky; top: 0; z-index: 20;
    background: rgba(9, 9, 11, 0.9); backdrop-filter: blur(12px);
    border-bottom: 1px solid var(--border);
  }
  .topbar-inner {
    max-width: 1380px; margin: 0 auto; padding: 12px 24px;
    display: flex; justify-content: space-between; align-items: center; gap: 16px;
  }
  .brand { display: flex; align-items: center; gap: 12px; }
  .brand-mark {
    width: 28px; height: 28px; border-radius: 6px;
    background: var(--surface-2); border: 1px solid var(--border-strong);
    display: grid; place-items: center; font-family: var(--mono);
    font-weight: 700; font-size: 12px; color: var(--ok);
  }
  .brand h1 { margin: 0; font-size: 15px; font-weight: 600; letter-spacing: -0.01em; }
  .brand span { font-size: 11px; color: var(--text-dim); font-family: var(--mono); }

  .top-actions { display: flex; align-items: center; gap: 10px; }
  .status-pill {
    display: inline-flex; align-items: center; gap: 8px;
    padding: 6px 14px; border-radius: 999px;
    font-family: var(--mono); font-size: 12px; font-weight: 600;
    border: 1px solid var(--border); background: var(--surface); color: var(--text-muted);
  }
  .status-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--text-dim); }
  .status-pill.SAFE_RUN { background: var(--ok-bg); border-color: rgba(16,185,129,0.4); color: var(--ok); }
  .status-pill.SAFE_RUN .status-dot { background: var(--ok); box-shadow: 0 0 10px var(--ok); }
  .status-pill.WARN_SLS { background: var(--warn-bg); border-color: rgba(245,158,11,0.45); color: var(--warn); }
  .status-pill.WARN_SLS .status-dot { background: var(--warn); box-shadow: 0 0 10px var(--warn); }
  .status-pill.INTRUSION_HALT { background: var(--danger-bg); border-color: rgba(239,68,68,0.5); color: var(--danger); }
  .status-pill.INTRUSION_HALT .status-dot { background: var(--danger); box-shadow: 0 0 12px var(--danger); }
  .status-pill.WATCHDOG_FAULT { background: var(--danger-bg); border-color: rgba(245,158,11,0.5); color: var(--warn); }
  .status-pill.WATCHDOG_FAULT .status-dot { background: var(--warn); }

  .container {
    max-width: 1380px; margin: 0 auto; padding: 20px 24px;
    display: flex; flex-direction: column; gap: 18px;
  }

  .kpi-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 12px; }
  .kpi {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 12px 14px;
  }
  .kpi-label { font-size: 11px; color: var(--text-muted); margin-bottom: 3px; }
  .kpi-value {
    font-family: var(--mono); font-size: 18px; font-weight: 600;
    font-variant-numeric: tabular-nums; color: var(--text);
  }
  .kpi-sub { font-size: 11px; color: var(--text-dim); font-family: var(--mono); margin-top: 2px; }

  .workspace {
    display: grid; grid-template-columns: 1.45fr 1fr; gap: 18px; align-items: start;
  }
  .panel {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 10px; overflow: hidden;
  }
  .panel-header {
    padding: 10px 14px; border-bottom: 1px solid var(--border);
    display: flex; justify-content: space-between; align-items: center; gap: 8px;
  }
  .panel-title { font-size: 13px; font-weight: 600; color: var(--text); }
  .panel-meta { font-family: var(--mono); font-size: 11px; color: var(--text-dim); }
  .panel-body { padding: 14px; }

  .viewport-wrap {
    position: relative; background: #050506;
    border: 1px solid var(--border); border-radius: 6px;
    overflow: hidden; aspect-ratio: 4 / 3; cursor: crosshair;
  }
  canvas { width: 100%; height: 100%; display: block; }

  .toolbar-strip {
    display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center;
    gap: 8px; margin-top: 10px;
  }
  .legend { display: flex; flex-wrap: wrap; gap: 14px; font-size: 11px; color: var(--text-muted); }
  .legend-item { display: inline-flex; align-items: center; gap: 6px; }
  .swatch { width: 9px; height: 9px; border-radius: 2px; }

  .right-stack { display: flex; flex-direction: column; gap: 16px; }

  .twin-wrap {
    background: #070709; border: 1px solid var(--border); border-radius: 6px; overflow: hidden;
  }
  .relay-grid {
    display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 10px;
  }
  .relay-box {
    background: var(--surface-2); border: 1px solid var(--border);
    border-radius: 6px; padding: 8px 10px; font-family: var(--mono);
  }
  .relay-lbl { font-size: 10px; color: var(--text-dim); }
  .relay-val { font-size: 13px; font-weight: 600; margin-top: 2px; }

  .btn-row { display: flex; flex-wrap: wrap; gap: 8px; }
  button {
    flex: 1; padding: 7px 11px; border-radius: 6px;
    font-family: var(--sans); font-size: 12px; font-weight: 500;
    cursor: pointer; border: 1px solid var(--border-strong);
    background: var(--surface-2); color: var(--text);
    transition: background 0.15s, border-color 0.15s;
  }
  button:hover { background: #27272a; }
  button.primary { background: var(--text); color: var(--bg); border-color: var(--text); font-weight: 600; }
  button.primary:hover { background: #e4e4e7; }
  button.danger-btn { border-color: rgba(239,68,68,0.45); color: #fca5a5; }
  button.danger-btn:hover { background: rgba(239,68,68,0.16); }

  .dropzone {
    border: 1px dashed var(--border-strong); background: var(--surface-2);
    border-radius: 8px; padding: 12px; text-align: center; cursor: pointer;
  }
  .dropzone:hover, .dropzone.dragover { border-color: var(--text-muted); background: #1f1f23; }
  .dropzone-title { font-size: 12px; font-weight: 500; color: var(--text); }
  .dropzone-sub { font-size: 11px; color: var(--text-dim); margin-top: 2px; }

  .console-box {
    background: #050506; font-family: var(--mono); font-size: 12px;
    line-height: 1.6; height: 220px; overflow-y: auto; padding: 10px 14px;
  }
  .log-line { display: flex; gap: 10px; border-bottom: 1px solid rgba(39,39,42,0.35); padding: 2px 0; }
  .log-ts { color: var(--text-dim); flex-shrink: 0; }
  .log-tag { flex-shrink: 0; font-weight: 600; width: 78px; }
  .log-ok { color: var(--ok); }
  .log-warn { color: var(--warn); }
  .log-danger { color: var(--danger); }

  .filter-btn {
    flex: 0 0 auto; padding: 3px 9px; font-size: 11px; border-radius: 5px;
    background: transparent; border: 1px solid var(--border); color: var(--text-muted);
  }
  .filter-btn.active { background: var(--surface-2); border-color: var(--border-strong); color: var(--text); }

  .topo-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 10px; }
  .topo-node {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 10px 12px;
  }
  .topo-prio { font-family: var(--mono); font-size: 10px; color: var(--text-dim); }
  .topo-name { font-family: var(--mono); font-size: 12px; font-weight: 600; color: var(--text); margin: 2px 0; }
  .topo-desc { font-size: 11px; color: var(--text-muted); }

  @media (max-width: 1024px) {
    .kpi-grid { grid-template-columns: repeat(2, 1fr); }
    .workspace { grid-template-columns: 1fr; }
    .topo-grid { grid-template-columns: repeat(2, 1fr); }
  }
</style>
</head>
<body>

<header class="topbar">
  <div class="topbar-inner">
    <div class="brand">
      <div class="brand-mark">Q8</div>
      <div>
        <h1>Workcell Guard &bull; Type-4 Virtual Light Curtain</h1>
        <span>QNX 8.0 Neutrino RTOS &bull; /dev/safety_gate (ISO 13849-1 Cat. 4)</span>
      </div>
    </div>
    <div class="top-actions">
      <button class="primary" style="flex:0 0 auto; padding:6px 14px;" onclick="resetGate()">Reset Interlock</button>
      <div id="stateBadge" class="status-pill">
        <span class="status-dot"></span>
        <span id="stateText">CONNECTING...</span>
      </div>
    </div>
  </div>
</header>

<main class="container">
  <section class="kpi-grid">
    <div class="kpi">
      <div class="kpi-label">OSSD Safety Relay Output</div>
      <div class="kpi-value" id="mOssd" style="color:var(--ok);">24.0 V DC</div>
      <div class="kpi-sub" id="mBrake">Mechanical Brake: RELEASED</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Deterministic IPC Latency</div>
      <div class="kpi-value" id="mLatency">-- us</div>
      <div class="kpi-sub" id="mTripLat">Last E-Stop trip: -- us</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Laser Curtain Clearance</div>
      <div class="kpi-value" id="mDist">-- px</div>
      <div class="kpi-sub">SLS Warn &le; 45 px &bull; Cutoff = 0 px</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Processed Frame / Trips</div>
      <div class="kpi-value" id="mFrame">#0</div>
      <div class="kpi-sub" id="mTrips">Safety trips: 0</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Active Video Stream</div>
      <div class="kpi-value" id="mSource" style="font-size:14px; padding-top:3px;">Synthetic Stream</div>
      <div class="kpi-sub">320&times;240 HD /dev/shmem</div>
    </div>
  </section>

  <section class="workspace">
    <div class="panel">
      <div class="panel-header">
        <span class="panel-title">Optical Light Curtain &amp; Custom Polygon Boundary</span>
        <span class="panel-meta">Drag vertices to reshape &bull; Double-click edge to add point</span>
      </div>
      <div class="panel-body">
        <div class="viewport-wrap">
          <canvas id="camCanvas" width="960" height="720"></canvas>
        </div>
        <div class="toolbar-strip">
          <div class="legend">
            <span class="legend-item"><span class="swatch" style="background:#10b981;"></span>Clear Laser Curtain (24V)</span>
            <span class="legend-item"><span class="swatch" style="background:#f59e0b;"></span>Proximity SLS Throttle</span>
            <span class="legend-item"><span class="swatch" style="background:#ef4444;"></span>Intrusion Cutoff (0V)</span>
          </div>
          <div class="btn-row" style="flex:0 0 auto;">
            <button style="padding:4px 9px; font-size:11px;" onclick="presetPoly('right')">Right Hazard</button>
            <button style="padding:4px 9px; font-size:11px;" onclick="presetPoly('trap')">Angled Table</button>
            <button style="padding:4px 9px; font-size:11px;" onclick="presetPoly('lshape')">L-Guard</button>
          </div>
        </div>
      </div>
    </div>

    <div class="right-stack">
      <div class="panel">
        <div class="panel-header">
          <span class="panel-title">Protected Actuator: 40-Ton Servo Stamping Press</span>
          <span class="panel-meta" id="pressMode">NOMINAL 100%</span>
        </div>
        <div class="panel-body">
          <div class="twin-wrap">
            <canvas id="pressCanvas" width="480" height="190"></canvas>
          </div>
          <div class="relay-grid">
            <div class="relay-box">
              <div class="relay-lbl">OSSD CH1 / CH2</div>
              <div class="relay-val" id="rOssd" style="color:var(--ok);">24.0V / 24.0V</div>
            </div>
            <div class="relay-box">
              <div class="relay-lbl">SERVO DRIVE</div>
              <div class="relay-val" id="rRpm" style="color:var(--ok);">120 SPM</div>
            </div>
            <div class="relay-box">
              <div class="relay-lbl">ISO-13849 LATCH</div>
              <div class="relay-val" id="rLatch">ARMED (ON)</div>
            </div>
          </div>
          <div class="btn-row" style="margin-top:10px;">
            <button class="primary" onclick="resetGate()">Reset E-Stop Interlock</button>
            <button id="latchBtn" onclick="toggleLatch()">Latch Mode: ON</button>
            <button class="danger-btn" onclick="triggerChaos()">Inject Vision Crash</button>
          </div>
        </div>
      </div>

      <div class="panel">
        <div class="panel-header">
          <span class="panel-title">Workcell Camera Feed Source</span>
          <span class="panel-meta">TCP :9877 &rarr; /dev/shmem</span>
        </div>
        <div class="panel-body">
          <input type="file" id="videoInput" accept="video/*,.bin" style="display:none" onchange="uploadVideo(this.files[0])">
          <div class="dropzone" id="dropzone" onclick="document.getElementById('videoInput').click()">
            <div class="dropzone-title" id="uploadTitle">Click or drop a test video (.mp4, .mov)</div>
            <div class="dropzone-sub">Streams crisp 320&times;240 frames directly into QNX shared memory</div>
          </div>
          <div class="btn-row" style="margin-top:8px;">
            <button onclick="useSynthetic()">Use Synthetic Workcell Feed</button>
          </div>
        </div>
      </div>
    </div>
  </section>

  <section class="panel">
    <div class="panel-header">
      <div style="display:flex; align-items:center; gap:10px;">
        <span class="panel-title">Live QNX Kernel &amp; Safety Gate Event Stream</span>
        <span class="panel-meta" id="logCount">0 events</span>
      </div>
      <div style="display:flex; align-items:center; gap:6px;">
        <button class="filter-btn active" onclick="setFilter('ALL', this)">All</button>
        <button class="filter-btn" onclick="setFilter('OK', this)">OK (Green)</button>
        <button class="filter-btn" onclick="setFilter('WARN', this)">Warn (Amber)</button>
        <button class="filter-btn" onclick="setFilter('DANGER', this)">Halt (Red)</button>
        <button class="filter-btn" onclick="clearLogs()">Clear</button>
      </div>
    </div>
    <div class="console-box" id="consoleBox"></div>
  </section>

  <section class="topo-grid">
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 25</div>
      <div class="topo-name">/dev/safety_gate</div>
      <div class="topo-desc">24V OSSD relay manager &amp; 5ms watchdog</div>
    </div>
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 20</div>
      <div class="topo-name">supervisor</div>
      <div class="topo-desc">Self-healing process monitor (250ms respawn)</div>
    </div>
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 15</div>
      <div class="topo-name">cam_feed</div>
      <div class="topo-desc">30 FPS zero-copy /dev/shmem producer</div>
    </div>
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 10</div>
      <div class="topo-name">vision_worker</div>
      <div class="topo-desc">2D polygon clearance &amp; cell-density CV</div>
    </div>
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 5</div>
      <div class="topo-name">telemetry_bridge</div>
      <div class="topo-desc">devctl gateway + HD TCP video server</div>
    </div>
  </section>
</main>

<script>
const camCanvas = document.getElementById('camCanvas');
const ctx = camCanvas.getContext('2d');
const offCanvas = document.createElement('canvas');
const offCtx = offCanvas.getContext('2d');

const pressCanvas = document.getElementById('pressCanvas');
const pctx = pressCanvas.getContext('2d');
const consoleBox = document.getElementById('consoleBox');

const WARN_CLEARANCE_PX = 45.0;
let logFilter = 'ALL';
let logs = [];
let lastFrameLogged = -1;
let lastSeverity = '';
let lastHeartbeatMs = 0;
let connected = false;

let polyPts = [
  { x: 0.50, y: 0.20 },
  { x: 0.92, y: 0.20 },
  { x: 0.92, y: 0.85 },
  { x: 0.50, y: 0.85 }
];
let dragIdx = -1;
let hoverIdx = -1;
let isDragging = false;

let telem = null;
let latestFrameImg = null;
let pressPhase = 0;
let scanPhase = 0;

function tsNow() {
  const d = new Date();
  return d.toTimeString().split(' ')[0] + '.' + String(d.getMilliseconds()).padStart(3, '0');
}

function pushLog(level, tag, msg) {
  logs.push({ ts: tsNow(), level, tag, msg });
  if (logs.length > 200) logs.shift();
  document.getElementById('logCount').innerText = logs.length + ' events';
  if (logFilter !== 'ALL' && logFilter !== level) return;
  appendLogDom(logs[logs.length - 1]);
}

function appendLogDom(entry) {
  const cls = entry.level === 'OK' ? 'log-ok' : (entry.level === 'WARN' ? 'log-warn' : 'log-danger');
  const row = document.createElement('div');
  row.className = 'log-line ' + cls;
  row.innerHTML = `<span class="log-ts">${entry.ts}</span><span class="log-tag">[${entry.tag}]</span><span>${entry.msg}</span>`;
  consoleBox.appendChild(row);
  while (consoleBox.children.length > 200) consoleBox.removeChild(consoleBox.firstChild);
  consoleBox.scrollTop = consoleBox.scrollHeight;
}

function setFilter(f, btn) {
  logFilter = f;
  document.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
  consoleBox.innerHTML = '';
  logs.filter(l => f === 'ALL' || l.level === f).forEach(appendLogDom);
}

function clearLogs() {
  logs = [];
  consoleBox.innerHTML = '';
  document.getElementById('logCount').innerText = '0 events';
}

function getNormCoords(e) {
  const rect = camCanvas.getBoundingClientRect();
  return {
    x: Math.max(0.02, Math.min(0.98, (e.clientX - rect.left) / rect.width)),
    y: Math.max(0.02, Math.min(0.98, (e.clientY - rect.top) / rect.height))
  };
}

camCanvas.addEventListener('mousedown', e => {
  const p = getNormCoords(e);
  for (let i = 0; i < polyPts.length; i++) {
    if (Math.hypot(polyPts[i].x - p.x, polyPts[i].y - p.y) < 0.035) {
      dragIdx = i;
      isDragging = true;
      return;
    }
  }
});

camCanvas.addEventListener('mousemove', e => {
  const p = getNormCoords(e);
  if (isDragging && dragIdx >= 0) {
    polyPts[dragIdx] = p;
    return;
  }
  hoverIdx = -1;
  for (let i = 0; i < polyPts.length; i++) {
    if (Math.hypot(polyPts[i].x - p.x, polyPts[i].y - p.y) < 0.035) {
      hoverIdx = i;
      break;
    }
  }
  camCanvas.style.cursor = hoverIdx >= 0 ? 'grab' : 'crosshair';
});

window.addEventListener('mouseup', () => {
  if (isDragging) {
    isDragging = false;
    dragIdx = -1;
    sendPolygonToQnx();
  }
});

camCanvas.addEventListener('dblclick', e => {
  if (polyPts.length >= 8) return;
  const p = getNormCoords(e);
  let bestEdge = 0, bestD = 999;
  for (let i = 0; i < polyPts.length; i++) {
    const a = polyPts[i], b = polyPts[(i + 1) % polyPts.length];
    const mx = (a.x + b.x) * 0.5, my = (a.y + b.y) * 0.5;
    const d = Math.hypot(mx - p.x, my - p.y);
    if (d < bestD) { bestD = d; bestEdge = i + 1; }
  }
  polyPts.splice(bestEdge, 0, p);
  sendPolygonToQnx();
});

function presetPoly(kind) {
  if (kind === 'right') {
    polyPts = [{x:0.50,y:0.20},{x:0.92,y:0.20},{x:0.92,y:0.85},{x:0.50,y:0.85}];
  } else if (kind === 'trap') {
    polyPts = [{x:0.54,y:0.18},{x:0.90,y:0.28},{x:0.94,y:0.86},{x:0.46,y:0.78}];
  } else if (kind === 'lshape') {
    polyPts = [{x:0.55,y:0.15},{x:0.92,y:0.15},{x:0.92,y:0.88},{x:0.42,y:0.88},{x:0.42,y:0.55},{x:0.55,y:0.55}];
  }
  sendPolygonToQnx();
}

async function sendPolygonToQnx() {
  await fetch('/api/zone', { method: 'POST', body: JSON.stringify({ pts: polyPts }) });
  pushLog('WARN', 'DEVCTL', `DCMD_SAFETY_SET_ZONE pushed ${polyPts.length}-vertex custom polygon to /dev/safety_gate`);
}

async function resetGate() {
  await fetch('/api/reset', { method: 'POST' });
  pushLog('OK', 'DEVCTL', 'DCMD_SAFETY_RESET -> Interlock cleared, OSSD 24.0V DC re-energized');
}

async function toggleLatch() {
  const next = (telem && telem.latch_enabled) ? 0 : 1;
  await fetch('/api/latch', { method: 'POST', body: JSON.stringify({ enabled: next }) });
  pushLog('WARN', 'DEVCTL', `DCMD_SAFETY_SET_LATCH -> ISO 13849-1 Latch Mode ${next ? 'ENABLED' : 'DISABLED'}`);
}

async function triggerChaos() {
  pushLog('DANGER', 'CHAOS', 'Injecting SIGKILL into vision_worker PID -> testing QNX 150ms watchdog & 250ms supervisor...');
  await fetch('/api/chaos', { method: 'POST' });
}

const dz = document.getElementById('dropzone');
dz.addEventListener('dragover', e => { e.preventDefault(); dz.classList.add('dragover'); });
dz.addEventListener('dragleave', () => dz.classList.remove('dragover'));
dz.addEventListener('drop', e => {
  e.preventDefault(); dz.classList.remove('dragover');
  if (e.dataTransfer.files.length) uploadVideo(e.dataTransfer.files[0]);
});

async function uploadVideo(file) {
  if (!file) return;
  const title = document.getElementById('uploadTitle');
  title.innerText = 'Streaming ' + file.name + ' to QNX...';
  pushLog('WARN', 'INGEST', `Decoding "${file.name}" to 320x240 grayscale -> /dev/shmem/workcell_cam`);
  try {
    const res = await fetch('/api/upload-video', {
      method: 'POST',
      headers: { 'X-Filename': encodeURIComponent(file.name) },
      body: file
    });
    const out = await res.json();
    if (out.ok) {
      title.innerText = 'Active: ' + file.name + ' (' + out.frames + ' frames)';
      document.getElementById('mSource').innerText = file.name;
      pushLog('OK', 'CAM_FEED', `Hot-swapped /dev/shmem/workcell_cam -> ${file.name} (${out.frames} frames @ 30 FPS)`);
    } else {
      title.innerText = 'Upload failed — click to retry';
      pushLog('DANGER', 'INGEST', out.error || 'Video decode error');
    }
  } catch (err) {
    pushLog('DANGER', 'INGEST', 'Upload error: ' + err);
  }
}

async function useSynthetic() {
  await fetch('/api/use-synthetic', { method: 'POST' });
  document.getElementById('uploadTitle').innerText = 'Click or drop a test video (.mp4, .mov)';
  document.getElementById('mSource').innerText = 'Synthetic Stream';
  pushLog('WARN', 'CAM_FEED', 'Switched /dev/shmem/workcell_cam back to synthetic workcell stream');
}

async function pollTelemetry() {
  try {
    const res = await fetch('/api/telemetry');
    const d = await res.json();
    if (!d.ok) {
      if (connected) {
        connected = false;
        pushLog('DANGER', 'LINK', 'Lost UDP telemetry connection to QNX node');
      }
      return;
    }
    telem = d;
    if (!connected) {
      connected = true;
      pushLog('OK', 'LINK', 'Connected to QNX 8.0 /dev/safety_gate (OSSD 24V safety relay online)');
    }

    if (!isDragging && d.pts && d.pts.length >= 3) {
      polyPts = d.pts;
    }

    const isProx = (d.state === 0 && d.bbox[2] > 0 && d.dist_px <= WARN_CLEARANCE_PX);
    const badge = document.getElementById('stateBadge');
    badge.className = 'status-pill ' + (
      d.state === 2 ? 'WATCHDOG_FAULT' :
      d.state === 1 ? 'INTRUSION_HALT' :
      isProx ? 'WARN_SLS' : 'SAFE_RUN'
    );
    document.getElementById('stateText').innerText =
      d.state === 2 ? 'WATCHDOG_FAULT — RELAY CUTOFF' :
      d.state === 1 ? (d.latched ? 'INTRUSION_HALT — LATCHED E-STOP' : 'INTRUSION_HALT — E-STOP') :
      isProx ? 'PROXIMITY — SLS THROTTLED' : 'SAFE_RUN — CURTAIN CLEAR';

    const voltStr = (d.ossd_mv / 1000).toFixed(1) + ' V DC';
    const mOssd = document.getElementById('mOssd');
    mOssd.innerText = voltStr;
    mOssd.style.color = d.ossd_mv > 0 ? 'var(--ok)' : 'var(--danger)';
    document.getElementById('mBrake').innerText = d.ossd_mv > 0 ? 'Mechanical Brake: RELEASED' : 'Mechanical Brake: ENGAGED (LOCKED)';

    document.getElementById('mLatency').innerText = d.latency_us + ' us';
    document.getElementById('mTripLat').innerText = d.last_trip_us > 0 ? `Last E-Stop trip: ${d.last_trip_us} us` : 'Last E-Stop trip: none';
    document.getElementById('mDist').innerText = d.dist_px >= 900 ? 'Clear' : d.dist_px.toFixed(1) + ' px';
    document.getElementById('mFrame').innerText = '#' + d.frame_id;
    document.getElementById('mTrips').innerText = 'Safety trips: ' + d.trip_count;

    document.getElementById('rOssd').innerText = d.ossd_mv > 0 ? '24.0V / 24.0V' : '0.0V / 0.0V';
    document.getElementById('rOssd').style.color = d.ossd_mv > 0 ? 'var(--ok)' : 'var(--danger)';
    document.getElementById('rRpm').innerText = d.ossd_mv === 0 ? '0 SPM (HALT)' : (isProx ? '30 SPM (SLS)' : '120 SPM');
    document.getElementById('rRpm').style.color = d.ossd_mv === 0 ? 'var(--danger)' : (isProx ? 'var(--warn)' : 'var(--ok)');
    document.getElementById('rLatch').innerText = d.latch_enabled ? (d.latched ? 'TRIPPED (LOCKED)' : 'ARMED (ON)') : 'AUTO-RESET (OFF)';
    document.getElementById('latchBtn').innerText = 'Latch Mode: ' + (d.latch_enabled ? 'ON' : 'OFF');
    document.getElementById('pressMode').innerText = d.ossd_mv === 0 ? 'BRAKE LOCKED' : (isProx ? 'SLS 25% SPEED' : 'NOMINAL 100%');

    const severity = (d.state === 2 || d.state === 1) ? 'DANGER' : (isProx ? 'WARN' : 'OK');
    const nowMs = Date.now();
    if (d.frame_id !== lastFrameLogged && (severity !== lastSeverity || (nowMs - lastHeartbeatMs >= 1000))) {
      lastFrameLogged = d.frame_id;
      lastSeverity = severity;
      lastHeartbeatMs = nowMs;

      if (d.state === 2) {
        pushLog('DANGER', 'WATCHDOG', `HEARTBEAT LOST (>150ms) -> /dev/safety_gate dropped OSSD 24V->0V & locked press brake`);
      } else if (d.state === 1) {
        pushLog('DANGER', 'E-STOP', `INTRUSION at Frame #${d.frame_id} | Clearance: 0.0 px | IPC: ${d.latency_us} us -> OSSD 0V CUTOFF`);
      } else if (isProx) {
        pushLog('WARN', 'SLS-WARN', `Proximity breach (${d.dist_px.toFixed(1)} px) -> Press throttled to 25% speed`);
      } else {
        const clr = d.dist_px >= 900 ? 'zone clear' : `clearance ${d.dist_px.toFixed(1)} px`;
        pushLog('OK', 'OK', `Curtain clear (${clr}) | OSSD 24.0V DC | Press 120 SPM | IPC ${d.latency_us} us`);
      }
    }
  } catch (e) {}
}

async function pollFrame() {
  try {
    const res = await fetch('/api/frame');
    const f = await res.json();
    if (f.ok && f.w > 0 && f.h > 0 && f.pixels_b64) {
      offCanvas.width = f.w;
      offCanvas.height = f.h;
      const raw = atob(f.pixels_b64);
      const img = offCtx.createImageData(f.w, f.h);
      for (let i = 0; i < raw.length; i++) {
        const v = raw.charCodeAt(i);
        img.data[i * 4] = v;
        img.data[i * 4 + 1] = v;
        img.data[i * 4 + 2] = v;
        img.data[i * 4 + 3] = 255;
      }
      offCtx.putImageData(img, 0, 0);
      latestFrameImg = offCanvas;
    }
  } catch (e) {}
}

function renderLoop() {
  const W = camCanvas.width, H = camCanvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#09090b';
  ctx.fillRect(0, 0, W, H);

  if (latestFrameImg) {
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(latestFrameImg, 0, 0, W, H);
  }

  const st = telem ? telem.state : 0;
  const isProx = (telem && st === 0 && telem.bbox[2] > 0 && telem.dist_px <= WARN_CLEARANCE_PX);
  const zoneColor = (st === 1 || st === 2) ? '#ef4444' : (isProx ? '#f59e0b' : '#10b981');
  const zoneFill  = (st === 1 || st === 2) ? 'rgba(239, 68, 68, 0.22)' : (isProx ? 'rgba(245, 158, 11, 0.14)' : 'rgba(16, 185, 129, 0.08)');

  if (polyPts.length >= 3) {
    ctx.save();
    ctx.beginPath();
    polyPts.forEach((pt, idx) => {
      const px = pt.x * W, py = pt.y * H;
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.closePath();
    ctx.fillStyle = zoneFill;
    ctx.fill();

    ctx.save();
    ctx.clip();
    scanPhase = (scanPhase + (st === 0 ? 2.5 : 0)) % 28;
    ctx.strokeStyle = (st === 0) ? 'rgba(16, 185, 129, 0.22)' : 'rgba(239, 68, 68, 0.25)';
    ctx.lineWidth = 1;
    for (let y = scanPhase; y < H; y += 28) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(W, y);
      ctx.stroke();
    }
    ctx.restore();

    ctx.strokeStyle = zoneColor;
    ctx.lineWidth = 1.5;
    ctx.stroke();

    polyPts.forEach((pt, idx) => {
      const px = pt.x * W, py = pt.y * H;
      ctx.beginPath();
      ctx.arc(px, py, (idx === hoverIdx || idx === dragIdx) ? 6.5 : 4.5, 0, Math.PI * 2);
      ctx.fillStyle = '#09090b';
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = zoneColor;
      ctx.stroke();
    });
    ctx.restore();
  }

  if (telem && telem.bbox[2] > 0 && telem.bbox[3] > 0 && telem.frame_w > 0) {
    const sx = W / telem.frame_w, sy = H / telem.frame_h;
    const bx = telem.bbox[0] * sx, by = telem.bbox[1] * sy;
    const bw = telem.bbox[2] * sx, bh = telem.bbox[3] * sy;
    const boxColor = (st === 1) ? '#ef4444' : (isProx ? '#f59e0b' : '#38bdf8');

    ctx.strokeStyle = boxColor;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(bx, by, bw, bh);

    ctx.fillStyle = boxColor;
    ctx.font = '600 12px ui-monospace, monospace';
    const lbl = st === 1 ? 'INTRUSION [0.0 px]' : `TRACKED [${telem.dist_px.toFixed(1)} px]`;
    ctx.fillText(lbl, bx, Math.max(16, by - 6));
  }

  drawServoPressTwin(st, isProx);
  requestAnimationFrame(renderLoop);
}

function drawServoPressTwin(st, isProx) {
  const W = pressCanvas.width, H = pressCanvas.height;
  pctx.clearRect(0, 0, W, H);
  pctx.fillStyle = '#070709';
  pctx.fillRect(0, 0, W, H);

  const running = (telem && telem.ossd_mv > 0);
  if (running) {
    pressPhase += isProx ? 0.025 : 0.095;
  }
  const stroke = (Math.sin(pressPhase) + 1) * 0.5;
  const ramY = 36 + stroke * 68;
  const accent = !running ? '#ef4444' : (isProx ? '#f59e0b' : '#10b981');

  pctx.fillStyle = accent;
  pctx.fillRect(0, 0, W, 3);

  pctx.strokeStyle = '#3f3f46';
  pctx.lineWidth = 1.5;
  pctx.strokeRect(120, 18, 240, 18);
  pctx.strokeRect(120, 152, 240, 22);
  pctx.strokeStyle = '#27272a';
  pctx.strokeRect(136, 36, 12, 116);
  pctx.strokeRect(332, 36, 12, 116);

  pctx.fillStyle = !running ? 'rgba(239,68,68,0.22)' : (isProx ? 'rgba(245,158,11,0.18)' : 'rgba(16,185,129,0.18)');
  pctx.strokeStyle = accent;
  pctx.lineWidth = 1.5;
  pctx.fillRect(160, ramY, 160, 26);
  pctx.strokeRect(160, ramY, 160, 26);

  pctx.beginPath();
  pctx.moveTo(240, 36);
  pctx.lineTo(240, ramY);
  pctx.stroke();

  const partX = 160 + ((pressPhase * 28) % 140);
  pctx.fillStyle = '#52525b';
  pctx.fillRect(partX, 142, 24, 10);

  pctx.fillStyle = accent;
  pctx.fillRect(102, 42, 5, 104);
  pctx.fillRect(373, 42, 5, 104);

  pctx.strokeStyle = !running ? 'rgba(239,68,68,0.55)' : (isProx ? 'rgba(245,158,11,0.45)' : 'rgba(16,185,129,0.45)');
  pctx.lineWidth = 1;
  for (let y = 48; y <= 140; y += 15) {
    pctx.beginPath();
    pctx.moveTo(107, y);
    pctx.lineTo(373, y);
    pctx.stroke();
  }

  pctx.fillStyle = accent;
  pctx.font = '600 11px ui-monospace, monospace';
  const statusMsg = !running
    ? (st === 2 ? 'E-STOP: WATCHDOG CUTOFF (BRAKE LOCKED)' : 'E-STOP: LIGHT CURTAIN TRIPPED (0V)')
    : (isProx ? 'SLS MODE: REDUCED SPEED (25%)' : 'RUN: OSSD 24V ARMED — CURTAIN CLEAR');
  pctx.fillText(statusMsg, 14, 22);
}

async function telemetryLoop() {
  await pollTelemetry();
  setTimeout(telemetryLoop, 45);
}

async function frameLoop() {
  await pollFrame();
  setTimeout(frameLoop, 45);
}

telemetryLoop();
frameLoop();
requestAnimationFrame(renderLoop);
</script>
</body>
</html>
"""

class FrameClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.lock = threading.Lock()
        self.last_frame = None

    def get_frame(self):
        with self.lock:
            try:
                if self.sock is None:
                    self.sock = socket.create_connection((self.host, self.port), timeout=0.5)
                    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.sendall(b"FRAM")
                hdr = self._recv_exact(16)
                magic, w, h, fid = struct.unpack("<IIII", hdr)
                if magic != FRAME_MAGIC or w == 0 or h == 0:
                    raise RuntimeError("Bad frame header")
                px = self._recv_exact(w * h)
                self.last_frame = (w, h, fid, base64.b64encode(px).decode("ascii"))
            except Exception:
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None
            return self.last_frame

    def _recv_exact(self, n):
        chunks = []
        got = 0
        while got < n:
            part = self.sock.recv(n - got)
            if not part:
                raise ConnectionError("EOF")
            chunks.append(part)
            got += len(part)
        return b"".join(chunks)

def decode_video_to_gray_frames(raw_bytes, filename, w=320, h=240, max_frames=450):
    if filename.lower().endswith(".bin") and len(raw_bytes) >= 12:
        bw, bh, n = struct.unpack("<III", raw_bytes[:12])
        if len(raw_bytes) == 12 + bw * bh * n:
            return bw, bh, n, raw_bytes[12:]

    fd, tmp_path = tempfile.mkstemp(suffix=os.path.splitext(filename)[1] or ".mp4")
    os.close(fd)
    try:
        with open(tmp_path, "wb") as f:
            f.write(raw_bytes)
        try:
            import cv2
            cap = cv2.VideoCapture(tmp_path)
            frames = []
            while len(frames) < max_frames:
                ok, frame = cap.read()
                if not ok:
                    break
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                resized = cv2.resize(gray, (w, h), interpolation=cv2.INTER_AREA)
                frames.append(resized.tobytes())
            cap.release()
            if frames:
                return w, h, len(frames), b"".join(frames)
        except ImportError:
            pass

        if shutil.which("ffmpeg"):
            cmd = [
                "ffmpeg", "-v", "error", "-i", tmp_path,
                "-vf", f"scale={w}:{h},format=gray",
                "-frames:v", str(max_frames),
                "-f", "rawvideo", "pipe:1"
            ]
            proc = subprocess.run(cmd, capture_output=True, check=False)
            frame_sz = w * h
            n = len(proc.stdout) // frame_sz
            if n > 0:
                return w, h, n, proc.stdout[: n * frame_sz]

        raise RuntimeError("Install OpenCV (pip install opencv-python) on Windows to decode .mp4/.mov clips.")
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)

def push_frames_to_qnx(qnx_ip, ingest_port, w, h, nframes, pixel_blob):
    header = struct.pack("<IIII", CAM_INGEST_MAGIC, w, h, nframes)
    with socket.create_connection((qnx_ip, ingest_port), timeout=5.0) as s:
        s.sendall(header)
        if pixel_blob:
            s.sendall(pixel_blob)

class Handler(BaseHTTPRequestHandler):
    qnx_ip = "192.168.184.3"
    telemetry_port = 9876
    ingest_port = 9877
    frame_client = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
        elif self.path == "/api/frame":
            fr = self.frame_client.get_frame()
            if fr:
                self._json({"ok": True, "w": fr[0], "h": fr[1], "fid": fr[2], "pixels_b64": fr[3]})
            else:
                self._json({"ok": False})
        elif self.path == "/api/telemetry":
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(0.25)
            try:
                sock.sendto(b"POLL", (self.qnx_ip, self.telemetry_port))
                data, _ = sock.recvfrom(4096)
                if len(data) >= PKT_SIZE:
                    fields = struct.unpack(PKT_FMT, data[:PKT_SIZE])
                    if fields[0] == TELEMETRY_MAGIC:
                        npts = fields[16]
                        vx = fields[17:25]
                        vy = fields[25:33]
                        pts = [{"x": vx[i], "y": vy[i]} for i in range(min(npts, 8))]
                        self._json({
                            "ok": True,
                            "state": fields[1],
                            "latched": fields[2],
                            "latch_enabled": fields[3],
                            "trip_count": fields[4],
                            "ossd_mv": fields[5],
                            "frame_id": fields[6],
                            "latency_us": fields[7],
                            "last_trip_us": fields[8],
                            "dist_px": fields[9],
                            "bbox": list(fields[10:14]),
                            "frame_w": fields[14],
                            "frame_h": fields[15],
                            "pts": pts,
                        })
                        return
            except Exception:
                pass
            finally:
                sock.close()
            self._json({"ok": False})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b""

        if self.path == "/api/upload-video":
            fname = unquote(self.headers.get("X-Filename", "upload.mp4"))
            try:
                w, h, nframes, blob = decode_video_to_gray_frames(raw, fname)
                push_frames_to_qnx(self.qnx_ip, self.ingest_port, w, h, nframes, blob)
                self._json({"ok": True, "frames": nframes})
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)})
            return

        if self.path == "/api/use-synthetic":
            try:
                push_frames_to_qnx(self.qnx_ip, self.ingest_port, 320, 240, 0, b"")
                self._json({"ok": True})
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)})
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if self.path == "/api/zone":
                data = json.loads(raw.decode("utf-8"))
                pts = data.get("pts", [])
                if 3 <= len(pts) <= 8:
                    coords = " ".join(f"{p['x']:.4f} {p['y']:.4f}" for p in pts)
                    sock.sendto(f"ZONE {len(pts)} {coords}".encode(), (self.qnx_ip, self.telemetry_port))
            elif self.path == "/api/reset":
                sock.sendto(b"RESET", (self.qnx_ip, self.telemetry_port))
            elif self.path == "/api/latch":
                data = json.loads(raw.decode("utf-8"))
                val = 1 if data.get("enabled") else 0
                sock.sendto(f"LATCH {val}".encode(), (self.qnx_ip, self.telemetry_port))
            elif self.path == "/api/chaos":
                sock.sendto(b"CHAOS", (self.qnx_ip, self.telemetry_port))
        finally:
            sock.close()
        self._json({"ok": True})

    def _json(self, obj):
        try:
            body = json.dumps(obj).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)
        except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
            pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--qnx-ip", default="192.168.184.3")
    parser.add_argument("--qnx-port", type=int, default=9876)
    parser.add_argument("--ingest-port", type=int, default=9877)
    parser.add_argument("--frame-port", type=int, default=9878)
    parser.add_argument("--web-port", type=int, default=8080)
    args = parser.parse_args()
    Handler.qnx_ip = args.qnx_ip
    Handler.telemetry_port = args.qnx_port
    Handler.ingest_port = args.ingest_port
    Handler.frame_client = FrameClient(args.qnx_ip, args.frame_port)
    print(f"Workcell Guard Control Plane live at http://localhost:{args.web_port} (QNX {args.qnx_ip})")
    ThreadingHTTPServer(("127.0.0.1", args.web_port), Handler).serve_forever()
'''

for name, content in FILES.items():
    Path(name).write_text(content.lstrip("\n"), encoding="utf-8", newline="\n")
    print(f"Updated {name}")