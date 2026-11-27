#include <devctl.h>
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
#include "vision_core.h"

// Exit codes the supervisor / operator can distinguish.
#define EXIT_CAMERA_STALL 3
#define EXIT_GATE_LOST    4
#define EXIT_ALLOC        5

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
        else return EXIT_FAILURE;
    }

    workcell_config_t cfg;
    if (config_load(cfg_path, &cfg) != 0) return EXIT_FAILURE;

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_vision;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) perror("sched_setscheduler (continuing; gate watchdog covers stalls)");

    int gate_fd = open(cfg.device_path, O_RDWR);
    if (gate_fd < 0) { perror("open safety_gate"); return EXIT_GATE_LOST; }

    int shm_fd = shm_open(cfg.shm_name, O_RDONLY, 0);
    if (shm_fd < 0) { perror("shm_open camera"); return EXIT_FAILURE; }

    shm_camera_t *cam = (shm_camera_t *)mmap(NULL, sizeof(shm_camera_t), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (cam == MAP_FAILED) { perror("mmap"); return EXIT_FAILURE; }

    // Everything is allocated once, up front. A failure here exits non-zero -> gate watchdog trips.
    vision_ctx_t *vc = vision_create();
    uint8_t *frame = (uint8_t *)malloc(MAX_FRAME_PIXELS);
    if (!vc || !frame) { fprintf(stderr, "vision_worker: out of memory\n"); return EXIT_ALLOC; }

    vision_params_t vp = {
        .diff_threshold   = (float)cfg.diff_threshold,
        .cutoff_margin_px = cfg.cutoff_margin_px,
        .freeze_margin_px = cfg.freeze_margin_px,
        .zone_adapt_rate  = 0.002f,
        .cell_hit_min     = (uint32_t)cfg.cell_hit_min,
        .min_blob_cells   = (uint32_t)cfg.min_blob_cells,
        .warmup_frames    = cfg.bg_warmup_frames,
        .learn_rate       = 0.05f,
        .learn_quiet_frames = (uint32_t)cfg.bg_learn_quiet_frames
    };

    printf("[vision_worker] Online at Prio %d | Polygon CV -> %s | stall-exit %dms\n",
           cfg.prio_vision, cfg.device_path, cfg.frame_stall_exit_ms);
    fflush(stdout);

    uint32_t last_seq = 0;
    uint64_t last_new_frame_ns = now_ns();
    const uint64_t stall_ns = (uint64_t)cfg.frame_stall_exit_ms * 1000000ULL;

    while (1) {
        uint32_t seq1 = __atomic_load_n(&cam->write_seq, __ATOMIC_ACQUIRE);
        if ((seq1 & 1U) || seq1 == 0 || seq1 == last_seq) {
            // No new frame. A stalled camera (or a re-created shm object we still map the old copy of)
            // must not leave us silently idle: exit so the supervisor respawns us with a fresh mapping.
            if (now_ns() - last_new_frame_ns > stall_ns) {
                fprintf(stderr, "[vision_worker] no new camera frame for %d ms - exiting for respawn\n",
                        cfg.frame_stall_exit_ms);
                return EXIT_CAMERA_STALL;
            }
            usleep(1000);
            continue;
        }

        // Seqlock read: everything is copied inside the sequence window and re-validated afterwards.
        uint32_t w     = cam->width;
        uint32_t h     = cam->height;
        uint32_t fid   = cam->frame_id;
        uint32_t epoch = cam->stream_epoch;
        uint64_t t_cap = cam->capture_timestamp_ns;
        if (w < 16 || h < 16 || w > MAX_FRAME_DIM || h > MAX_FRAME_DIM || (size_t)w * h > MAX_FRAME_PIXELS) {
            usleep(1000);
            continue;
        }
        memcpy(frame, (const void *)cam->pixels, (size_t)w * h);
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        uint32_t seq2 = __atomic_load_n(&cam->write_seq, __ATOMIC_ACQUIRE);
        if (seq1 != seq2) continue;     // torn read: discard, never process
        last_seq = seq2;
        last_new_frame_ns = now_ns();

        // The gate is the single source of truth for the polygon.
        safety_status_t st;
        if (devctl(gate_fd, DCMD_SAFETY_GET_STATUS, &st, sizeof(st), NULL) != 0) {
            perror("devctl GET_STATUS");
            return EXIT_GATE_LOST;
        }
        if (zone_validate(&st.zone) != ZONE_OK) continue;   // no report -> heartbeat lapses -> gate stays/goes safe

        vision_result_t res;
        if (vision_process(vc, frame, w, h, epoch, &st.zone, &vp, &res) != 0) continue;

        vision_report_t report;
        memset(&report, 0, sizeof(report));
        report.capture_timestamp_ns = t_cap;
        report.frame_id             = fid;
        report.stream_epoch         = epoch;
        report.intrusion_detected   = res.intrusion;
        report.baseline_valid       = res.baseline_valid;
        report.min_zone_distance_px = res.dist_px;
        report.bbox_x               = res.bbox_x;
        report.bbox_y               = res.bbox_y;
        report.bbox_w               = res.bbox_w;
        report.bbox_h               = res.bbox_h;
        report.frame_w              = (uint16_t)w;
        report.frame_h              = (uint16_t)h;
        report.active_zone          = st.zone;

        if (write(gate_fd, &report, sizeof(report)) != (ssize_t)sizeof(report)) {
            perror("write safety_gate");
            return EXIT_GATE_LOST;
        }
    }
}
