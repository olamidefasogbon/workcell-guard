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
            last_seen_seq = 0;
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
