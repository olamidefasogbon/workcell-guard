// Portable vision core: background model + blob detection + polygon clearance.
//
// vision_worker.c does the QNX I/O (shared memory, devctl); everything that
// decides "is something in or near the zone" lives here so it can be tested
// with synthetic frames on any host.
//
// Safety properties this file is responsible for:
//   * Nothing is allocated after vision_create() -> no runtime allocation faults.
//   * The background model is re-seeded ONLY when the camera stream changes
//     (stream_epoch bump, or frame size change) - never by a guess.
//   * The background is never updated inside, or within freeze_margin_px of, the
//     danger zone while anything is visible. A person who walks in and stops can
//     not be "learned" into the background.
//   * EVERY blob is checked against the zone (not just the largest), over the
//     whole frame (no blind borders / blind regions).
#ifndef VISION_CORE_H
#define VISION_CORE_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "safety_protocol.h"

#define VC_CELL     8U
#define VC_QUIET_PX 16U   // at most this many foreground pixels in the zone region still counts as "quiet"
#define VC_MAX_COLS (MAX_FRAME_DIM / VC_CELL)
#define VC_MAX_ROWS (MAX_FRAME_DIM / VC_CELL)

typedef struct {
    float    diff_threshold;     // |pixel - background| above this is foreground
    float    cutoff_margin_px;   // a blob this close to the polygon counts as an intrusion
    float    freeze_margin_px;   // no background updates this close to the polygon
    float    zone_adapt_rate;    // very slow drift compensation inside the freeze region (scene clear only)
    uint32_t cell_hit_min;       // foreground pixels (of 64) for a cell to be active
    uint32_t min_blob_cells;     // smallest blob (in cells) that is reported
    uint32_t warmup_frames;      // minimum learning frames after a (re)seed
    float    learn_rate;         // fast background adaptation while learning (heals a seed that contained a moving object)
    uint32_t learn_quiet_frames; // learning ends only after the zone region has been quiet this many frames in a row
} vision_params_t;

typedef struct {
    uint8_t  intrusion;
    uint8_t  baseline_valid;
    uint8_t  reseeded;           // this frame started a new baseline
    uint32_t blob_count;
    float    dist_px;            // distance of the most hazardous blob (999 = none)
    uint16_t bbox_x, bbox_y, bbox_w, bbox_h;
} vision_result_t;

typedef struct {
    uint32_t w, h, epoch;
    int      have_stream;
    uint32_t frames_since_reseed;
    uint32_t quiet_run;          // consecutive frames with (almost) no foreground in the zone region
    int      learning;           // 1 until the zone region has been quiet long enough
    int      mask_valid;
    safety_zone_t mask_zone;
    uint8_t  smooth[MAX_FRAME_PIXELS];
    float    bg[MAX_FRAME_PIXELS];
    uint8_t  freeze[MAX_FRAME_PIXELS];
    uint8_t  active[VC_MAX_ROWS][VC_MAX_COLS];
    uint8_t  visited[VC_MAX_ROWS][VC_MAX_COLS];
    uint16_t qx[VC_MAX_ROWS * VC_MAX_COLS];
    uint16_t qy[VC_MAX_ROWS * VC_MAX_COLS];
} vision_ctx_t;

static inline vision_ctx_t *vision_create(void) {
    return (vision_ctx_t *)calloc(1, sizeof(vision_ctx_t));   // NULL on failure: caller must exit non-zero
}

static inline void vision_destroy(vision_ctx_t *c) { free(c); }

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------
static inline int vc_point_in_poly(float px, float py, const float *vx, const float *vy, uint32_t n) {
    int inside = 0;
    for (uint32_t i = 0, j = n - 1; i < n; j = i++) {
        if (((vy[i] > py) != (vy[j] > py)) &&
            (px < (vx[j] - vx[i]) * (py - vy[i]) / (vy[j] - vy[i]) + vx[i])) {
            inside = !inside;
        }
    }
    return inside;
}

static inline float vc_orient(float ax, float ay, float bx, float by, float cx, float cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static inline int vc_on_segment(float ax, float ay, float bx, float by, float px, float py) {
    return px >= fminf(ax, bx) - 1e-3f && px <= fmaxf(ax, bx) + 1e-3f &&
           py >= fminf(ay, by) - 1e-3f && py <= fmaxf(ay, by) + 1e-3f;
}

static inline int vc_segments_intersect(float ax, float ay, float bx, float by,
                                        float cx, float cy, float dx, float dy) {
    float o1 = vc_orient(ax, ay, bx, by, cx, cy);
    float o2 = vc_orient(ax, ay, bx, by, dx, dy);
    float o3 = vc_orient(cx, cy, dx, dy, ax, ay);
    float o4 = vc_orient(cx, cy, dx, dy, bx, by);
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) &&
        ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return 1;
    if (fabsf(o1) < 1e-3f && vc_on_segment(ax, ay, bx, by, cx, cy)) return 1;
    if (fabsf(o2) < 1e-3f && vc_on_segment(ax, ay, bx, by, dx, dy)) return 1;
    if (fabsf(o3) < 1e-3f && vc_on_segment(cx, cy, dx, dy, ax, ay)) return 1;
    if (fabsf(o4) < 1e-3f && vc_on_segment(cx, cy, dx, dy, bx, by)) return 1;
    return 0;
}

static inline float vc_dist_pt_to_seg(float px, float py, float ax, float ay, float bx, float by) {
    float vx = bx - ax, vy = by - ay;
    float len2 = vx * vx + vy * vy;
    float t = (len2 > 1e-6f) ? ((px - ax) * vx + (py - ay) * vy) / len2 : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return hypotf(px - (ax + t * vx), py - (ay + t * vy));
}

static inline float vc_dist_pt_to_rect(float px, float py, float rx1, float ry1, float rx2, float ry2) {
    float dx = 0.0f, dy = 0.0f;
    if (px < rx1) dx = rx1 - px; else if (px > rx2) dx = px - rx2;
    if (py < ry1) dy = ry1 - py; else if (py > ry2) dy = py - ry2;
    return hypotf(dx, dy);
}

// Distance between a polygon (pixel coordinates) and an axis-aligned rectangle.
// Returns 0 and sets *intrusion when they overlap.
static inline void vc_eval_poly_rect(const float *px, const float *py, uint32_t n,
                                     float rx1, float ry1, float rx2, float ry2,
                                     uint8_t *intrusion, float *dist) {
    *intrusion = 1;
    *dist = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        if (px[i] >= rx1 && px[i] <= rx2 && py[i] >= ry1 && py[i] <= ry2) return;
    }
    float rcx[4] = { rx1, rx2, rx2, rx1 };
    float rcy[4] = { ry1, ry1, ry2, ry2 };
    for (int c = 0; c < 4; c++) {
        if (vc_point_in_poly(rcx[c], rcy[c], px, py, n)) return;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t j = (i + 1) % n;
        for (int e = 0; e < 4; e++) {
            int en = (e + 1) % 4;
            if (vc_segments_intersect(px[i], py[i], px[j], py[j], rcx[e], rcy[e], rcx[en], rcy[en])) return;
        }
    }

    float min_d = 9999.0f;
    for (int c = 0; c < 4; c++) {
        for (uint32_t i = 0; i < n; i++) {
            uint32_t j = (i + 1) % n;
            float d = vc_dist_pt_to_seg(rcx[c], rcy[c], px[i], py[i], px[j], py[j]);
            if (d < min_d) min_d = d;
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        float d = vc_dist_pt_to_rect(px[i], py[i], rx1, ry1, rx2, ry2);
        if (d < min_d) min_d = d;
    }
    *intrusion = 0;
    *dist = min_d;
}

// ---------------------------------------------------------------------------
// Internals
// ---------------------------------------------------------------------------
static inline int vc_zone_matches(const safety_zone_t *a, const safety_zone_t *b) {
    if (a->num_pts != b->num_pts) return 0;
    for (uint32_t i = 0; i < a->num_pts && i < MAX_ZONE_VERTICES; i++) {
        if (a->vx[i] != b->vx[i] || a->vy[i] != b->vy[i]) return 0;
    }
    return 1;
}

// 3x3 box filter with edge replication, so the frame border is monitored like everything else.
static inline void vc_smooth(const uint8_t *src, uint8_t *dst, uint32_t w, uint32_t h) {
    for (uint32_t y = 0; y < h; y++) {
        uint32_t ym = y ? y - 1 : 0, yp = (y + 1 < h) ? y + 1 : h - 1;
        const uint8_t *r0 = src + (size_t)ym * w, *r1 = src + (size_t)y * w, *r2 = src + (size_t)yp * w;
        for (uint32_t x = 0; x < w; x++) {
            uint32_t xm = x ? x - 1 : 0, xp = (x + 1 < w) ? x + 1 : w - 1;
            uint32_t sum = (uint32_t)r0[xm] + r0[x] + r0[xp] + r1[xm] + r1[x] + r1[xp] + r2[xm] + r2[x] + r2[xp];
            dst[(size_t)y * w + x] = (uint8_t)(sum / 9U);
        }
    }
}

static inline void vc_zone_to_px(const safety_zone_t *z, uint32_t w, uint32_t h, float *px, float *py) {
    for (uint32_t i = 0; i < z->num_pts; i++) {
        px[i] = z->vx[i] * (float)w;
        py[i] = z->vy[i] * (float)h;
    }
}

static inline void vc_build_freeze_mask(vision_ctx_t *c, const safety_zone_t *z, uint32_t w, uint32_t h, float margin) {
    float px[MAX_ZONE_VERTICES] = {0}, py[MAX_ZONE_VERTICES] = {0};
    uint32_t n = z->num_pts;
    vc_zone_to_px(z, w, h, px, py);

    float minx = px[0], maxx = px[0], miny = py[0], maxy = py[0];
    for (uint32_t i = 1; i < n; i++) {
        if (px[i] < minx) minx = px[i];
        if (px[i] > maxx) maxx = px[i];
        if (py[i] < miny) miny = py[i];
        if (py[i] > maxy) maxy = py[i];
    }
    memset(c->freeze, 0, (size_t)w * h);
    int x_lo = (int)floorf(minx - margin), x_hi = (int)ceilf(maxx + margin);
    int y_lo = (int)floorf(miny - margin), y_hi = (int)ceilf(maxy + margin);
    if (x_lo < 0) x_lo = 0;
    if (y_lo < 0) y_lo = 0;
    if (x_hi >= (int)w) x_hi = (int)w - 1;
    if (y_hi >= (int)h) y_hi = (int)h - 1;

    for (int y = y_lo; y <= y_hi; y++) {
        for (int x = x_lo; x <= x_hi; x++) {
            float fx = (float)x, fy = (float)y;
            int frozen = vc_point_in_poly(fx, fy, px, py, n);
            for (uint32_t i = 0; !frozen && i < n; i++) {
                uint32_t j = (i + 1) % n;
                if (vc_dist_pt_to_seg(fx, fy, px[i], py[i], px[j], py[j]) <= margin) frozen = 1;
            }
            if (frozen) c->freeze[(size_t)y * w + (size_t)x] = 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Per-frame processing
// ---------------------------------------------------------------------------
// Returns 0 on success, -2 on invalid arguments. `frame` is w*h 8-bit grayscale.
static inline int vision_process(vision_ctx_t *c, const uint8_t *frame, uint32_t w, uint32_t h,
                                 uint32_t epoch, const safety_zone_t *zone,
                                 const vision_params_t *p, vision_result_t *out) {
    if (!c || !frame || !zone || !p || !out) return -2;
    if (w < 16 || h < 16 || w > MAX_FRAME_DIM || h > MAX_FRAME_DIM || (size_t)w * h > MAX_FRAME_PIXELS) return -2;
    if (zone->num_pts < 3 || zone->num_pts > MAX_ZONE_VERTICES) return -2;

    memset(out, 0, sizeof(*out));
    out->dist_px = 999.0f;
    size_t total = (size_t)w * h;

    vc_smooth(frame, c->smooth, w, h);

    if (!c->have_stream || w != c->w || h != c->h || epoch != c->epoch) {
        c->w = w; c->h = h; c->epoch = epoch; c->have_stream = 1;
        for (size_t i = 0; i < total; i++) c->bg[i] = (float)c->smooth[i];
        c->frames_since_reseed = 0;
        c->quiet_run = 0;
        c->learning = 1;      // teach-in: baseline_valid stays 0 until the zone has settled
        c->mask_valid = 0;
        out->reseeded = 1;
    }
    if (!c->mask_valid || !vc_zone_matches(zone, &c->mask_zone)) {
        vc_build_freeze_mask(c, zone, w, h, p->freeze_margin_px);
        c->mask_zone = *zone;
        c->mask_valid = 1;
    }

    // --- foreground cells ---
    uint32_t cols = w / VC_CELL, rows = h / VC_CELL;
    memset(c->active, 0, sizeof(c->active));
    memset(c->visited, 0, sizeof(c->visited));
    for (uint32_t cy = 0; cy < rows; cy++) {
        for (uint32_t cx = 0; cx < cols; cx++) {
            uint32_t hits = 0;
            for (uint32_t dy = 0; dy < VC_CELL; dy++) {
                size_t base = (size_t)(cy * VC_CELL + dy) * w + (size_t)cx * VC_CELL;
                for (uint32_t dx = 0; dx < VC_CELL; dx++) {
                    if (fabsf((float)c->smooth[base + dx] - c->bg[base + dx]) > p->diff_threshold) hits++;
                }
            }
            if (hits >= p->cell_hit_min) c->active[cy][cx] = 1;
        }
    }

    // --- evaluate EVERY blob against the polygon, keep the most hazardous ---
    float px[MAX_ZONE_VERTICES] = {0}, py[MAX_ZONE_VERTICES] = {0};
    vc_zone_to_px(zone, w, h, px, py);
    static const int ddx[4] = { -1, 1, 0, 0 };
    static const int ddy[4] = { 0, 0, -1, 1 };

    float best_dist = 999.0f;
    uint8_t best_intr = 0;
    uint16_t bx = 0, by = 0, bw = 0, bh = 0;
    uint32_t blobs = 0;

    for (uint32_t cy = 0; cy < rows; cy++) {
        for (uint32_t cx = 0; cx < cols; cx++) {
            if (!c->active[cy][cx] || c->visited[cy][cx]) continue;

            uint32_t head = 0, tail = 0;
            uint32_t cmin_x = w, cmin_y = h, cmax_x = 0, cmax_y = 0;
            c->visited[cy][cx] = 1;
            c->qx[tail] = (uint16_t)cx; c->qy[tail] = (uint16_t)cy; tail++;
            while (head < tail) {
                uint32_t ux = c->qx[head], uy = c->qy[head];
                head++;
                uint32_t x0 = ux * VC_CELL, y0 = uy * VC_CELL;
                if (x0 < cmin_x) cmin_x = x0;
                if (y0 < cmin_y) cmin_y = y0;
                if (x0 + VC_CELL - 1 > cmax_x) cmax_x = x0 + VC_CELL - 1;
                if (y0 + VC_CELL - 1 > cmax_y) cmax_y = y0 + VC_CELL - 1;
                for (int k = 0; k < 4; k++) {
                    int nx = (int)ux + ddx[k], ny = (int)uy + ddy[k];
                    if (nx >= 0 && ny >= 0 && (uint32_t)nx < cols && (uint32_t)ny < rows &&
                        c->active[ny][nx] && !c->visited[ny][nx]) {
                        c->visited[ny][nx] = 1;
                        c->qx[tail] = (uint16_t)nx; c->qy[tail] = (uint16_t)ny; tail++;
                    }
                }
            }
            if (tail < p->min_blob_cells) continue;
            blobs++;

            // Tighten the box to the actual foreground pixels (against the un-updated background).
            uint32_t tx0 = cmax_x, ty0 = cmax_y, tx1 = cmin_x, ty1 = cmin_y;
            for (uint32_t yy = cmin_y; yy <= cmax_y && yy < h; yy++) {
                for (uint32_t xx = cmin_x; xx <= cmax_x && xx < w; xx++) {
                    size_t i = (size_t)yy * w + xx;
                    if (fabsf((float)c->smooth[i] - c->bg[i]) > p->diff_threshold) {
                        if (xx < tx0) tx0 = xx;
                        if (xx > tx1) tx1 = xx;
                        if (yy < ty0) ty0 = yy;
                        if (yy > ty1) ty1 = yy;
                    }
                }
            }
            if (tx1 < tx0 || ty1 < ty0) { tx0 = cmin_x; ty0 = cmin_y; tx1 = cmax_x; ty1 = cmax_y; }

            uint8_t intr = 0;
            float d = 999.0f;
            vc_eval_poly_rect(px, py, zone->num_pts, (float)tx0, (float)ty0, (float)tx1, (float)ty1, &intr, &d);
            if (!intr && d <= p->cutoff_margin_px) intr = 1;

            int better = (intr && !best_intr) || (intr == best_intr && d < best_dist) || blobs == 1;
            if (better) {
                best_intr = intr; best_dist = d;
                bx = (uint16_t)tx0; by = (uint16_t)ty0;
                bw = (uint16_t)(tx1 - tx0 + 1); bh = (uint16_t)(ty1 - ty0 + 1);
            }
        }
    }

    out->blob_count = blobs;
    if (blobs > 0) {
        out->intrusion = best_intr;
        out->dist_px   = best_dist;
        out->bbox_x = bx; out->bbox_y = by; out->bbox_w = bw; out->bbox_h = bh;
    }

    // --- background update (after detection) ---
    // Foreground pixels in the zone region (zone + freeze margin), measured against the background as it is now.
    uint32_t fg_zone = 0;
    for (size_t i = 0; i < total; i++) {
        if (c->freeze[i] && fabsf((float)c->smooth[i] - c->bg[i]) > p->diff_threshold) fg_zone++;
    }
    const int zone_quiet = fg_zone <= VC_QUIET_PX;

    if (c->learning) {
        // TEACH-IN. Adapt everywhere, fast: a seed that happened to contain a moving object (an operator
        // standing in the zone when the worker started) heals within ~1 s instead of becoming a permanent
        // phantom. Detection still runs, but baseline_valid stays 0, so the gate keeps the machine stopped
        // and refuses a reset until the zone has been quiet for learn_quiet_frames in a row.
        // A person who stays in the zone for the whole teach-in is learned as background: commissioning
        // requires an empty zone, and the operator's deliberate reset is the confirmation.
        for (size_t i = 0; i < total; i++)
            c->bg[i] = (1.0f - p->learn_rate) * c->bg[i] + p->learn_rate * (float)c->smooth[i];
        c->quiet_run = zone_quiet ? c->quiet_run + 1 : 0;
        if (c->frames_since_reseed + 1 >= p->warmup_frames && c->quiet_run >= p->learn_quiet_frames) c->learning = 0;
    } else {
        for (size_t i = 0; i < total; i++) {
            float diff = fabsf((float)c->smooth[i] - c->bg[i]);
            float rate;
            if (c->freeze[i]) {
                // Inside / next to the hazard zone: never adapt while anything is there (a stationary
                // intruder must not be learned); only compensate slow drift while the zone region is clear.
                if (!zone_quiet) continue;
                rate = p->zone_adapt_rate;
            } else {
                rate = (diff < 18.0f) ? 0.05f : 0.005f;
            }
            c->bg[i] = (1.0f - rate) * c->bg[i] + rate * (float)c->smooth[i];
        }
    }

    if (c->frames_since_reseed < 0xFFFFFFFFU) c->frames_since_reseed++;
    out->baseline_valid = c->learning ? 0 : 1;
    return 0;
}

#endif
