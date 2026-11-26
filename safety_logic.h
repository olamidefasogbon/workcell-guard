// Portable safety arbitration core (no QNX, no I/O, no allocation).
//
// safety_gate.c owns the QNX resource manager; every decision about whether the
// OSSD output may be energised lives here so it can be unit-tested on any host.
//
// Design rules:
//   * Fail safe. The gate boots with the output OFF (startup interlock).
//   * The output is ON only in STATE_SAFE_RUN. Anything unexpected -> fault.
//   * Faults latch. Only gate_request_reset() leaves a fault, and it refuses
//     unless the hazard is gone *and* has stayed gone for N consecutive frames.
//   * Invalid input (stale frame, bad report, bad polygon) is a fault, never
//     something to guess around.
#ifndef SAFETY_LOGIC_H
#define SAFETY_LOGIC_H

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "safety_protocol.h"

// ---------------------------------------------------------------------------
// Polygon validation
// ---------------------------------------------------------------------------
#define ZONE_MIN_AREA 0.002   // normalised units (0.2 % of the frame)
#define ZL_EPS        1e-9

enum {
    ZONE_OK = 0,
    ZONE_ERR_COUNT,        // fewer than 3 or more than MAX_ZONE_VERTICES points
    ZONE_ERR_RANGE,        // NaN / Inf / outside [0,1]
    ZONE_ERR_DUPLICATE,    // two consecutive identical vertices
    ZONE_ERR_AREA,         // degenerate / too small
    ZONE_ERR_SELF_CROSS    // edges cross or touch
};

static inline double zl_orient(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static inline int zl_on_seg(double ax, double ay, double bx, double by, double px, double py) {
    double lox = ax < bx ? ax : bx, hix = ax < bx ? bx : ax;
    double loy = ay < by ? ay : by, hiy = ay < by ? by : ay;
    return px >= lox - ZL_EPS && px <= hix + ZL_EPS && py >= loy - ZL_EPS && py <= hiy + ZL_EPS;
}

static inline int zl_seg_intersect(double ax, double ay, double bx, double by,
                                   double cx, double cy, double dx, double dy) {
    double d1 = zl_orient(cx, cy, dx, dy, ax, ay);
    double d2 = zl_orient(cx, cy, dx, dy, bx, by);
    double d3 = zl_orient(ax, ay, bx, by, cx, cy);
    double d4 = zl_orient(ax, ay, bx, by, dx, dy);
    if (((d1 > ZL_EPS && d2 < -ZL_EPS) || (d1 < -ZL_EPS && d2 > ZL_EPS)) &&
        ((d3 > ZL_EPS && d4 < -ZL_EPS) || (d3 < -ZL_EPS && d4 > ZL_EPS))) return 1;
    if (fabs(d1) <= ZL_EPS && zl_on_seg(cx, cy, dx, dy, ax, ay)) return 1;
    if (fabs(d2) <= ZL_EPS && zl_on_seg(cx, cy, dx, dy, bx, by)) return 1;
    if (fabs(d3) <= ZL_EPS && zl_on_seg(ax, ay, bx, by, cx, cy)) return 1;
    if (fabs(d4) <= ZL_EPS && zl_on_seg(ax, ay, bx, by, dx, dy)) return 1;
    return 0;
}

static inline int zone_validate(const safety_zone_t *z) {
    if (!z) return ZONE_ERR_COUNT;
    uint32_t n = z->num_pts;
    if (n < 3 || n > MAX_ZONE_VERTICES) return ZONE_ERR_COUNT;

    for (uint32_t i = 0; i < n; i++) {
        float x = z->vx[i], y = z->vy[i];
        // Written as !(in range) so NaN fails the test instead of slipping through.
        if (!(isfinite(x) && isfinite(y) && x >= 0.0f && x <= 1.0f && y >= 0.0f && y <= 1.0f))
            return ZONE_ERR_RANGE;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t j = (i + 1) % n;
        if (fabs((double)z->vx[i] - z->vx[j]) <= ZL_EPS && fabs((double)z->vy[i] - z->vy[j]) <= ZL_EPS)
            return ZONE_ERR_DUPLICATE;
    }

    for (uint32_t i = 0; i < n; i++) {
        uint32_t i2 = (i + 1) % n;
        for (uint32_t j = i + 1; j < n; j++) {
            uint32_t j2 = (j + 1) % n;
            if (j == i2 || j2 == i) continue;   // adjacent edges share a vertex by definition
            if (zl_seg_intersect(z->vx[i], z->vy[i], z->vx[i2], z->vy[i2],
                                 z->vx[j], z->vy[j], z->vx[j2], z->vy[j2]))
                return ZONE_ERR_SELF_CROSS;
        }
    }

    double area2 = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t j = (i + 1) % n;
        area2 += (double)z->vx[i] * z->vy[j] - (double)z->vx[j] * z->vy[i];
    }
    if (fabs(area2) * 0.5 < ZONE_MIN_AREA) return ZONE_ERR_AREA;
    return ZONE_OK;
}

static inline int zone_equal(const safety_zone_t *a, const safety_zone_t *b) {
    if (a->num_pts != b->num_pts || a->num_pts > MAX_ZONE_VERTICES) return 0;
    for (uint32_t i = 0; i < a->num_pts; i++) {
        if (a->vx[i] != b->vx[i] || a->vy[i] != b->vy[i]) return 0;
    }
    return 1;
}

// ---------------------------------------------------------------------------
// Gate state machine
// ---------------------------------------------------------------------------
#define GATE_MAX_ZONE_MISMATCH_FRAMES 5U
#define GATE_STREAK_CAP               1000000U

typedef struct {
    uint32_t heartbeat_timeout_ms;  // no valid report for this long -> fault
    uint32_t max_frame_age_ms;      // capture->arrival older than this -> stale-frame fault
    uint32_t reset_clear_frames;    // consecutive clear frames required before a reset is accepted
    int      allow_latch_disable;   // test builds only
} gate_params_t;

typedef struct {
    gate_params_t  p;
    gate_state_t   state;
    fault_reason_t reason;
    int            latched;
    int            latch_enabled;
    uint32_t       trip_count;
    uint64_t       last_heartbeat_ns;
    int            seen_report;          // at least one valid report since boot
    uint64_t       last_latency_us;
    uint64_t       last_trip_us;
    vision_report_t last_report;
    uint32_t       clear_streak;
    uint32_t       reset_refusals;
    int32_t        last_reset_result;
    uint32_t       zone_mismatch_frames;
    safety_zone_t  zone;
} gate_t;

static inline uint32_t gate_ossd_mv(const gate_t *g) {
    return g->state == STATE_SAFE_RUN ? OSSD_ON_MV : 0U;
}

// Records a fault. State severity only ever increases until a reset.
static inline void gate_enter_fault(gate_t *g, gate_state_t st, fault_reason_t why, uint64_t trip_us) {
    if (g->state == STATE_SAFE_RUN) {
        g->trip_count++;
        g->last_trip_us = trip_us;
    }
    if ((int)st > (int)g->state) {
        g->state  = st;
        g->reason = why;
    } else if (g->state == STATE_SAFE_RUN) {   // st == SAFE_RUN is a caller bug: fail safe
        g->state  = STATE_WATCHDOG_FAULT;
        g->reason = why;
    }
    if (g->latch_enabled) g->latched = 1;
    g->clear_streak = 0;
}

// Returns 0 on success. On an invalid zone the gate still comes up, but faulted.
static inline int gate_init(gate_t *g, const gate_params_t *p, const safety_zone_t *zone, uint64_t now_ns) {
    memset(g, 0, sizeof(*g));
    g->p                 = *p;
    g->state             = STATE_WATCHDOG_FAULT;      // startup interlock: output OFF until reset
    g->reason            = FAULT_POWER_UP;
    g->latch_enabled     = 1;
    g->latched           = 1;
    g->last_heartbeat_ns = now_ns;
    g->last_report.min_zone_distance_px = 999.0f;
    g->last_reset_result = RESET_REFUSED_NO_HEARTBEAT;
    g->zone              = *zone;
    int zr = zone_validate(zone);
    if (zr != ZONE_OK) {
        g->reason = FAULT_INVALID_ZONE;
        return zr;
    }
    return 0;
}

static inline uint64_t gate_heartbeat_age_ns(const gate_t *g, uint64_t now_ns) {
    return now_ns > g->last_heartbeat_ns ? now_ns - g->last_heartbeat_ns : 0;
}

static inline int gate_report_is_sane(const vision_report_t *r) {
    if (r->frame_w < 16 || r->frame_w > MAX_FRAME_DIM || r->frame_h < 16 || r->frame_h > MAX_FRAME_DIM) return 0;
    if ((uint32_t)r->frame_w * r->frame_h > MAX_FRAME_PIXELS) return 0;
    if (r->intrusion_detected > 1 || r->baseline_valid > 1) return 0;
    if (!isfinite(r->min_zone_distance_px) || r->min_zone_distance_px < 0.0f) return 0;
    if (zone_validate(&r->active_zone) != ZONE_OK) return 0;
    return 1;
}

// Decides whether a reset would be accepted *now*. Pure: no state change.
static inline int gate_reset_check(const gate_t *g, uint64_t now_ns) {
    uint64_t timeout_ns = (uint64_t)g->p.heartbeat_timeout_ms * 1000000ULL;
    if (!g->seen_report || gate_heartbeat_age_ns(g, now_ns) > timeout_ns) return RESET_REFUSED_NO_HEARTBEAT;
    if (zone_validate(&g->zone) != ZONE_OK) return RESET_REFUSED_ZONE;
    if (g->last_report.intrusion_detected) return RESET_REFUSED_HAZARD;
    if (!g->last_report.baseline_valid) return RESET_REFUSED_SETTLING;
    if (g->clear_streak < g->p.reset_clear_frames) return RESET_REFUSED_SETTLING;
    return RESET_OK;
}

// Operator reset. Accepted only if the hazard is gone and has stayed gone.
// A refused reset changes nothing except the refusal counter.
static inline int gate_request_reset(gate_t *g, uint64_t now_ns) {
    int r = gate_reset_check(g, now_ns);
    g->last_reset_result = r;
    if (r != RESET_OK) {
        g->reset_refusals++;
        return r;
    }
    g->state   = STATE_SAFE_RUN;
    g->reason  = FAULT_NONE;
    g->latched = 0;
    return RESET_OK;
}

// Watchdog tick. Returns 1 if this tick caused a new trip.
static inline int gate_on_tick(gate_t *g, uint64_t now_ns) {
    uint64_t timeout_ns = (uint64_t)g->p.heartbeat_timeout_ms * 1000000ULL;
    uint64_t age = gate_heartbeat_age_ns(g, now_ns);
    if (age > timeout_ns && g->state != STATE_WATCHDOG_FAULT) {
        gate_enter_fault(g, STATE_WATCHDOG_FAULT, FAULT_HEARTBEAT_LOST, age / 1000ULL);
        return 1;
    }
    return 0;
}

// Handles one vision report. Returns 1 if the state changed.
static inline int gate_on_report(gate_t *g, const vision_report_t *r, uint64_t now_ns) {
    gate_state_t before = g->state;

    if (!gate_report_is_sane(r)) {
        gate_enter_fault(g, STATE_WATCHDOG_FAULT, FAULT_BAD_REPORT, 0);
        return g->state != before;
    }

    // Capture time must not be in the future (same monotonic clock) and must be recent.
    int64_t age_ns = (int64_t)now_ns - (int64_t)r->capture_timestamp_ns;
    if (age_ns < -1000000LL) {
        gate_enter_fault(g, STATE_WATCHDOG_FAULT, FAULT_BAD_REPORT, 0);
        return g->state != before;
    }
    uint64_t latency_us = age_ns > 0 ? (uint64_t)age_ns / 1000ULL : 0;
    if (latency_us > (uint64_t)g->p.max_frame_age_ms * 1000ULL) {
        // A stale frame is not a heartbeat: do NOT refresh last_heartbeat_ns.
        g->last_latency_us = latency_us;
        gate_enter_fault(g, STATE_WATCHDOG_FAULT, FAULT_STALE_FRAME, latency_us);
        return g->state != before;
    }

    g->last_heartbeat_ns = now_ns;
    g->seen_report       = 1;
    g->last_report       = *r;
    g->last_latency_us   = latency_us;

    // Vision must be evaluating the polygon the gate holds (allow a few frames for a zone change to propagate).
    if (zone_equal(&r->active_zone, &g->zone)) {
        g->zone_mismatch_frames = 0;
    } else if (++g->zone_mismatch_frames > GATE_MAX_ZONE_MISMATCH_FRAMES) {
        gate_enter_fault(g, STATE_WATCHDOG_FAULT, FAULT_ZONE_MISMATCH, latency_us);
        return g->state != before;
    }

    if (r->intrusion_detected || !r->baseline_valid) {
        g->clear_streak = 0;
    } else if (g->clear_streak < GATE_STREAK_CAP) {
        g->clear_streak++;
    }

    if (r->intrusion_detected && g->state == STATE_SAFE_RUN) {
        gate_enter_fault(g, STATE_INTRUSION_HALT, FAULT_INTRUSION, latency_us);
    }

    // Auto-recovery exists only when latching has been disabled (test builds).
    // It still requires the full clear-streak, so it can never "snap back".
    if (!g->latch_enabled && g->state != STATE_SAFE_RUN && gate_reset_check(g, now_ns) == RESET_OK) {
        g->state   = STATE_SAFE_RUN;
        g->reason  = FAULT_NONE;
        g->latched = 0;
    }
    return g->state != before;
}

// Replaces the polygon. An invalid polygon is rejected AND faults the gate.
static inline int gate_set_zone(gate_t *g, const safety_zone_t *z) {
    if (zone_validate(z) != ZONE_OK) {
        gate_enter_fault(g, STATE_WATCHDOG_FAULT, FAULT_INVALID_ZONE, 0);
        return EINVAL;
    }
    g->zone = *z;
    g->zone_mismatch_frames = 0;
    return 0;
}

// Latch can be switched on at any time; switching it off needs a test build.
static inline int gate_set_latch(gate_t *g, int enable) {
    if (!enable && !g->p.allow_latch_disable) return EPERM;
    g->latch_enabled = enable ? 1 : 0;
    if (!g->latch_enabled) g->latched = 0;
    return 0;
}

#endif
