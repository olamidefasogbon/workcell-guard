// Unit tests for the portable safety arbitration core (safety_logic.h).
#include <math.h>
#include "check.h"
#include "safety_logic.h"

#define MS 1000000ULL

static safety_zone_t rect_zone(void) {
    safety_zone_t z = { .num_pts = 4 };
    z.vx[0] = 0.5f; z.vy[0] = 0.2f; z.vx[1] = 0.9f; z.vy[1] = 0.2f;
    z.vx[2] = 0.9f; z.vy[2] = 0.8f; z.vx[3] = 0.5f; z.vy[3] = 0.8f;
    return z;
}

static gate_params_t params(void) {
    gate_params_t p = { .heartbeat_timeout_ms = 150, .max_frame_age_ms = 100, .reset_clear_frames = 5, .allow_latch_disable = 0 };
    return p;
}

static uint64_t T;     // simulated monotonic clock
static uint32_t FID;

static vision_report_t report(int intrusion, int baseline_valid, const safety_zone_t *z) {
    vision_report_t r;
    memset(&r, 0, sizeof(r));
    r.capture_timestamp_ns = T - 2 * MS;        // 2 ms old
    r.frame_id = ++FID;
    r.stream_epoch = 1;
    r.intrusion_detected = (uint8_t)intrusion;
    r.baseline_valid = (uint8_t)baseline_valid;
    r.min_zone_distance_px = intrusion ? 0.0f : 80.0f;
    r.frame_w = 320; r.frame_h = 240;
    r.active_zone = *z;
    return r;
}

static void feed(gate_t *g, int intrusion, int n) {
    for (int i = 0; i < n; i++) {
        T += 33 * MS;
        vision_report_t r = report(intrusion, 1, &g->zone);
        gate_on_report(g, &r, T);
    }
}

static void test_boot_is_safe(void) {
    TEST("boots with output OFF, latched, and refuses reset until vision is alive");
    safety_zone_t z = rect_zone();
    gate_t g; T = 1000 * MS;
    CHECK(gate_init(&g, &(gate_params_t){150, 100, 5, 0}, &z, T) == 0, "valid zone should init cleanly");
    CHECK(g.state == STATE_WATCHDOG_FAULT && gate_ossd_mv(&g) == 0, "must boot de-energised");
    CHECK(g.reason == FAULT_POWER_UP && g.latched, "startup interlock must be latched");
    CHECK(gate_request_reset(&g, T) == RESET_REFUSED_NO_HEARTBEAT, "reset with no vision must be refused");
    CHECK(g.state != STATE_SAFE_RUN, "still stopped");
}

static void test_reset_needs_clear_streak(void) {
    TEST("reset only accepted after N consecutive clear frames");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    feed(&g, 0, 3);
    CHECK(gate_request_reset(&g, T) == RESET_REFUSED_SETTLING, "3 clear frames < 5 required");
    CHECK(gate_ossd_mv(&g) == 0, "still off after refused reset");
    feed(&g, 0, 2);
    CHECK(gate_request_reset(&g, T) == RESET_OK, "5 clear frames -> reset accepted");
    CHECK(g.state == STATE_SAFE_RUN && gate_ossd_mv(&g) == OSSD_ON_MV, "now energised");
    CHECK(!g.latched, "latch cleared by reset");
}

static void test_intrusion_latches_and_no_snap_back(void) {
    TEST("intrusion trips and stays tripped when the object steps back");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    feed(&g, 0, 5); gate_request_reset(&g, T);
    CHECK(g.state == STATE_SAFE_RUN, "running");

    feed(&g, 1, 1);
    CHECK(g.state == STATE_INTRUSION_HALT && gate_ossd_mv(&g) == 0, "intrusion -> halt, 0 V");
    CHECK(g.latched && g.reason == FAULT_INTRUSION && g.trip_count == 1, "latched, counted once");

    feed(&g, 0, 100);      // object leaves and stays away for 3 s
    CHECK(g.state == STATE_INTRUSION_HALT && gate_ossd_mv(&g) == 0, "must NOT snap back on its own");
    CHECK(g.trip_count == 1, "no extra trips");
}

static void test_reset_refused_while_hazard_present(void) {
    TEST("reset during an ongoing intrusion is refused (no 'two-click bug')");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    feed(&g, 0, 5); gate_request_reset(&g, T);
    feed(&g, 1, 3);
    CHECK(gate_request_reset(&g, T) == RESET_REFUSED_HAZARD, "hazard still present");
    CHECK(g.state == STATE_INTRUSION_HALT && gate_ossd_mv(&g) == 0, "still stopped");
    CHECK(g.reset_refusals == 1 && g.last_reset_result == RESET_REFUSED_HAZARD, "refusal recorded for the UI");
    CHECK(g.trip_count == 1, "refused reset does not add trips");

    feed(&g, 0, 4);
    CHECK(gate_request_reset(&g, T) == RESET_REFUSED_SETTLING, "clear, but not long enough yet");
    feed(&g, 0, 1);
    CHECK(gate_request_reset(&g, T) == RESET_OK, "clear for 5 frames -> accepted");
    CHECK(g.state == STATE_SAFE_RUN, "running again, single deliberate reset");
}

static void test_baseline_blocks_reset(void) {
    TEST("reset refused while the background baseline is still settling");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    for (int i = 0; i < 10; i++) { T += 33 * MS; vision_report_t r = report(0, 0, &z); gate_on_report(&g, &r, T); }
    CHECK(gate_request_reset(&g, T) == RESET_REFUSED_SETTLING, "baseline_valid=0 must block reset");
}

static void test_watchdog(void) {
    TEST("lost heartbeat -> 0 V, latched; recovered worker cannot re-energise");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    feed(&g, 0, 5); gate_request_reset(&g, T);
    CHECK(g.state == STATE_SAFE_RUN, "running");

    uint64_t last = T;
    int tripped = 0;
    for (int ms = 5; ms <= 200; ms += 5) if (gate_on_tick(&g, last + (uint64_t)ms * MS)) { tripped = ms; break; }
    CHECK(tripped > 150 && tripped <= 160, "trips just after 150 ms (tripped at %d ms)", tripped);
    CHECK(g.state == STATE_WATCHDOG_FAULT && gate_ossd_mv(&g) == 0 && g.reason == FAULT_HEARTBEAT_LOST, "fault, 0 V");
    CHECK(g.latched, "latched");
    CHECK(gate_on_tick(&g, last + 400 * MS) == 0 && g.trip_count == 1, "only one trip counted");

    T = last + 500 * MS;           // worker respawns and reports clear frames
    feed(&g, 0, 50);
    CHECK(g.state == STATE_WATCHDOG_FAULT && gate_ossd_mv(&g) == 0, "respawned vision alone must not re-energise");
    CHECK(gate_request_reset(&g, T) == RESET_OK, "operator reset works once vision is healthy");
}

static void test_stale_and_bad_reports(void) {
    TEST("stale frames and malformed reports are faults, not heartbeats");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    feed(&g, 0, 5); gate_request_reset(&g, T);

    T += 33 * MS;
    vision_report_t r = report(0, 1, &z);
    r.capture_timestamp_ns = T - 250 * MS;           // 250 ms old
    uint64_t hb_before = g.last_heartbeat_ns;
    gate_on_report(&g, &r, T);
    CHECK(g.state == STATE_WATCHDOG_FAULT && g.reason == FAULT_STALE_FRAME, "stale frame faults");
    CHECK(g.last_heartbeat_ns == hb_before, "stale frame must not refresh the heartbeat");

    struct { const char *name; void (*mut)(vision_report_t *); } cases[] = {
        { "NaN distance",     NULL }, { "zero frame size", NULL }, { "intrusion flag = 7", NULL },
        { "future timestamp", NULL }, { "invalid zone in report", NULL },
    };
    for (int c = 0; c < 5; c++) {
        gate_t g2; T = 5000 * MS; gate_init(&g2, &p, &z, T);
        feed(&g2, 0, 5); gate_request_reset(&g2, T);
        T += 33 * MS;
        vision_report_t b = report(0, 1, &z);
        switch (c) {
            case 0: b.min_zone_distance_px = NAN; break;
            case 1: b.frame_w = 0; break;
            case 2: b.intrusion_detected = 7; break;
            case 3: b.capture_timestamp_ns = T + 50 * MS; break;
            case 4: b.active_zone.vx[0] = NAN; break;
        }
        gate_on_report(&g2, &b, T);
        CHECK(g2.state == STATE_WATCHDOG_FAULT && gate_ossd_mv(&g2) == 0, "%s must fault", cases[c].name);
    }
}

static void test_zone_validation(void) {
    TEST("polygon validation: NaN, range, degenerate, self-crossing all rejected");
    safety_zone_t ok = rect_zone();
    CHECK(zone_validate(&ok) == ZONE_OK, "rectangle valid");

    safety_zone_t trap = { .num_pts = 4 };
    trap.vx[0]=0.54f; trap.vy[0]=0.18f; trap.vx[1]=0.90f; trap.vy[1]=0.28f; trap.vx[2]=0.94f; trap.vy[2]=0.86f; trap.vx[3]=0.46f; trap.vy[3]=0.78f;
    CHECK(zone_validate(&trap) == ZONE_OK, "dashboard 'Angled Table' preset valid");

    safety_zone_t ell = { .num_pts = 6 };
    float lx[6] = {0.55f,0.92f,0.92f,0.42f,0.42f,0.55f}, ly[6] = {0.15f,0.15f,0.88f,0.88f,0.55f,0.55f};
    for (int i = 0; i < 6; i++) { ell.vx[i] = lx[i]; ell.vy[i] = ly[i]; }
    CHECK(zone_validate(&ell) == ZONE_OK, "dashboard 'L-Guard' preset (concave) valid");

    safety_zone_t bad = ok; bad.num_pts = 2;
    CHECK(zone_validate(&bad) == ZONE_ERR_COUNT, "2 points");
    bad = ok; bad.num_pts = MAX_ZONE_VERTICES + 1;
    CHECK(zone_validate(&bad) == ZONE_ERR_COUNT, "too many points");
    bad = ok; bad.vx[1] = NAN;
    CHECK(zone_validate(&bad) == ZONE_ERR_RANGE, "NaN x (old check let this through)");
    bad = ok; bad.vy[2] = INFINITY;
    CHECK(zone_validate(&bad) == ZONE_ERR_RANGE, "Inf y");
    bad = ok; bad.vx[0] = -0.1f;
    CHECK(zone_validate(&bad) == ZONE_ERR_RANGE, "negative");
    bad = ok; bad.vx[3] = 1.5f;
    CHECK(zone_validate(&bad) == ZONE_ERR_RANGE, ">1");
    bad = ok; bad.vx[1] = bad.vx[0]; bad.vy[1] = bad.vy[0];
    CHECK(zone_validate(&bad) == ZONE_ERR_DUPLICATE, "duplicate consecutive vertex");

    safety_zone_t line = { .num_pts = 3 };
    line.vx[0]=0.2f; line.vy[0]=0.2f; line.vx[1]=0.5f; line.vy[1]=0.5f; line.vx[2]=0.8f; line.vy[2]=0.8f;
    CHECK(zone_validate(&line) == ZONE_ERR_AREA, "collinear points (zero area)");

    safety_zone_t tiny = { .num_pts = 3 };
    tiny.vx[0]=0.5f; tiny.vy[0]=0.5f; tiny.vx[1]=0.52f; tiny.vy[1]=0.5f; tiny.vx[2]=0.5f; tiny.vy[2]=0.52f;
    CHECK(zone_validate(&tiny) == ZONE_ERR_AREA, "too small to be useful");

    safety_zone_t bow = { .num_pts = 4 };       // figure-eight
    bow.vx[0]=0.2f; bow.vy[0]=0.2f; bow.vx[1]=0.8f; bow.vy[1]=0.8f; bow.vx[2]=0.8f; bow.vy[2]=0.2f; bow.vx[3]=0.2f; bow.vy[3]=0.8f;
    CHECK(zone_validate(&bow) == ZONE_ERR_SELF_CROSS, "bow-tie polygon");
}

static void test_invalid_zone_command_faults(void) {
    TEST("invalid polygon command is rejected AND stops the machine");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    feed(&g, 0, 5); gate_request_reset(&g, T);
    CHECK(g.state == STATE_SAFE_RUN, "running");

    safety_zone_t nan_zone = z; nan_zone.vx[0] = NAN;
    CHECK(gate_set_zone(&g, &nan_zone) == EINVAL, "rejected");
    CHECK(g.state == STATE_WATCHDOG_FAULT && gate_ossd_mv(&g) == 0 && g.reason == FAULT_INVALID_ZONE, "machine stopped");
    CHECK(zone_equal(&g.zone, &z), "previous valid polygon kept");

    feed(&g, 0, 50);
    CHECK(gate_request_reset(&g, T) == RESET_OK, "reset after the bad input is gone");

    safety_zone_t moved = z; moved.vx[1] = 0.85f;
    CHECK(gate_set_zone(&g, &moved) == 0 && zone_equal(&g.zone, &moved), "valid zone accepted");
    CHECK(g.state == STATE_SAFE_RUN, "valid zone does not stop the machine");
}

static void test_zone_mismatch(void) {
    TEST("vision evaluating a different polygon than the gate holds is a fault (after propagation grace)");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    feed(&g, 0, 5); gate_request_reset(&g, T);

    safety_zone_t other = z; other.vx[1] = 0.7f;
    gate_set_zone(&g, &other);
    for (int i = 0; i < 3; i++) { T += 33 * MS; vision_report_t r = report(0, 1, &z); gate_on_report(&g, &r, T); }
    CHECK(g.state == STATE_SAFE_RUN, "a few frames of lag are tolerated");
    for (int i = 0; i < 5; i++) { T += 33 * MS; vision_report_t r = report(0, 1, &z); gate_on_report(&g, &r, T); }
    CHECK(g.state == STATE_WATCHDOG_FAULT && g.reason == FAULT_ZONE_MISMATCH, "persistent mismatch faults");
}

static void test_latch_cannot_be_disabled(void) {
    TEST("ISO 13849-1 latch cannot be switched off at runtime (production build)");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    CHECK(gate_set_latch(&g, 0) == EPERM && g.latch_enabled, "disable refused");
    CHECK(gate_set_latch(&g, 1) == 0, "enable is always allowed");

    feed(&g, 0, 5); gate_request_reset(&g, T);
    feed(&g, 1, 1);
    feed(&g, 0, 60);
    CHECK(g.state == STATE_INTRUSION_HALT, "still latched");
}

static void test_test_build_autorecovery_still_gated(void) {
    TEST("test-build auto-recovery still requires the clear streak (no instant snap-back)");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); p.allow_latch_disable = 1; gate_init(&g, &p, &z, T);
    CHECK(gate_set_latch(&g, 0) == 0 && !g.latch_enabled, "latch off allowed in test builds");
    feed(&g, 0, 4);
    CHECK(g.state != STATE_SAFE_RUN, "4 clear frames: not yet");
    feed(&g, 0, 1);
    CHECK(g.state == STATE_SAFE_RUN, "5th clear frame recovers");
    feed(&g, 1, 1);
    CHECK(g.state == STATE_INTRUSION_HALT, "intrusion trips");
    feed(&g, 1, 20);
    CHECK(g.state == STATE_INTRUSION_HALT, "stays stopped while the intruder is present");
    feed(&g, 0, 4);
    CHECK(g.state == STATE_INTRUSION_HALT, "not recovered after 4 clear frames");
    feed(&g, 0, 1);
    CHECK(g.state == STATE_SAFE_RUN, "recovered after 5");
}

static void test_severity_never_downgrades(void) {
    TEST("fault severity never downgrades before a reset");
    safety_zone_t z = rect_zone(); gate_t g; T = 1000 * MS;
    gate_params_t p = params(); gate_init(&g, &p, &z, T);
    feed(&g, 0, 5); gate_request_reset(&g, T);
    gate_on_tick(&g, T + 300 * MS);
    CHECK(g.state == STATE_WATCHDOG_FAULT, "watchdog fault");
    T += 300 * MS;
    feed(&g, 1, 1);
    CHECK(g.state == STATE_WATCHDOG_FAULT, "an intrusion report cannot downgrade a watchdog fault");
}

int main(void) {
    printf("test_gate\n");
    test_boot_is_safe();
    test_reset_needs_clear_streak();
    test_intrusion_latches_and_no_snap_back();
    test_reset_refused_while_hazard_present();
    test_baseline_blocks_reset();
    test_watchdog();
    test_stale_and_bad_reports();
    test_zone_validation();
    test_invalid_zone_command_faults();
    test_zone_mismatch();
    test_latch_cannot_be_disabled();
    test_test_build_autorecovery_still_gated();
    test_severity_never_downgrades();
    return check_summary("test_gate");
}
