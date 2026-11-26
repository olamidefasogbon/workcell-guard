// End-to-end scenario on the host: real vision core + real gate core, simulated 30 fps camera.
// Mirrors the demo run: reset -> safe run -> warning/halt -> reset -> real video -> crash/respawn -> reset.
#include "check.h"
#include "safety_logic.h"
#include "vision_core.h"

#define MS 1000000ULL
#define W 320
#define H 240

static uint8_t frame[W * H];
static uint64_t T;
static uint32_t FID, EPOCH = 1;
static gate_t g;
static vision_ctx_t *vc;
static vision_params_t vp = { .diff_threshold = 38, .cutoff_margin_px = 15, .freeze_margin_px = 30,
                              .zone_adapt_rate = 0.002f, .cell_hit_min = 34, .min_blob_cells = 4, .warmup_frames = 20 };
static vision_result_t last;

static void scene(int bright) {
    for (uint32_t y = 0; y < H; y++)
        for (uint32_t x = 0; x < W; x++)
            frame[y * W + x] = (uint8_t)((bright ? 120 : 28) + (y * 18) / H + (((x / 32) + (y / 32)) & 1 ? 6 : 0));
}

static void person(int x0) {      // 24x60 px bright figure whose right edge is at x0+24
    for (int y = 90; y < 150; y++) for (int x = x0; x < x0 + 24; x++) if (x >= 0 && x < W) frame[y * W + x] = 225;
}

// One camera frame through vision into the gate (all timestamps on the simulated clock).
static void step(int bright, int person_x) {
    T += 33 * MS;
    scene(bright);
    if (person_x >= -1000) person(person_x);
    vision_result_t r;
    if (vision_process(vc, frame, W, H, EPOCH, &g.zone, &vp, &r) != 0) { CHECK(0, "vision_process failed"); return; }
    last = r;
    vision_report_t rep;
    memset(&rep, 0, sizeof(rep));
    rep.capture_timestamp_ns = T - 3 * MS;
    rep.frame_id = ++FID; rep.stream_epoch = EPOCH;
    rep.intrusion_detected = r.intrusion; rep.baseline_valid = r.baseline_valid;
    rep.min_zone_distance_px = r.dist_px;
    rep.bbox_x = r.bbox_x; rep.bbox_y = r.bbox_y; rep.bbox_w = r.bbox_w; rep.bbox_h = r.bbox_h;
    rep.frame_w = W; rep.frame_h = H; rep.active_zone = g.zone;
    gate_on_report(&g, &rep, T);
    gate_on_tick(&g, T);
}
#define NOBODY (-100000)

int main(void) {
    printf("test_e2e\n");
    gate_params_t gp = { .heartbeat_timeout_ms = 150, .max_frame_age_ms = 100, .reset_clear_frames = 15, .allow_latch_disable = 0 };
    safety_zone_t z = { .num_pts = 4 };
    z.vx[0]=0.50f; z.vy[0]=0.20f; z.vx[1]=0.92f; z.vy[1]=0.20f; z.vx[2]=0.92f; z.vy[2]=0.85f; z.vx[3]=0.50f; z.vy[3]=0.85f;
    T = 10000 * MS;
    gate_init(&g, &gp, &z, T);
    vc = vision_create();

    TEST("1. power-up: output OFF, reset refused while the baseline settles, accepted afterwards");
    CHECK(gate_ossd_mv(&g) == 0, "boots off");
    for (int i = 0; i < 10; i++) step(0, NOBODY);
    CHECK(gate_request_reset(&g, T) == RESET_REFUSED_SETTLING, "too early");
    for (int i = 0; i < 25; i++) step(0, NOBODY);
    CHECK(gate_request_reset(&g, T) == RESET_OK && g.state == STATE_SAFE_RUN, "reset accepted -> SAFE_RUN");

    TEST("2. operator approaches: warning band first, then halt before contact");
    int saw_warn = 0, halted_at = -1;
    for (int x = 20; x <= 200; x += 3) {
        step(0, x);
        if (g.state == STATE_SAFE_RUN && last.blob_count && last.dist_px <= 45.0f) saw_warn = 1;
        if (g.state != STATE_SAFE_RUN) { halted_at = x; break; }
    }
    CHECK(saw_warn, "passed through the SLS warning band (<=45 px) while still running");
    CHECK(halted_at > 0 && gate_ossd_mv(&g) == 0 && g.state == STATE_INTRUSION_HALT, "halted, 0 V (at x=%d)", halted_at);
    CHECK(halted_at + 24 < 160 + 5, "cut off at the margin, before reaching the zone edge (right edge %d vs zone x=160)", halted_at + 24);

    TEST("3. reset while the operator is still there is refused; nothing re-trips or snaps back");
    for (int i = 0; i < 40; i++) step(0, halted_at + 20);     // walks on in and stands in the zone
    CHECK(gate_request_reset(&g, T) == RESET_REFUSED_HAZARD && g.state == STATE_INTRUSION_HALT, "refused: hazard present");
    CHECK(g.trip_count == 1, "single trip, no re-latch storm (trips=%u)", g.trip_count);

    TEST("4. operator leaves: still latched until a deliberate reset, which then succeeds first time");
    for (int i = 0; i < 20; i++) step(0, NOBODY);
    CHECK(g.state == STATE_INTRUSION_HALT && gate_ossd_mv(&g) == 0, "no automatic restart");
    CHECK(gate_request_reset(&g, T) == RESET_OK && gate_ossd_mv(&g) == OSSD_ON_MV, "one click -> SAFE_RUN");

    TEST("5. stream swap to bright real video: epoch bump, no false trip, no stale background");
    EPOCH++;
    for (int i = 0; i < 40; i++) step(1, NOBODY);
    CHECK(g.state == STATE_SAFE_RUN && g.trip_count == 1, "still running, no new trips (trips=%u)", g.trip_count);
    CHECK(g.last_report.baseline_valid, "baseline settled again");

    TEST("6. vision crash: watchdog cuts 0 V within the timeout; respawned worker cannot re-energise");
    uint64_t t_crash = T;
    int cut_ms = -1;
    for (int ms = 5; ms <= 400; ms += 5) { if (gate_on_tick(&g, t_crash + (uint64_t)ms * MS)) { cut_ms = ms; break; } }
    CHECK(cut_ms > 150 && cut_ms <= 160 && gate_ossd_mv(&g) == 0, "cut at %d ms", cut_ms);
    T = t_crash + 450 * MS;                                    // supervisor respawns after ~250 ms + startup
    vision_destroy(vc);
    vc = vision_create();                                      // fresh process: new background model
    for (int i = 0; i < 40; i++) step(1, NOBODY);
    CHECK(g.state == STATE_WATCHDOG_FAULT && gate_ossd_mv(&g) == 0, "worker is back but output stays off");
    CHECK(gate_request_reset(&g, T) == RESET_OK && g.state == STATE_SAFE_RUN, "deliberate reset restores SAFE_RUN");

    TEST("7. invalid polygon from the UI: rejected and machine stopped; valid one accepted after reset");
    safety_zone_t bow = z; bow.vx[1] = 0.2f; bow.vy[1] = 0.85f; bow.vx[2] = 0.92f; bow.vy[2] = 0.20f;
    CHECK(gate_set_zone(&g, &bow) == EINVAL && g.state == STATE_WATCHDOG_FAULT && gate_ossd_mv(&g) == 0, "stopped");
    for (int i = 0; i < 25; i++) step(1, NOBODY);
    CHECK(gate_request_reset(&g, T) == RESET_OK, "reset after rejection");

    return check_summary("test_e2e");
}
