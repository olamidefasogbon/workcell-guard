// Unit tests for the portable vision core (vision_core.h) using synthetic frames.
#include "check.h"
#include "vision_core.h"

static vision_params_t params(void) {
    vision_params_t p = { .diff_threshold = 38, .cutoff_margin_px = 15, .freeze_margin_px = 30,
                          .zone_adapt_rate = 0.002f, .cell_hit_min = 34, .min_blob_cells = 4, .warmup_frames = 20,
                          .learn_rate = 0.05f, .learn_quiet_frames = 30 };
    return p;
}

static safety_zone_t right_zone(void) {   // x 50..92 %, y 20..85 %
    safety_zone_t z = { .num_pts = 4 };
    z.vx[0]=0.50f; z.vy[0]=0.20f; z.vx[1]=0.92f; z.vy[1]=0.20f; z.vx[2]=0.92f; z.vy[2]=0.85f; z.vx[3]=0.50f; z.vy[3]=0.85f;
    return z;
}

static void scene(uint8_t *f, uint32_t w, uint32_t h, int bright) {
    for (uint32_t y = 0; y < h; y++)
        for (uint32_t x = 0; x < w; x++)
            f[y * w + x] = (uint8_t)((bright ? 120 : 28) + (y * 18) / h + (((x / 32) + (y / 32)) & 1 ? 6 : 0));
}

static void box(uint8_t *f, uint32_t w, uint32_t h, int x0, int y0, int x1, int y1, uint8_t v) {
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            if (x >= 0 && y >= 0 && x < (int)w && y < (int)h) f[(uint32_t)y * w + (uint32_t)x] = v;
}

static vision_result_t run(vision_ctx_t *c, const uint8_t *f, uint32_t w, uint32_t h, uint32_t epoch, const safety_zone_t *z) {
    vision_params_t p = params();
    vision_result_t r;
    int rc = vision_process(c, f, w, h, epoch, z, &p, &r);
    CHECK(rc == 0, "vision_process rc=%d", rc);
    return r;
}

static void settle(vision_ctx_t *c, uint8_t *f, uint32_t w, uint32_t h, const safety_zone_t *z, int n) {
    for (int i = 0; i < n; i++) run(c, f, w, h, 1, z);
}

#define W 320
#define H 240
static uint8_t frame[W * H];

static void test_empty_scene_and_warmup(void) {
    TEST("empty scene: no detections; baseline valid once teach-in is done (30 quiet frames)");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0);
    vision_result_t r = run(c, frame, W, H, 1, &z);
    CHECK(r.reseeded && !r.intrusion && r.blob_count == 0 && !r.baseline_valid, "first frame seeds, baseline not valid yet");
    for (int i = 2; i <= 29; i++) { r = run(c, frame, W, H, 1, &z); CHECK(!r.baseline_valid, "frame %d: still learning", i); }
    r = run(c, frame, W, H, 1, &z);
    CHECK(r.baseline_valid && !r.intrusion && r.dist_px >= 900.0f, "valid on frame 30, nothing detected");
    vision_destroy(c);
}

static void test_intruder_detected_and_never_absorbed(void) {
    TEST("a person who walks into the zone and STOPS is never learned into the background");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 30);

    int first_hit = -1;
    for (int x = 20; x <= 220; x += 4) {        // walks in from the left, ends inside the zone
        scene(frame, W, H, 0); box(frame, W, H, x, 90, x + 24, 150, 225);
        vision_result_t r = run(c, frame, W, H, 1, &z);
        if (r.intrusion && first_hit < 0) first_hit = x;
    }
    CHECK(first_hit > 0 && first_hit < 200, "intrusion raised while walking in (at x=%d)", first_hit);

    int missed = 0;
    for (int i = 0; i < 1800; i++) {            // stands perfectly still for 60 s @30 fps
        scene(frame, W, H, 0); box(frame, W, H, 220, 90, 244, 150, 225);
        vision_result_t r = run(c, frame, W, H, 1, &z);
        if (!r.intrusion) missed++;
    }
    CHECK(missed == 0, "stationary intruder must stay detected for the whole minute (missed %d frames)", missed);

    scene(frame, W, H, 0);                      // leaves
    vision_result_t r = run(c, frame, W, H, 1, &z);
    CHECK(!r.intrusion, "clears immediately once the person is gone (no ghost trail in the zone)");
    vision_destroy(c);
}

static void test_warning_distance(void) {
    TEST("clearance is reported accurately and the cutoff margin trips before contact");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 30);
    // zone left edge is x=160. A 24px-wide box ending at x=160-D has clearance ~D.
    scene(frame, W, H, 0); box(frame, W, H, 100, 90, 135, 150, 225);   // right edge 135 -> clearance ~25
    vision_result_t r = run(c, frame, W, H, 1, &z);
    CHECK(!r.intrusion && r.dist_px > 15.0f && r.dist_px < 45.0f, "25px away: warn range, not cut off (dist %.1f)", r.dist_px);
    scene(frame, W, H, 0); box(frame, W, H, 100, 90, 148, 150, 225);   // clearance ~12 < 15 margin
    r = run(c, frame, W, H, 1, &z);
    CHECK(r.intrusion, "inside the 15px cutoff margin trips (dist %.1f)", r.dist_px);
    vision_destroy(c);
}

static void test_every_blob_is_checked(void) {
    TEST("a small intruder in the zone is detected even when a bigger object is elsewhere");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 30);
    scene(frame, W, H, 0);
    box(frame, W, H, 10, 40, 80, 200, 225);        // big blob far from the zone (70x160)
    box(frame, W, H, 200, 100, 223, 123, 225);     // small blob inside the zone (24x24)
    vision_result_t r = run(c, frame, W, H, 1, &z);
    CHECK(r.blob_count == 2, "both blobs seen (%u)", r.blob_count);
    CHECK(r.intrusion && r.bbox_x >= 190, "intrusion reported for the blob INSIDE the zone (bbox x=%u)", r.bbox_x);
    vision_destroy(c);
}

static void test_arm_is_judged_by_its_hand(void) {
    TEST("an arm reaching in from the edge: box follows the hand, clearance is the hand's, L-shapes do not false-trip");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 40);
    // arm from the left edge to x=135 (zone starts at x=160): hand clearance ~25
    scene(frame, W, H, 0); box(frame, W, H, 0, 90, 135, 120, 225);
    vision_result_t r = run(c, frame, W, H, 1, &z);
    CHECK(!r.intrusion && r.dist_px > 15.0f && r.dist_px < 45.0f, "arm tip 25px away: warning, not cut off (dist %.1f)", r.dist_px);
    CHECK(r.bbox_x + r.bbox_w >= 130 && r.bbox_x + r.bbox_w <= 140, "box ends at the hand (right edge %u)", r.bbox_x + r.bbox_w);
    CHECK(r.bbox_w < 90, "box is the hand/forearm, not the whole arm (width %u of 136)", r.bbox_w);
    // hand enters the zone: still tripped
    scene(frame, W, H, 0); box(frame, W, H, 0, 90, 175, 120, 225);
    r = run(c, frame, W, H, 1, &z);
    CHECK(r.intrusion, "hand inside the zone trips");
    // L-shaped blob wrapping around the zone at a safe distance: its bounding rectangle covers the zone, its pixels do not
    scene(frame, W, H, 0); box(frame, W, H, 20, 0, 40, 239, 225); box(frame, W, H, 20, 224, 319, 239, 225);
    r = run(c, frame, W, H, 1, &z);
    CHECK(!r.intrusion && r.dist_px > 15.0f, "L-shape around the zone is not a false stop (dist %.1f)", r.dist_px);
    vision_destroy(c);
}

static void test_ghost_heals_and_tracker_follows_the_mover(void) {
    TEST("REGRESSION: object rests, leaves; the empty spot it left must not keep the tracker (box follows the real mover)");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 40);
    // hand rests 55px from the zone (outside the freeze/warn ring) for 4 s: the background partly absorbs it
    for (int i = 0; i < 120; i++) { scene(frame, W, H, 0); box(frame, W, H, 70, 100, 105, 140, 225); run(c, frame, W, H, 1, &z); }
    // it moves away to the left and keeps moving (small jitter); the vacated spot is nearer the zone than the hand
    vision_result_t r = {0};
    int wrong = 0, seen = 0;
    for (int i = 0; i < 90; i++) {
        int x = (i * 3) % 80; if (x > 40) x = 80 - x;   // keeps travelling at 3 px/frame, 0..40
        scene(frame, W, H, 0); box(frame, W, H, x, 100, x + 30, 140, 225);
        r = run(c, frame, W, H, 1, &z);
        if (i >= 50) { seen++; if (!(r.blob_count >= 1 && r.bbox_x + r.bbox_w <= 72)) wrong++; }
    }
    CHECK(wrong == 0, "within 1.7 s the box is on the mover, not on the vacated spot (%d/%d frames wrong, last box x=%u w=%u)", wrong, seen, r.bbox_x, r.bbox_w);
    vision_destroy(c);
}

static void test_still_person_in_warning_ring_is_not_learned_away(void) {
    TEST("a person standing still inside the 45px warning ring stays detected (fast healing never applies there)");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 40);
    int lost = 0;
    for (int i = 0; i < 600; i++) {            // 20 s, box right edge 125 -> clearance ~35
        scene(frame, W, H, 0); box(frame, W, H, 95, 100, 125, 140, 225);
        vision_result_t r = run(c, frame, W, H, 1, &z);
        if (r.blob_count < 1 || r.dist_px > 45.0f) lost++;
    }
    CHECK(lost == 0, "warning stays up for 20 s of standing still (%d frames lost)", lost);
    vision_destroy(c);
}

static void test_no_blind_border(void) {
    TEST("intruder touching the frame border is detected (old code ignored a 6-16 px border band)");
    vision_ctx_t *c = vision_create();
    safety_zone_t z = { .num_pts = 4 };
    z.vx[0]=0.90f; z.vy[0]=0.20f; z.vx[1]=0.99f; z.vy[1]=0.20f; z.vx[2]=0.99f; z.vy[2]=0.85f; z.vx[3]=0.90f; z.vy[3]=0.85f;
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 30);
    scene(frame, W, H, 0); box(frame, W, H, 300, 90, W - 1, 150, 225);
    vision_result_t r = run(c, frame, W, H, 1, &z);
    CHECK(r.intrusion, "border-hugging intruder detected");
    vision_destroy(c);
}

static void test_large_frame_full_coverage(void) {
    TEST("frames wider than 512 px are fully monitored (old code only looked at the top-left 512x512)");
    static uint8_t big[800 * 600];
    vision_ctx_t *c = vision_create();
    safety_zone_t z = { .num_pts = 4 };
    z.vx[0]=0.70f; z.vy[0]=0.40f; z.vx[1]=0.95f; z.vy[1]=0.40f; z.vx[2]=0.95f; z.vy[2]=0.90f; z.vx[3]=0.70f; z.vy[3]=0.90f;
    scene(big, 800, 600, 0); for (int i = 0; i < 30; i++) run(c, big, 800, 600, 1, &z);
    scene(big, 800, 600, 0); box(big, 800, 600, 620, 300, 680, 420, 225);   // x>512 AND y>512-ish region
    vision_result_t r = run(c, big, 800, 600, 1, &z);
    CHECK(r.intrusion, "intruder at x=620..680 in an 800x600 frame detected");
    vision_destroy(c);
}

static void test_stream_swap_epoch(void) {
    TEST("stream swap: epoch bump re-seeds cleanly (no false trip); without it the stale background would trip");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 40);

    scene(frame, W, H, 1);                                    // bright real-video-like scene
    vision_result_t r = run(c, frame, W, H, 2, &z);           // epoch 1 -> 2
    CHECK(r.reseeded && !r.intrusion && r.blob_count == 0, "no false intrusion on swap");
    CHECK(!r.baseline_valid, "baseline not valid right after a swap -> reset stays blocked");
    for (int i = 0; i < 35; i++) r = run(c, frame, W, H, 2, &z);
    CHECK(r.baseline_valid && !r.intrusion, "baseline valid after teach-in");

    vision_ctx_t *c2 = vision_create();                       // control: same swap but forgetting the epoch bump
    scene(frame, W, H, 0); settle(c2, frame, W, H, &z, 40);
    scene(frame, W, H, 1);
    r = run(c2, frame, W, H, 1, &z);
    CHECK(r.intrusion, "control: stale background on an un-signalled swap DOES trip (fail-safe, not silent)");
    vision_destroy(c); vision_destroy(c2);
}

static void test_frame_size_change_reseeds(void) {
    TEST("frame size change re-seeds");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 35);
    static uint8_t small[160 * 120];
    scene(small, 160, 120, 0);
    vision_result_t r = run(c, small, 160, 120, 1, &z);
    CHECK(r.reseeded && !r.intrusion, "size change reseeds without a false trip");
    vision_destroy(c);
}

static void test_outside_object_no_ghost_in_zone(void) {
    TEST("an object parked outside the zone for a long time then removed does not cause a false trip");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    scene(frame, W, H, 0); settle(c, frame, W, H, &z, 30);
    for (int i = 0; i < 600; i++) { scene(frame, W, H, 0); box(frame, W, H, 20, 90, 60, 150, 225); run(c, frame, W, H, 1, &z); }
    scene(frame, W, H, 0);
    int trips = 0;
    for (int i = 0; i < 100; i++) { vision_result_t r = run(c, frame, W, H, 1, &z); if (r.intrusion) trips++; }
    CHECK(trips == 0, "no false intrusion from a removed far-away object (%d)", trips);
    vision_destroy(c);
}

static void test_bad_arguments(void) {
    TEST("invalid arguments are rejected, not processed");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone(); vision_params_t p = params(); vision_result_t r;
    scene(frame, W, H, 0);
    CHECK(vision_process(c, frame, 8, 8, 1, &z, &p, &r) == -2, "tiny frame");
    CHECK(vision_process(c, frame, 1300, 100, 1, &z, &p, &r) == -2, "oversized dimension");
    safety_zone_t bad = z; bad.num_pts = 2;
    CHECK(vision_process(c, frame, W, H, 1, &bad, &p, &r) == -2, "bad zone");
    CHECK(vision_process(NULL, frame, W, H, 1, &z, &p, &r) == -2, "null ctx");
    vision_destroy(c);
}

// Same patrol motion as cam_feed's synthetic operator (cosine ease, lingers near the zone centre).
static void patrol_frame(uint8_t *f, uint32_t fid) {
    scene(f, W, H, 0);
    float phase = (float)(fid % 240U) / 240.0f * 2.0f * 3.1415926f, prog = 0.5f - 0.5f * cosf(phase);
    int cx = (int)((0.12f + (0.71f - 0.12f) * prog) * W), cy = (int)(0.525f * H);
    box(f, W, H, cx - 17, cy - 31, cx + 17, cy + 31, 225);
}
static int patrol_x(uint32_t fid) {
    float phase = (float)(fid % 240U) / 240.0f * 2.0f * 3.1415926f, prog = 0.5f - 0.5f * cosf(phase);
    return (int)((0.12f + (0.71f - 0.12f) * prog) * W);
}

static void test_restart_mid_patrol_never_leaves_a_phantom(void) {
    TEST("REGRESSION: worker (re)start while the operator is in/near the zone must not leave a phantom tracker");
    safety_zone_t z = right_zone();
    const uint32_t starts[] = { 0, 40, 80, 100, 110, 120, 130, 140, 160, 200 };
    int phantom_starts = 0, never_valid = 0, worst = 0;
    for (unsigned k = 0; k < sizeof(starts) / sizeof(starts[0]); k++) {
        vision_ctx_t *c = vision_create();
        int valid_at = -1, phantoms = 0;
        for (int i = 0; i < 800; i++) {
            patrol_frame(frame, starts[k] + (uint32_t)i);
            vision_result_t r = run(c, frame, W, H, 1, &z);
            if (r.baseline_valid && valid_at < 0) valid_at = i;
            if (valid_at >= 0 && i > valid_at + 240 && r.intrusion) {
                int ox = patrol_x(starts[k] + (uint32_t)i);
                if (!((int)r.bbox_x <= ox + 25 && (int)(r.bbox_x + r.bbox_w) >= ox - 25)) phantoms++;   // box not on the operator
            }
        }
        if (valid_at < 0) never_valid++; else if (valid_at > worst) worst = valid_at;
        if (phantoms) phantom_starts++;
        vision_destroy(c);
    }
    CHECK(phantom_starts == 0, "%d restart phases ended with the tracker stuck away from the operator", phantom_starts);
    CHECK(never_valid == 0, "%d restart phases never finished teach-in", never_valid);
    CHECK(worst <= 260, "teach-in finishes within 260 frames even in the worst phase (worst %d)", worst);
}

static void test_seed_with_object_that_leaves_heals(void) {
    TEST("a seed that contained an object which then leaves heals during teach-in (no permanent ghost)");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    vision_result_t r = {0};
    for (int i = 0; i < 6; i++) { scene(frame, W, H, 0); box(frame, W, H, 200, 90, 235, 150, 225); r = run(c, frame, W, H, 1, &z); }
    int valid_at = -1, ghost = 0;
    for (int i = 0; i < 400; i++) {
        scene(frame, W, H, 0);
        r = run(c, frame, W, H, 1, &z);
        if (r.baseline_valid && valid_at < 0) valid_at = i;
        if (valid_at >= 0 && r.intrusion) ghost++;
    }
    CHECK(valid_at > 0 && valid_at < 120, "teach-in completes once the zone is empty (valid after %d frames)", valid_at);
    CHECK(ghost == 0, "no phantom intrusion after teach-in (%d frames)", ghost);
    vision_destroy(c);
}

static void test_busy_zone_never_validates(void) {
    TEST("a zone that is never quiet never gets a trusted baseline (stays fail-safe)");
    vision_ctx_t *c = vision_create(); safety_zone_t z = right_zone();
    int valid = 0;
    for (int i = 0; i < 600; i++) {
        scene(frame, W, H, 0);
        int x = 180 + ((i / 4) % 2 ? 30 : 0) + (i % 4) * 7;     // keeps moving inside the zone
        box(frame, W, H, x, 90, x + 35, 150, 225);
        vision_result_t r = run(c, frame, W, H, 1, &z);
        if (r.baseline_valid) valid++;
    }
    CHECK(valid == 0, "baseline must not become valid while something keeps moving in the zone (%d frames valid)", valid);
    vision_destroy(c);
}

int main(void) {
    printf("test_vision\n");
    test_empty_scene_and_warmup();
    test_intruder_detected_and_never_absorbed();
    test_warning_distance();
    test_every_blob_is_checked();
    test_arm_is_judged_by_its_hand();
    test_ghost_heals_and_tracker_follows_the_mover();
    test_still_person_in_warning_ring_is_not_learned_away();
    test_no_blind_border();
    test_large_frame_full_coverage();
    test_stream_swap_epoch();
    test_frame_size_change_reseeds();
    test_outside_object_no_ghost_in_zone();
    test_bad_arguments();
    test_restart_mid_patrol_never_leaves_a_phantom();
    test_seed_with_object_that_leaves_heals();
    test_busy_zone_never_validates();
    return check_summary("test_vision");
}
