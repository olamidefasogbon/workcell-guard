// Config loader: strict parsing and cross-field validation. Run from the repo root.
#include <string.h>
#include "check.h"
#include "config_loader.h"

static int load_text(const char *text, workcell_config_t *cfg) {
    const char *path = "build/_test.conf";
    FILE *f = fopen(path, "w");
    if (!f) { perror("fopen"); exit(2); }
    fputs(text, f);
    fclose(f);
    return config_load(path, cfg);
}

int main(void) {
    printf("test_config\n");
    workcell_config_t c;

    TEST("shipped workcell.conf loads and validates");
    CHECK(config_load("workcell.conf", &c) == 0, "workcell.conf must be valid");
    CHECK(c.latch_faults == 1 && c.diff_threshold == 38 && c.heartbeat_timeout_ms == 150, "expected values");
    CHECK(c.zone.num_pts == 4 && c.zone.vx[0] == 0.50f && c.zone.vy[2] == 0.85f, "zone_norm is actually applied (it used to be ignored)");

    TEST("missing file is fatal, not a silent default");
    CHECK(config_load("build/does_not_exist.conf", &c) != 0, "missing config must fail");

    TEST("unknown keys / malformed values are fatal");
    CHECK(load_text("bogus_key=1\n", &c) != 0, "unknown key");
    CHECK(load_text("target_fps=30abc\n", &c) != 0, "trailing junk in number");
    CHECK(load_text("target_fps=\n", &c) != 0, "empty number");
    CHECK(load_text("diff_threshold=-5\n", &c) != 0, "negative threshold");
    CHECK(load_text("cutoff_margin_px=nan\n", &c) != 0, "NaN float");
    CHECK(load_text("justtext\n", &c) != 0, "line without =");
    CHECK(load_text("device_path=/dev/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n", &c) != 0, "overlong string");

    TEST("priority isolation is enforced");
    CHECK(load_text("prio_vision=30\n", &c) != 0, "vision above gate");
    CHECK(load_text("prio_telemetry=15\n", &c) != 0, "telemetry == camera");
    CHECK(load_text("prio_supervisor=26\n", &c) != 0, "supervisor above gate");

    TEST("timing relations are enforced");
    CHECK(load_text("heartbeat_timeout_ms=10\n", &c) != 0, "timeout shorter than minimum");
    CHECK(load_text("target_fps=5\n", &c) != 0, "timeout must cover 2 frame periods (150 < 400)");
    CHECK(load_text("target_fps=5\nheartbeat_timeout_ms=500\nframe_stall_exit_ms=600\nhang_kill_ms=1000\nmax_frame_age_ms=100\n", &c) == 0, "consistent slow-camera config OK");
    CHECK(load_text("max_frame_age_ms=200\n", &c) != 0, "frame age above heartbeat timeout");
    CHECK(load_text("hang_kill_ms=200\n", &c) != 0, "hang kill must be >= 2x heartbeat");

    TEST("latch cannot be disabled in a production build");
#ifdef WORKCELL_TEST_HOOKS
    CHECK(load_text("latch_faults=0\n", &c) == 0, "allowed in hooks build");
#else
    CHECK(load_text("latch_faults=0\n", &c) != 0, "latch_faults=0 refused");
#endif

    TEST("ports");
    CHECK(load_text("telemetry_port=9877\n", &c) != 0, "duplicate port");
    CHECK(load_text("telemetry_port=80\n", &c) != 0, "privileged port");

    TEST("control_allow_ip");
    CHECK(load_text("control_allow_ip=192.168.184.1\n", &c) == 0, "valid ip");
    CHECK(load_text("control_allow_ip=192.168.184.256\n", &c) != 0, "octet > 255");
    CHECK(load_text("control_allow_ip=192.168.1\n", &c) != 0, "too few octets");
    CHECK(load_text("control_allow_ip=1.2.3.4.5\n", &c) != 0, "too many octets");
    CHECK(load_text("control_allow_ip=a.b.c.d\n", &c) != 0, "not numeric");

    TEST("zones");
    CHECK(load_text("zone_pts=0.1,0.1;0.9,0.1;0.9,0.9;0.1,0.9\n", &c) == 0 && c.zone.num_pts == 4, "polygon accepted");
    CHECK(load_text("zone_pts=0.2,0.2;0.8,0.8;0.8,0.2;0.2,0.8\n", &c) != 0, "bow-tie rejected at config time");
    CHECK(load_text("zone_pts=0.1,0.1;0.9,0.1\n", &c) != 0, "two points rejected");
    CHECK(load_text("zone_norm=0.5,0.2,0.5,0.85\n", &c) != 0, "zero-width rectangle rejected");
    CHECK(load_text("zone_norm=0.5,0.2,1.5,0.85\n", &c) != 0, "out of range rejected");
    CHECK(load_text("# comment\n\n  zone_norm = 0.1 , 0.1 , 0.4 , 0.4 \n", &c) != 0, "spaces inside the value are rejected (strict)");
    CHECK(load_text("# comment\n\n  zone_norm = 0.1,0.1,0.4,0.4  \n", &c) == 0, "surrounding whitespace is fine");

    return check_summary("test_config");
}
