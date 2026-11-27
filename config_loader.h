#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "safety_logic.h"
#include "safety_protocol.h"

// A safety configuration is either correct or the process refuses to start.
// There are no silent defaults for a missing file, no ignored keys and no
// partially-parsed numbers.

typedef struct {
    char device_path[64];
    char shm_name[64];

    // Scheduling (SCHED_FIFO). Must satisfy gate > supervisor > camera > vision > telemetry.
    int prio_gate;
    int prio_supervisor;
    int prio_camera;
    int prio_vision;
    int prio_telemetry;

    // Watchdog / timing
    int watchdog_tick_ms;
    int heartbeat_timeout_ms;   // no valid vision report for this long -> 0 V
    int max_frame_age_ms;       // capture->gate older than this -> 0 V
    int reset_clear_frames;     // consecutive clear frames needed before a reset is accepted
    int frame_stall_exit_ms;    // vision exits (and is respawned) if the camera stalls this long
    int hang_kill_ms;           // supervisor kills a worker whose heartbeat is older than this
    int respawn_delay_ms;
    int respawn_backoff_max_ms;

    // Camera
    int target_fps;
    uint32_t sim_width;
    uint32_t sim_height;

    // Vision
    uint32_t bg_warmup_frames;
    int bg_learn_quiet_frames;  // zone must be quiet this many consecutive frames before a (re)learned background is trusted
    int diff_threshold;
    float cutoff_margin_px;
    float freeze_margin_px;
    int cell_hit_min;
    int min_blob_cells;

    int latch_faults;           // must stay 1 unless built with WORKCELL_TEST_HOOKS

    int telemetry_port;
    int cam_ingest_port;
    int frame_stream_port;
    char control_allow_ip[40];  // "" = accept control from anywhere (logged as a warning)

    safety_zone_t zone;
} workcell_config_t;

#ifdef WORKCELL_TEST_HOOKS
#define WORKCELL_HOOKS_ENABLED 1
#else
#define WORKCELL_HOOKS_ENABLED 0
#endif

static inline void config_set_defaults(workcell_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    snprintf(cfg->device_path, sizeof(cfg->device_path), "%s", "/dev/safety_gate");
    snprintf(cfg->shm_name, sizeof(cfg->shm_name), "%s", "/workcell_cam");
    cfg->prio_gate               = 25;
    cfg->prio_supervisor         = 20;
    cfg->prio_camera             = 15;
    cfg->prio_vision             = 10;
    cfg->prio_telemetry          = 5;
    cfg->watchdog_tick_ms        = 5;
    cfg->heartbeat_timeout_ms    = 150;
    cfg->max_frame_age_ms        = 100;
    cfg->reset_clear_frames      = 15;
    cfg->frame_stall_exit_ms     = 500;
    cfg->hang_kill_ms            = 400;
    cfg->respawn_delay_ms        = 250;
    cfg->respawn_backoff_max_ms  = 5000;
    cfg->target_fps              = 30;
    cfg->sim_width               = 320;
    cfg->sim_height              = 240;
    cfg->bg_warmup_frames        = 20;
    cfg->bg_learn_quiet_frames   = 30;
    cfg->diff_threshold          = 38;
    cfg->cutoff_margin_px        = 15.0f;
    cfg->freeze_margin_px        = 30.0f;
    cfg->cell_hit_min            = 34;
    cfg->min_blob_cells          = 4;
    cfg->latch_faults            = 1;
    cfg->telemetry_port          = 9876;
    cfg->cam_ingest_port         = 9877;
    cfg->frame_stream_port       = 9878;

    cfg->zone.num_pts = 4;
    cfg->zone.vx[0] = 0.50f; cfg->zone.vy[0] = 0.20f;
    cfg->zone.vx[1] = 0.92f; cfg->zone.vy[1] = 0.20f;
    cfg->zone.vx[2] = 0.92f; cfg->zone.vy[2] = 0.85f;
    cfg->zone.vx[3] = 0.50f; cfg->zone.vy[3] = 0.85f;
}

static inline int cfg_parse_int(const char *v, long lo, long hi, int *out) {
    char *end = NULL;
    errno = 0;
    long x = strtol(v, &end, 10);
    if (errno != 0 || end == v || *end != '\0' || x < lo || x > hi) return -1;
    *out = (int)x;
    return 0;
}

static inline int cfg_parse_float(const char *v, float lo, float hi, float *out) {
    char *end = NULL;
    errno = 0;
    float x = strtof(v, &end);
    if (errno != 0 || end == v || *end != '\0' || !isfinite(x) || x < lo || x > hi) return -1;
    *out = x;
    return 0;
}

// Dotted-quad IPv4 -> host-order integer. Strict: exactly 4 octets 0..255, nothing else.
static inline int cfg_parse_ipv4(const char *s, uint32_t *out) {
    uint32_t addr = 0;
    for (int part = 0; part < 4; part++) {
        if (*s < '0' || *s > '9') return -1;
        uint32_t v = 0;
        int digits = 0;
        while (*s >= '0' && *s <= '9') {
            v = v * 10 + (uint32_t)(*s - '0');
            if (++digits > 3 || v > 255) return -1;
            s++;
        }
        addr = (addr << 8) | v;
        if (part < 3) { if (*s != '.') return -1; s++; }
    }
    if (*s != '\0') return -1;
    *out = addr;
    return 0;
}

static inline int cfg_copy_str(char *dst, size_t cap, const char *v) {
    int n = snprintf(dst, cap, "%s", v);
    return (n < 0 || (size_t)n >= cap) ? -1 : 0;
}

// "x1,y1,x2,y2"  -> axis-aligned rectangle
static inline int cfg_parse_zone_rect(const char *v, safety_zone_t *z) {
    float f[4];
    char buf[128];
    if (cfg_copy_str(buf, sizeof(buf), v) != 0) return -1;
    char *save = NULL;
    char *tok = strtok_r(buf, ",", &save);
    for (int i = 0; i < 4; i++) {
        if (!tok || cfg_parse_float(tok, 0.0f, 1.0f, &f[i]) != 0) return -1;
        tok = strtok_r(NULL, ",", &save);
    }
    if (tok) return -1;
    z->num_pts = 4;
    z->vx[0] = f[0]; z->vy[0] = f[1];
    z->vx[1] = f[2]; z->vy[1] = f[1];
    z->vx[2] = f[2]; z->vy[2] = f[3];
    z->vx[3] = f[0]; z->vy[3] = f[3];
    return 0;
}

// "x,y;x,y;x,y[;...]" -> arbitrary polygon (3..MAX_ZONE_VERTICES points)
static inline int cfg_parse_zone_pts(const char *v, safety_zone_t *z) {
    char buf[256];
    if (cfg_copy_str(buf, sizeof(buf), v) != 0) return -1;
    memset(z, 0, sizeof(*z));
    char *save1 = NULL;
    for (char *pt = strtok_r(buf, ";", &save1); pt; pt = strtok_r(NULL, ";", &save1)) {
        if (z->num_pts >= MAX_ZONE_VERTICES) return -1;
        char *save2 = NULL;
        char *sx = strtok_r(pt, ",", &save2);
        char *sy = strtok_r(NULL, ",", &save2);
        if (!sx || !sy || strtok_r(NULL, ",", &save2)) return -1;
        if (cfg_parse_float(sx, 0.0f, 1.0f, &z->vx[z->num_pts]) != 0) return -1;
        if (cfg_parse_float(sy, 0.0f, 1.0f, &z->vy[z->num_pts]) != 0) return -1;
        z->num_pts++;
    }
    return 0;
}

// Cross-field validation. Returns 0, or -1 with a message in err.
static inline int config_validate(const workcell_config_t *c, char *err, size_t cap) {
#define CFG_FAIL(...) do { snprintf(err, cap, __VA_ARGS__); return -1; } while (0)
    if (strncmp(c->device_path, "/dev/", 5) != 0 || strlen(c->device_path) < 6)
        CFG_FAIL("device_path must start with /dev/");
    if (c->shm_name[0] != '/' || strlen(c->shm_name) < 2 || strchr(c->shm_name + 1, '/'))
        CFG_FAIL("shm_name must look like /name (single leading slash)");

    int p[5] = { c->prio_gate, c->prio_supervisor, c->prio_camera, c->prio_vision, c->prio_telemetry };
    for (int i = 0; i < 5; i++) if (p[i] < 1 || p[i] > 255) CFG_FAIL("priorities must be in 1..255");
    for (int i = 0; i < 4; i++)
        if (p[i] <= p[i + 1])
            CFG_FAIL("priority isolation violated: need prio_gate > prio_supervisor > prio_camera > prio_vision > prio_telemetry");

    if (c->watchdog_tick_ms < 1 || c->watchdog_tick_ms > 100) CFG_FAIL("watchdog_tick_ms must be 1..100");
    if (c->target_fps < 1 || c->target_fps > 120) CFG_FAIL("target_fps must be 1..120");
    int frame_ms = 1000 / c->target_fps;
    if (c->heartbeat_timeout_ms < 20 || c->heartbeat_timeout_ms > 2000) CFG_FAIL("heartbeat_timeout_ms must be 20..2000");
    if (c->heartbeat_timeout_ms < 3 * c->watchdog_tick_ms) CFG_FAIL("heartbeat_timeout_ms must be >= 3 * watchdog_tick_ms");
    if (c->heartbeat_timeout_ms < 2 * frame_ms) CFG_FAIL("heartbeat_timeout_ms must cover at least 2 frame periods");
    if (c->max_frame_age_ms < 10 || c->max_frame_age_ms > c->heartbeat_timeout_ms)
        CFG_FAIL("max_frame_age_ms must be 10..heartbeat_timeout_ms");
    if (c->reset_clear_frames < 1 || c->reset_clear_frames > 300) CFG_FAIL("reset_clear_frames must be 1..300");
    if (c->frame_stall_exit_ms < c->heartbeat_timeout_ms || c->frame_stall_exit_ms > 10000)
        CFG_FAIL("frame_stall_exit_ms must be heartbeat_timeout_ms..10000");
    if (c->hang_kill_ms < 2 * c->heartbeat_timeout_ms || c->hang_kill_ms > 10000)
        CFG_FAIL("hang_kill_ms must be 2*heartbeat_timeout_ms..10000");
    if (c->respawn_delay_ms < 0 || c->respawn_delay_ms > 10000) CFG_FAIL("respawn_delay_ms must be 0..10000");
    if (c->respawn_backoff_max_ms < c->respawn_delay_ms || c->respawn_backoff_max_ms > 60000)
        CFG_FAIL("respawn_backoff_max_ms must be respawn_delay_ms..60000");

    if (c->sim_width < 16 || c->sim_width > MAX_FRAME_DIM || c->sim_height < 16 || c->sim_height > 720 ||
        (size_t)c->sim_width * c->sim_height > MAX_FRAME_PIXELS)
        CFG_FAIL("sim_width/sim_height out of range");

    if (c->bg_warmup_frames < 1 || c->bg_warmup_frames > 10000) CFG_FAIL("bg_warmup_frames must be 1..10000");
    if (c->bg_learn_quiet_frames < 1 || c->bg_learn_quiet_frames > 1000) CFG_FAIL("bg_learn_quiet_frames must be 1..1000");
    if (c->diff_threshold < 5 || c->diff_threshold > 250) CFG_FAIL("diff_threshold must be 5..250");
    if (c->cutoff_margin_px < 0.0f || c->cutoff_margin_px > 200.0f) CFG_FAIL("cutoff_margin_px must be 0..200");
    if (c->freeze_margin_px < c->cutoff_margin_px || c->freeze_margin_px > 300.0f)
        CFG_FAIL("freeze_margin_px must be cutoff_margin_px..300");
    if (c->cell_hit_min < 1 || c->cell_hit_min > 64) CFG_FAIL("cell_hit_min must be 1..64");
    if (c->min_blob_cells < 1 || c->min_blob_cells > 64) CFG_FAIL("min_blob_cells must be 1..64");

    if (!WORKCELL_HOOKS_ENABLED && c->latch_faults != 1)
        CFG_FAIL("latch_faults=0 is only allowed in a WORKCELL_TEST_HOOKS build (ISO 13849-1 restart interlock)");

    int ports[3] = { c->telemetry_port, c->cam_ingest_port, c->frame_stream_port };
    for (int i = 0; i < 3; i++) if (ports[i] < 1024 || ports[i] > 65535) CFG_FAIL("ports must be 1024..65535");
    if (ports[0] == ports[1] || ports[0] == ports[2] || ports[1] == ports[2]) CFG_FAIL("ports must be distinct");

    if (c->control_allow_ip[0]) {
        uint32_t a;
        if (cfg_parse_ipv4(c->control_allow_ip, &a) != 0) CFG_FAIL("control_allow_ip is not a valid IPv4 address");
    }

    int zr = zone_validate(&c->zone);
    if (zr != ZONE_OK) CFG_FAIL("configured zone is invalid (zone_validate error %d)", zr);
    return 0;
#undef CFG_FAIL
}

// Returns 0 on success. On any problem prints the reason to stderr and returns -1;
// callers must exit non-zero.
static inline int config_load(const char *path, workcell_config_t *cfg) {
    config_set_defaults(cfg);
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "config: cannot open '%s': %s\n", path, strerror(errno));
        return -1;
    }

    char line[256], err[200];
    int lineno = 0, rc = 0;
    while (rc == 0 && fgets(line, sizeof(line), f)) {
        lineno++;
        if (!strchr(line, '\n') && !feof(f)) { snprintf(err, sizeof(err), "line too long"); rc = -1; break; }

        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        char *eq = strchr(p, '=');
        if (!eq) { snprintf(err, sizeof(err), "expected key=value"); rc = -1; break; }
        *eq = '\0';
        char *k = p, *v = eq + 1;
        char *kend = k + strlen(k);
        while (kend > k && (kend[-1] == ' ' || kend[-1] == '\t')) *--kend = '\0';
        while (*v == ' ' || *v == '\t') v++;
        char *vend = v + strlen(v);
        while (vend > v && (vend[-1] == '\n' || vend[-1] == '\r' || vend[-1] == ' ' || vend[-1] == '\t')) *--vend = '\0';

        int bad = 0;
        if      (!strcmp(k, "device_path"))             bad = cfg_copy_str(cfg->device_path, sizeof(cfg->device_path), v);
        else if (!strcmp(k, "shm_name"))                bad = cfg_copy_str(cfg->shm_name, sizeof(cfg->shm_name), v);
        else if (!strcmp(k, "control_allow_ip"))        bad = cfg_copy_str(cfg->control_allow_ip, sizeof(cfg->control_allow_ip), v);
        else if (!strcmp(k, "prio_gate"))               bad = cfg_parse_int(v, 1, 255, &cfg->prio_gate);
        else if (!strcmp(k, "prio_supervisor"))         bad = cfg_parse_int(v, 1, 255, &cfg->prio_supervisor);
        else if (!strcmp(k, "prio_camera"))             bad = cfg_parse_int(v, 1, 255, &cfg->prio_camera);
        else if (!strcmp(k, "prio_vision"))             bad = cfg_parse_int(v, 1, 255, &cfg->prio_vision);
        else if (!strcmp(k, "prio_telemetry"))          bad = cfg_parse_int(v, 1, 255, &cfg->prio_telemetry);
        else if (!strcmp(k, "watchdog_tick_ms"))        bad = cfg_parse_int(v, 1, 100, &cfg->watchdog_tick_ms);
        else if (!strcmp(k, "heartbeat_timeout_ms"))    bad = cfg_parse_int(v, 1, 100000, &cfg->heartbeat_timeout_ms);
        else if (!strcmp(k, "max_frame_age_ms"))        bad = cfg_parse_int(v, 1, 100000, &cfg->max_frame_age_ms);
        else if (!strcmp(k, "reset_clear_frames"))      bad = cfg_parse_int(v, 1, 100000, &cfg->reset_clear_frames);
        else if (!strcmp(k, "frame_stall_exit_ms"))     bad = cfg_parse_int(v, 1, 100000, &cfg->frame_stall_exit_ms);
        else if (!strcmp(k, "hang_kill_ms"))            bad = cfg_parse_int(v, 1, 100000, &cfg->hang_kill_ms);
        else if (!strcmp(k, "respawn_delay_ms"))        bad = cfg_parse_int(v, 0, 100000, &cfg->respawn_delay_ms);
        else if (!strcmp(k, "respawn_backoff_max_ms"))  bad = cfg_parse_int(v, 0, 1000000, &cfg->respawn_backoff_max_ms);
        else if (!strcmp(k, "target_fps"))              bad = cfg_parse_int(v, 1, 1000, &cfg->target_fps);
        else if (!strcmp(k, "sim_width"))   { int t = 0; bad = cfg_parse_int(v, 1, 100000, &t); cfg->sim_width = (uint32_t)t; }
        else if (!strcmp(k, "sim_height"))  { int t = 0; bad = cfg_parse_int(v, 1, 100000, &t); cfg->sim_height = (uint32_t)t; }
        else if (!strcmp(k, "bg_warmup_frames")) { int t = 0; bad = cfg_parse_int(v, 1, 1000000, &t); cfg->bg_warmup_frames = (uint32_t)t; }
        else if (!strcmp(k, "bg_learn_quiet_frames"))   bad = cfg_parse_int(v, 1, 100000, &cfg->bg_learn_quiet_frames);
        else if (!strcmp(k, "diff_threshold"))          bad = cfg_parse_int(v, 1, 1000, &cfg->diff_threshold);
        else if (!strcmp(k, "cutoff_margin_px"))        bad = cfg_parse_float(v, 0.0f, 10000.0f, &cfg->cutoff_margin_px);
        else if (!strcmp(k, "freeze_margin_px"))        bad = cfg_parse_float(v, 0.0f, 10000.0f, &cfg->freeze_margin_px);
        else if (!strcmp(k, "cell_hit_min"))            bad = cfg_parse_int(v, 1, 1000, &cfg->cell_hit_min);
        else if (!strcmp(k, "min_blob_cells"))          bad = cfg_parse_int(v, 1, 1000, &cfg->min_blob_cells);
        else if (!strcmp(k, "latch_faults"))            bad = cfg_parse_int(v, 0, 1, &cfg->latch_faults);
        else if (!strcmp(k, "telemetry_port"))          bad = cfg_parse_int(v, 1, 65535, &cfg->telemetry_port);
        else if (!strcmp(k, "cam_ingest_port"))         bad = cfg_parse_int(v, 1, 65535, &cfg->cam_ingest_port);
        else if (!strcmp(k, "frame_stream_port"))       bad = cfg_parse_int(v, 1, 65535, &cfg->frame_stream_port);
        else if (!strcmp(k, "zone_norm"))               bad = cfg_parse_zone_rect(v, &cfg->zone);
        else if (!strcmp(k, "zone_pts"))                bad = cfg_parse_zone_pts(v, &cfg->zone);
        else { snprintf(err, sizeof(err), "unknown key '%s'", k); rc = -1; break; }

        if (bad) { snprintf(err, sizeof(err), "invalid value for '%s'", k); rc = -1; }
    }
    fclose(f);

    if (rc != 0) {
        fprintf(stderr, "config %s:%d: %s\n", path, lineno, err);
        return -1;
    }
    if (config_validate(cfg, err, sizeof(err)) != 0) {
        fprintf(stderr, "config %s: %s\n", path, err);
        return -1;
    }
    return 0;
}

#endif
