#ifndef CONFIG_LOADER_H
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
