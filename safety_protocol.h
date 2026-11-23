#ifndef SAFETY_PROTOCOL_H
#define SAFETY_PROTOCOL_H

#include <devctl.h>
#include <stdint.h>

#define MAX_FRAME_PIXELS (1280 * 720)
#define MAX_ZONE_VERTICES 8

typedef enum {
    STATE_SAFE_RUN       = 0,
    STATE_INTRUSION_HALT = 1,
    STATE_WATCHDOG_FAULT = 2
} gate_state_t;

// Dynamic N-point normalized danger-zone polygon (0.0 to 1.0)
typedef struct {
    uint32_t num_pts;
    float    vx[MAX_ZONE_VERTICES];
    float    vy[MAX_ZONE_VERTICES];
} safety_zone_t;

// Telemetry packet sent from vision_worker -> /dev/safety_gate
typedef struct {
    uint64_t capture_timestamp_ns;
    uint32_t frame_id;
    uint8_t  intrusion_detected;
    float    min_zone_distance_px;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t frame_w;
    uint16_t frame_h;
    safety_zone_t active_zone;
} vision_report_t;

// Snapshot returned via devctl(DCMD_SAFETY_GET_STATUS)
typedef struct {
    int32_t  state;
    int32_t  latched;
    int32_t  latch_enabled;
    uint32_t trip_count;
    uint32_t ossd_mv;       // 24000 mV (24V DC) when SAFE_RUN, 0 mV on HALT/FAULT
    uint32_t frame_id;
    uint64_t latency_us;
    uint64_t last_trip_us;
    float    min_zone_distance_px;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t frame_w;
    uint16_t frame_h;
    safety_zone_t zone;
} safety_status_t;

// POSIX Shared Memory layout for decoupled camera frames (/dev/shmem/workcell_cam)
typedef struct {
    volatile uint32_t write_seq;
    uint32_t width;
    uint32_t height;
    uint32_t frame_id;
    uint64_t capture_timestamp_ns;
    uint8_t  pixels[MAX_FRAME_PIXELS];
} shm_camera_t;

// Native QNX devctl commands for /dev/safety_gate
#define DCMD_SAFETY_GET_STATUS __DIOF(_DCMD_MISC, 1, safety_status_t)
#define DCMD_SAFETY_SET_ZONE   __DIOT(_DCMD_MISC, 2, safety_zone_t)
#define DCMD_SAFETY_RESET      __DION(_DCMD_MISC, 3)
#define DCMD_SAFETY_SET_LATCH  __DIOT(_DCMD_MISC, 4, int32_t)

#endif
