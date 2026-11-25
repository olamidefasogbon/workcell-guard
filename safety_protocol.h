#ifndef SAFETY_PROTOCOL_H
#define SAFETY_PROTOCOL_H

#include <stdint.h>

#ifdef __QNX__
#include <devctl.h>
#endif

#define MAX_FRAME_PIXELS  (1280 * 720)
#define MAX_FRAME_DIM     1280
#define MAX_ZONE_VERTICES 8
#define OSSD_ON_MV        24000U

typedef enum {
    STATE_SAFE_RUN       = 0,
    STATE_INTRUSION_HALT = 1,
    STATE_WATCHDOG_FAULT = 2   // generic "fault, outputs de-energised" state
} gate_state_t;

// Why the gate is not in SAFE_RUN. Reported to the operator; never used to
// decide whether the output is energised (only gate_state_t does that).
typedef enum {
    FAULT_NONE           = 0,
    FAULT_POWER_UP       = 1,  // startup interlock: outputs stay off until a deliberate reset
    FAULT_INTRUSION      = 2,
    FAULT_HEARTBEAT_LOST = 3,
    FAULT_STALE_FRAME    = 4,
    FAULT_BAD_REPORT     = 5,
    FAULT_INVALID_ZONE   = 6,
    FAULT_ZONE_MISMATCH  = 7   // vision is not evaluating the polygon the gate holds
} fault_reason_t;

// Result of the last reset request. A refused reset leaves the outputs off.
typedef enum {
    RESET_OK                   = 0,
    RESET_REFUSED_NO_HEARTBEAT = 1,  // no fresh, valid vision report
    RESET_REFUSED_HAZARD       = 2,  // vision still reports an intrusion
    RESET_REFUSED_SETTLING     = 3,  // not enough consecutive clear frames / baseline not valid yet
    RESET_REFUSED_ZONE         = 4   // held polygon is invalid
} reset_result_t;

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
    uint32_t stream_epoch;
    uint8_t  intrusion_detected;  // must be exactly 0 or 1
    uint8_t  baseline_valid;      // background model settled since the last reseed
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
    uint32_t ossd_mv;       // 24000 mV (24V DC) only in SAFE_RUN, 0 mV otherwise
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
    int32_t  fault_reason;
    int32_t  reset_ready;        // 1 if a reset request would be accepted right now
    int32_t  last_reset_result;  // reset_result_t of the most recent request
    uint32_t reset_refusals;
    uint32_t clear_streak;
    uint32_t stream_epoch;
    uint32_t heartbeat_age_ms;   // saturates; UINT32_MAX = no valid report yet
    int32_t  test_hooks;         // 1 if built with WORKCELL_TEST_HOOKS
} safety_status_t;

// POSIX Shared Memory layout for decoupled camera frames (/dev/shmem/workcell_cam)
typedef struct {
    volatile uint32_t write_seq;      // odd = write in progress
    uint32_t width;
    uint32_t height;
    uint32_t frame_id;
    uint32_t stream_epoch;            // bumped by cam_feed on every stream swap
    uint32_t reserved;
    uint64_t capture_timestamp_ns;
    uint8_t  pixels[MAX_FRAME_PIXELS];
} shm_camera_t;

#ifdef __QNX__
// Native QNX devctl commands for /dev/safety_gate
#define DCMD_SAFETY_GET_STATUS __DIOF(_DCMD_MISC, 1, safety_status_t)
#define DCMD_SAFETY_SET_ZONE   __DIOT(_DCMD_MISC, 2, safety_zone_t)
#define DCMD_SAFETY_RESET      __DION(_DCMD_MISC, 3)
#define DCMD_SAFETY_SET_LATCH  __DIOT(_DCMD_MISC, 4, int32_t)
#endif

#endif
