# Workcell Guard — QNX 8.0 Type-4 Virtual Optical Light Curtain

A deterministic real-time safety controller built on **QNX Neutrino RTOS 8.0** (`x86_64`) that provides a programmable 2D polygon optical safety boundary, sub-millisecond IPC cutoff (`24V -> 0V` OSSD relay simulation), a `5 ms` kernel pulse watchdog (`150 ms` heartbeat timeout), and automatic process recovery (`250 ms` respawn with crash-loop backoff and hang detection).

## QNX 8.0 Architecture

| Process | Priority | QNX / POSIX Mechanism | Role |
| :--- | :---: | :--- | :--- |
| `safety_gate` (`safety_gate.c`) | `25` | `resmgr_attach`, `io_devctl`, `timer_create` + `SIGEV_PULSE_INIT` | Safety relay manager (`/dev/safety_gate`, `24V` OSSD) + `150 ms` watchdog |
| `supervisor` (`supervisor.c`) | `20` | `fork`, `execv`, `waitpid` | Monitors and respawns `vision_worker` within `250 ms` after crashes |
| `cam_feed` (`cam_feed.c`) | `15` | `shm_open`, `mmap` (`/dev/shmem/workcell_cam`), TCP (`9877`) | `30 FPS` (`320x240`) shared-memory frame producer and video clip ingest |
| `vision_worker` (`vision_worker.c`) | `10` | `mmap` seqlock reader, `devctl`, `write` | Computes 2D polygon intrusion/clearance and reports to `/dev/safety_gate` |
| `telemetry_bridge` (`telemetry_bridge.c`) | `5` | `devctl` (`DCMD_SAFETY_*`), UDP (`9876`), TCP (`9878`) | Bridges `devctl` controls and live frames to the host control plane |

## Target, Toolchain & Build Instructions

- **OS & Target:** QNX Neutrino RTOS **8.0** (`x86_64` VM / QEMU / x86_64 target)
- **Toolchain:** QNX SDP 8.0 (`qcc -V gcc_ntox86_64`)
- **Host UI:** Python 3.10+ (`dashboard.py`; optional `opencv-python` for `.mp4`/`.mov` decoding)

### Build
```bash
make -B                # production build
make -B TEST_HOOKS=1   # demo build: enables the CHAOS vision-crash button (and latch-off for testing)
```

### Deploy & Run on QNX 8.0
```bash
scp -O safety_gate cam_feed vision_worker supervisor telemetry_bridge workcell.conf root@192.168.184.3:/tmp/
ssh root@192.168.184.3 "slay -f safety_gate cam_feed supervisor vision_worker telemetry_bridge 2>/dev/null; mkdir -p /data/home/qnxuser && chmod +x /tmp/safety_gate /tmp/cam_feed /tmp/vision_worker /tmp/supervisor /tmp/telemetry_bridge && mv -f /tmp/safety_gate /tmp/cam_feed /tmp/vision_worker /tmp/supervisor /tmp/telemetry_bridge /tmp/workcell.conf /data/home/qnxuser/ && cd /data/home/qnxuser && on -d ./safety_gate -c workcell.conf >/tmp/gate.log 2>&1 && on -d ./cam_feed -c workcell.conf >/tmp/cam.log 2>&1 && sleep 1 && on -d ./supervisor -c workcell.conf -b ./vision_worker >/tmp/sup.log 2>&1 && on -d ./telemetry_bridge -c workcell.conf >/tmp/tb.log 2>&1"
```

### Run Host Dashboard
```bash
python dashboard.py --qnx-ip 192.168.184.3
```

## Safety behaviour (what the code guarantees)

| Requirement | Where | How |
| :--- | :--- | :--- |
| Fail-safe default | `safety_logic.h` | Gate **boots with OSSD at 0 V** (startup interlock). Output is on only in `SAFE_RUN`. |
| Lost heartbeat / stale frame / bad report / invalid polygon -> stop | `safety_logic.h` | Each is a latched fault; a stale or malformed report is *not* counted as a heartbeat. |
| Latched faults, deliberate reset (ISO 13849-1) | `safety_logic.h` | Latch cannot be disabled at runtime (production build). A reset is **refused** unless vision is alive, the baseline has settled, and the scene has stayed clear for `reset_clear_frames` consecutive frames. A refused reset changes nothing and is reported to the operator. |
| Deterministic stream switches | `cam_feed.c`, `vision_core.h` | `cam_feed` bumps a `stream_epoch` on every swap; vision re-seeds only on an epoch/size change - no heuristics. |
| Stationary intruder is never "learned" | `vision_core.h` | Background is frozen in/near the zone while anything is visible. Every blob is evaluated (not only the largest), over the whole frame. |
| Priority isolation | `config_loader.h` | Startup is refused unless `prio_gate > prio_supervisor > prio_camera > prio_vision > prio_telemetry`; gate and supervisor refuse to run without `SCHED_FIFO`. |
| UI is observational | `dashboard.py` | Telemetry loss shows `COMMS LOST - STATE UNKNOWN`, never a stale "24 V". Control POSTs are origin-checked. Closing the browser has no effect on the gate. |
| Strict config | `config_loader.h` | Unknown keys, bad numbers, missing file, inconsistent timings are fatal. |

Reset/latch states exposed in telemetry: `fault_reason`, `reset_ready`, `last_reset_result`, `clear_streak`, `heartbeat_age_ms`, `cam_age_ms`.

## Verification

```bash
make test           # host-side: gate core, vision core, config loader, end-to-end demo scenario
make check-syntax   # compile-checks the QNX sources against stub QNX headers (not a substitute for a qcc build)
```

`make test` runs the scripted scenario *reset -> safe run -> warning -> halt -> refused reset while the operator is present ->
clean reset -> real-video swap -> vision crash/respawn -> reset -> invalid polygon* against the real gate and vision logic.

## Known limits (be upfront about these in a review)

- **Simulation, not certified hardware.** `ossd_mv` models a relay; a real Cat. 4 system needs redundant, monitored OSSD outputs
  and a hardware watchdog that also covers `safety_gate` itself dying.
- **Single-channel vision.** One camera, one CV channel, no diagnostic coverage. Cat. 4 requires redundancy/diversity.
- **Baseline must be captured with the zone empty.** An object already inside the zone when a stream starts becomes the baseline.
- **Control channel is not authenticated.** `control_allow_ip` is a source-address filter only; use an isolated safety network or
  add TLS/HMAC before deployment. `/dev/safety_gate` is owner-only (0600) and shared memory is 0600.
- **Clip rewinds do not bump the epoch** (only stream swaps do), so a looping clip that jumps scenes can legitimately trip the gate.
- The QNX binaries could not be built or run in the authoring environment; only the portable cores are executed by the tests.
