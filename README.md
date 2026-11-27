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

### Build & deploy (one command)
```bash
./deploy.sh              # demo build: enables the "Inject Vision Crash" button and latch-off for testing
./deploy.sh production   # production build: no test hooks, latch cannot be disabled
```
`deploy.sh` compiles all five binaries with `qcc`, copies them and `workcell.conf` to the target, and starts them in the
right order (gate -> camera -> supervisor/vision -> telemetry bridge). Set `QNX_HOST`, `QNX_TARGET` and `QNX_IP` if your
SDK/target differ. A plain `make` (with `qcc` on the path) builds the same binaries.

### Run the host dashboard
```bash
python dashboard.py --qnx-ip 192.168.184.3                      # full control, localhost only
python dashboard.py --qnx-ip 192.168.184.3 --read-only          # view-only (safe to expose)
python dashboard.py --qnx-ip 192.168.184.3 --read-only --allow-host my.tunnel.example   # behind ngrok/cloudflared/nginx
```
The dashboard polls the QNX node once and serves every browser from a cache, so opening it in many tabs (or through a
slow tunnel) does not add load to the safety node. **Never expose the control dashboard publicly** - it has no login.
A judge cannot reach `localhost`; give them a `--read-only` tunnel link and a recorded run as a backup.

### Demo run (about 3 minutes)
1. Start the stack (`./deploy.sh`), start the dashboard. The gate boots **stopped (0 V)**.
2. **Start Simulation.** Status shows *learning background*; once the zone has been clear for ~1 s the reset is accepted -> 24 V.
3. The operator approaches: proximity warning, then **halt (0 V) before contact**.
4. **Reset while the operator is still in the zone:** refused, with the reason in the event log.
5. The operator leaves; the machine stays latched until you press Reset, which is then accepted first time.
6. Load a real video: the machine **stops while the new background is learned**; press Reset when ready.
7. **Inject Vision Crash:** 0 V within ~150 ms, the supervisor respawns vision, output stays off until Reset.

## Safety behaviour (what the code guarantees)

| Requirement | Where | How |
| :--- | :--- | :--- |
| Fail-safe default | `safety_logic.h` | Gate **boots with OSSD at 0 V** (startup interlock). Output is on only in `SAFE_RUN`. |
| Lost heartbeat / stale frame / bad report / invalid polygon -> stop | `safety_logic.h` | Each is a latched fault; a stale or malformed report is *not* counted as a heartbeat. |
| Latched faults, deliberate reset (ISO 13849-1) | `safety_logic.h` | Latch cannot be disabled at runtime (production build). A reset is **refused** unless vision is alive, the baseline has settled, and the scene has stayed clear for `reset_clear_frames` consecutive frames. A refused reset changes nothing and is reported to the operator. |
| Deterministic stream switches | `cam_feed.c`, `vision_core.h`, `safety_logic.h` | `cam_feed` bumps a `stream_epoch` on every swap; vision re-seeds only on an epoch/size change and then **teaches in**: fast background learning until the zone has been quiet for `bg_learn_quiet_frames`. Until then `baseline_valid=0` and the gate keeps the output OFF (reason *learning background*) and refuses reset. A vision restart does the same, so a worker that respawns while the operator is in the zone cannot leave a phantom tracker. |
| Stationary intruder is never "learned" | `vision_core.h` | Background is frozen in/near the zone while anything is visible. Every blob is evaluated (not only the largest), over the whole frame. |
| Priority isolation | `config_loader.h` | Startup is refused unless `prio_gate > prio_supervisor > prio_camera > prio_vision > prio_telemetry`; gate and supervisor refuse to run without `SCHED_FIFO`. |
| UI is observational | `dashboard.py` | A missed poll shows `STALE DATA`; a 1.2 s gap shows `COMMS LOST - STATE UNKNOWN`, never a stale "24 V". Control POSTs are origin-checked; `--read-only` disables them. Viewer count does not change the load on the safety node. Closing the browser has no effect on the gate. |
| Strict config | `config_loader.h` | Unknown keys, bad numbers, missing file, inconsistent timings are fatal. |

Reset/latch states exposed in telemetry: `fault_reason`, `reset_ready`, `last_reset_result`, `clear_streak`, `heartbeat_age_ms`, `cam_age_ms`.

## Verification

```bash
make test           # host-side: gate core, vision core, config loader, dashboard, end-to-end demo scenario
make check-syntax   # compile-checks the QNX sources against stub QNX headers (not a substitute for a qcc build)
```

`make test` runs the scripted scenario *learn -> reset -> safe run -> warning -> halt -> refused reset while the operator is present ->
clean reset -> real-video swap (stops, relearns) -> vision crash/respawn -> reset -> invalid polygon* against the real gate and vision logic.

## Known limits (be upfront about these in a review)

- **Simulation, not certified hardware.** `ossd_mv` models a relay; a real Cat. 4 system needs redundant, monitored OSSD outputs
  and a hardware watchdog that also covers `safety_gate` itself dying.
- **Single-channel vision.** One camera, one CV channel, no diagnostic coverage. Cat. 4 requires redundancy/diversity.
- **Teach-in needs an empty zone.** Moving objects are learned away, but something that stays *stationary inside the zone for the entire teach-in* is learned as background. The operator's deliberate reset is the confirmation that the zone was clear.
- **Control channel is not authenticated.** `control_allow_ip` is a source-address filter only; use an isolated safety network or
  add TLS/HMAC before deployment. `/dev/safety_gate` is owner-only (0600) and shared memory is 0600.
- **Clip rewinds do not bump the epoch** (only stream swaps do), so a looping clip that jumps scenes can legitimately trip the gate.
- The QNX binaries could not be built or run in the authoring environment; only the portable cores are executed by the tests.
