# Workcell Guard — QNX 8.0 Type-4 Virtual Optical Light Curtain

A deterministic real-time safety controller built on **QNX Neutrino RTOS 8.0** (`x86_64`) that provides a programmable 2D polygon optical safety boundary, sub-millisecond IPC cutoff (`24V -> 0V` OSSD relay simulation), a `5 ms` kernel pulse watchdog (`150 ms` heartbeat timeout), and `250 ms` automatic process recovery.

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
make -B
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

## Updates Made During This Submission Period
- Added dynamic 3-to-8 vertex polygon zones (`DCMD_SAFETY_SET_ZONE`) and 2D segment/clearance checks in `vision_worker.c`.
- Added ISO 13849-1 fault latching (`DCMD_SAFETY_SET_LATCH`) and manual interlock reset (`DCMD_SAFETY_RESET`) in `safety_gate.c`.
- Added shared-memory video clip hot-swapping (port `9877`) and frame streaming (port `9878`) in `cam_feed.c` and `telemetry_bridge.c`.
- Added interactive polygon editing, a 40-ton servo stamping press digital twin, and one-click `vision_worker` crash injection (`CHAOS`) in `dashboard.py`.
