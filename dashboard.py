import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import shutil
import socket
import struct
import subprocess
import tempfile
import sys
import threading
import time
from pathlib import Path
from urllib.parse import unquote

# v2 telemetry: original fields + fault/reset/heartbeat/camera fields (must match telemetry_bridge.c)
PKT_FMT = "<IiiiIIIQQfHHHHHH I 8f 8f i i i I I I I I i"
PKT_SIZE = struct.calcsize(PKT_FMT)
TELEMETRY_MAGIC = 0x514E5838
FRAME_MAGIC = 0x46524D38
CAM_INGEST_MAGIC = 0x43414D38
MAX_UPLOAD_BYTES = 200 * 1024 * 1024
FRAME_MAX_AGE_S = 2.0

RESET_REASONS = {
    0: "accepted",
    1: "no fresh vision heartbeat",
    2: "hazard still present in the zone",
    3: "scene not clear long enough / background baseline still settling",
    4: "zone polygon invalid",
}


def poll_telemetry(ip, port, timeout=0.25):
    """One status poll. Returns a dict, or None if the node did not answer with a valid packet.
    The UI must treat None as 'state unknown, assume stopped' - never as 'unchanged'."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(b"POLL", (ip, port))
        data, _ = sock.recvfrom(4096)
    except OSError:
        return None
    finally:
        sock.close()
    if len(data) < PKT_SIZE:
        return None
    f = struct.unpack(PKT_FMT, data[:PKT_SIZE])
    if f[0] != TELEMETRY_MAGIC:
        return None
    npts = f[16]
    return {
        "ok": True,
        "state": f[1], "latched": f[2], "latch_enabled": f[3], "trip_count": f[4], "ossd_mv": f[5],
        "frame_id": f[6], "latency_us": f[7], "last_trip_us": f[8], "dist_px": f[9],
        "bbox": list(f[10:14]), "frame_w": f[14], "frame_h": f[15],
        "pts": [{"x": f[17 + i], "y": f[25 + i]} for i in range(min(npts, 8))],
        "fault_reason": f[33], "reset_ready": f[34], "last_reset_result": f[35],
        "reset_refusals": f[36], "clear_streak": f[37], "stream_epoch": f[38],
        "heartbeat_age_ms": f[39], "cam_age_ms": f[40], "test_hooks": f[41],
    }


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Workcell Guard — QNX 8.0 Type-4 Optical Safety Controller</title>
<style>
  :root {
    --bg: #09090b;
    --surface: #111113;
    --surface-2: #18181b;
    --border: #27272a;
    --border-strong: #3f3f46;
    --text: #fafafa;
    --text-muted: #a1a1aa;
    --text-dim: #71717a;
    --ok: #10b981;
    --ok-bg: rgba(16, 185, 129, 0.12);
    --warn: #f59e0b;
    --warn-bg: rgba(245, 158, 11, 0.12);
    --danger: #ef4444;
    --danger-bg: rgba(239, 68, 68, 0.14);
    --mono: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace;
    --sans: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  }

  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--text);
    font-family: var(--sans); font-size: 13px; line-height: 1.5;
    -webkit-font-smoothing: antialiased;
  }

  .topbar {
    position: sticky; top: 0; z-index: 20;
    background: rgba(9, 9, 11, 0.9); backdrop-filter: blur(12px);
    border-bottom: 1px solid var(--border);
  }
  .topbar-inner {
    max-width: 1380px; margin: 0 auto; padding: 12px 24px;
    display: flex; justify-content: space-between; align-items: center; gap: 16px;
  }
  .brand { display: flex; align-items: center; gap: 12px; }
  .brand-mark {
    width: 28px; height: 28px; border-radius: 6px;
    background: var(--surface-2); border: 1px solid var(--border-strong);
    display: grid; place-items: center; font-family: var(--mono);
    font-weight: 700; font-size: 12px; color: var(--ok);
  }
  .brand h1 { margin: 0; font-size: 15px; font-weight: 600; letter-spacing: -0.01em; }
  .brand span { font-size: 11px; color: var(--text-dim); font-family: var(--mono); }

  .top-actions { display: flex; align-items: center; gap: 10px; }
  .status-pill {
    display: inline-flex; align-items: center; gap: 8px;
    padding: 6px 14px; border-radius: 999px;
    font-family: var(--mono); font-size: 12px; font-weight: 600;
    border: 1px solid var(--border); background: var(--surface); color: var(--text-muted);
  }
  .status-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--text-dim); }
  .status-pill.SAFE_RUN { background: var(--ok-bg); border-color: rgba(16,185,129,0.4); color: var(--ok); }
  .status-pill.SAFE_RUN .status-dot { background: var(--ok); box-shadow: 0 0 10px var(--ok); }
  .status-pill.WARN_SLS { background: var(--warn-bg); border-color: rgba(245,158,11,0.45); color: var(--warn); }
  .status-pill.WARN_SLS .status-dot { background: var(--warn); box-shadow: 0 0 10px var(--warn); }
  .status-pill.INTRUSION_HALT { background: var(--danger-bg); border-color: rgba(239,68,68,0.5); color: var(--danger); }
  .status-pill.INTRUSION_HALT .status-dot { background: var(--danger); box-shadow: 0 0 12px var(--danger); }
  .status-pill.WATCHDOG_FAULT { background: var(--danger-bg); border-color: rgba(245,158,11,0.5); color: var(--warn); }
  .status-pill.WATCHDOG_FAULT .status-dot { background: var(--warn); }

  .container {
    max-width: 1380px; margin: 0 auto; padding: 20px 24px;
    display: flex; flex-direction: column; gap: 18px;
  }

  .kpi-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 12px; }
  .kpi {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 12px 14px;
  }
  .kpi-label { font-size: 11px; color: var(--text-muted); margin-bottom: 3px; }
  .kpi-value {
    font-family: var(--mono); font-size: 18px; font-weight: 600;
    font-variant-numeric: tabular-nums; color: var(--text);
  }
  .kpi-sub { font-size: 11px; color: var(--text-dim); font-family: var(--mono); margin-top: 2px; }

  .workspace {
    display: grid; grid-template-columns: 1.45fr 1fr; gap: 18px; align-items: start;
  }
  .panel {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 10px; overflow: hidden;
  }
  .panel-header {
    padding: 10px 14px; border-bottom: 1px solid var(--border);
    display: flex; justify-content: space-between; align-items: center; gap: 8px;
  }
  .panel-title { font-size: 13px; font-weight: 600; color: var(--text); }
  .panel-meta { font-family: var(--mono); font-size: 11px; color: var(--text-dim); }
  .panel-body { padding: 14px; }

  .viewport-wrap {
    position: relative; background: #050506;
    border: 1px solid var(--border); border-radius: 6px;
    overflow: hidden; aspect-ratio: 4 / 3; cursor: crosshair;
  }
  canvas { width: 100%; height: 100%; display: block; }

  .toolbar-strip {
    display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center;
    gap: 8px; margin-top: 10px;
  }
  .legend { display: flex; flex-wrap: wrap; gap: 14px; font-size: 11px; color: var(--text-muted); }
  .legend-item { display: inline-flex; align-items: center; gap: 6px; }
  .swatch { width: 9px; height: 9px; border-radius: 2px; }

  .right-stack { display: flex; flex-direction: column; gap: 16px; }

  .twin-wrap {
    background: #070709; border: 1px solid var(--border); border-radius: 6px; overflow: hidden;
  }
  .relay-grid {
    display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 10px;
  }
  .relay-box {
    background: var(--surface-2); border: 1px solid var(--border);
    border-radius: 6px; padding: 8px 10px; font-family: var(--mono);
  }
  .relay-lbl { font-size: 10px; color: var(--text-dim); }
  .relay-val { font-size: 13px; font-weight: 600; margin-top: 2px; }

  .btn-row { display: flex; flex-wrap: wrap; gap: 8px; }
  button {
    flex: 1; padding: 7px 11px; border-radius: 6px;
    font-family: var(--sans); font-size: 12px; font-weight: 500;
    cursor: pointer; border: 1px solid var(--border-strong);
    background: var(--surface-2); color: var(--text);
    transition: background 0.15s, border-color 0.15s;
  }
  button:hover { background: #27272a; }
  button.primary { background: var(--text); color: var(--bg); border-color: var(--text); font-weight: 600; }
  button.primary:hover { background: #e4e4e7; }
  button.danger-btn { border-color: rgba(239,68,68,0.45); color: #fca5a5; }
  button.danger-btn:hover { background: rgba(239,68,68,0.16); }

  .dropzone {
    border: 1px dashed var(--border-strong); background: var(--surface-2);
    border-radius: 8px; padding: 12px; text-align: center; cursor: pointer;
  }
  .dropzone:hover, .dropzone.dragover { border-color: var(--text-muted); background: #1f1f23; }
  .dropzone-title { font-size: 12px; font-weight: 500; color: var(--text); }
  .dropzone-sub { font-size: 11px; color: var(--text-dim); margin-top: 2px; }

  .console-box {
    background: #050506; font-family: var(--mono); font-size: 12px;
    line-height: 1.6; height: 220px; overflow-y: auto; padding: 10px 14px;
  }
  .log-line { display: flex; gap: 10px; border-bottom: 1px solid rgba(39,39,42,0.35); padding: 2px 0; }
  .log-ts { color: var(--text-dim); flex-shrink: 0; }
  .log-tag { flex-shrink: 0; font-weight: 600; width: 78px; }
  .log-ok { color: var(--ok); }
  .log-warn { color: var(--warn); }
  .log-danger { color: var(--danger); }

  .filter-btn {
    flex: 0 0 auto; padding: 3px 9px; font-size: 11px; border-radius: 5px;
    background: transparent; border: 1px solid var(--border); color: var(--text-muted);
  }
  .filter-btn.active { background: var(--surface-2); border-color: var(--border-strong); color: var(--text); }

  .topo-grid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 10px; }
  .topo-node {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 10px 12px;
  }
  .topo-prio { font-family: var(--mono); font-size: 10px; color: var(--text-dim); }
  .topo-name { font-family: var(--mono); font-size: 12px; font-weight: 600; color: var(--text); margin: 2px 0; }
  .topo-desc { font-size: 11px; color: var(--text-muted); }

  @media (max-width: 1024px) {
    .kpi-grid { grid-template-columns: repeat(2, 1fr); }
    .workspace { grid-template-columns: 1fr; }
    .topo-grid { grid-template-columns: repeat(2, 1fr); }
  }
</style>
</head>
<body>

<header class="topbar">
  <div class="topbar-inner">
    <div class="brand">
      <div class="brand-mark">Q8</div>
      <div>
        <h1>Workcell Guard &bull; Type-4 Virtual Light Curtain</h1>
        <span>QNX 8.0 Neutrino RTOS &bull; /dev/safety_gate (ISO 13849-1 Cat. 4)</span>
      </div>
    </div>
    <div class="top-actions">
      <button class="primary" style="flex:0 0 auto; padding:6px 14px;" onclick="resetGate()">Reset Interlock</button>
      <div id="stateBadge" class="status-pill">
        <span class="status-dot"></span>
        <span id="stateText">CONNECTING...</span>
      </div>
    </div>
  </div>
</header>

<main class="container">
  <section class="kpi-grid">
    <div class="kpi">
      <div class="kpi-label">OSSD Safety Relay Output</div>
      <div class="kpi-value" id="mOssd" style="color:var(--ok);">24.0 V DC</div>
      <div class="kpi-sub" id="mBrake">Mechanical Brake: RELEASED</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Deterministic IPC Latency</div>
      <div class="kpi-value" id="mLatency">-- us</div>
      <div class="kpi-sub" id="mTripLat">Last E-Stop trip: -- us</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Laser Curtain Clearance</div>
      <div class="kpi-value" id="mDist">-- px</div>
      <div class="kpi-sub">SLS Warn &le; 45 px &bull; Cutoff = 0 px</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Processed Frame / Trips</div>
      <div class="kpi-value" id="mFrame">#0</div>
      <div class="kpi-sub" id="mTrips">Safety trips: 0</div>
    </div>
    <div class="kpi">
      <div class="kpi-label">Active Video Stream</div>
      <div class="kpi-value" id="mSource" style="font-size:14px; padding-top:3px;">Synthetic Stream</div>
      <div class="kpi-sub">320&times;240 HD /dev/shmem</div>
    </div>
  </section>

  <section class="workspace">
    <div class="panel">
      <div class="panel-header">
        <span class="panel-title">Optical Light Curtain &amp; Custom Polygon Boundary</span>
        <span class="panel-meta">Drag vertices to reshape &bull; Double-click edge to add point</span>
      </div>
      <div class="panel-body">
        <div class="viewport-wrap">
          <canvas id="camCanvas" width="960" height="720"></canvas>
        </div>
        <div class="toolbar-strip">
          <div class="legend">
            <span class="legend-item"><span class="swatch" style="background:#10b981;"></span>Clear Laser Curtain (24V)</span>
            <span class="legend-item"><span class="swatch" style="background:#f59e0b;"></span>Proximity SLS Throttle</span>
            <span class="legend-item"><span class="swatch" style="background:#ef4444;"></span>Intrusion Cutoff (0V)</span>
          </div>
          <div class="btn-row" style="flex:0 0 auto;">
            <button style="padding:4px 9px; font-size:11px;" onclick="presetPoly('right')">Right Hazard</button>
            <button style="padding:4px 9px; font-size:11px;" onclick="presetPoly('trap')">Angled Table</button>
            <button style="padding:4px 9px; font-size:11px;" onclick="presetPoly('lshape')">L-Guard</button>
          </div>
        </div>
      </div>
    </div>

    <div class="right-stack">
      <div class="panel">
        <div class="panel-header">
          <span class="panel-title">Protected Actuator: 40-Ton Servo Stamping Press</span>
          <span class="panel-meta" id="pressMode">NOMINAL 100%</span>
        </div>
        <div class="panel-body">
          <div class="twin-wrap">
            <canvas id="pressCanvas" width="480" height="190"></canvas>
          </div>
          <div class="relay-grid">
            <div class="relay-box">
              <div class="relay-lbl">OSSD CH1 / CH2</div>
              <div class="relay-val" id="rOssd" style="color:var(--ok);">24.0V / 24.0V</div>
            </div>
            <div class="relay-box">
              <div class="relay-lbl">SERVO DRIVE</div>
              <div class="relay-val" id="rRpm" style="color:var(--ok);">120 SPM</div>
            </div>
            <div class="relay-box">
              <div class="relay-lbl">ISO-13849 LATCH</div>
              <div class="relay-val" id="rLatch">ARMED (ON)</div>
            </div>
          </div>
          <div class="btn-row" style="margin-top:10px;">
            <button class="primary" onclick="resetGate()">Reset E-Stop Interlock</button>
            <button id="latchBtn" onclick="toggleLatch()">Latch Mode: ON</button>
            <button class="danger-btn" onclick="triggerChaos()">Inject Vision Crash</button>
          </div>
        </div>
      </div>

      <div class="panel">
        <div class="panel-header">
          <span class="panel-title">Workcell Camera Feed Source</span>
          <span class="panel-meta">TCP :9877 &rarr; /dev/shmem</span>
        </div>
        <div class="panel-body">
          <input type="file" id="videoInput" accept="video/*,.bin" style="display:none" onchange="uploadVideo(this.files[0])">
          <div class="dropzone" id="dropzone" onclick="document.getElementById('videoInput').click()">
            <div class="dropzone-title" id="uploadTitle">Click or drop a test video (.mp4, .mov)</div>
            <div class="dropzone-sub">Streams crisp 320&times;240 frames directly into QNX shared memory</div>
          </div>
          <div class="btn-row" style="margin-top:8px;">
            <button onclick="useSynthetic()">Use Synthetic Workcell Feed</button>

<div style="display:flex;gap:8px;margin-top:8px;">
  <button type="button" onclick="startSimFeed()" style="flex:1;">Start Simulation</button>
  <button type="button" onclick="loadSampleRealVideo()" style="flex:1;">Load Sample Real Video</button>
</div>

          </div>
        </div>
      </div>
    </div>
  </section>

  <section class="panel">
    <div class="panel-header">
      <div style="display:flex; align-items:center; gap:10px;">
        <span class="panel-title">Live QNX Kernel &amp; Safety Gate Event Stream</span>
        <span class="panel-meta" id="logCount">0 events</span>
      </div>
      <div style="display:flex; align-items:center; gap:6px;">
        <button class="filter-btn active" onclick="setFilter('ALL', this)">All</button>
        <button class="filter-btn" onclick="setFilter('OK', this)">OK (Green)</button>
        <button class="filter-btn" onclick="setFilter('WARN', this)">Warn (Amber)</button>
        <button class="filter-btn" onclick="setFilter('DANGER', this)">Halt (Red)</button>
        <button class="filter-btn" onclick="clearLogs()">Clear</button>
      </div>
    </div>
    <div class="console-box" id="consoleBox"></div>
  </section>

  <section class="topo-grid">
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 25</div>
      <div class="topo-name">/dev/safety_gate</div>
      <div class="topo-desc">24V OSSD relay manager &amp; 5ms watchdog</div>
    </div>
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 20</div>
      <div class="topo-name">supervisor</div>
      <div class="topo-desc">Self-healing process monitor (250ms respawn)</div>
    </div>
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 15</div>
      <div class="topo-name">cam_feed</div>
      <div class="topo-desc">30 FPS zero-copy /dev/shmem producer</div>
    </div>
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 10</div>
      <div class="topo-name">vision_worker</div>
      <div class="topo-desc">2D polygon clearance &amp; cell-density CV</div>
    </div>
    <div class="topo-node">
      <div class="topo-prio">SCHED_FIFO &bull; PRIO 5</div>
      <div class="topo-name">telemetry_bridge</div>
      <div class="topo-desc">devctl gateway + HD TCP video server</div>
    </div>
  </section>
</main>

<script>
const camCanvas = document.getElementById('camCanvas');
const ctx = camCanvas.getContext('2d');
const offCanvas = document.createElement('canvas');
const offCtx = offCanvas.getContext('2d');

const pressCanvas = document.getElementById('pressCanvas');
const pctx = pressCanvas.getContext('2d');
const consoleBox = document.getElementById('consoleBox');

const WARN_CLEARANCE_PX = 45.0;
let logFilter = 'ALL';
let logs = [];
let lastFrameLogged = -1;
let lastSeverity = '';
let lastHeartbeatMs = 0;
let connected = false;

let polyPts = [
  { x: 0.50, y: 0.20 },
  { x: 0.92, y: 0.20 },
  { x: 0.92, y: 0.85 },
  { x: 0.50, y: 0.85 }
];
let dragIdx = -1;
let hoverIdx = -1;
let isDragging = false;

let telem = null;
let latestFrameImg = null;
let pressPhase = 0;
let scanPhase = 0;

function tsNow() {
  const d = new Date();
  return d.toTimeString().split(' ')[0] + '.' + String(d.getMilliseconds()).padStart(3, '0');
}

function pushLog(level, tag, msg) {
  logs.push({ ts: tsNow(), level, tag, msg });
  if (logs.length > 200) logs.shift();
  document.getElementById('logCount').innerText = logs.length + ' events';
  if (logFilter !== 'ALL' && logFilter !== level) return;
  appendLogDom(logs[logs.length - 1]);
}

function appendLogDom(entry) {
  const cls = entry.level === 'OK' ? 'log-ok' : (entry.level === 'WARN' ? 'log-warn' : 'log-danger');
  const row = document.createElement('div');
  row.className = 'log-line ' + cls;
  row.innerHTML = `<span class="log-ts">${entry.ts}</span><span class="log-tag">[${entry.tag}]</span><span>${entry.msg}</span>`;
  consoleBox.appendChild(row);
  while (consoleBox.children.length > 200) consoleBox.removeChild(consoleBox.firstChild);
  consoleBox.scrollTop = consoleBox.scrollHeight;
}

function setFilter(f, btn) {
  logFilter = f;
  document.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
  consoleBox.innerHTML = '';
  logs.filter(l => f === 'ALL' || l.level === f).forEach(appendLogDom);
}

function clearLogs() {
  logs = [];
  consoleBox.innerHTML = '';
  document.getElementById('logCount').innerText = '0 events';
}

function getNormCoords(e) {
  const rect = camCanvas.getBoundingClientRect();
  return {
    x: Math.max(0.02, Math.min(0.98, (e.clientX - rect.left) / rect.width)),
    y: Math.max(0.02, Math.min(0.98, (e.clientY - rect.top) / rect.height))
  };
}

camCanvas.addEventListener('mousedown', e => {
  const p = getNormCoords(e);
  for (let i = 0; i < polyPts.length; i++) {
    if (Math.hypot(polyPts[i].x - p.x, polyPts[i].y - p.y) < 0.035) {
      dragIdx = i;
      isDragging = true;
      return;
    }
  }
});

camCanvas.addEventListener('mousemove', e => {
  const p = getNormCoords(e);
  if (isDragging && dragIdx >= 0) {
    polyPts[dragIdx] = p;
    return;
  }
  hoverIdx = -1;
  for (let i = 0; i < polyPts.length; i++) {
    if (Math.hypot(polyPts[i].x - p.x, polyPts[i].y - p.y) < 0.035) {
      hoverIdx = i;
      break;
    }
  }
  camCanvas.style.cursor = hoverIdx >= 0 ? 'grab' : 'crosshair';
});

window.addEventListener('mouseup', () => {
  if (isDragging) {
    isDragging = false;
    dragIdx = -1;
    sendPolygonToQnx();
  }
});

camCanvas.addEventListener('dblclick', e => {
  if (polyPts.length >= 8) return;
  const p = getNormCoords(e);
  let bestEdge = 0, bestD = 999;
  for (let i = 0; i < polyPts.length; i++) {
    const a = polyPts[i], b = polyPts[(i + 1) % polyPts.length];
    const mx = (a.x + b.x) * 0.5, my = (a.y + b.y) * 0.5;
    const d = Math.hypot(mx - p.x, my - p.y);
    if (d < bestD) { bestD = d; bestEdge = i + 1; }
  }
  polyPts.splice(bestEdge, 0, p);
  sendPolygonToQnx();
});

function presetPoly(kind) {
  if (kind === 'right') {
    polyPts = [{x:0.50,y:0.20},{x:0.92,y:0.20},{x:0.92,y:0.85},{x:0.50,y:0.85}];
  } else if (kind === 'trap') {
    polyPts = [{x:0.54,y:0.18},{x:0.90,y:0.28},{x:0.94,y:0.86},{x:0.46,y:0.78}];
  } else if (kind === 'lshape') {
    polyPts = [{x:0.55,y:0.15},{x:0.92,y:0.15},{x:0.92,y:0.88},{x:0.42,y:0.88},{x:0.42,y:0.55},{x:0.55,y:0.55}];
  }
  sendPolygonToQnx();
}

async function sendPolygonToQnx() {
  await fetch('/api/zone', { method: 'POST', body: JSON.stringify({ pts: polyPts }) });
  pushLog('WARN', 'DEVCTL', `DCMD_SAFETY_SET_ZONE pushed ${polyPts.length}-vertex custom polygon to /dev/safety_gate`);
}

let latchStateLocal = 1;
let actionInFlight = false;

async function resetGate() {
  if (actionInFlight) return;
  actionInFlight = true;
  try {
    const res = await fetch('/api/reset', { method: 'POST' });
    const out = await res.json();
    await pollTelemetry();
    if (out.accepted) {
      pushLog('OK', 'DEVCTL', 'DCMD_SAFETY_RESET accepted -> OSSD 24.0V DC re-energized (SAFE_RUN)');
    } else {
      pushLog('DANGER', 'DEVCTL', 'RESET REFUSED: ' + (out.reason || 'unknown') + ' -> OSSD stays 0V');
    }
  } finally {
    actionInFlight = false;
  }
}

async function toggleLatch() {
  if (actionInFlight) return;
  actionInFlight = true;
  try {
    if (!(telem && telem.test_hooks)) {
      pushLog('WARN', 'DEVCTL', 'Latch mode is locked ON (ISO 13849-1 restart interlock); it can only be changed in a test-hooks build');
      return;
    }
    const cur = (telem && typeof telem.latch_enabled === 'number') ? telem.latch_enabled : latchStateLocal;
    const next = cur ? 0 : 1;
    latchStateLocal = next;
    document.getElementById('latchBtn').innerText = 'Latch Mode: ' + (next ? 'ON' : 'OFF');
    await fetch('/api/latch', { method: 'POST', body: JSON.stringify({ enabled: next }) });
    await pollTelemetry();
    pushLog('WARN', 'DEVCTL', `DCMD_SAFETY_SET_LATCH -> ISO 13849-1 Latch Mode ${next ? 'ENABLED' : 'DISABLED'}`);
  } finally {
    actionInFlight = false;
  }
}

async function triggerChaos() {
  if (actionInFlight) return;
  actionInFlight = true;
  try {
    pushLog('DANGER', 'CHAOS', 'Injecting SIGKILL into vision_worker PID -> testing QNX 150ms watchdog & 250ms supervisor...');
    await fetch('/api/chaos', { method: 'POST' });
    setTimeout(pollTelemetry, 60);
  } finally {
    actionInFlight = false;
  }
}

const dz = document.getElementById('dropzone');
dz.addEventListener('dragover', e => { e.preventDefault(); dz.classList.add('dragover'); });
dz.addEventListener('dragleave', () => dz.classList.remove('dragover'));
dz.addEventListener('drop', e => {
  e.preventDefault(); dz.classList.remove('dragover');
  if (e.dataTransfer.files.length) uploadVideo(e.dataTransfer.files[0]);
});

async function uploadVideo(file) {
  if (!file) return;
  const title = document.getElementById('uploadTitle');
  title.innerText = 'Streaming ' + file.name + ' to QNX...';
  pushLog('WARN', 'INGEST', `Decoding "${file.name}" to 320x240 grayscale -> /dev/shmem/workcell_cam`);
  try {
    const res = await fetch('/api/upload-video', {
      method: 'POST',
      headers: { 'X-Filename': encodeURIComponent(file.name) },
      body: file
    });
    const out = await res.json();
    if (out.ok) {
      title.innerText = 'Active: ' + file.name + ' (' + out.frames + ' frames)';
      document.getElementById('mSource').innerText = file.name;
      pushLog('OK', 'CAM_FEED', `Hot-swapped /dev/shmem/workcell_cam -> ${file.name} (${out.frames} frames @ 30 FPS)`);
    } else {
      title.innerText = 'Upload failed — click to retry';
      pushLog('DANGER', 'INGEST', out.error || 'Video decode error');
    }
  } catch (err) {
    pushLog('DANGER', 'INGEST', 'Upload error: ' + err);
  }
}

async function useSynthetic() {
  await fetch('/api/use-synthetic', { method: 'POST' });
  document.getElementById('uploadTitle').innerText = 'Click or drop a test video (.mp4, .mov)';
  document.getElementById('mSource').innerText = 'Synthetic Stream';
  pushLog('WARN', 'CAM_FEED', 'Switched /dev/shmem/workcell_cam back to synthetic workcell stream');
}

const FAULT_NAMES = {
  0: 'none', 1: 'power-up interlock (reset required)', 2: 'intrusion', 3: 'vision heartbeat lost',
  4: 'stale camera frame', 5: 'invalid vision report', 6: 'invalid zone polygon', 7: 'vision/gate zone mismatch'
};
const RESET_REASON = {
  0: 'ok', 1: 'no fresh vision heartbeat', 2: 'hazard still present', 3: 'settling (scene must stay clear)', 4: 'invalid zone'
};

// Telemetry is gone: never keep showing the last good "24 V / running". Unknown means STOPPED.
function showLinkLost() {
  telem = null;
  if (connected) {
    connected = false;
    pushLog('DANGER', 'LINK', 'Telemetry lost - machine state UNKNOWN, assume STOPPED (the gate fails safe independently of this UI)');
  }
  document.getElementById('stateBadge').className = 'status-pill WATCHDOG_FAULT';
  document.getElementById('stateText').innerText = 'COMMS LOST — STATE UNKNOWN';
  const o = document.getElementById('mOssd'); o.innerText = '-- V'; o.style.color = 'var(--danger)';
  document.getElementById('mBrake').innerText = 'Link lost — assume brake ENGAGED';
  const r = document.getElementById('rOssd'); r.innerText = '-- / --'; r.style.color = 'var(--danger)';
  document.getElementById('rRpm').innerText = 'UNKNOWN';
  document.getElementById('pressMode').innerText = 'LINK LOST';
}

async function pollTelemetry() {
  try {
    const res = await fetch('/api/telemetry');
    const d = await res.json();
    if (!d.ok) { showLinkLost(); return; }
    telem = d;
    if (!connected) {
      connected = true;
      pushLog('OK', 'LINK', 'Connected to QNX 8.0 /dev/safety_gate (OSSD 24V safety relay online)');
    }

    if (!isDragging && d.pts && d.pts.length >= 3) {
      polyPts = d.pts;
    }

    const isProx = (d.state === 0 && d.bbox[2] > 0 && d.dist_px <= WARN_CLEARANCE_PX);
    const badge = document.getElementById('stateBadge');
    badge.className = 'status-pill ' + (
      d.state === 2 ? 'WATCHDOG_FAULT' :
      d.state === 1 ? 'INTRUSION_HALT' :
      isProx ? 'WARN_SLS' : 'SAFE_RUN'
    );
    document.getElementById('stateText').innerText =
      d.state === 2 ? ('FAULT — RELAY CUTOFF (' + (FAULT_NAMES[d.fault_reason] || 'unknown') + ')') :
      d.state === 1 ? (d.latched ? 'INTRUSION_HALT — LATCHED E-STOP' : 'INTRUSION_HALT — E-STOP') :
      isProx ? 'PROXIMITY — SLS THROTTLED' : 'SAFE_RUN — CURTAIN CLEAR';

    const voltStr = (d.ossd_mv / 1000).toFixed(1) + ' V DC';
    const mOssd = document.getElementById('mOssd');
    mOssd.innerText = voltStr;
    mOssd.style.color = d.ossd_mv > 0 ? 'var(--ok)' : 'var(--danger)';
    document.getElementById('mBrake').innerText = d.ossd_mv > 0 ? 'Mechanical Brake: RELEASED' : 'Mechanical Brake: ENGAGED (LOCKED)';

    document.getElementById('mLatency').innerText = d.latency_us + ' us';
    document.getElementById('mTripLat').innerText = d.last_trip_us > 0 ? `Last E-Stop trip: ${d.last_trip_us} us` : 'Last E-Stop trip: none';
    document.getElementById('mDist').innerText = d.dist_px >= 900 ? 'Clear' : d.dist_px.toFixed(1) + ' px';
    document.getElementById('mFrame').innerText = '#' + d.frame_id;
    document.getElementById('mTrips').innerText = 'Safety trips: ' + d.trip_count + (d.cam_age_ms > 500 ? ' | CAMERA STALLED ' + d.cam_age_ms + ' ms' : '');

    document.getElementById('rOssd').innerText = d.ossd_mv > 0 ? '24.0V / 24.0V' : '0.0V / 0.0V';
    document.getElementById('rOssd').style.color = d.ossd_mv > 0 ? 'var(--ok)' : 'var(--danger)';
    document.getElementById('rRpm').innerText = d.ossd_mv === 0 ? '0 SPM (HALT)' : (isProx ? '30 SPM (SLS)' : '120 SPM');
    document.getElementById('rRpm').style.color = d.ossd_mv === 0 ? 'var(--danger)' : (isProx ? 'var(--warn)' : 'var(--ok)');
    document.getElementById('rLatch').innerText = d.latch_enabled ? (d.latched ? (d.reset_ready ? 'TRIPPED — RESET READY' : 'TRIPPED — ' + (RESET_REASON[d.last_reset_result] || 'wait') + ' (' + d.clear_streak + ' clear)') : 'ARMED (ON)') : 'AUTO-RESET (TEST BUILD)';
    document.getElementById('latchBtn').innerText = 'Latch Mode: ' + (d.latch_enabled ? 'ON' : 'OFF');
    document.getElementById('pressMode').innerText = d.ossd_mv === 0 ? 'BRAKE LOCKED' : (isProx ? 'SLS 25% SPEED' : 'NOMINAL 100%');

    const severity = (d.state === 2 || d.state === 1) ? 'DANGER' : (isProx ? 'WARN' : 'OK');
    const nowMs = Date.now();
    if (d.frame_id !== lastFrameLogged && (severity !== lastSeverity || (nowMs - lastHeartbeatMs >= 1000))) {
      lastFrameLogged = d.frame_id;
      lastSeverity = severity;
      lastHeartbeatMs = nowMs;

      if (d.state === 2) {
        pushLog('DANGER', 'FAULT', `${FAULT_NAMES[d.fault_reason] || 'fault'} -> /dev/safety_gate OSSD 0V & press brake locked`);
      } else if (d.state === 1) {
        pushLog('DANGER', 'E-STOP', `INTRUSION at Frame #${d.frame_id} | Clearance: 0.0 px | IPC: ${d.latency_us} us -> OSSD 0V CUTOFF`);
      } else if (isProx) {
        pushLog('WARN', 'SLS-WARN', `Proximity breach (${d.dist_px.toFixed(1)} px) -> Press throttled to 25% speed`);
      } else {
        const clr = d.dist_px >= 900 ? 'zone clear' : `clearance ${d.dist_px.toFixed(1)} px`;
        pushLog('OK', 'OK', `Curtain clear (${clr}) | OSSD 24.0V DC | Press 120 SPM | IPC ${d.latency_us} us`);
      }
    }
  } catch (e) { showLinkLost(); }
}

async function pollFrame() {
  try {
    const res = await fetch('/api/frame');
    const f = await res.json();
    if (f.ok && f.w > 0 && f.h > 0 && f.pixels_b64) {
      offCanvas.width = f.w;
      offCanvas.height = f.h;
      const raw = atob(f.pixels_b64);
      const img = offCtx.createImageData(f.w, f.h);
      for (let i = 0; i < raw.length; i++) {
        const v = raw.charCodeAt(i);
        img.data[i * 4] = v;
        img.data[i * 4 + 1] = v;
        img.data[i * 4 + 2] = v;
        img.data[i * 4 + 3] = 255;
      }
      offCtx.putImageData(img, 0, 0);
      latestFrameImg = offCanvas;
    }
  } catch (e) {}
}

function renderLoop() {
  const W = camCanvas.width, H = camCanvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#09090b';
  ctx.fillRect(0, 0, W, H);

  if (latestFrameImg) {
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(latestFrameImg, 0, 0, W, H);
  }

  const st = telem ? telem.state : 2;
  const isProx = (telem && st === 0 && telem.bbox[2] > 0 && telem.dist_px <= WARN_CLEARANCE_PX);
  const zoneColor = (st === 1 || st === 2) ? '#ef4444' : (isProx ? '#f59e0b' : '#10b981');
  const zoneFill  = (st === 1 || st === 2) ? 'rgba(239, 68, 68, 0.22)' : (isProx ? 'rgba(245, 158, 11, 0.14)' : 'rgba(16, 185, 129, 0.08)');

  if (polyPts.length >= 3) {
    ctx.save();
    ctx.beginPath();
    polyPts.forEach((pt, idx) => {
      const px = pt.x * W, py = pt.y * H;
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.closePath();
    ctx.fillStyle = zoneFill;
    ctx.fill();

    ctx.save();
    ctx.clip();
    scanPhase = (scanPhase + (st === 0 ? 2.5 : 0)) % 28;
    ctx.strokeStyle = (st === 0) ? 'rgba(16, 185, 129, 0.22)' : 'rgba(239, 68, 68, 0.25)';
    ctx.lineWidth = 1;
    for (let y = scanPhase; y < H; y += 28) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(W, y);
      ctx.stroke();
    }
    ctx.restore();

    ctx.strokeStyle = zoneColor;
    ctx.lineWidth = 1.5;
    ctx.stroke();

    polyPts.forEach((pt, idx) => {
      const px = pt.x * W, py = pt.y * H;
      ctx.beginPath();
      ctx.arc(px, py, (idx === hoverIdx || idx === dragIdx) ? 6.5 : 4.5, 0, Math.PI * 2);
      ctx.fillStyle = '#09090b';
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = zoneColor;
      ctx.stroke();
    });
    ctx.restore();
  }

  if (telem && telem.bbox[2] > 0 && telem.bbox[3] > 0 && telem.frame_w > 0) {
    const sx = W / telem.frame_w, sy = H / telem.frame_h;
    const bx = telem.bbox[0] * sx, by = telem.bbox[1] * sy;
    const bw = telem.bbox[2] * sx, bh = telem.bbox[3] * sy;
    const boxColor = (st === 1) ? '#ef4444' : (isProx ? '#f59e0b' : '#38bdf8');

    ctx.strokeStyle = boxColor;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(bx, by, bw, bh);

    ctx.fillStyle = boxColor;
    ctx.font = '600 12px ui-monospace, monospace';
    const lbl = st === 1 ? 'INTRUSION [0.0 px]' : `TRACKED [${telem.dist_px.toFixed(1)} px]`;
    ctx.fillText(lbl, bx, Math.max(16, by - 6));
  }

  drawServoPressTwin(st, isProx);
  requestAnimationFrame(renderLoop);
}

function drawServoPressTwin(st, isProx) {
  const W = pressCanvas.width, H = pressCanvas.height;
  pctx.clearRect(0, 0, W, H);
  pctx.fillStyle = '#070709';
  pctx.fillRect(0, 0, W, H);

  const running = (telem && telem.ossd_mv > 0);
  if (running) {
    pressPhase += isProx ? 0.025 : 0.095;
  }
  const stroke = (Math.sin(pressPhase) + 1) * 0.5;
  const ramY = 36 + stroke * 68;
  const accent = !running ? '#ef4444' : (isProx ? '#f59e0b' : '#10b981');

  pctx.fillStyle = accent;
  pctx.fillRect(0, 0, W, 3);

  pctx.strokeStyle = '#3f3f46';
  pctx.lineWidth = 1.5;
  pctx.strokeRect(120, 18, 240, 18);
  pctx.strokeRect(120, 152, 240, 22);
  pctx.strokeStyle = '#27272a';
  pctx.strokeRect(136, 36, 12, 116);
  pctx.strokeRect(332, 36, 12, 116);

  pctx.fillStyle = !running ? 'rgba(239,68,68,0.22)' : (isProx ? 'rgba(245,158,11,0.18)' : 'rgba(16,185,129,0.18)');
  pctx.strokeStyle = accent;
  pctx.lineWidth = 1.5;
  pctx.fillRect(160, ramY, 160, 26);
  pctx.strokeRect(160, ramY, 160, 26);

  pctx.beginPath();
  pctx.moveTo(240, 36);
  pctx.lineTo(240, ramY);
  pctx.stroke();

  const partX = 160 + ((pressPhase * 28) % 140);
  pctx.fillStyle = '#52525b';
  pctx.fillRect(partX, 142, 24, 10);

  pctx.fillStyle = accent;
  pctx.fillRect(102, 42, 5, 104);
  pctx.fillRect(373, 42, 5, 104);

  pctx.strokeStyle = !running ? 'rgba(239,68,68,0.55)' : (isProx ? 'rgba(245,158,11,0.45)' : 'rgba(16,185,129,0.45)');
  pctx.lineWidth = 1;
  for (let y = 48; y <= 140; y += 15) {
    pctx.beginPath();
    pctx.moveTo(107, y);
    pctx.lineTo(373, y);
    pctx.stroke();
  }

  pctx.fillStyle = accent;
  pctx.font = '600 11px ui-monospace, monospace';
  const statusMsg = !running
    ? (st === 2 ? 'E-STOP: WATCHDOG CUTOFF (BRAKE LOCKED)' : 'E-STOP: LIGHT CURTAIN TRIPPED (0V)')
    : (isProx ? 'SLS MODE: REDUCED SPEED (25%)' : 'RUN: OSSD 24V ARMED — CURTAIN CLEAR');
  pctx.fillText(statusMsg, 14, 22);
}

async function telemetryLoop() {
  await pollTelemetry();
  setTimeout(telemetryLoop, 45);
}

async function frameLoop() {
  await pollFrame();
  setTimeout(frameLoop, 45);
}

telemetryLoop();
frameLoop();
requestAnimationFrame(renderLoop);

async function startSimFeed() {
  await fetch('/api/use-synthetic', { method: 'POST' });
  pushLog('OK', 'CAM_FEED', 'Started synthetic workcell simulation - waiting for the background baseline, then requesting reset');
  // The gate refuses a reset until the baseline has settled and the scene has stayed clear; wait for that.
  await new Promise(r => setTimeout(r, 1600));
  await resetGate();
}

async function loadSampleRealVideo() {
  pushLog('WARN', 'INGEST', 'Loading sample real video from disk...');
  const r = await fetch('/api/load-sample', { method: 'POST' });
  const j = await r.json();
  if (j.ok) {
    pushLog('OK', 'INGEST', `Loaded ${j.filename} (${j.frames} frames) -> /dev/shmem/workcell_cam`);
  } else {
    pushLog('DANGER', 'INGEST', j.error || 'Failed to load sample video');
  }
}

</script>
</body>
</html>
"""

class FrameClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.lock = threading.Lock()
        self.last_frame = None
        self.last_frame_t = 0.0

    def get_frame(self):
        with self.lock:
            try:
                if self.sock is None:
                    self.sock = socket.create_connection((self.host, self.port), timeout=0.5)
                    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.sendall(b"FRAM")
                hdr = self._recv_exact(16)
                magic, w, h, fid = struct.unpack("<IIII", hdr)
                if magic != FRAME_MAGIC or w == 0 or h == 0:
                    raise RuntimeError("Bad frame header")
                px = self._recv_exact(w * h)
                self.last_frame = (w, h, fid, base64.b64encode(px).decode("ascii"))
                self.last_frame_t = time.monotonic()
            except (OSError, ConnectionError, RuntimeError, struct.error):
                if self.sock:
                    try:
                        self.sock.close()
                    except OSError:
                        pass
                    self.sock = None
            # A frozen picture must never look live: expire old frames.
            if self.last_frame and time.monotonic() - self.last_frame_t > FRAME_MAX_AGE_S:
                self.last_frame = None
            return self.last_frame

    def _recv_exact(self, n):
        chunks = []
        got = 0
        while got < n:
            part = self.sock.recv(n - got)
            if not part:
                raise ConnectionError("EOF")
            chunks.append(part)
            got += len(part)
        return b"".join(chunks)


def decode_video_to_gray_frames(raw_bytes, filename, w=320, h=240, max_frames=450):
    if filename.lower().endswith(".bin") and len(raw_bytes) >= 12:
        bw, bh, n = struct.unpack("<III", raw_bytes[:12])
        if len(raw_bytes) == 12 + bw * bh * n:
            return bw, bh, n, raw_bytes[12:]

    fd, tmp_path = tempfile.mkstemp(suffix=os.path.splitext(filename)[1] or ".mp4")
    os.close(fd)
    try:
        with open(tmp_path, "wb") as f:
            f.write(raw_bytes)
        try:
            import cv2
            cap = cv2.VideoCapture(tmp_path)
            frames = []
            while len(frames) < max_frames:
                ok, frame = cap.read()
                if not ok:
                    break
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                resized = cv2.resize(gray, (w, h), interpolation=cv2.INTER_AREA)
                frames.append(resized.tobytes())
            cap.release()
            if frames:
                return w, h, len(frames), b"".join(frames)
        except ImportError:
            pass

        if shutil.which("ffmpeg"):
            cmd = [
                "ffmpeg", "-v", "error", "-i", tmp_path,
                "-vf", f"scale={w}:{h},format=gray",
                "-frames:v", str(max_frames),
                "-f", "rawvideo", "pipe:1"
            ]
            proc = subprocess.run(cmd, capture_output=True, check=False)
            frame_sz = w * h
            n = len(proc.stdout) // frame_sz
            if n > 0:
                return w, h, n, proc.stdout[: n * frame_sz]

        raise RuntimeError("Install OpenCV (pip install opencv-python) on Windows to decode .mp4/.mov clips.")
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)

def push_frames_to_qnx(qnx_ip, ingest_port, w, h, nframes, pixel_blob):
    header = struct.pack("<IIII", CAM_INGEST_MAGIC, w, h, nframes)
    with socket.create_connection((qnx_ip, ingest_port), timeout=5.0) as s:
        s.sendall(header)
        if pixel_blob:
            s.sendall(pixel_blob)

class Handler(BaseHTTPRequestHandler):
    qnx_ip = "192.168.184.3"
    telemetry_port = 9876
    ingest_port = 9877
    frame_client = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
        elif self.path == "/api/frame":
            fr = self.frame_client.get_frame()
            if fr:
                self._json({"ok": True, "w": fr[0], "h": fr[1], "fid": fr[2], "pixels_b64": fr[3]})
            else:
                self._json({"ok": False})
        elif self.path == "/api/telemetry":
            t = poll_telemetry(self.qnx_ip, self.telemetry_port)
            self._json(t if t else {"ok": False, "error": "no telemetry from QNX node"})

    web_port = 8080

    def _origin_ok(self):
        # Blocks cross-site requests (a web page in this browser POSTing /api/reset) and DNS rebinding.
        hosts = (f"localhost:{self.web_port}", f"127.0.0.1:{self.web_port}")
        if self.headers.get("Host", "") not in hosts:
            return False
        origin = self.headers.get("Origin")
        return origin is None or origin in tuple("http://" + h for h in hosts)

    def _do_reset(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rs:
            rs.sendto(b"RESET", (self.qnx_ip, self.telemetry_port))
        time.sleep(0.15)
        t = poll_telemetry(self.qnx_ip, self.telemetry_port)
        if not t:
            return {"ok": False, "accepted": False, "reason": "no telemetry - reset result unknown, assume stopped"}
        code = t["last_reset_result"]
        return {"ok": True, "accepted": code == 0 and t["state"] == 0,
                "reason": RESET_REASONS.get(code, "unknown"), "state": t["state"]}

    def do_POST(self):
        if not self._origin_ok():
            self.send_response(403)
            self.end_headers()
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
        except ValueError:
            length = -1
        if length < 0 or length > MAX_UPLOAD_BYTES:
            self.send_response(413)
            self.end_headers()
            return
        raw = self.rfile.read(length) if length else b""

        if self.path == "/api/upload-video":
            fname = unquote(self.headers.get("X-Filename", "upload.mp4"))
            try:
                w, h, nframes, blob = decode_video_to_gray_frames(raw, fname)
                push_frames_to_qnx(self.qnx_ip, self.ingest_port, w, h, nframes, blob)
                self._json({"ok": True, "frames": nframes})
            except (OSError, RuntimeError, ValueError, struct.error) as exc:
                self._json({"ok": False, "error": str(exc)})
            return

        if self.path == "/api/load-sample":
            try:
                search_dirs = [Path("."), Path.home() / "Downloads", Path.home() / "Desktop"]
                vids = []
                for d in search_dirs:
                    if d.exists():
                        vids.extend(sorted(d.glob("*.mp4"), key=lambda x: x.stat().st_mtime, reverse=True))
                        vids.extend(sorted(d.glob("*.mov"), key=lambda x: x.stat().st_mtime, reverse=True))
                if not vids:
                    self._json({"ok": False, "error": "No .mp4 or .mov found in project folder, Downloads, or Desktop"})
                    return
                chosen = vids[0]
                w, h, nframes, blob = decode_video_to_gray_frames(chosen.read_bytes(), chosen.name)
                push_frames_to_qnx(self.qnx_ip, self.ingest_port, w, h, nframes, blob)
                self._json({"ok": True, "filename": chosen.name, "frames": nframes})
            except (OSError, RuntimeError, ValueError, struct.error) as exc:
                self._json({"ok": False, "error": str(exc)})
            return

        if self.path == "/api/use-synthetic":
            try:
                push_frames_to_qnx(self.qnx_ip, self.ingest_port, 320, 240, 0, b"")
                self._json({"ok": True})
            except (OSError, RuntimeError) as exc:
                self._json({"ok": False, "error": str(exc)})
            return

        if self.path == "/api/reset":
            self._json(self._do_reset())
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if self.path == "/api/zone":
                try:
                    pts = json.loads(raw.decode("utf-8")).get("pts", [])
                    coords = " ".join(f"{float(p['x']):.4f} {float(p['y']):.4f}" for p in pts)
                except (ValueError, KeyError, TypeError, AttributeError):
                    self._json({"ok": False, "error": "malformed polygon"})
                    return
                if 3 <= len(pts) <= 8:
                    # The gate is the authority: it rejects (and stops the machine on) an invalid polygon.
                    sock.sendto(f"ZONE {len(pts)} {coords}".encode(), (self.qnx_ip, self.telemetry_port))
            elif self.path == "/api/latch":
                data = json.loads(raw.decode("utf-8"))
                val = 1 if data.get("enabled") else 0
                sock.sendto(f"LATCH {val}".encode(), (self.qnx_ip, self.telemetry_port))
            elif self.path == "/api/chaos":
                sock.sendto(b"CHAOS", (self.qnx_ip, self.telemetry_port))
        finally:
            sock.close()
        self._json({"ok": True})

    def _json(self, obj):
        try:
            body = json.dumps(obj).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)
        except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
            return  # browser went away; nothing to do (the gate never depends on this UI)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--qnx-ip", default="192.168.184.3")
    parser.add_argument("--qnx-port", type=int, default=9876)
    parser.add_argument("--ingest-port", type=int, default=9877)
    parser.add_argument("--frame-port", type=int, default=9878)
    parser.add_argument("--web-port", type=int, default=8080)
    args = parser.parse_args()
    Handler.qnx_ip = args.qnx_ip
    Handler.web_port = args.web_port
    Handler.telemetry_port = args.qnx_port
    Handler.ingest_port = args.ingest_port
    Handler.frame_client = FrameClient(args.qnx_ip, args.frame_port)
    print(f"Workcell Guard Control Plane live at http://localhost:{args.web_port} (QNX {args.qnx_ip})")
    ThreadingHTTPServer(("127.0.0.1", args.web_port), Handler).serve_forever()
