import json, socket, struct, sys, threading, time, urllib.request, urllib.error, zlib, base64
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import dashboard as d
from http.server import ThreadingHTTPServer

W, H = 320, 240
PIX = bytes((x * 3 + y * 5) & 0xFF for y in range(H) for x in range(W))
state = {"reset_result": 2, "st": 1, "polls": 0, "frames": 0, "fid": 100, "frame_on": True}

def fake_qnx(sock):
    while True:
        data, addr = sock.recvfrom(512)
        if data.startswith(b"RESET"):
            if state["reset_result"] == 0: state["st"] = 0
            continue
        state["polls"] += 1
        vx = [0.5, 0.9, 0.9, 0.5] + [0]*4; vy = [0.2, 0.2, 0.8, 0.8] + [0]*4
        pkt = struct.pack(d.PKT_FMT, d.TELEMETRY_MAGIC, state["st"], 1, 1, 1, 0 if state["st"] else 24000, 7, 900, 800, 0.0,
                          10, 10, 20, 20, 320, 240, 4, *vx, *vy, 8, 0, state["reset_result"], 1, 3, 1, 20, 33, 0)
        sock.sendto(pkt, addr)

def fake_frames(srv):
    while True:
        c, _ = srv.accept()
        def serve(c=c):
            try:
                while c.recv(4) == b"FRAM":
                    if not state["frame_on"]: return
                    state["frames"] += 1; state["fid"] += 1
                    c.sendall(struct.pack("<IIII", d.FRAME_MAGIC, W, H, state["fid"]) + PIX)
            except OSError: pass
            finally: c.close()
        threading.Thread(target=serve, daemon=True).start()

q = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); q.bind(("127.0.0.1", 0)); qport = q.getsockname()[1]
threading.Thread(target=fake_qnx, args=(q,), daemon=True).start()
fs = socket.socket(); fs.bind(("127.0.0.1", 0)); fs.listen(4); fport = fs.getsockname()[1]
threading.Thread(target=fake_frames, args=(fs,), daemon=True).start()

d.Handler.qnx_ip = "127.0.0.1"; d.Handler.telemetry_port = qport; d.Handler.web_port = 18080
d.Handler.frame_client = d.FrameClient("127.0.0.1", fport)
d.Handler.cache = d.LiveCache("127.0.0.1", qport, d.Handler.frame_client)
d.Handler.cache.IDLE_AFTER_S = 1.0
d.Handler.cache.start()
srv = ThreadingHTTPServer(("127.0.0.1", 18080), d.Handler); threading.Thread(target=srv.serve_forever, daemon=True).start()

def call(path, method="GET", headers=None, raw=False):
    req = urllib.request.Request("http://127.0.0.1:18080" + path, method=method, data=b"" if method == "POST" else None, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            body = r.read(); return r.status, (body if raw else json.loads(body or b"{}"))
    except urllib.error.HTTPError as e:
        b = e.read(); return e.code, (json.loads(b) if b.startswith(b"{") else {})

ok = True
def check(c, m):
    global ok; ok &= bool(c); print(("PASS " if c else "FAIL ") + m)

time.sleep(0.4)
code, t = call("/api/telemetry")
check(code == 200 and t["ok"] and t["state"] == 1 and t["fault_reason"] == 8 and t["clear_streak"] == 3 and len(t["pts"]) == 4, "v2 telemetry decodes (fault_reason=8 learning, clear_streak, pts)")
check(d.FAULT_NAMES[8].startswith("learning background"), "fault 8 has an operator-readable name")

# ---- constant load on the QNX node regardless of viewers ----
p0, f0, t0 = state["polls"], state["frames"], time.monotonic()
def hammer():
    end = time.monotonic() + 2.0
    n = 0
    while time.monotonic() < end:
        call("/api/telemetry"); call("/api/frame?since=-1"); n += 2
    results.append(n)
results = []
ths = [threading.Thread(target=hammer) for _ in range(8)]       # 8 browsers polling flat out
[x.start() for x in ths]; [x.join() for x in ths]
dt = time.monotonic() - t0
http_reqs = sum(results); polls = state["polls"] - p0; frames = state["frames"] - f0
print(f"     {http_reqs} HTTP requests from 8 viewers in {dt:.1f}s -> QNX saw {polls} status polls and {frames} frame fetches")
check(polls <= dt / d.LiveCache.TELEMETRY_PERIOD_S * 1.3 and polls < http_reqs / 4, "status polls to QNX do not scale with viewers")
check(frames <= dt / d.LiveCache.FRAME_PERIOD_S * 1.5 + 2 and frames < http_reqs / 8, "frame fetches from QNX do not scale with viewers")

# ---- PNG frame is correct and 'unchanged' works ----
code, f = call("/api/frame?since=-1")
check(code == 200 and f["ok"] and f["w"] == W and f["h"] == H and "png_b64" in f, "frame served as PNG")
png = base64.b64decode(f["png_b64"])
check(png[:8] == b"\x89PNG\r\n\x1a\n", "PNG signature")
# decode IDAT by hand and compare to the source pixels
pos, idat, ihdr = 8, b"", None
while pos < len(png):
    ln, tag = struct.unpack(">I4s", png[pos:pos + 8]); data = png[pos + 8:pos + 8 + ln]
    crc = struct.unpack(">I", png[pos + 8 + ln:pos + 12 + ln])[0]
    assert crc == (zlib.crc32(tag + data) & 0xFFFFFFFF), "bad CRC in " + tag.decode()
    if tag == b"IHDR": ihdr = struct.unpack(">IIBBBBB", data)
    if tag == b"IDAT": idat += data
    pos += 12 + ln
rows = zlib.decompress(idat)
decoded = b"".join(rows[y * (W + 1) + 1:(y + 1) * (W + 1)] for y in range(H))
check(ihdr == (W, H, 8, 0, 0, 0, 0) and decoded == PIX, "PNG decodes to exactly the camera pixels (CRCs valid)")
check(len(f["png_b64"]) < len(base64.b64encode(PIX)), f"PNG is smaller than raw frame ({len(f['png_b64'])} vs {len(base64.b64encode(PIX))} base64 bytes)")
code, u = call("/api/frame?since=%d" % f["fid"])
check(u.get("unchanged") is True or u.get("fid", 0) > f["fid"], "unchanged frame is not resent")

# ---- idle mode: no viewers -> slow polling, no frame fetching ----
time.sleep(1.6)
p1, f1 = state["polls"], state["frames"]; time.sleep(2.0)
check(state["polls"] - p1 <= 6, f"idle status polling is slow ({state['polls'] - p1} polls in 2 s)")
check(state["frames"] - f1 == 0, "no frame fetching while nobody is watching")

# ---- stale frames are never shown as live ----
call("/api/frame?since=-1"); time.sleep(0.5)
state["frame_on"] = False
time.sleep(d.FRAME_MAX_AGE_S + 1.5); call("/api/frame?since=-1"); time.sleep(0.3)
code, f2 = call("/api/frame?since=-1")
check(f2.get("ok") is False, "camera feed gone -> frame endpoint reports not-ok instead of a frozen picture")
state["frame_on"] = True

# ---- control path unchanged ----
state["reset_result"] = 2
code, r = call("/api/reset", "POST")
check(r["ok"] and not r["accepted"] and "hazard" in r["reason"], f"refused reset reported honestly: {r['reason']}")
state["reset_result"] = 0
code, r = call("/api/reset", "POST")
check(r["accepted"] and r["state"] == 0, "accepted reset reported only when the gate really is SAFE_RUN")
check(call("/api/reset", "POST", {"Origin": "http://evil.example"})[0] == 403, "cross-site POST /api/reset blocked (CSRF)")
check(call("/api/reset", "POST", {"Host": "demo.example.com"})[0] == 403, "unlisted tunnel hostname blocked by default")
d.Handler.allowed_hosts = {"localhost", "127.0.0.1", "demo.example.com"}
code, r = call("/api/reset", "POST", {"Host": "demo.example.com:8443", "Origin": "https://demo.example.com"})
check(code == 200 and r["ok"], "listed tunnel hostname works")
d.Handler.allowed_hosts = {"localhost", "127.0.0.1"}
check(call("/api/mode")[1] == {"read_only": False}, "/api/mode reports control mode")
d.Handler.read_only = True
code, r = call("/api/reset", "POST")
check(code == 403 and r.get("accepted") is False and "view-only" in r.get("reason", ""), "--read-only refuses control with a JSON reason the UI can show")
check(call("/api/mode")[1] == {"read_only": True}, "/api/mode reports view-only")
check(call("/api/telemetry")[0] == 200, "--read-only still serves telemetry")
d.Handler.read_only = False
req = urllib.request.Request("http://127.0.0.1:18080/api/upload-video", method="POST", data=b"x", headers={"Content-Length": str(300*1024*1024)})
try: urllib.request.urlopen(req, timeout=3); c = False
except urllib.error.HTTPError as e: c = e.code == 413
except OSError: c = True
check(c, "oversized upload rejected")

# ---- node gone ----
d.Handler.cache.telemetry_port = 9; d.Handler.cache.TELEMETRY_MAX_AGE_S = 0.3
d.Handler.cache.touch(); time.sleep(0.8)
code, t = call("/api/telemetry")
check(t.get("ok") is False, "dead node -> ok:false (UI shows STALE then COMMS LOST, never the old 24 V)")
sys.exit(0 if ok else 1)
