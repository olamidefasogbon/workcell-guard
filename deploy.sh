#!/usr/bin/env bash
# Build all five QNX binaries with qcc, copy them to the target and (re)start the stack.
#   ./deploy.sh              demo build (CHAOS crash button + latch-off enabled)
#   ./deploy.sh production   production build (no test hooks; latch cannot be disabled)
# Override with env vars:  QNX_HOST, QNX_TARGET, QNX_IP (default 192.168.184.3), QNX_USER (root), QNX_DIR
set -euo pipefail

MODE="${1:-demo}"
export QNX_HOST="${QNX_HOST:-$HOME/qnx800/host/win64/x86_64}"
export QNX_TARGET="${QNX_TARGET:-$HOME/qnx800/target/qnx}"
QNX_IP="${QNX_IP:-192.168.184.3}"
QNX_USER="${QNX_USER:-root}"
QNX_DIR="${QNX_DIR:-/data/home/qnxuser}"

FLAGS="-V gcc_ntox86_64 -Wall -Wextra -O2"
[ "$MODE" = "production" ] || FLAGS="$FLAGS -DWORKCELL_TEST_HOOKS"
QCC="$QNX_HOST/usr/bin/qcc"
[ -x "$QCC" ] || { echo "qcc not found at $QCC (set QNX_HOST)"; exit 1; }

echo "== building ($MODE)"
"$QCC" $FLAGS -o safety_gate      safety_gate.c      -lm
"$QCC" $FLAGS -o cam_feed         cam_feed.c         -lm -lsocket
"$QCC" $FLAGS -o vision_worker    vision_worker.c    -lm
"$QCC" $FLAGS -o supervisor       supervisor.c       -lm
"$QCC" $FLAGS -o telemetry_bridge telemetry_bridge.c -lm -lsocket

echo "== deploying to $QNX_USER@$QNX_IP:$QNX_DIR"
scp -O safety_gate cam_feed vision_worker supervisor telemetry_bridge workcell.conf "$QNX_USER@$QNX_IP:/tmp/"
# Start order matters: gate first, then camera, then supervisor (vision), then the telemetry bridge.
ssh "$QNX_USER@$QNX_IP" "slay -f safety_gate cam_feed supervisor vision_worker telemetry_bridge 2>/dev/null; \
  cd /tmp && chmod +x safety_gate cam_feed vision_worker supervisor telemetry_bridge && \
  mv -f safety_gate cam_feed vision_worker supervisor telemetry_bridge workcell.conf $QNX_DIR/ && cd $QNX_DIR && \
  on -d ./safety_gate -c workcell.conf >/tmp/gate.log 2>&1 && \
  on -d ./cam_feed -c workcell.conf >/tmp/cam.log 2>&1 && sleep 1 && \
  on -d ./supervisor -c workcell.conf -b ./vision_worker >/tmp/sup.log 2>&1 && \
  on -d ./telemetry_bridge -c workcell.conf >/tmp/tb.log 2>&1; \
  sleep 1; pidin | grep -E 'safety_gate|cam_feed|vision_worker|supervisor|telemetry_bridge'"
echo "== done. The gate starts STOPPED (0 V): wait for 'learning background' to finish, then press Reset."
