CC      = qcc
ARCH    = gcc_ntox86_64
CFLAGS  = -V $(ARCH) -Wall -Wextra -O2

TARGETS = safety_gate cam_feed vision_worker supervisor telemetry_bridge

all: $(TARGETS)

safety_gate: safety_gate.c safety_protocol.h config_loader.h
	$(CC) $(CFLAGS) -o $@ safety_gate.c

cam_feed: cam_feed.c safety_protocol.h config_loader.h
	$(CC) $(CFLAGS) -o $@ cam_feed.c -lm -lsocket

vision_worker: vision_worker.c safety_protocol.h config_loader.h
	$(CC) $(CFLAGS) -o $@ vision_worker.c -lm

supervisor: supervisor.c config_loader.h
	$(CC) $(CFLAGS) -o $@ supervisor.c

telemetry_bridge: telemetry_bridge.c safety_protocol.h config_loader.h
	$(CC) $(CFLAGS) -o $@ telemetry_bridge.c -lsocket

clean:
	rm -f $(TARGETS)