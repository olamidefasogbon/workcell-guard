# ---- QNX target build -------------------------------------------------------
#   make                 production build (no fault-injection hooks, latch cannot be disabled)
#   make TEST_HOOKS=1    demo/test build  (enables the CHAOS vision-crash hook and latch-off)
CC      = qcc
ARCH    = gcc_ntox86_64
CFLAGS  = -V $(ARCH) -Wall -Wextra -O2
ifeq ($(TEST_HOOKS),1)
CFLAGS += -DWORKCELL_TEST_HOOKS
endif

TARGETS = safety_gate cam_feed vision_worker supervisor telemetry_bridge

COMMON  = safety_protocol.h safety_logic.h config_loader.h

all: $(TARGETS)

safety_gate: safety_gate.c $(COMMON)
	$(CC) $(CFLAGS) -o $@ safety_gate.c

cam_feed: cam_feed.c $(COMMON) net_acl.h
	$(CC) $(CFLAGS) -o $@ cam_feed.c -lm -lsocket

vision_worker: vision_worker.c vision_core.h $(COMMON)
	$(CC) $(CFLAGS) -o $@ vision_worker.c -lm

supervisor: supervisor.c $(COMMON)
	$(CC) $(CFLAGS) -o $@ supervisor.c

telemetry_bridge: telemetry_bridge.c $(COMMON) net_acl.h
	$(CC) $(CFLAGS) -o $@ telemetry_bridge.c -lm -lsocket

clean:
	rm -f $(TARGETS)
	rm -rf build

# ---- Host-side verification (no QNX required) ------------------------------
#   make test            unit tests for the safety core, vision core, config, dashboard, and an end-to-end scenario
#   make check-syntax    compile-check the QNX sources against stub QNX headers
HOSTCC    ?= cc
HOSTFLAGS  = -std=gnu11 -Wall -Wextra -Werror -O2 -I.
TESTS      = test_gate test_config test_vision test_e2e

build:
	mkdir -p build

build/%: tests/%.c $(COMMON) vision_core.h | build
	$(HOSTCC) $(HOSTFLAGS) -o $@ $< -lm

test: $(addprefix build/,$(TESTS))
	@set -e; for t in $(TESTS); do echo "== $$t"; ./build/$$t; done; echo "== test_dashboard"; python3 tests/test_dashboard.py; echo "ALL TESTS PASSED"

check-syntax: | build
	@set -e; for f in vision_worker cam_feed telemetry_bridge supervisor; do \
	  echo "syntax: $$f.c"; \
	  $(HOSTCC) -std=gnu11 -D_GNU_SOURCE -Wall -Wextra -D__QNX__ -fsyntax-only -I. -Itests/stubs $$f.c; \
	done
	@echo "syntax: safety_gate.c"; $(HOSTCC) -std=gnu11 -D_GNU_SOURCE -Wall -Wextra -D__QNX__ -fsyntax-only -I. -Itests/stubs safety_gate.c
	@echo "syntax OK (hooks on)"; $(HOSTCC) -std=gnu11 -D_GNU_SOURCE -DWORKCELL_TEST_HOOKS -D__QNX__ -fsyntax-only -I. -Itests/stubs telemetry_bridge.c

.PHONY: all clean test check-syntax
