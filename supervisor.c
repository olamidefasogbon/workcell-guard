#include <devctl.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sched.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "config_loader.h"
#include "safety_protocol.h"

extern char **environ;

// The supervisor restarts vision_worker; it never touches the safety state.
// While the worker is down, safety_gate's heartbeat watchdog holds OSSD at 0 V,
// and the latched fault means a recovered worker cannot silently re-energise it.

static volatile sig_atomic_t stop_requested = 0;
static void on_signal(int sig) { (void)sig; stop_requested = 1; }

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void sleep_ms_interruptible(uint64_t ms) {
    uint64_t end = now_ms() + ms;
    while (!stop_requested && now_ms() < end) {
        struct timespec ts = { 0, 10 * 1000000L };
        nanosleep(&ts, NULL);
    }
}

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    const char *worker_arg = "./vision_worker";
    int opt;
    while ((opt = getopt(argc, argv, "c:b:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
        else if (opt == 'b') worker_arg = optarg;
        else return EXIT_FAILURE;
    }

    workcell_config_t cfg;
    if (config_load(cfg_path, &cfg) != 0) return EXIT_FAILURE;

    // Supervision must out-rank the thing it supervises, or it can be starved by it.
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_supervisor;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("sched_setscheduler(SCHED_FIFO) - refusing to run without real-time priority");
        return EXIT_FAILURE;
    }

    // Absolute worker path, checked once: a relative path that stops resolving later must not
    // turn into an endless silent respawn failure.
    char worker_bin[PATH_MAX];
    if (!realpath(worker_arg, worker_bin) || access(worker_bin, X_OK) != 0) {
        fprintf(stderr, "[supervisor] worker '%s' not found or not executable: %s\n", worker_arg, strerror(errno));
        return EXIT_FAILURE;
    }
    char cfg_abs[PATH_MAX];
    if (!realpath(cfg_path, cfg_abs)) { perror("realpath(config)"); return EXIT_FAILURE; }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    char *const child_argv[] = { worker_bin, "-c", cfg_abs, NULL };
    printf("[supervisor] Active at Prio %d | Managing '%s' | respawn %dms (backoff to %dms) | hang-kill %dms\n",
           cfg.prio_supervisor, worker_bin, cfg.respawn_delay_ms, cfg.respawn_backoff_max_ms, cfg.hang_kill_ms);
    fflush(stdout);

    int gate_fd = -1;
    uint64_t delay_ms = (uint64_t)cfg.respawn_delay_ms;
    const uint64_t stable_ms = 5000;     // a worker that lived this long resets the backoff
    const uint64_t grace_ms  = (uint64_t)cfg.hang_kill_ms > 1000 ? (uint64_t)cfg.hang_kill_ms : 1000;
    unsigned restarts = 0;

    while (!stop_requested) {
        pid_t pid = 0;
        int rc = posix_spawn(&pid, worker_bin, NULL, NULL, child_argv, environ);
        if (rc != 0) {
            fprintf(stderr, "[supervisor] posix_spawn(%s) failed: %s\n", worker_bin, strerror(rc));
            sleep_ms_interruptible(delay_ms);
            delay_ms = delay_ms * 2 > (uint64_t)cfg.respawn_backoff_max_ms ? (uint64_t)cfg.respawn_backoff_max_ms : delay_ms * 2;
            continue;
        }

        uint64_t started = now_ms();
        printf("[supervisor] Spawned worker PID %d (restart #%u)\n", (int)pid, restarts);
        fflush(stdout);

        int status = 0, exited = 0, killed_for_hang = 0;
        while (!exited && !stop_requested) {
            pid_t w = waitpid(pid, &status, WNOHANG);
            if (w == pid) { exited = 1; break; }
            if (w < 0 && errno != EINTR) { exited = 1; break; }

            // Hang detection: the worker is alive but the gate has not seen a valid report for too long.
            if (now_ms() - started > grace_ms) {
                if (gate_fd < 0) gate_fd = open(cfg.device_path, O_RDONLY);
                safety_status_t st;
                if (gate_fd >= 0 && devctl(gate_fd, DCMD_SAFETY_GET_STATUS, &st, sizeof(st), NULL) == 0 &&
                    st.heartbeat_age_ms > (uint32_t)cfg.hang_kill_ms) {
                    fprintf(stderr, "[supervisor] worker PID %d silent for %u ms - killing\n", (int)pid, st.heartbeat_age_ms);
                    kill(pid, SIGKILL);
                    waitpid(pid, &status, 0);
                    exited = 1;
                    killed_for_hang = 1;
                    break;
                }
            }
            struct timespec ts = { 0, 20 * 1000000L };
            nanosleep(&ts, NULL);
        }

        if (stop_requested) {
            if (!exited) { kill(pid, SIGTERM); waitpid(pid, &status, 0); }
            break;
        }

        uint64_t lived = now_ms() - started;
        restarts++;
        if (lived >= stable_ms) delay_ms = (uint64_t)cfg.respawn_delay_ms;   // healthy run: back to the base delay
        printf("[supervisor] Worker PID %d %s after %llu ms. Respawning in %llu ms...\n", (int)pid,
               killed_for_hang ? "hung and was killed" : "exited", (unsigned long long)lived, (unsigned long long)delay_ms);
        fflush(stdout);

        sleep_ms_interruptible(delay_ms);
        if (lived < stable_ms) {           // crash loop: back off exponentially, bounded
            delay_ms = delay_ms * 2;
            if (delay_ms > (uint64_t)cfg.respawn_backoff_max_ms) delay_ms = (uint64_t)cfg.respawn_backoff_max_ms;
            if (delay_ms == 0) delay_ms = 1;
        }
    }

    printf("[supervisor] Stopping\n");
    return EXIT_SUCCESS;
}
