#include <sched.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "config_loader.h"

extern char **environ;

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    const char *worker_bin = "./vision_worker";
    int opt;
    while ((opt = getopt(argc, argv, "c:b:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
        else if (opt == 'b') worker_bin = optarg;
    }

    workcell_config_t cfg;
    config_load(cfg_path, &cfg);

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_supervisor;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    char *const child_argv[] = { (char *)worker_bin, "-c", (char *)cfg_path, NULL };
    printf("[supervisor] Active at Prio %d | Managing '%s' (respawn delay %dms)\n",
           cfg.prio_supervisor, worker_bin, cfg.respawn_delay_ms);
    fflush(stdout);

    while (1) {
        pid_t pid = 0;
        int rc = posix_spawn(&pid, worker_bin, NULL, NULL, child_argv, environ);
        if (rc != 0) {
            fprintf(stderr, "[supervisor] posix_spawn(%s) failed: %s\n", worker_bin, strerror(rc));
            sleep(1);
            continue;
        }

        printf("[supervisor] Spawned worker PID %d\n", (int)pid);
        fflush(stdout);

        int status = 0;
        waitpid(pid, &status, 0);

        printf("[supervisor] Worker PID %d exited! Respawning in %dms...\n",
               (int)pid, cfg.respawn_delay_ms);
        fflush(stdout);
        usleep((useconds_t)cfg.respawn_delay_ms * 1000U);
    }
    return 0;
}