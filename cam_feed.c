#include <devctl.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "config_loader.h"
#include "safety_protocol.h"

#define CAM_INGEST_MAGIC 0x43414D38U // "CAM8"
#define MAX_CUSTOM_FRAMES 450

static workcell_config_t cfg;
static pthread_mutex_t clip_lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t *custom_frames = NULL;
static uint32_t custom_w = 0;
static uint32_t custom_h = 0;
static uint32_t custom_count = 0;
static uint32_t custom_idx = 0;

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static int recv_exact(int fd, void *buf, size_t len) {
    size_t got = 0;
    uint8_t *p = (uint8_t *)buf;
    while (got < len) {
        ssize_t n = recv(fd, p + got, len - got, 0);
        if (n <= 0) return -1;
        got += (size_t)n;
    }
    return 0;
}

static void *ingest_thread(void *arg) {
    (void)arg;
    int port = (cfg.cam_ingest_port > 0) ? cfg.cam_ingest_port : 9877;

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("cam ingest socket"); return NULL; }

    int yes = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(srv, 2) < 0) {
        perror("cam ingest bind/listen");
        close(srv);
        return NULL;
    }

    printf("[cam_feed] TCP video ingest listening on port %d\n", port);
    fflush(stdout);

    while (1) {
        int cli = accept(srv, NULL, NULL);
        if (cli < 0) continue;

        uint32_t hdr[4];
        if (recv_exact(cli, hdr, sizeof(hdr)) == 0 && hdr[0] == CAM_INGEST_MAGIC) {
            uint32_t w = hdr[1];
            uint32_t h = hdr[2];
            uint32_t n = hdr[3];

            if (n == 0) {
                pthread_mutex_lock(&clip_lock);
                free(custom_frames);
                custom_frames = NULL;
                custom_count = 0;
                custom_idx = 0;
                pthread_mutex_unlock(&clip_lock);
                printf("[cam_feed] Switched back to synthetic workcell stream\n");
                fflush(stdout);
            } else if (w >= 16 && h >= 16 && (size_t)w * h <= MAX_FRAME_PIXELS && n <= MAX_CUSTOM_FRAMES) {
                size_t total = (size_t)w * h * n;
                uint8_t *buf = (uint8_t *)malloc(total);
                if (buf && recv_exact(cli, buf, total) == 0) {
                    pthread_mutex_lock(&clip_lock);
                    free(custom_frames);
                    custom_frames = buf;
                    custom_w = w;
                    custom_h = h;
                    custom_count = n;
                    custom_idx = 0;
                    pthread_mutex_unlock(&clip_lock);
                    printf("[cam_feed] Hot-swapped live video clip (%ux%u, %u frames)\n", w, h, n);
                    fflush(stdout);
                } else {
                    free(buf);
                }
            }
        }
        close(cli);
    }
    return NULL;
}

static void render_synthetic_frame(uint8_t *buf, uint32_t w, uint32_t h, uint32_t fid, float target_cx, float target_cy) {
    // Rich industrial workcell floor & stamping bed grid
    for (uint32_t y = 0; y < h; y++) {
        uint8_t base = (uint8_t)(28 + (y * 18) / h);
        for (uint32_t x = 0; x < w; x++) {
            uint8_t v = base;
            if ((x % 32 == 0) || (y % 32 == 0)) v = (uint8_t)(v + 12);
            // Hazard press bed on right side
            if (x > (w * 52) / 100 && x < (w * 90) / 100 && y > (h * 22) / 100 && y < (h * 82) / 100) {
                v = (uint8_t)(v + 16);
            }
            buf[y * w + x] = v;
        }
    }

    // Smoothly patrolling operator / AGV arm that periodically approaches & enters right-side zone
        float phase = (float)(fid % 240U) / 240.0f * 2.0f * 3.1415926f;
    float progress = 0.5f - 0.5f * cosf(phase);
    float start_x = (target_cx > 0.5f) ? 0.12f : 0.88f;
    float norm_x  = start_x + (target_cx - start_x) * progress;
    float norm_y  = target_cy;

    int cx = (int)(norm_x * (float)w);
    int cy = (int)(norm_y * (float)h);
    int rw = (int)(w * 0.055f);
    int rh = (int)(h * 0.13f);

    int x0 = cx - rw, x1 = cx + rw;
    int y0 = cy - rh, y1 = cy + rh;
    if (x0 < 2) x0 = 2;
    if (y0 < 2) y0 = 2;
    if (x1 >= (int)w - 2) x1 = (int)w - 3;
    if (y1 >= (int)h - 2) y1 = (int)h - 3;

    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            buf[(uint32_t)y * w + (uint32_t)x] = 225;
        }
    }
}

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
    }

    config_load(cfg_path, &cfg);
    if (cfg.sim_width == 0 || cfg.sim_width > 1280) cfg.sim_width = 320;
    if (cfg.sim_height == 0 || cfg.sim_height > 720) cfg.sim_height = 240;
    if (cfg.target_fps <= 0) cfg.target_fps = 30;

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_camera;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    shm_unlink(cfg.shm_name);
    int shm_fd = shm_open(cfg.shm_name, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) { perror("shm_open"); return EXIT_FAILURE; }
    if (ftruncate(shm_fd, sizeof(shm_camera_t)) == -1) { perror("ftruncate"); return EXIT_FAILURE; }

    shm_camera_t *cam = (shm_camera_t *)mmap(NULL, sizeof(shm_camera_t),
                                             PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (cam == MAP_FAILED) { perror("mmap"); return EXIT_FAILURE; }
    memset(cam, 0, sizeof(shm_camera_t));

    int gate_fd = open(cfg.device_path, O_RDONLY);
    pthread_t itid;
    pthread_create(&itid, NULL, ingest_thread, NULL);

    printf("[cam_feed] Online at Prio %d | %ux%u @ %d FPS -> %s\n",
           cfg.prio_camera, cfg.sim_width, cfg.sim_height, cfg.target_fps, cfg.shm_name);
    fflush(stdout);

    uint32_t fid = 0;
    uint32_t seq = 0;
    useconds_t frame_us = (useconds_t)(1000000 / cfg.target_fps);

    while (1) {
        fid++;
        uint32_t w = cfg.sim_width;
        uint32_t h = cfg.sim_height;

        seq++;
        cam->write_seq = seq; // Odd = write in progress
        __sync_synchronize();

        pthread_mutex_lock(&clip_lock);
        if (custom_frames && custom_count > 0) {
            w = custom_w;
            h = custom_h;
            size_t bytes = (size_t)w * h;
            memcpy(cam->pixels, custom_frames + (size_t)custom_idx * bytes, bytes);
            custom_idx = (custom_idx + 1) % custom_count;
        } else {
            float tcx = 0.81f, tcy = 0.525f;
            if (gate_fd < 0) gate_fd = open(cfg.device_path, O_RDONLY);
            if (gate_fd >= 0) {
                safety_status_t st;
                if (devctl(gate_fd, DCMD_SAFETY_GET_STATUS, &st, sizeof(st), NULL) == 0 && st.zone.num_pts >= 3) {
                    float sx = 0.0f, sy = 0.0f;
                    for (uint32_t i = 0; i < st.zone.num_pts; i++) { sx += st.zone.vx[i]; sy += st.zone.vy[i]; }
                    tcx = sx / (float)st.zone.num_pts;
                    tcy = sy / (float)st.zone.num_pts;
                }
            }
            render_synthetic_frame(cam->pixels, w, h, fid, tcx, tcy);
        }
        pthread_mutex_unlock(&clip_lock);

        cam->width = w;
        cam->height = h;
        cam->frame_id = fid;
        cam->capture_timestamp_ns = now_ns();

        __sync_synchronize();
        seq++;
        cam->write_seq = seq; // Even = consistent frame ready

        usleep(frame_us);
    }

    return 0;
}
