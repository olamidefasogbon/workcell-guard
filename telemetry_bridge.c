#include <arpa/inet.h>
#include <devctl.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include "config_loader.h"
#include "safety_protocol.h"

#define TELEMETRY_MAGIC 0x514E5838U // "QNX8"
#define FRAME_MAGIC     0x46524D38U // "FRM8"

typedef struct __attribute__((packed)) {
    uint32_t magic;
    int32_t  state;
    int32_t  latched;
    int32_t  latch_enabled;
    uint32_t trip_count;
    uint32_t ossd_mv;
    uint32_t frame_id;
    uint64_t latency_us;
    uint64_t last_trip_us;
    float    min_zone_distance_px;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t frame_w;
    uint16_t frame_h;
    uint32_t num_pts;
    float    vx[MAX_ZONE_VERTICES];
    float    vy[MAX_ZONE_VERTICES];
} udp_telemetry_pkt_t;

static workcell_config_t cfg;
static shm_camera_t *cam = NULL;

static int send_exact(int fd, const void *buf, size_t len) {
    size_t sent = 0;
    const uint8_t *p = (const uint8_t *)buf;
    while (sent < len) {
        ssize_t n = send(fd, p + sent, len - sent, 0);
        if (n <= 0) return -1;
        sent += (size_t)n;
    }
    return 0;
}

static void *frame_stream_thread(void *arg) {
    (void)arg;
    int port = (cfg.frame_stream_port > 0) ? cfg.frame_stream_port : 9878;

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("frame stream socket"); return NULL; }

    int yes = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(srv, 4) < 0) {
        perror("frame stream bind/listen");
        close(srv);
        return NULL;
    }

    printf("[telemetry_bridge] Full-Res TCP video stream active on port %d\n", port);
    fflush(stdout);

    uint8_t *snap = (uint8_t *)malloc(MAX_FRAME_PIXELS);
    if (!snap) return NULL;

    while (1) {
        int cli = accept(srv, NULL, NULL);
        if (cli < 0) continue;
        setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        char req[16];
        while (recv(cli, req, 4, 0) == 4) {
            uint32_t w = cam->width;
            uint32_t h = cam->height;
            uint32_t fid = cam->frame_id;
            if (w == 0 || h == 0 || (size_t)w * h > MAX_FRAME_PIXELS) break;

            size_t bytes = (size_t)w * h;
            memcpy(snap, cam->pixels, bytes);
            uint32_t hdr[4] = { FRAME_MAGIC, w, h, fid };
            if (send_exact(cli, hdr, sizeof(hdr)) < 0 || send_exact(cli, snap, bytes) < 0) {
                break;
            }
        }
        close(cli);
    }
    free(snap);
    return NULL;
}

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
    }

    config_load(cfg_path, &cfg);
    if (cfg.prio_telemetry <= 0) cfg.prio_telemetry = 5;
    if (cfg.telemetry_port <= 0) cfg.telemetry_port = 9876;

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_telemetry;
    sched_setscheduler(0, SCHED_FIFO, &sp);

    int gate_fd = open(cfg.device_path, O_RDWR);
    if (gate_fd < 0) { perror("open safety_gate"); return EXIT_FAILURE; }

    int shm_fd = shm_open(cfg.shm_name, O_RDONLY, 0666);
    if (shm_fd < 0) { perror("shm_open camera"); return EXIT_FAILURE; }

    cam = (shm_camera_t *)mmap(NULL, sizeof(shm_camera_t),
                               PROT_READ, MAP_SHARED, shm_fd, 0);
    if (cam == MAP_FAILED) { perror("mmap"); return EXIT_FAILURE; }

    pthread_t ftid;
    pthread_create(&ftid, NULL, frame_stream_thread, NULL);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return EXIT_FAILURE; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)cfg.telemetry_port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind UDP telemetry port");
        return EXIT_FAILURE;
    }

    printf("[telemetry_bridge] Online at Prio %d | UDP port %d -> %s\n",
           cfg.prio_telemetry, cfg.telemetry_port, cfg.device_path);
    fflush(stdout);

    char cmd_buf[512];
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        ssize_t n = recvfrom(sock, cmd_buf, sizeof(cmd_buf) - 1, 0,
                             (struct sockaddr *)&client_addr, &client_len);
        if (n <= 0) continue;
        cmd_buf[n] = '\0';

        if (strncmp(cmd_buf, "ZONE ", 5) == 0) {
            safety_zone_t z;
            memset(&z, 0, sizeof(z));
            char *tok = strtok(cmd_buf + 5, " \t\r\n");
            if (tok) {
                z.num_pts = (uint32_t)atoi(tok);
                if (z.num_pts >= 3 && z.num_pts <= MAX_ZONE_VERTICES) {
                    uint32_t ok = 1;
                    for (uint32_t i = 0; i < z.num_pts; i++) {
                        char *sx = strtok(NULL, " \t\r\n");
                        char *sy = strtok(NULL, " \t\r\n");
                        if (!sx || !sy) { ok = 0; break; }
                        z.vx[i] = strtof(sx, NULL);
                        z.vy[i] = strtof(sy, NULL);
                    }
                    if (ok) devctl(gate_fd, DCMD_SAFETY_SET_ZONE, &z, sizeof(z), NULL);
                }
            }
            continue;
        }

        if (strncmp(cmd_buf, "RESET", 5) == 0) {
            devctl(gate_fd, DCMD_SAFETY_RESET, NULL, 0, NULL);
            continue;
        }

        if (strncmp(cmd_buf, "LATCH ", 6) == 0) {
            int32_t val = atoi(cmd_buf + 6);
            devctl(gate_fd, DCMD_SAFETY_SET_LATCH, &val, sizeof(val), NULL);
            continue;
        }

        if (strncmp(cmd_buf, "CHAOS", 5) == 0) {
            int rc = system("slay vision_worker >/dev/null 2>&1");
            (void)rc;
            continue;
        }

        safety_status_t st;
        memset(&st, 0, sizeof(st));
        if (devctl(gate_fd, DCMD_SAFETY_GET_STATUS, &st, sizeof(st), NULL) != 0) {
            continue;
        }

        udp_telemetry_pkt_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.magic                = TELEMETRY_MAGIC;
        pkt.state                = st.state;
        pkt.latched              = st.latched;
        pkt.latch_enabled        = st.latch_enabled;
        pkt.trip_count           = st.trip_count;
        pkt.ossd_mv              = st.ossd_mv;
        pkt.frame_id             = st.frame_id;
        pkt.latency_us           = st.latency_us;
        pkt.last_trip_us         = st.last_trip_us;
        pkt.min_zone_distance_px = st.min_zone_distance_px;
        pkt.bbox_x               = st.bbox_x;
        pkt.bbox_y               = st.bbox_y;
        pkt.bbox_w               = st.bbox_w;
        pkt.bbox_h               = st.bbox_h;
        pkt.frame_w              = st.frame_w;
        pkt.frame_h              = st.frame_h;
        pkt.num_pts              = st.zone.num_pts;
        memcpy(pkt.vx, st.zone.vx, sizeof(pkt.vx));
        memcpy(pkt.vy, st.zone.vy, sizeof(pkt.vy));

        sendto(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&client_addr, client_len);
    }

    close(sock);
    close(gate_fd);
    return 0;
}
