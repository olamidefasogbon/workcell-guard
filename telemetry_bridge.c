#include <arpa/inet.h>
#include <devctl.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "config_loader.h"
#include "net_acl.h"
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
    // --- v2 fields (dashboard.py PKT_FMT must match) ---
    int32_t  fault_reason;
    int32_t  reset_ready;
    int32_t  last_reset_result;
    uint32_t reset_refusals;
    uint32_t clear_streak;
    uint32_t stream_epoch;
    uint32_t heartbeat_age_ms;
    uint32_t cam_age_ms;
    int32_t  test_hooks;
} udp_telemetry_pkt_t;

static workcell_config_t cfg;
static const shm_camera_t *cam = NULL;

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

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

// Seqlock-consistent snapshot of the current frame. Returns pixel count, or 0 if no stable frame.
static size_t snapshot_frame(uint8_t *dst, uint32_t *w, uint32_t *h, uint32_t *fid) {
    for (int tries = 0; tries < 4; tries++) {
        uint32_t s1 = __atomic_load_n(&cam->write_seq, __ATOMIC_ACQUIRE);
        if ((s1 & 1U) || s1 == 0) { usleep(1000); continue; }
        uint32_t cw = cam->width, ch = cam->height, cf = cam->frame_id;
        if (cw < 16 || ch < 16 || cw > MAX_FRAME_DIM || ch > MAX_FRAME_DIM || (size_t)cw * ch > MAX_FRAME_PIXELS) return 0;
        memcpy(dst, (const void *)cam->pixels, (size_t)cw * ch);
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (__atomic_load_n(&cam->write_seq, __ATOMIC_ACQUIRE) == s1) {
            *w = cw; *h = ch; *fid = cf;
            return (size_t)cw * ch;
        }
    }
    return 0;
}

static void *frame_stream_thread(void *arg) {
    (void)arg;
    int port = cfg.frame_stream_port;

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
    if (!snap) { fprintf(stderr, "[telemetry_bridge] frame stream: out of memory\n"); close(srv); return NULL; }

    while (1) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int cli = accept(srv, (struct sockaddr *)&peer, &plen);
        if (cli < 0) continue;
        if (!acl_allows(&cfg, &peer)) { close(cli); continue; }

        setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        struct timeval tv = { 5, 0 };
        setsockopt(cli, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(cli, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        char req[4];
        while (recv(cli, req, sizeof(req), 0) == (ssize_t)sizeof(req)) {
            uint32_t w = 0, h = 0, fid = 0;
            size_t bytes = snapshot_frame(snap, &w, &h, &fid);
            if (bytes == 0) break;
            uint32_t hdr[4] = { FRAME_MAGIC, w, h, fid };
            if (send_exact(cli, hdr, sizeof(hdr)) < 0 || send_exact(cli, snap, bytes) < 0) break;
        }
        close(cli);
    }
    free(snap);
    return NULL;
}

// ZONE <n> x0 y0 x1 y1 ...   Strict: any malformed token rejects the whole command.
static int parse_zone(char *save, safety_zone_t *z) {
    memset(z, 0, sizeof(*z));
    char *tok = strtok_r(NULL, " \t\r\n", &save);
    char *end = NULL;
    if (!tok) return -1;
    long n = strtol(tok, &end, 10);
    if (*end != '\0' || n < 3 || n > MAX_ZONE_VERTICES) return -1;
    z->num_pts = (uint32_t)n;
    for (uint32_t i = 0; i < z->num_pts; i++) {
        char *sx = strtok_r(NULL, " \t\r\n", &save);
        char *sy = strtok_r(NULL, " \t\r\n", &save);
        if (!sx || !sy) return -1;
        z->vx[i] = strtof(sx, &end); if (*end != '\0') return -1;
        z->vy[i] = strtof(sy, &end); if (*end != '\0') return -1;
    }
    if (strtok_r(NULL, " \t\r\n", &save)) return -1;   // trailing garbage
    return 0;       // range / geometry is validated by the gate, which is the authority
}

int main(int argc, char **argv) {
    const char *cfg_path = "workcell.conf";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        if (opt == 'c') cfg_path = optarg;
        else return EXIT_FAILURE;
    }

    if (config_load(cfg_path, &cfg) != 0) return EXIT_FAILURE;
    acl_warn_if_open(&cfg, "telemetry_bridge");

    // A peer that disconnects mid-transfer must produce an error from send()/recv(), not kill this process.
    signal(SIGPIPE, SIG_IGN);

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = cfg.prio_telemetry;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) perror("sched_setscheduler (continuing)");

    int gate_fd = open(cfg.device_path, O_RDWR);
    if (gate_fd < 0) { perror("open safety_gate"); return EXIT_FAILURE; }

    int shm_fd = shm_open(cfg.shm_name, O_RDONLY, 0);
    if (shm_fd < 0) { perror("shm_open camera"); return EXIT_FAILURE; }

    cam = (const shm_camera_t *)mmap(NULL, sizeof(shm_camera_t), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (cam == MAP_FAILED) { perror("mmap"); return EXIT_FAILURE; }

    pthread_t ftid;
    if (pthread_create(&ftid, NULL, frame_stream_thread, NULL) != 0) {
        fprintf(stderr, "[telemetry_bridge] cannot start frame stream thread\n");
    }

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

    printf("[telemetry_bridge] Online at Prio %d | UDP port %d -> %s | hooks=%d\n",
           cfg.prio_telemetry, cfg.telemetry_port, cfg.device_path, WORKCELL_HOOKS_ENABLED);
    fflush(stdout);

    char cmd_buf[512];
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        ssize_t n = recvfrom(sock, cmd_buf, sizeof(cmd_buf) - 1, 0,
                             (struct sockaddr *)&client_addr, &client_len);
        if (n <= 0) continue;
        cmd_buf[n] = '\0';

        char *save = NULL;
        char *cmd = strtok_r(cmd_buf, " \t\r\n", &save);
        if (!cmd) continue;

        if (strcmp(cmd, "POLL") == 0) {
            // fall through to the status reply below
        } else {
            // Everything that is not a read-only poll is a control command and needs an allowed source.
            if (!acl_allows(&cfg, &client_addr)) {
                fprintf(stderr, "[telemetry_bridge] control command '%s' refused from %s\n", cmd, inet_ntoa(client_addr.sin_addr));
                continue;
            }

            if (strcmp(cmd, "ZONE") == 0) {
                safety_zone_t z;
                if (parse_zone(save, &z) == 0) {
                    if (devctl(gate_fd, DCMD_SAFETY_SET_ZONE, &z, sizeof(z), NULL) != 0) {
                        fprintf(stderr, "[telemetry_bridge] gate rejected zone\n");
                    }
                } else {
                    fprintf(stderr, "[telemetry_bridge] malformed ZONE command ignored\n");
                }
            } else if (strcmp(cmd, "RESET") == 0) {
                // The gate decides; a refusal is visible in the next status poll (last_reset_result).
                devctl(gate_fd, DCMD_SAFETY_RESET, NULL, 0, NULL);
            } else if (strcmp(cmd, "LATCH") == 0) {
                char *arg = strtok_r(NULL, " \t\r\n", &save);
                if (arg && (strcmp(arg, "0") == 0 || strcmp(arg, "1") == 0)) {
                    int32_t val = (arg[0] == '1');
                    devctl(gate_fd, DCMD_SAFETY_SET_LATCH, &val, sizeof(val), NULL);   // gate refuses val==0 outside test builds
                }
#ifdef WORKCELL_TEST_HOOKS
            } else if (strcmp(cmd, "CHAOS") == 0) {
                // Fault injection for the watchdog/supervisor demo. Compiled out of production builds.
                int rc = system("slay vision_worker >/dev/null 2>&1");
                (void)rc;
#endif
            } else {
                fprintf(stderr, "[telemetry_bridge] unknown command '%s' ignored\n", cmd);
            }
            continue;
        }

        safety_status_t st;
        memset(&st, 0, sizeof(st));
        if (devctl(gate_fd, DCMD_SAFETY_GET_STATUS, &st, sizeof(st), NULL) != 0) {
            continue;   // no reply -> the dashboard shows COMMS LOST (never a stale "all clear")
        }

        uint64_t now = now_ns();
        uint64_t cap = cam->capture_timestamp_ns;
        uint64_t cam_age_ms;
        if (cap == 0) {
            cam_age_ms = UINT32_MAX;                       // camera has never produced a frame
        } else {
            cam_age_ms = (now > cap) ? (now - cap) / 1000000ULL : 0;
            if (cam_age_ms >= UINT32_MAX) cam_age_ms = UINT32_MAX - 1U;
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
        pkt.fault_reason         = st.fault_reason;
        pkt.reset_ready          = st.reset_ready;
        pkt.last_reset_result    = st.last_reset_result;
        pkt.reset_refusals       = st.reset_refusals;
        pkt.clear_streak         = st.clear_streak;
        pkt.stream_epoch         = st.stream_epoch;
        pkt.heartbeat_age_ms     = st.heartbeat_age_ms;
        pkt.cam_age_ms           = (uint32_t)cam_age_ms;
        pkt.test_hooks           = st.test_hooks;

        sendto(sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&client_addr, client_len);
    }

    close(sock);
    close(gate_fd);
    return 0;
}
