// Source-address allow-list for control/ingest sockets.
//
// This is a minimum guard against accidental or casual access (anyone on the
// subnet being able to send RESET). It is NOT authentication: IPv4 source
// addresses can be spoofed. A production deployment must put these ports on an
// isolated safety network or add an authenticated channel (TLS / HMAC + nonce).
#ifndef NET_ACL_H
#define NET_ACL_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>

#include "config_loader.h"

// Returns 1 if `peer` may issue control commands.
static inline int acl_allows(const workcell_config_t *cfg, const struct sockaddr_in *peer) {
    if (cfg->control_allow_ip[0] == '\0') return 1;   // open (a warning is printed at start-up)
    uint32_t allowed;
    if (cfg_parse_ipv4(cfg->control_allow_ip, &allowed) != 0) return 0;
    return peer->sin_family == AF_INET && peer->sin_addr.s_addr == htonl(allowed);
}

static inline void acl_warn_if_open(const workcell_config_t *cfg, const char *who) {
    if (cfg->control_allow_ip[0] == '\0') {
        fprintf(stderr, "[%s] WARNING: control_allow_ip is not set - any host can send control commands\n", who);
    }
}

#endif
