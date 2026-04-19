#pragma once

#include <stdint.h>

#include "frame_pool.h"

typedef struct {
    uint32_t queue_drops[2];
    uint32_t queue_fill[2];
    uint32_t queue_depth;
    uint32_t send_block_events[2];
    uint32_t send_fatal_errors[2];
    uint32_t reconnect_events[2];
    uint32_t connected_mask;
} ntrip_client_diag_t;

void ntrip_client_init(void);
int  ntrip_client_active_count(void);
void ntrip_client_publish(pool_frame_t *f);
void ntrip_client_get_diagnostics(ntrip_client_diag_t *diag);
