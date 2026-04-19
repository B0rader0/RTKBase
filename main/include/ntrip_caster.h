#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "frame_pool.h"

typedef struct {
    uint32_t queue_drops;
    uint32_t queue_fill;
    uint32_t queue_depth;
    uint32_t rover_slow_drops;
    uint32_t rover_block_events;
    uint32_t rover_send_errors;
    uint32_t active_rovers;
} ntrip_caster_diag_t;

void ntrip_caster_init(void);
int  ntrip_caster_active_count(void);
void ntrip_caster_publish(pool_frame_t *f);
int  ntrip_caster_rover_count(void);
void ntrip_caster_get_diagnostics(ntrip_caster_diag_t *diag);
