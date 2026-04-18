#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

// ─── Frame / Pool Sizing ──────────────────────────────────────────────────────
#define MAX_RTCM_FRAME              1029    // 3 hdr + 1023 payload + 3 CRC
#define FRAME_QUEUE_DEPTH           16

// Pool: 3 RTCM consumers (2 uplinks + 1 local caster) × 16 depth + 8 slack
#define FRAME_POOL_SIZE             56

typedef struct {
    uint8_t     data[MAX_RTCM_FRAME];
    size_t      len;
    atomic_int  ref_count;
} pool_frame_t;

void          pool_init(void);
pool_frame_t *pool_alloc(void);
void          pool_set_refs(pool_frame_t *f, int n);
void          pool_release(pool_frame_t *f);
