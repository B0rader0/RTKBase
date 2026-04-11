#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ─── Hard UART constants (not user-configurable via web UI) ───────────────────
// User-configurable values (baud rate, pins etc.) live in config.h / g_cfg.
#define UART_RX_BUF_SIZE            8192

// ─── Frame / Pool Sizing ──────────────────────────────────────────────────────
#define MAX_RTCM_FRAME              1029    // 3 hdr + 1023 payload + 3 CRC
#define MAX_NMEA_FRAME              128
#define MAX_FRAME_SIZE              MAX_RTCM_FRAME

// Each queue carries pool_frame_t* pointers.
#define FRAME_QUEUE_DEPTH           16

// Pool: 4 queues × 16 depth + 8 slack = 72
// (socket_server + ntrip1 + ntrip2 + ntrip_server = 4 queues, each with 16 frames = 64 frames, plus 8 slack frames for safety)
#define FRAME_POOL_SIZE             72

// ─── Message Types ────────────────────────────────────────────────────────────
typedef enum {
    MSG_RTCM3,
    MSG_NMEA,
} msg_type_t;

// ─── Static Frame Pool Entry ──────────────────────────────────────────────────
typedef struct {
    uint8_t     data[MAX_FRAME_SIZE];
    size_t      len;
    msg_type_t  type;
    atomic_int  ref_count;
} pool_frame_t;

// ─── Pool API  (frame_pool.c) ─────────────────────────────────────────────────
void          pool_init(void);
pool_frame_t *pool_alloc(void);
void          pool_set_refs(pool_frame_t *f, int n);
void          pool_release(pool_frame_t *f);

// ─── Shared Queue Handles ─────────────────────────────────────────────────────
// NULL if the corresponding service is disabled in g_cfg.
// publish_frame() NULL-checks each handle before enqueuing.
extern QueueHandle_t q_ntrip_1;        // RTCM3 only   → NTRIP client 1
extern QueueHandle_t q_ntrip_2;        // RTCM3 only   → NTRIP client 2
extern QueueHandle_t q_ntrip_server;   // RTCM3 only   → NTRIP server (LAN rovers)

// ─── Misc helpers ─────────────────────────────────────────────────────────────
int ntrip_server_rover_count(void);
