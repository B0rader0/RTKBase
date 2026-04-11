#pragma once

#include <freertos/queue.h>

// ─── Per-instance configuration ───────────────────────────────────────────────
// Passed as the void *arg to xTaskCreatePinnedToCore().
// The struct must remain valid for the lifetime of the task (allocated in main).
typedef struct {
    char        *host;
    uint16_t    port;
    char        *mountpoint;
    char        *username;
    char        *password;
    QueueHandle_t  queue;
    char        *task_name;   // for log messages, e.g. "ntrip1" / "ntrip2"
} ntrip_client_cfg_t;