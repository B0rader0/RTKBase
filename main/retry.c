/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * Derived from the ESP32-XBee project:
 * https://github.com/nebkat/esp32-xbee
 *
 * Original work:
 * Copyright (c) 2019 Nebojsa Cvetkovic.
 *
 * Modified for RTKBase.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "retry.h"

struct retry_delay {
    uint16_t attempts;

    bool first_instant;

    uint8_t short_count;
    int short_delay;

    int max_delay;

    uint8_t delays_offset;
};

static const int delays[] = {1000, 2000, 5000, 10000, 15000, 30000, 45000, 60000, 90000,
        120000, 300000, 600000, 900000, 1800000, 2700000, 3600000};
static const int delays_count = sizeof(delays) / sizeof(int);

retry_delay_handle_t retry_init(bool first_instant, uint8_t short_count, int short_delay, int max_delay) {
    retry_delay_handle_t handle = malloc(sizeof(struct retry_delay));
    *handle = (struct retry_delay) {
            .attempts = 0,

            .first_instant = first_instant,

            .short_count = short_count,
            .short_delay = short_delay,

            .max_delay = max_delay,

            .delays_offset = 0
    };

    while (handle->delays_offset < delays_count && delays[handle->delays_offset] < short_delay) {
        handle->delays_offset++;
    }

    return handle;
}

int retry_delay(retry_delay_handle_t handle) {
    int attempts = handle->attempts;
    int delay;
    if (attempts == 0 && handle->first_instant) {
        delay = 0;
    } else if (attempts < handle->short_count) {
        delay = handle->short_delay;
    } else {
        attempts -= handle->short_count;
        attempts += handle->delays_offset;

        if (attempts < delays_count) {
            delay = delays[attempts];
        } else {
            delay = delays[delays_count - 1];
        }

        if (handle->max_delay > 0 && delay > handle->max_delay) delay = handle->max_delay;
    }

    handle->attempts++;

    if (delay > 0) vTaskDelay(pdMS_TO_TICKS(delay));

    return handle->attempts;
}

void retry_reset(retry_delay_handle_t handle) {
    handle->attempts = 0;
}
