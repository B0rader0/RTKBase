/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * Derived from the ESP32-XBee project:
 * https://github.com/nebkat/esp32-xbee
 *
 * Original work:
 * Copyright (c) 2020 Nebojsa Cvetkovic.
 *
 * Modified for RTKBase.
 */

#pragma once

#include <stdint.h>

typedef struct stream_stats_values {
    const char *name;

    uint32_t total_in;
    uint32_t total_out;

    uint32_t rate_in;
    uint32_t rate_out;
} stream_stats_values_t;

typedef struct stream_stats *stream_stats_handle_t;

void stream_stats_init();
stream_stats_handle_t stream_stats_new(const char *name);

void stream_stats_increment(stream_stats_handle_t stats, uint32_t in, uint32_t out);
void stream_stats_values(stream_stats_handle_t stats, stream_stats_values_t *values);

stream_stats_handle_t stream_stats_first();
stream_stats_handle_t stream_stats_next(stream_stats_handle_t stats);
