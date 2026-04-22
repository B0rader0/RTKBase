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

#pragma once

#include <stdint.h>

typedef struct retry_delay *retry_delay_handle_t;

retry_delay_handle_t retry_init(bool first_instant, uint8_t short_count, int short_delay, int max_delay);
int retry_delay(retry_delay_handle_t handle);
void retry_reset(retry_delay_handle_t handle);
