/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * Derived from the ESP32-XBee project:
 * https://github.com/nebkat/esp32-xbee
 *
 * Original work:
 * Copyright (c) 2019 Nebojsa Cvetkovic.
 *
 * Modified and hardened for RTKBase.
 */

#pragma once

#include <stddef.h>
#include <esp_err.h>

void core_dump_check();
size_t core_dump_available();
esp_err_t core_dump_read(size_t offset, void *buffer, size_t len);
