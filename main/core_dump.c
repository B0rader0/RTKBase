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

#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <esp_core_dump.h>
#include "core_dump.h"

static const char *TAG = "CORE_DUMP";
static const esp_partition_t *core_dump_partition;
static size_t core_dump_size = 0;

void core_dump_check() {
    size_t core_dump_addr = 0;
    core_dump_partition = NULL;
    core_dump_size = 0;

    if (esp_core_dump_image_get(&core_dump_addr, &core_dump_size) != ESP_OK || core_dump_size == 0) {
        return;
    }

    core_dump_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
    if (core_dump_partition == NULL) {
        ESP_LOGE(TAG, "Core dump image reported but coredump partition is missing");
        core_dump_size = 0;
        return;
    }

    if (core_dump_size > core_dump_partition->size) {
        ESP_LOGE(TAG, "Core dump size %u exceeds partition size %u",
                 (unsigned)core_dump_size, (unsigned)core_dump_partition->size);
        core_dump_size = 0;
    }
}

size_t core_dump_available() {
    return core_dump_partition != NULL ? core_dump_size : 0;
}

esp_err_t core_dump_read(size_t offset, void *buffer, size_t len) {
    if (core_dump_partition == NULL || buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (offset > core_dump_size || len > core_dump_size - offset) {
        return ESP_ERR_INVALID_SIZE;
    }

    return esp_partition_read(core_dump_partition, offset, buffer, len);
}
