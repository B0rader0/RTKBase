#pragma once

#include <esp_err.h>
#include <stddef.h>

#define LOG_SNAPSHOT_BUFFER_SIZE 16385

esp_err_t log_init(void);
size_t log_snapshot(char *dest, size_t dest_size);
