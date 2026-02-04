#pragma once

#include <stdarg.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

esp_err_t log_init();
int log_vprintf(const char * format, va_list arg);
void *log_receive(size_t *length, TickType_t ticksToWait);
void log_return(void *item);