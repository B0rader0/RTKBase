#pragma once

#include <stdint.h>
#include <stddef.h>
#include <esp_err.h>

#define UART_RX_BUF_SIZE 8192
#define GNSS_CONSOLE_SNAPSHOT_BUFFER_SIZE 8193

typedef struct {
    uint32_t rtcm_frames;
    uint32_t last_rtcm_ms;
    uint32_t rtcm_bad_len_count;
    uint32_t rtcm_crc_fail_count;
} gnss_uart_diag_t;

void gnss_uart_init(void);
void gnss_uart_get_diagnostics(gnss_uart_diag_t *diag);
esp_err_t gnss_uart_send_command(const char *command, size_t len);
size_t gnss_uart_console_snapshot(char *dest, size_t dest_size);
void gnss_uart_console_clear(void);
