#pragma once

#include <stdint.h>

#define UART_RX_BUF_SIZE 8192

typedef struct {
    uint32_t rtcm_frames;
    uint32_t last_rtcm_ms;
    uint32_t rtcm_bad_len_count;
    uint32_t rtcm_crc_fail_count;
} gnss_uart_diag_t;

void gnss_uart_init(void);
void gnss_uart_get_diagnostics(gnss_uart_diag_t *diag);

