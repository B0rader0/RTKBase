#pragma once

#include <stdint.h>
#include <stddef.h>

#define UART_READ_CHUNK   128 // Number of bytes to read from UART in one go, when processing a UART_DATA event. Should be smaller than UART_RX_BUF_SIZE to allow processing of other events (e.g. overflow) in a timely manner.

typedef struct {
    uint8_t     data[UART_READ_CHUNK];
    size_t      len;
} raw_frame_t;

void post_raw_gnss_data_tcp(const raw_frame_t *frame);