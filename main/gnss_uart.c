/** 
 * @file gnss_uart.c
 * @brief GNSS UART driver implementation
 * This module implements the UART driver for receiving GNSS data, including
 * RTCM3 correction messages and NMEA sentences. It uses the ESP-IDF UART driver in event-driven mode,
 * with a state machine to split the incoming byte stream into complete frames, validate them, and publish them to the appropriate queues for further processing by other tasks.
 * The UART configuration is read from NVS at startup, allowing for flexible hardware setups. The driver also handles various UART events such as data reception,
 * The COM2 of the UM980 module is used (irrelevant for this code but importnat for the content of some commands sent). 
*/

#include <driver/uart.h> // IDF 5.x: provided by esp_driver_uart component
#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_err.h>
#include <esp_log.h>
#include <string.h>
#include <stream_stats.h>
#include <esp_timer.h>
#include <stdlib.h>             // strtol()
#include <fcntl.h>

#include "gnss_uart.h"
#include "nvs_config.h"
#include "tcp_server.h"
#include "rtk_base.h"



static const char *TAG = "GNSS_UART";

/// ─── CRC-24Q (RTCM3) ─────────────────────────────────────────────────────────

static const uint32_t CRC24Q_POLY = 0x1864CFB;

static uint32_t crc24q(const uint8_t *buf, size_t len)
{
    uint32_t crc = 0;
    while (len--) {
        crc ^= (uint32_t)(*buf++) << 16;
        for (int i = 0; i < 8; i++) {
            crc <<= 1;
            if (crc & 0x1000000) crc ^= CRC24Q_POLY;
        }
    }
    return crc & 0xFFFFFF;
}

// ─── NMEA XOR checksum ────────────────────────────────────────────────────────

static bool nmea_checksum_valid(const uint8_t *buf, size_t len)
{
    if (len < 7 || buf[0] != '$') return false;

    int star = -1;
    for (size_t i = 1; i < len - 4; i++) {
        if (buf[i] == '*') { star = (int)i; break; }
    }
    if (star < 0) return false;

    uint8_t computed = 0;
    for (int i = 1; i < star; i++) computed ^= buf[i];

    char hex[3] = { (char)buf[star+1], (char)buf[star+2], '\0' };
    uint8_t received = (uint8_t)strtol(hex, NULL, 16);

    return computed == received;
}

// ─── Publish a validated frame to all active queues except the TCP server which transfers raw data ────────────────────────────
//
// Acquires ONE pool slot, copies the data in, then enqueues the same pointer
// to every non-NULL queue.  ref_count is set to exactly the number of queues
// that will receive the frame — determined dynamically at publish time so that
// disabled services (NULL queue handles) are never counted.
//
// This means no code change is needed here when services are enabled/disabled
// at startup: main.c simply leaves unused queue handles as NULL.

static void publish_frame(msg_type_t type, const uint8_t *data, size_t len)
{
    // ── Count active destinations ─────────────────────────────────────────────
    int refs = 0;
    //if (q_tcp_server)        refs++;          // send raw data NMEA + RTCM3
    if (type == MSG_RTCM3) {
        if (q_ntrip_1)       refs++;
        if (q_ntrip_2)       refs++;
        if (q_ntrip_server)  refs++;
    }

    if (refs == 0) return;  // no active consumers; discard silently

    // ── Allocate pool slot ────────────────────────────────────────────────────
    pool_frame_t *f = pool_alloc();
    if (f == NULL) return;   // pool_alloc already logged the error

    memcpy(f->data, data, len);
    f->len  = len;
    f->type = type;
    pool_set_refs(f, refs);

    // ── Enqueue to each active destination ────────────────────────────────────
    // If a queue is full we release that reference immediately so the pool
    // slot is returned correctly when the remaining consumers finish.
    #define TRY_ENQUEUE(q) do { \
        if ((q) != NULL) { \
            if (xQueueSend((q), &f, 0) != pdTRUE) { \
                ESP_LOGW(TAG, "Queue full (" #q "), dropping frame"); \
                pool_release(f); \
            } \
        } \
    } while(0)

    ///TRY_ENQUEUE(q_tcp_server);

    if (type == MSG_RTCM3) {
        TRY_ENQUEUE(q_ntrip_1);
        TRY_ENQUEUE(q_ntrip_2);
        TRY_ENQUEUE(q_ntrip_server);
    }

    #undef TRY_ENQUEUE
}

// ─── Frame Splitter State Machine ────────────────────────────────────────────

typedef enum {
    STATE_HUNT,         // scanning for a frame start byte
    STATE_RTCM_HDR,     // collected 0xD3, reading 2 more header bytes
    STATE_RTCM_BODY,    // reading payload + CRC
    STATE_NMEA_BODY,    // reading up to \r\n
} splitter_state_t;

typedef struct {
    splitter_state_t state;
    uint8_t          buf[MAX_RTCM_FRAME];  // reused for both types; RTCM is larger
    size_t           buf_pos;
    uint16_t         rtcm_payload_len;
    size_t           rtcm_total_len;       // 3 hdr + payload + 3 CRC
} splitter_t;

static void splitter_init(splitter_t *s)
{
    s->state   = STATE_HUNT;
    s->buf_pos = 0;
}

static void splitter_push_byte(splitter_t *s, uint8_t b)
{
restart:
    switch (s->state) {

    // ── HUNT ──────────────────────────────────────────────────────────────────
    case STATE_HUNT:
        if (b == 0xD3) {
            s->buf[0]  = b;
            s->buf_pos = 1;
            s->state   = STATE_RTCM_HDR;
        } else if (b == '$') {
            s->buf[0]  = b;
            s->buf_pos = 1;
            s->state   = STATE_NMEA_BODY;
        }
        // Any other byte: silently discard, remain in HUNT
        break;

    // ── RTCM3: two remaining header bytes ─────────────────────────────────────
    case STATE_RTCM_HDR:
        s->buf[s->buf_pos++] = b;
        if (s->buf_pos == 3) {
            uint16_t len = ((uint16_t)(s->buf[1] & 0x03) << 8) | s->buf[2];

            if (len == 0 || len > 1023) {
                ESP_LOGD(TAG, "RTCM implausible length %u — re-hunting from buf[1]", len);
                // Save both bytes before resetting: either could be a real frame
                // start (0xD3 or '$') that must not be silently discarded.
                uint8_t b1 = s->buf[1];
                uint8_t b2 = s->buf[2];  // same value as the current byte b
                splitter_init(s);
                splitter_push_byte(s, b1);
                splitter_push_byte(s, b2);
                return;                  // b already replayed; do not fall through
            }

            s->rtcm_payload_len = len;
            s->rtcm_total_len   = 3 + len + 3;
            s->state            = STATE_RTCM_BODY;
        }
        break;

    // ── RTCM3: payload + CRC ──────────────────────────────────────────────────
    case STATE_RTCM_BODY:
        // Defensive bounds check.  rtcm_total_len is always <= MAX_RTCM_FRAME by
        // construction, but guard explicitly against any future logic change.
        if (s->buf_pos >= MAX_RTCM_FRAME) {
            ESP_LOGE(TAG, "RTCM body overflow — re-hunting");
            splitter_init(s);
            goto restart;
        }
        s->buf[s->buf_pos++] = b;
        if (s->buf_pos == s->rtcm_total_len) {
            uint32_t computed = crc24q(s->buf, s->rtcm_total_len - 3);
            uint32_t received = ((uint32_t)s->buf[s->rtcm_total_len-3] << 16)
                              | ((uint32_t)s->buf[s->rtcm_total_len-2] <<  8)
                              |  (uint32_t)s->buf[s->rtcm_total_len-1];

            if (computed == received) {
                uint16_t msg_type = ((uint16_t)s->buf[3] << 4) | (s->buf[4] >> 4);
                ESP_LOGD(TAG, "RTCM3 type=%u len=%u OK", msg_type, s->rtcm_total_len);
                publish_frame(MSG_RTCM3, s->buf, s->rtcm_total_len);
            } else {
                ESP_LOGW(TAG, "RTCM3 CRC FAIL (got 0x%06lX, expected 0x%06lX)",
                         (unsigned long)received, (unsigned long)computed);
            }
            splitter_init(s);
        }
        break;

    // ── NMEA: collect until \r\n ──────────────────────────────────────────────
    case STATE_NMEA_BODY:
        if (s->buf_pos >= MAX_NMEA_FRAME - 1) {
            ESP_LOGW(TAG, "NMEA sentence too long — re-hunting");
            splitter_init(s);
            goto restart;
        }
        s->buf[s->buf_pos++] = b;
        if (b == '\n' && s->buf_pos >= 2 && s->buf[s->buf_pos-2] == '\r') {
            if (nmea_checksum_valid(s->buf, s->buf_pos)) {
                ESP_LOGD(TAG, "NMEA OK: %.*s", (int)(s->buf_pos - 2), s->buf);
                publish_frame(MSG_NMEA, s->buf, s->buf_pos);
            } else {
                ESP_LOGW(TAG, "NMEA checksum FAIL");
            }
            splitter_init(s);
        }
        break;
    }
}

// ─── UART Event-Driven Reader Task ────────────────────────────────────────────
//
// Uses the UART driver's built-in event queue instead of polling.
// Blocks on xQueueReceive() with portMAX_DELAY — zero CPU until the hardware
// has data or an error to report.
//
// Events handled:
//   UART_DATA        — normal path; drain ring buffer into frame splitter
//   UART_FIFO_OVF    — HW FIFO overflow; bytes lost at silicon; flush + reset
//   UART_BUFFER_FULL — SW ring buffer full; flush + reset
//   UART_BREAK       — break condition; log only
//   UART_FRAME_ERR   — framing error (baud mismatch); log, let CRC catch it
//   UART_PARITY_ERR  — parity error; log, let CRC catch it
//   default          — log and ignore

#define UART_EVENT_QUEUE_DEPTH  20

// Configures the task from nvs and starts the reader task
void task_gnss_reader(void *pvParams)
{
    nvs_handle_t h_config;
    config_item_value_t cfg_var;

    // ── Configure UART ────────────────────────────────────────────────────────
    uart_config_t uart_cfg = {
        .flags.allow_pd = false, // Do not allow power down.
        .flags.backup_before_sleep = false, // Do not backup before sleep, as we do not allow power down
        .source_clk = UART_SCLK_DEFAULT, // Use default source clock.
    };
     
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_config));

    // UART configuration structure. The values are populated from NVS later, after reading them from NVS.
    // Populating the uart_config structure with the configuration values read from NVS.
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u32(h_config, KEY_CONFIG_UART_BAUD_RATE, &cfg_var.uint32));
    uart_cfg.baud_rate = cfg_var.uint32;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_DATA_BITS, &cfg_var.uint8));
    uart_cfg.data_bits = cfg_var.uint8;
    
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_PARITY, &cfg_var.uint8));
    uart_cfg.parity = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_STOP_BITS, &cfg_var.uint8));
    uart_cfg.stop_bits = cfg_var.uint8;

    // The flow control is set by bitwise ORing the RTS and CTS flow control values, which are defined in uart_hw_flowcontrol_t enum. If both are disabled, the flow control will be disabled. If only one of them is enabled, the flow control will be set to the corresponding value. If both are enabled, the flow control will be set to UART_HW_FLOWCTRL_CTS_RTS.
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; // default value, will be updated later based on the RTS and CTS flow control config values read from NVS.
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_FLOW_CTRL_RTS, &cfg_var.uint8));
    if (cfg_var.enabled) {
        uart_cfg.flow_ctrl = uart_cfg.flow_ctrl | UART_HW_FLOWCTRL_RTS;
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_FLOW_CTRL_CTS, &cfg_var.uint8));
    if (cfg_var.enabled) {
        uart_cfg.flow_ctrl = uart_cfg.flow_ctrl | UART_HW_FLOWCTRL_CTS;
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_NUM, &cfg_var.uint8));
    uart_port_t uart_port = cfg_var.uint8;

    // UART pin confguration.
    int pin_tx, pin_rx, pin_rts, pin_cts;
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_TX_PIN, &cfg_var.uint8));
    pin_tx = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_RX_PIN, &cfg_var.uint8));
    pin_rx = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_RTS_PIN, &cfg_var.uint8));
    pin_rts = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_CTS_PIN, &cfg_var.uint8));
    pin_cts = cfg_var.uint8;

    nvs_close(h_config);

    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_cfg));

    ESP_ERROR_CHECK(uart_set_pin(uart_port,
                                 pin_tx, pin_rx,
                                 pin_rts, pin_cts));

    QueueHandle_t uart_evt_queue;
    
    ESP_ERROR_CHECK(uart_driver_install(uart_port,
                                        UART_RX_BUF_SIZE,
                                        0,
                                        UART_EVENT_QUEUE_DEPTH,
                                        &uart_evt_queue,
                                        0));

    ESP_LOGI(TAG, "UART%d ready @ %d baud TX=%d RX=%d (event-driven)",
             uart_port, uart_cfg.baud_rate,
             pin_tx, pin_rx);

    splitter_t splitter;
    splitter_init(&splitter);

    static raw_frame_t raw_frame;  // static: off the task stack
    uart_event_t event;

    while (1) {
        if (xQueueReceive(uart_evt_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (event.type) {

        case UART_DATA: {
            size_t remaining = event.size;
            while (remaining > 0) {
                size_t to_read = remaining < UART_READ_CHUNK ? remaining : UART_READ_CHUNK;
                int n = uart_read_bytes(uart_port, raw_frame.data, to_read, pdMS_TO_TICKS(10));
                if (n <= 0) break;

                // send raw data to TCP servers queue
                raw_frame.len = (size_t)n;
                post_raw_gnss_data_tcp(&raw_frame);

/* to fix it later
                // Parse data byte-by-byte through the splitter state machine, which will publish complete frames to the queues when ready.
                for (int i = 0; i < n; i++) {
                    splitter_push_byte(&splitter, rx_buf[i]);
                }
 */
                remaining -= (size_t)n;
            }
            break;
        }

        case UART_FIFO_OVF:
            ESP_LOGW(TAG, "HW FIFO overflow — bytes lost, resetting splitter");
            uart_flush_input(uart_port);
            xQueueReset(uart_evt_queue);
            splitter_init(&splitter);
            break;

        case UART_BUFFER_FULL:
            ESP_LOGW(TAG, "SW buffer full — flushing and resetting splitter");
            uart_flush_input(uart_port);
            xQueueReset(uart_evt_queue);
            splitter_init(&splitter);
            break;

        case UART_BREAK:
            ESP_LOGW(TAG, "UART break condition");
            break;

        case UART_FRAME_ERR:
            ESP_LOGW(TAG, "UART framing error — check baud rate (configured %d)",
                     uart_cfg.baud_rate);
            break;

        case UART_PARITY_ERR:
            ESP_LOGW(TAG, "UART parity error");
            break;

        default:
            ESP_LOGD(TAG, "Unhandled UART event: %d", event.type);
            break;
        }
    }
} // task_gnss_reader


 /* 
esp_err_t gnss_uart_init()
{
    nvs_handle_t h_config;
    uart_config_t uart_config;
    config_item_value_t cfg_var;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_config));

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_LOG_FORWARD, &cfg_var.uint8));
    uart_log_forward = cfg_var.enabled;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_NUM, &cfg_var.uint8));
    gps_port = cfg_var.uint8;

    // UART configuration structure. The values are populated from NVS later, after reading them from NVS.
    // Populating the uart_config structure with the configuration values read from NVS.
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u32(h_config, KEY_CONFIG_UART_BAUD_RATE, &cfg_var.uint32));
    uart_config.baud_rate = cfg_var.uint32;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_DATA_BITS, &cfg_var.uint8));
    uart_config.data_bits = cfg_var.uint8;
    
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_PARITY, &cfg_var.uint8));
    uart_config.parity = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_STOP_BITS, &cfg_var.uint8));
    uart_config.stop_bits = cfg_var.uint8;

    // The flow control is set by bitwise ORing the RTS and CTS flow control values, which are defined in uart_hw_flowcontrol_t enum. If both are disabled, the flow control will be disabled. If only one of them is enabled, the flow control will be set to the corresponding value. If both are enabled, the flow control will be set to UART_HW_FLOWCTRL_CTS_RTS.
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; // default value, will be updated later based on the RTS and CTS flow control config values read from NVS.
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_FLOW_CTRL_RTS, &cfg_var.uint8));
    if (cfg_var.enabled) {
        uart_config.flow_ctrl = uart_config.flow_ctrl | UART_HW_FLOWCTRL_RTS;
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_FLOW_CTRL_CTS, &cfg_var.uint8));
    if (cfg_var.enabled) {
        uart_config.flow_ctrl = uart_config.flow_ctrl | UART_HW_FLOWCTRL_CTS;
    };

    
    // UART pin confguration.
    int pin_tx, pin_rx, pin_rts, pin_cts;
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_TX_PIN, &cfg_var.uint8));
    pin_tx = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_RX_PIN, &cfg_var.uint8));
    pin_rx = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_RTS_PIN, &cfg_var.uint8));
    pin_rts = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_CTS_PIN, &cfg_var.uint8));
    pin_cts = cfg_var.uint8;

    nvs_close(h_config);

    // A. Setup Event Loop with custom queue size for bursts
    // The received GSM messages will be posted to this event loop, 
    // and all the tasks that want to receive the messages will subscribe to this event loop.
    // This is created before the UART driver is set up, so that we can post events to it from the UART event reader task 
    // as soon as we start receiving data from the GPS.
    esp_event_loop_args_t loop_args = {
        .queue_size = EVENT_QUEUE_SIZE,
        .task_name = "gps_evt_loop_task",
        .task_priority = 15,
        .task_stack_size = 3072,
        .task_core_id = tskNO_AFFINITY
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &ntrip_event_loop));
 

    // Setup UART
    // uart_config is already populated with the values read from NVS. 
    // We just need to call the UART driver functions to apply the configuration and set up the UART.
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(gps_port, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_param_config(gps_port, &uart_config)); 
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_pin(gps_port, pin_tx, pin_rx, pin_rts, pin_cts));
   
    // D. Enable Pattern Detect for '\n' (ASCII 10)
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_enable_pattern_det_baud_intr(gps_port, '\n', PATTERN_CHR_NUM, 9, 0, 0)); // 9 What are the correct parameters?
    
    //Reset the pattern queue length to record at most 20 pattern positions.
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_pattern_queue_reset(gps_port, 20));//
    
    //Create a task to handler UART event from ISR
    //xTaskCreate(uart_event_reader_task, "uart_event_reader", 4096, NULL, 12, NULL);
    
    //stream_stats = stream_stats_new("uart");

    return ESP_OK;
} // uart_init */