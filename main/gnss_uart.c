/** 
 * @file gnss_uart.c
 * @brief GNSS UART driver implementation
 * This module receives GNSS UART data in event-driven mode. Raw UART chunks are
 * forwarded to the TCP server for UPrecise, while a lightweight splitter
 * extracts complete RTCM3 frames for the NTRIP uplinks and local caster.
 * The UART configuration is read from NVS at startup, allowing for flexible hardware setups. The driver also handles various UART events such as data reception,
 * The COM2 of the UM980 module is used (irrelevant for this code but importnat for the content of some commands sent). 
*/

#include <driver/uart.h> // IDF 5.x: provided by esp_driver_uart component
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <freertos/semphr.h>

#include "gnss_uart.h"
#include "frame_pool.h"
#include "ntrip_caster.h"
#include "ntrip_client.h"
#include "nvs_config.h"
#include "tcp_server.h"



static const char *TAG = "GNSS_UART";

#define LOG_RTCM_SPLITTER_ERRORS   1
#define RTCM_ERROR_LOG_MASK        0x3F
#define GNSS_CONSOLE_BUFFER_SIZE   8192
#define GNSS_CONSOLE_LINE_SIZE     256
#define GNSS_COMMAND_MAX_LEN       256

static uart_port_t s_uart_port = UART_NUM_MAX;
static SemaphoreHandle_t s_uart_tx_mutex;
static char s_console_buffers[2][GNSS_CONSOLE_BUFFER_SIZE];
static uint8_t s_console_active_buffer;
static size_t s_console_head;
static size_t s_console_used;
static char s_console_line[GNSS_CONSOLE_LINE_SIZE];
static size_t s_console_line_len;
static bool s_console_line_binary;
static uint64_t s_console_suppress_until_us;
static portMUX_TYPE s_console_mux = portMUX_INITIALIZER_UNLOCKED;

/// ─── CRC-24Q (RTCM3) ─────────────────────────────────────────────────────────

static const uint32_t CRC24Q_POLY = 0x1864CFB;
static uint32_t rtcm_bad_len_count;
static uint32_t rtcm_crc_fail_count;
static uint32_t rtcm_frame_count;
static uint64_t last_rtcm_ms;

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

static void log_rtcm_bad_len(uint16_t len)
{
#if LOG_RTCM_SPLITTER_ERRORS
    if ((++rtcm_bad_len_count & RTCM_ERROR_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "RTCM bad length=%u count=%lu",
                 len, (unsigned long)rtcm_bad_len_count);
    }
#else
    (void)len;
#endif
}

static void log_rtcm_crc_fail(void)
{
#if LOG_RTCM_SPLITTER_ERRORS
    if ((++rtcm_crc_fail_count & RTCM_ERROR_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "RTCM CRC failures=%lu",
                 (unsigned long)rtcm_crc_fail_count);
    }
#endif
}

static bool console_has_checksum_suffix(const char *line)
{
    const char *star = strrchr(line, '*');
    if (star == NULL || star == line) {
        return false;
    }

    size_t digits = strlen(star + 1);
    if (digits != 2 && digits != 8) {
        return false;
    }

    for (const char *p = star + 1; *p != '\0'; p++) {
        if (!isxdigit((unsigned char)*p)) {
            return false;
        }
    }

    return true;
}

static bool console_line_interesting(const char *line)
{
    while (*line == ' ' || *line == '\t') {
        line++;
    }

    if (*line == '\0') {
        return false;
    }

    // Binary RTCM can occasionally contain printable fragments followed by
    // CR/LF-like bytes. Keep receiver text forms, but reject fragments such
    // as "$", "<p", or "$NHnR".
    for (const char *p = line; *p != '\0'; p++) {
        if (!isprint((unsigned char)*p) && *p != '\t') {
            return false;
        }
    }

    if (line[0] == '$' || line[0] == '#') {
        return console_has_checksum_suffix(line);
    }

    if (strcasecmp(line, "OK") == 0 ||
        strncasecmp(line, "ERR", 3) == 0 ||
        strncasecmp(line, "ERROR", 5) == 0 ||
        strncasecmp(line, "WARNING", 7) == 0) {
        return true;
    }

    return false;
}

static bool console_is_likely_line_start(uint8_t b)
{
    return b == '$' || b == '#' ||
           b == 'O' || b == 'o' ||
           b == 'E' || b == 'e' ||
           b == 'W' || b == 'w';
}

static void console_append_locked(const char *data, size_t len)
{
    char *buffer = s_console_buffers[s_console_active_buffer];

    if (len >= GNSS_CONSOLE_BUFFER_SIZE) {
        data += len - GNSS_CONSOLE_BUFFER_SIZE + 1;
        len = GNSS_CONSOLE_BUFFER_SIZE - 1;
    }

    for (size_t i = 0; i < len; i++) {
        buffer[s_console_head] = data[i];
        s_console_head = (s_console_head + 1) % GNSS_CONSOLE_BUFFER_SIZE;
        if (s_console_used < GNSS_CONSOLE_BUFFER_SIZE) {
            s_console_used++;
        }
    }
}

static void console_store_line_locked(void)
{
    if (s_console_line_len == 0) {
        return;
    }

    s_console_line[s_console_line_len] = '\0';
    if (!s_console_line_binary && console_line_interesting(s_console_line)) {
        console_append_locked(s_console_line, s_console_line_len);
        console_append_locked("\n", 1);
    }

    s_console_line_len = 0;
    s_console_line_binary = false;
}

static void console_push_bytes(const uint8_t *data, size_t len)
{
    portENTER_CRITICAL(&s_console_mux);
    if (s_console_suppress_until_us != 0 &&
        (uint64_t)esp_timer_get_time() < s_console_suppress_until_us) {
        s_console_line_len = 0;
        s_console_line_binary = false;
        portEXIT_CRITICAL(&s_console_mux);
        return;
    }

    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];

        if (b == '\r' || b == '\n') {
            console_store_line_locked();
            continue;
        }

        if (b == '\t' || (b >= 0x20 && b <= 0x7E)) {
            if (s_console_line_binary) {
                if (!console_is_likely_line_start(b)) {
                    continue;
                }

                s_console_line_binary = false;
                s_console_line_len = 0;
            }

            if (s_console_line_len + 1 >= sizeof(s_console_line)) {
                console_store_line_locked();
            }
            s_console_line[s_console_line_len++] = (char)b;
        } else {
            // Binary data, typically RTCM, invalidates the current text line.
            s_console_line_binary = true;
            s_console_line_len = 0;
        }
    }
    portEXIT_CRITICAL(&s_console_mux);
}

// ─── Publish a validated RTCM3 frame to all active NTRIP destinations ───────
//
// Acquires ONE pool slot, copies the data in, then enqueues the same pointer
// to every non-NULL queue.  ref_count is set to exactly the number of queues
// that will receive the frame — determined dynamically at publish time so that
// disabled services (NULL queue handles) are never counted.
//
// This means no code change is needed here when services are enabled/disabled
// at startup: main.c simply leaves unused queue handles as NULL.

static void publish_rtcm_frame(const uint8_t *data, size_t len)
{
    last_rtcm_ms = (uint64_t)(esp_timer_get_time() / 1000);
    rtcm_frame_count++;

    int refs = ntrip_client_active_count() + ntrip_caster_active_count();

    if (refs == 0) return;  // no active consumers; discard silently

    pool_frame_t *f = pool_alloc();
    if (f == NULL) return;   // pool_alloc already logged the error

    memcpy(f->data, data, len);
    f->len = len;
    pool_set_refs(f, refs);

    ntrip_client_publish(f);
    ntrip_caster_publish(f);
}

// ─── Frame Splitter State Machine ────────────────────────────────────────────

typedef enum {
    STATE_HUNT,         // scanning for a frame start byte
    STATE_RTCM_HDR,     // collected 0xD3, reading 2 more header bytes
    STATE_RTCM_BODY,    // reading payload + CRC
} splitter_state_t;

typedef struct {
    splitter_state_t state;
    uint8_t          buf[MAX_RTCM_FRAME];
    size_t           buf_pos;
    uint16_t         rtcm_payload_len;
    size_t           rtcm_total_len;       // 3 hdr + payload + 3 CRC
} splitter_t;

static uint8_t replay_buf[MAX_RTCM_FRAME];

static int splitter_push_byte(splitter_t *s, uint8_t b);
static void splitter_process_bytes(splitter_t *s, const uint8_t *data, size_t len);

static void splitter_init(splitter_t *s)
{
    s->state   = STATE_HUNT;
    s->buf_pos = 0;
}

static int splitter_push_byte(splitter_t *s, uint8_t b)
{
restart:
    switch (s->state) {

    // ── HUNT ──────────────────────────────────────────────────────────────────
    case STATE_HUNT:
        if (b == 0xD3) {
            s->buf[0]  = b;
            s->buf_pos = 1;
            s->state   = STATE_RTCM_HDR;
        }
        return -1;

    // ── RTCM3: two remaining header bytes ─────────────────────────────────────
    case STATE_RTCM_HDR:
        s->buf[s->buf_pos++] = b;
        if (s->buf_pos == 3) {
            uint16_t len = ((uint16_t)(s->buf[1] & 0x03) << 8) | s->buf[2];

            if (len == 0 || len > 1023) {
                log_rtcm_bad_len(len);
                return 1;
            }

            s->rtcm_payload_len = len;
            s->rtcm_total_len   = 3 + len + 3;
            s->state            = STATE_RTCM_BODY;
        }
        return -1;

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
                publish_rtcm_frame(s->buf, s->rtcm_total_len);
                splitter_init(s);
            } else {
                (void)received;
                (void)computed;
                log_rtcm_crc_fail();
                return 1;
            }
        }
        return -1;
    }

    return -1;
}

static void splitter_process_bytes(splitter_t *s, const uint8_t *data, size_t len)
{
    const uint8_t *src = data;
    size_t src_len = len;
    size_t i = 0;
    size_t resume_i = 0;

    while (1) {
        while (i < src_len) {
            int replay_start = splitter_push_byte(s, src[i++]);
            if (replay_start < 0) {
                continue;
            }

            size_t replay_len = s->buf_pos > (size_t)replay_start
                              ? s->buf_pos - (size_t)replay_start
                              : 0;
            splitter_init(s);

            if (replay_len == 0) {
                continue;
            }

            memcpy(replay_buf, &s->buf[replay_start], replay_len);

            if (src != replay_buf) {
                resume_i = i;
            }

            src = replay_buf;
            src_len = replay_len;
            i = 0;
        }

        if (src == replay_buf) {
            src = data;
            src_len = len;
            i = resume_i;
            continue;
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
static void task_gnss_reader(void *pvParams)
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

    // The GNSS receiver may already be streaming while the ESP32 is still
    // booting. Start from a clean boundary instead of feeding stale boot-time
    // bytes into the splitter as soon as the task begins servicing UART.
    ESP_ERROR_CHECK(uart_flush_input(uart_port));
    xQueueReset(uart_evt_queue);

    ESP_LOGI(TAG, "UART%d ready @ %d baud TX=%d RX=%d (event-driven)",
             uart_port, uart_cfg.baud_rate,
             pin_tx, pin_rx);
    s_uart_port = uart_port;

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

                // Forward the raw byte stream to the TCP bridge for UPrecise.
                raw_frame.len = (size_t)n;
                post_raw_gnss_data_tcp(&raw_frame);
                console_push_bytes(raw_frame.data, (size_t)n);

                // Extract only validated RTCM3 frames for the NTRIP uplinks and
                // local caster. All other UART bytes are ignored here.
                splitter_process_bytes(&splitter, raw_frame.data, (size_t)n);

                remaining -= (size_t)n;
            }
            break;
        }

        case UART_FIFO_OVF:
            ESP_LOGW(TAG, "HW FIFO overflow - bytes lost, resetting splitter");
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

void gnss_uart_init(void)
{
    if (s_uart_tx_mutex == NULL) {
        s_uart_tx_mutex = xSemaphoreCreateMutex();
        if (s_uart_tx_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create UART TX mutex");
            return;
        }
    }
    xTaskCreatePinnedToCore(task_gnss_reader, "gnss_reader", 4096, NULL, 5, NULL, 1);
}

void gnss_uart_get_diagnostics(gnss_uart_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }

    diag->rtcm_frames = rtcm_frame_count;
    diag->last_rtcm_ms = last_rtcm_ms;
    diag->rtcm_bad_len_count = rtcm_bad_len_count;
    diag->rtcm_crc_fail_count = rtcm_crc_fail_count;
}

esp_err_t gnss_uart_send_command(const char *command, size_t len)
{
    if (command == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_uart_port == UART_NUM_MAX || s_uart_tx_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    while (len > 0 && (command[len - 1] == '\r' || command[len - 1] == '\n')) {
        len--;
    }

    if (len == 0 || len > GNSS_COMMAND_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_uart_tx_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int written = uart_write_bytes(s_uart_port, command, len);
    if (written == (int)len) {
        written += uart_write_bytes(s_uart_port, "\r\n", 2);
    }
    xSemaphoreGive(s_uart_tx_mutex);

    return written == (int)(len + 2) ? ESP_OK : ESP_FAIL;
}

size_t gnss_uart_console_snapshot(char *dest, size_t dest_size)
{
    if (dest == NULL || dest_size == 0) {
        return 0;
    }

    portENTER_CRITICAL(&s_console_mux);
    char *buffer = s_console_buffers[s_console_active_buffer];
    size_t used = s_console_used;
    size_t start = (s_console_head + GNSS_CONSOLE_BUFFER_SIZE - used) % GNSS_CONSOLE_BUFFER_SIZE;
    size_t to_copy = used < dest_size - 1 ? used : dest_size - 1;
    size_t skip = used - to_copy;
    start = (start + skip) % GNSS_CONSOLE_BUFFER_SIZE;

    for (size_t i = 0; i < to_copy; i++) {
        dest[i] = buffer[(start + i) % GNSS_CONSOLE_BUFFER_SIZE];
    }
    portEXIT_CRITICAL(&s_console_mux);

    dest[to_copy] = '\0';
    return to_copy;
}

size_t gnss_uart_console_consume(char *dest, size_t dest_size)
{
    if (dest == NULL || dest_size == 0) {
        return 0;
    }

    char *buffer;
    size_t start;
    size_t to_copy;

    portENTER_CRITICAL(&s_console_mux);
    size_t used = s_console_used;
    start = (s_console_head + GNSS_CONSOLE_BUFFER_SIZE - used) % GNSS_CONSOLE_BUFFER_SIZE;
    to_copy = used < dest_size - 1 ? used : dest_size - 1;
    size_t skip = used - to_copy;
    start = (start + skip) % GNSS_CONSOLE_BUFFER_SIZE;
    buffer = s_console_buffers[s_console_active_buffer];

    s_console_active_buffer ^= 1U;
    s_console_head = 0;
    s_console_used = 0;

    portEXIT_CRITICAL(&s_console_mux);

    for (size_t i = 0; i < to_copy; i++) {
        dest[i] = buffer[(start + i) % GNSS_CONSOLE_BUFFER_SIZE];
    }

    dest[to_copy] = '\0';
    return to_copy;
}

void gnss_uart_console_clear(void)
{
    portENTER_CRITICAL(&s_console_mux);
    s_console_head = 0;
    s_console_used = 0;
    s_console_line_len = 0;
    s_console_line_binary = false;
    // Drop a short burst of already-queued text bytes after clear so stale
    // pre-clear console lines do not immediately repopulate the viewer.
    s_console_suppress_until_us = (uint64_t)esp_timer_get_time() + 500000ULL;
    portEXIT_CRITICAL(&s_console_mux);
}
