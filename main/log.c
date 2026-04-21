#include "log.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <esp_log.h>
#include <esp_log_write.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#define WEB_LOG_BUFFER_SIZE 16384
#define WEB_LOG_LINE_SIZE 512
#define WEB_LOG_INITIAL_LINE "R (00:00:00.000) ESP32: Device Restarted\n"

static char s_log_buffer[WEB_LOG_BUFFER_SIZE];
static size_t s_log_head;
static size_t s_log_used;
static char s_pending_line[WEB_LOG_LINE_SIZE];
static size_t s_pending_len;
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_prev_vprintf;

static const char *skip_ansi_escape(const char *text)
{
    while (text[0] == '\033' && text[1] == '[') {
        text += 2;
        while (*text != '\0' && (*text < '@' || *text > '~')) {
            text++;
        }
        if (*text != '\0') {
            text++;
        }
    }

    return text;
}

static bool should_store_line(const char *line)
{
    line = skip_ansi_escape(line);
    return (line[0] == 'W' || line[0] == 'E') && line[1] == ' ';
}

static char strip_ansi_escape(const char **cursor)
{
    const char *text = *cursor;
    if (text[0] == '\033' && text[1] == '[') {
        text += 2;
        while (*text != '\0' && (*text < '@' || *text > '~')) {
            text++;
        }
        if (*text != '\0') {
            text++;
        }
        *cursor = text;
        return '\0';
    }

    char c = *text++;
    *cursor = text;
    return c;
}

static size_t sanitize_line(char *line)
{
    char clean[WEB_LOG_LINE_SIZE];
    size_t out = 0;
    const char *cursor = line;

    while (*cursor != '\0' && out + 1 < sizeof(clean)) {
        char c = strip_ansi_escape(&cursor);
        if (c == '\0') {
            continue;
        }
        if (c == '\r') {
            continue;
        }
        clean[out++] = c;
    }

    if (out == 0 || clean[out - 1] != '\n') {
        if (out + 1 < sizeof(clean)) {
            clean[out++] = '\n';
        }
    }

    memcpy(line, clean, out);
    line[out] = '\0';
    return out;
}

static void append_to_log_buffer_locked(const char *data, size_t len)
{
    if (len >= WEB_LOG_BUFFER_SIZE) {
        data += len - WEB_LOG_BUFFER_SIZE + 1;
        len = WEB_LOG_BUFFER_SIZE - 1;
    }

    for (size_t i = 0; i < len; i++) {
        s_log_buffer[s_log_head] = data[i];
        s_log_head = (s_log_head + 1) % WEB_LOG_BUFFER_SIZE;
        if (s_log_used < WEB_LOG_BUFFER_SIZE) {
            s_log_used++;
        }
    }
}

static void append_to_log_buffer(const char *data, size_t len)
{
    portENTER_CRITICAL(&s_log_mux);
    append_to_log_buffer_locked(data, len);
    portEXIT_CRITICAL(&s_log_mux);
}

static void store_pending_line_locked(void)
{
    if (s_pending_len == 0) {
        return;
    }

    s_pending_line[s_pending_len] = '\0';
    size_t len = sanitize_line(s_pending_line);
    if (should_store_line(s_pending_line)) {
        append_to_log_buffer_locked(s_pending_line, len);
    }
    s_pending_len = 0;
}

static void append_formatted_text(const char *text)
{
    const char *cursor = text;

    portENTER_CRITICAL(&s_log_mux);
    while (*cursor != '\0') {
        char c = strip_ansi_escape(&cursor);
        if (c == '\0' || c == '\r') {
            continue;
        }

        if (s_pending_len + 1 >= sizeof(s_pending_line)) {
            s_pending_line[s_pending_len++] = '\n';
            store_pending_line_locked();
        }

        s_pending_line[s_pending_len++] = c;
        if (c == '\n') {
            store_pending_line_locked();
        }
    }
    portEXIT_CRITICAL(&s_log_mux);
}

static int log_vprintf(const char *format, va_list args)
{
    va_list copy;
    va_copy(copy, args);

    int ret = s_prev_vprintf ? s_prev_vprintf(format, args) : vprintf(format, args);

    char text[WEB_LOG_LINE_SIZE];
    int written = vsnprintf(text, sizeof(text), format, copy);
    va_end(copy);

    if (written > 0) {
        size_t len = (size_t)written;
        if (len >= sizeof(text)) {
            len = sizeof(text) - 1;
            text[len] = '\0';
        }
        append_formatted_text(text);
    }

    return ret;
}

esp_err_t log_init(void)
{
    append_to_log_buffer(WEB_LOG_INITIAL_LINE, strlen(WEB_LOG_INITIAL_LINE));
    s_prev_vprintf = esp_log_set_vprintf(log_vprintf);
    return ESP_OK;
}

size_t log_snapshot(char *dest, size_t dest_size)
{
    if (dest == NULL || dest_size == 0) {
        return 0;
    }

    portENTER_CRITICAL(&s_log_mux);
    size_t used = s_log_used;
    size_t start = (s_log_head + WEB_LOG_BUFFER_SIZE - used) % WEB_LOG_BUFFER_SIZE;
    size_t to_copy = used < dest_size - 1 ? used : dest_size - 1;
    size_t skip = used - to_copy;
    start = (start + skip) % WEB_LOG_BUFFER_SIZE;

    for (size_t i = 0; i < to_copy; i++) {
        dest[i] = s_log_buffer[(start + i) % WEB_LOG_BUFFER_SIZE];
    }
    portEXIT_CRITICAL(&s_log_mux);

    dest[to_copy] = '\0';
    return to_copy;
}
