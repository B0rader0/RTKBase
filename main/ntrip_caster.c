#include "ntrip_caster.h"
#include "nvs_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include <lwip/tcp.h>
#include <mbedtls/base64.h>
#include <string.h>
#include <strings.h>            // strncasecmp(), strcasecmp() — POSIX
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>              // errno
#include <fcntl.h>

static const char *TAG = "NTRIP_CASTER";

#define MAX_ROVERS              4
#define ROVER_REQ_BUF_SIZE      512
#define HANDSHAKE_SEND_RETRY_MS 5
#define HANDSHAKE_SEND_RETRIES  20
#define TERMINAL_CLOSE_DRAIN_RETRIES 4
#define TERMINAL_CLOSE_DRAIN_MS 5
#define ROVER_STREAM_SEND_RETRY_MS 2
#define ROVER_STREAM_SEND_RETRIES 5
#define NTRIP_CASTER_DIAG_LOG_MASK 0x0F
#define NTRIP_CASTER_TASK_STACK_SIZE 5120
#define NTRIP_CASTER_TASK_PRIORITY   4
#define NTRIP_CASTER_TASK_CORE       0

// ─── Per-rover connection state ───────────────────────────────────────────────

typedef enum {
    ROVER_EMPTY,
    ROVER_HANDSHAKE,
    ROVER_STREAMING,
} rover_state_t;

typedef struct {
    int           fd;
    rover_state_t state;
    char          rx_buf[ROVER_REQ_BUF_SIZE];
    int           rx_pos;
    bool          is_ntrip_v2;
    uint8_t       tx_pending[MAX_RTCM_FRAME];
    size_t        tx_pending_len;
    size_t        tx_pending_pos;
} rover_conn_t;

typedef enum {
    ROVER_CLOSE_PEER_CLOSED,
    ROVER_CLOSE_RECV_ERROR,
    ROVER_CLOSE_HANDSHAKE_FAILED,
    ROVER_CLOSE_REQUEST_TOO_LARGE,
    ROVER_CLOSE_SEND_ERROR,
} rover_close_reason_t;

static rover_conn_t  s_rovers[MAX_ROVERS];
static SemaphoreHandle_t s_rovers_mutex;
static QueueHandle_t s_q_ntrip_caster;
static ntrip_caster_diag_t s_diag;

typedef struct {
    bool     loaded;
    bool     auth_required;
    uint16_t port;
    char     mountpoint[33];
    char     identifier[33];
    char     username[33];
    char     password[65];
} caster_config_t;

static caster_config_t s_cfg;
static void close_rover(rover_conn_t *rover, int index, rover_close_reason_t reason, int err);

// ─── Helpers ──────────────────────────────────────────────────────────────────

static const char *stristr(const char *hay, const char *needle)
{
    size_t nl = strlen(needle);
    for (; *hay; hay++) {
        if (strncasecmp(hay, needle, nl) == 0) return hay;
    }
    return NULL;
}

static bool load_cfg_string(const char *key, char *dst, size_t dst_size)
{
    char *tmp = NULL;
    esp_err_t err = cfg_get_str(key, &tmp);
    if (err != ESP_OK || tmp == NULL) {
        ESP_LOGE(TAG, "cfg_get_str(%s) failed: %s", key, esp_err_to_name(err));
        if (tmp != NULL) {
            free(tmp);
        }
        dst[0] = '\0';
        return false;
    }

    strlcpy(dst, tmp, dst_size);
    free(tmp);
    return true;
}

static void sanitize_sourcetable_field(char *value)
{
    for (char *p = value; *p != '\0'; ++p) {
        if (*p == ';' || *p == '\r' || *p == '\n') {
            *p = '_';
        }
    }
}

static bool load_caster_config(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));

    esp_err_t err = cfg_get_u16(KEY_CONFIG_CASTER_PORT, &s_cfg.port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cfg_get_u16(%s) failed: %s",
                 KEY_CONFIG_CASTER_PORT, esp_err_to_name(err));
        return false;
    }

    if (!load_cfg_string(KEY_CONFIG_CASTER_MOUNTPOINT,
                         s_cfg.mountpoint, sizeof(s_cfg.mountpoint))) {
        return false;
    }

    if (!load_cfg_string(KEY_CONFIG_STATION_HOSTNAME,
                         s_cfg.identifier, sizeof(s_cfg.identifier))) {
        strlcpy(s_cfg.identifier, "RTK Base", sizeof(s_cfg.identifier));
    }
    if (s_cfg.identifier[0] == '\0') {
        strlcpy(s_cfg.identifier, "RTK Base", sizeof(s_cfg.identifier));
    }
    sanitize_sourcetable_field(s_cfg.identifier);

    if (!load_cfg_string(KEY_CONFIG_CASTER_USERNAME,
                         s_cfg.username, sizeof(s_cfg.username))) {
        return false;
    }

    if (!load_cfg_string(KEY_CONFIG_CASTER_PASSWORD,
                         s_cfg.password, sizeof(s_cfg.password))) {
        return false;
    }

    s_cfg.auth_required = (s_cfg.password[0] != '\0');
    s_cfg.loaded = true;
    return true;
}

static bool send_all_nonblocking(int fd, const char *data, size_t len)
{
    size_t offset = 0;
    int retries = 0;

    while (offset < len) {
        int sent = send(fd, data + offset, len - offset, MSG_DONTWAIT);
        if (sent > 0) {
            offset += (size_t)sent;
            retries = 0;
            continue;
        }

        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) &&
            retries++ < HANDSHAKE_SEND_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_SEND_RETRY_MS));
            continue;
        }

        return false;
    }

    return true;
}

static void drain_and_shutdown_write(int fd)
{
    if (fd < 0) {
        return;
    }

    shutdown(fd, SHUT_WR);

    for (int i = 0; i < TERMINAL_CLOSE_DRAIN_RETRIES; i++) {
        uint8_t tmp[64];
        int n = recv(fd, tmp, sizeof(tmp), MSG_DONTWAIT);
        if (n == 0) {
            break;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(pdMS_TO_TICKS(TERMINAL_CLOSE_DRAIN_MS));
                continue;
            }
            break;
        }
    }
}

static bool respond_and_half_close(int fd, const char *data, size_t len)
{
    bool ok = send_all_nonblocking(fd, data, len);
    drain_and_shutdown_write(fd);
    return ok;
}

static bool respond_pair_and_half_close(int fd,
                                        const char *data1, size_t len1,
                                        const char *data2, size_t len2)
{
    bool ok = send_all_nonblocking(fd, data1, len1);
    if (ok) {
        ok = send_all_nonblocking(fd, data2, len2);
    }
    drain_and_shutdown_write(fd);
    return ok;
}

static void caster_log_queue_drop(void)
{
    s_diag.queue_drops++;
    if ((s_diag.queue_drops & NTRIP_CASTER_DIAG_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "Caster queue drop count=%lu queue=%lu/%u",
                 (unsigned long)s_diag.queue_drops,
                 (unsigned long)(s_q_ntrip_caster ? uxQueueMessagesWaiting(s_q_ntrip_caster) : 0),
                 (unsigned)FRAME_QUEUE_DEPTH);
    }
}

static void caster_log_rover_block(int rover_index, size_t remaining)
{
    s_diag.rover_block_events++;
    if ((s_diag.rover_block_events & NTRIP_CASTER_DIAG_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "Rover %d blocked, remaining=%uB count=%lu",
                 rover_index, (unsigned)remaining,
                 (unsigned long)s_diag.rover_block_events);
    }
}

static void caster_log_rover_slow_drop(int rover_index)
{
    s_diag.rover_slow_drops++;
    if ((s_diag.rover_slow_drops & NTRIP_CASTER_DIAG_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "Rover %d slow, dropped whole frame count=%lu",
                 rover_index, (unsigned long)s_diag.rover_slow_drops);
    }
}

static const char *rover_close_reason_text(rover_close_reason_t reason)
{
    switch (reason) {
    case ROVER_CLOSE_PEER_CLOSED:
        return "peer closed connection";
    case ROVER_CLOSE_RECV_ERROR:
        return "receive error";
    case ROVER_CLOSE_HANDSHAKE_FAILED:
        return "handshake failed";
    case ROVER_CLOSE_REQUEST_TOO_LARGE:
        return "request too large";
    case ROVER_CLOSE_SEND_ERROR:
        return "send error";
    }

    return "unknown";
}

static int send_bytes_to_rover(rover_conn_t *rover,
                               int rover_index,
                               const uint8_t *data,
                               size_t len,
                               size_t *offset)
{
    int retries = 0;

    while (*offset < len) {
        size_t remaining = len - *offset;
        int sent = send(rover->fd, data + *offset, remaining, MSG_DONTWAIT);
        if (sent > 0) {
            *offset += (size_t)sent;
            retries = 0;
            continue;
        }

        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (retries == 0) {
                caster_log_rover_block(rover_index, remaining);
            }
            if (++retries > ROVER_STREAM_SEND_RETRIES) {
                return 0;
            }
            vTaskDelay(pdMS_TO_TICKS(ROVER_STREAM_SEND_RETRY_MS));
            continue;
        }

        s_diag.rover_send_errors++;
        close_rover(rover, rover_index, ROVER_CLOSE_SEND_ERROR, errno);
        return -1;
    }

    return 1;
}

static bool flush_rover_pending(rover_conn_t *rover, int rover_index)
{
    if (rover->tx_pending_pos >= rover->tx_pending_len) {
        rover->tx_pending_len = 0;
        rover->tx_pending_pos = 0;
        return true;
    }

    int result = send_bytes_to_rover(rover, rover_index,
                                     rover->tx_pending,
                                     rover->tx_pending_len,
                                     &rover->tx_pending_pos);
    if (result < 0) {
        rover->tx_pending_len = 0;
        rover->tx_pending_pos = 0;
        return false;
    }
    if (result == 0) {
        return false;
    }

    rover->tx_pending_len = 0;
    rover->tx_pending_pos = 0;
    return true;
}

static bool send_frame_to_rover(rover_conn_t *rover, int rover_index, const uint8_t *data, size_t len)
{
    if (!flush_rover_pending(rover, rover_index)) {
        if (rover->state == ROVER_STREAMING) {
            caster_log_rover_slow_drop(rover_index);
        }
        return false;
    }

    size_t offset = 0;
    int result = send_bytes_to_rover(rover, rover_index, data, len, &offset);
    if (result > 0) {
        return true;
    }

    if (result < 0 || rover->state != ROVER_STREAMING) {
        return false;
    }

    if (offset == 0) {
        caster_log_rover_slow_drop(rover_index);
        return false;
    }

    size_t remaining = len - offset;
    memcpy(rover->tx_pending, data + offset, remaining);
    rover->tx_pending_len = remaining;
    rover->tx_pending_pos = 0;
    ESP_LOGW(TAG, "Rover %d deferred frame tail, pending=%uB",
             rover_index, (unsigned)remaining);
    return true;
}

static bool auth_valid(const char *b64)
{
    if (!s_cfg.loaded) {
        return false;
    }

    if (!s_cfg.auth_required) {
        return true;
    }

    unsigned char decoded[128];
    size_t out_len = 0;
    if (mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &out_len,
                              (const unsigned char *)b64, strlen(b64)) != 0) {
        return false;
    }
    decoded[out_len] = '\0';

    char expected[128];
    snprintf(expected, sizeof(expected), "%s:%s",
             s_cfg.username, s_cfg.password);
    bool ok = strcmp((char *)decoded, expected) == 0;
    return ok;
}

// ─── Sourcetable ─────────────────────────────────────────────────────────────

static bool send_sourcetable(int fd, bool v2)
{
    char str_record[256];
    int str_len = snprintf(str_record, sizeof(str_record),
        "STR;%s;%s;RTCM 3.2;"
        "1005(1),1074(1),1084(1),1094(1),1124(1);"
        "2;GPS+GLO+GAL+BDS;ESP32;DE;48.10;11.60;1;1;sNTRIP;none;N;N;0;\r\n"
        "ENDSOURCETABLE\r\n",
        s_cfg.mountpoint, s_cfg.identifier);

    char header[256];
    int hdr_len;
    if (v2) {
        hdr_len = snprintf(header, sizeof(header),
            "HTTP/1.1 200 OK\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: %d\r\n\r\n", str_len);
    } else {
        hdr_len = snprintf(header, sizeof(header),
            "SOURCETABLE 200 OK\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: %d\r\n\r\n", str_len);
    }
    return respond_pair_and_half_close(fd,
                                       header, (size_t)hdr_len,
                                       str_record, (size_t)str_len);
}

// ─── HTTP request handler ─────────────────────────────────────────────────────

static bool handle_request(rover_conn_t *rover)
{
    const char *req = rover->rx_buf;
    rover->is_ntrip_v2 = (stristr(req, "Ntrip-Version: Ntrip/2.0") != NULL);

    char method[8] = {0}, path[64] = {0};
    sscanf(req, "%7s %63s", method, path);

    if (strcasecmp(method, "GET") != 0) {
        const char *r = "HTTP/1.1 400 Bad Request\r\n\r\n";
        respond_and_half_close(rover->fd, r, strlen(r));
        return false;
    }

    if (strcmp(path, "/") == 0) {
        send_sourcetable(rover->fd, rover->is_ntrip_v2);
        return false;
    }

    char expected[72];
    snprintf(expected, sizeof(expected), "/%s", s_cfg.mountpoint);
    if (strcasecmp(path, expected) != 0) {
        const char *r = "HTTP/1.1 404 Not Found\r\n\r\n";
        respond_and_half_close(rover->fd, r, strlen(r));
        ESP_LOGW(TAG, "Unknown mountpoint: %s", path);
        return false;
    }

    // ── Auth ──────────────────────────────────────────────────────────────────
    if (s_cfg.auth_required) {
        const char *auth_hdr = stristr(req, "Authorization: Basic ");
        if (!auth_hdr) {
            const char *r = "HTTP/1.1 401 Unauthorized\r\n"
                            "WWW-Authenticate: Basic realm=\"NTRIP\"\r\n\r\n";
            respond_and_half_close(rover->fd, r, strlen(r));
            ESP_LOGW(TAG, "Rover provided no credentials");
            return false;
        }

        auth_hdr += strlen("Authorization: Basic ");
        char token[128] = {0};
        const char *end = strstr(auth_hdr, "\r");
        if (end) {
            size_t tlen = (size_t)(end - auth_hdr);
            if (tlen >= sizeof(token)) tlen = sizeof(token) - 1;
            memcpy(token, auth_hdr, tlen);
        }
        if (!auth_valid(token)) {
            const char *r = "HTTP/1.1 401 Unauthorized\r\n"
                            "WWW-Authenticate: Basic realm=\"NTRIP\"\r\n\r\n";
            respond_and_half_close(rover->fd, r, strlen(r));
            ESP_LOGW(TAG, "Rover failed authentication");
            return false;
        }
    }

    // ── Accept ────────────────────────────────────────────────────────────────
    if (rover->is_ntrip_v2) {
        const char *r = "HTTP/1.1 200 OK\r\n"
                        "Ntrip-Version: Ntrip/2.0\r\n"
                        "Content-Type: gnss/data\r\n\r\n";
        send_all_nonblocking(rover->fd, r, strlen(r));
    } else {
        const char *r = "ICY 200 OK\r\n\r\n";
        send_all_nonblocking(rover->fd, r, strlen(r));
    }

    ESP_LOGI(TAG, "Rover accepted on /%s (NTRIPv%d)",
             s_cfg.mountpoint, rover->is_ntrip_v2 ? 2 : 1);

    return true;
} //handle_request

// ─── Close a rover slot ───────────────────────────────────────────────────────

static void close_rover(rover_conn_t *rover, int index, rover_close_reason_t reason, int err)
{
    size_t pending = rover->tx_pending_len > rover->tx_pending_pos
                   ? rover->tx_pending_len - rover->tx_pending_pos
                   : 0;
    if (err != 0) {
        ESP_LOGW(TAG, "Rover %d disconnected: %s errno=%d pending=%uB",
                 index, rover_close_reason_text(reason), err, (unsigned)pending);
    } else {
        ESP_LOGI(TAG, "Rover %d disconnected: %s pending=%uB",
                 index, rover_close_reason_text(reason), (unsigned)pending);
    }
    if (rover->fd >= 0) {
        close(rover->fd);
    }
    rover->fd     = -1;
    rover->state  = ROVER_EMPTY;
    rover->rx_pos = 0;
    rover->is_ntrip_v2 = false;
    rover->tx_pending_len = 0;
    rover->tx_pending_pos = 0;
}

// ─── NTRIP Server Task ────────────────────────────────────────────────────────

static void task_ntrip_server(void *arg)
{
    memset(&s_diag, 0, sizeof(s_diag));

    s_rovers_mutex = xSemaphoreCreateMutex();
    if (s_rovers_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create rover mutex");
        vTaskDelete(NULL);
        return;
    }
    for (int i = 0; i < MAX_ROVERS; i++) {
        s_rovers[i].fd    = -1;
        s_rovers[i].state = ROVER_EMPTY;
        s_rovers[i].rx_pos = 0;
        s_rovers[i].is_ntrip_v2 = false;
        s_rovers[i].tx_pending_len = 0;
        s_rovers[i].tx_pending_pos = 0;
    }
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) != 0) {
        ESP_LOGW(TAG, "setsockopt(SO_REUSEADDR) failed: errno=%d", errno);
    }

    if (!s_cfg.loaded) {
        ESP_LOGE(TAG, "Caster configuration not loaded");
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    
    struct sockaddr_in srv = {
        .sin_family      = AF_INET,
        .sin_port        = htons(s_cfg.port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_fd, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_fd, MAX_ROVERS) != 0) {
        ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "NTRIP server listening on port %u, mountpoint /%s",
             (unsigned)s_cfg.port, s_cfg.mountpoint);

    while (1) {
        // ── fd_set ────────────────────────────────────────────────────────────
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(listen_fd, &read_fds);
        int max_fd = listen_fd;
        for (int i = 0; i < MAX_ROVERS; i++) {
            if (s_rovers[i].state != ROVER_EMPTY) {
                FD_SET(s_rovers[i].fd, &read_fds);
                if (s_rovers[i].fd > max_fd) max_fd = s_rovers[i].fd;
            }
        }

        struct timeval tv = { .tv_sec = 0, .tv_usec = 1000 };
        select(max_fd + 1, &read_fds, NULL, NULL, &tv);

        // ── Accept ────────────────────────────────────────────────────────────
        if (FD_ISSET(listen_fd, &read_fds)) {
            struct sockaddr_in cli_addr;
            socklen_t cli_len = sizeof(cli_addr);
            int new_fd = accept(listen_fd, (struct sockaddr *)&cli_addr, &cli_len);
            if (new_fd >= 0) {
                bool accepted = false;
                for (int i = 0; i < MAX_ROVERS; i++) {
                    if (s_rovers[i].state == ROVER_EMPTY) {
                        int flag = 1;
                        setsockopt(new_fd, IPPROTO_TCP, TCP_NODELAY,
                                   &flag, sizeof(flag));
                        int fl = fcntl(new_fd, F_GETFL, 0);
                        if (fl >= 0) {
                            fcntl(new_fd, F_SETFL, fl | O_NONBLOCK);
                        }

                        s_rovers[i].fd     = new_fd;
                        s_rovers[i].state  = ROVER_HANDSHAKE;
                        s_rovers[i].rx_pos = 0;
                        s_rovers[i].tx_pending_len = 0;
                        s_rovers[i].tx_pending_pos = 0;
                        memset(s_rovers[i].rx_buf, 0, sizeof(s_rovers[i].rx_buf));
                        char ip_str[16];
                        inet_ntop(AF_INET, &cli_addr.sin_addr, ip_str, sizeof(ip_str));
                        ESP_LOGI(TAG, "Rover %d connecting from %s", i, ip_str);
                        accepted = true;
                        break;
                    }
                }
                if (!accepted) {
                    const char *busy = "HTTP/1.1 503 Service Unavailable\r\n\r\n";
                    respond_and_half_close(new_fd, busy, strlen(busy));
                    close(new_fd);
                    ESP_LOGW(TAG, "Max rovers reached, connection refused");
                }
            }
        }

        // ── Service connected rovers ──────────────────────────────────────────
        for (int i = 0; i < MAX_ROVERS; i++) {
            rover_conn_t *r = &s_rovers[i];
            if (r->state == ROVER_EMPTY) continue;

            if (FD_ISSET(r->fd, &read_fds)) {
                if (r->state == ROVER_HANDSHAKE) {
                    int n = recv(r->fd,
                                 r->rx_buf + r->rx_pos,
                                 sizeof(r->rx_buf) - r->rx_pos - 1, 0);
                    if (n <= 0) {
                        close_rover(r, i,
                                    n == 0 ? ROVER_CLOSE_PEER_CLOSED : ROVER_CLOSE_RECV_ERROR,
                                    n == 0 ? 0 : errno);
                        continue;
                    }
                    r->rx_pos += n;
                    r->rx_buf[r->rx_pos] = '\0';
                    if (strstr(r->rx_buf, "\r\n\r\n")) {
                        if (handle_request(r)) {
                            r->state = ROVER_STREAMING;
                        } else {
                            close_rover(r, i, ROVER_CLOSE_HANDSHAKE_FAILED, 0);
                        }
                    } else if (r->rx_pos >= (int)sizeof(r->rx_buf) - 1) {
                        close_rover(r, i, ROVER_CLOSE_REQUEST_TOO_LARGE, 0);
                    }
                } else if (r->state == ROVER_STREAMING) {
                    // Incoming GGA from rover (for VRS future use) — drain cleanly
                    uint8_t tmp[64];
                    int n = recv(r->fd, tmp, sizeof(tmp), MSG_DONTWAIT);
                    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                        close_rover(r, i,
                                    n == 0 ? ROVER_CLOSE_PEER_CLOSED : ROVER_CLOSE_RECV_ERROR,
                                    n == 0 ? 0 : errno);
                    }
                }
            }
        }

        // ── Forward RTCM3 frames to streaming rovers ──────────────────────────
        pool_frame_t *f;
        while (xQueueReceive(s_q_ntrip_caster, &f, 0) == pdTRUE) {
            xSemaphoreTake(s_rovers_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_ROVERS; i++) {
                rover_conn_t *r = &s_rovers[i];
                if (r->state != ROVER_STREAMING) continue;
                send_frame_to_rover(r, i, f->data, f->len);
            }
            xSemaphoreGive(s_rovers_mutex);
            pool_release(f);    // done with this frame
        }
    }
}

// ─── Helper: count active streaming rovers ───────────────────────────────────

void ntrip_caster_init(void)
{
    uint8_t enabled = 0;
    esp_err_t err = cfg_get_u8(KEY_CONFIG_CASTER_ACTIVE, &enabled);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read caster enable flag");
        return;
    }
    if (!enabled) {
        ESP_LOGI(TAG, "Caster disabled in configuration");
        return;
    }

    if (!load_caster_config()) {
        ESP_LOGE(TAG, "Failed to load caster configuration");
        return;
    }

    s_q_ntrip_caster = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(pool_frame_t *));
    if (s_q_ntrip_caster == NULL) {
        ESP_LOGE(TAG, "Failed to create caster queue");
        return;
    }
    BaseType_t created = xTaskCreatePinnedToCore(task_ntrip_server, "ntrip_srv",
                                                 NTRIP_CASTER_TASK_STACK_SIZE, NULL,
                                                 NTRIP_CASTER_TASK_PRIORITY, NULL,
                                                 NTRIP_CASTER_TASK_CORE);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create NTRIP caster task");
        vQueueDelete(s_q_ntrip_caster);
        s_q_ntrip_caster = NULL;
    }
}

int ntrip_caster_active_count(void)
{
    return s_q_ntrip_caster != NULL ? 1 : 0;
}

void ntrip_caster_publish(pool_frame_t *f)
{
    if (s_q_ntrip_caster == NULL) {
        return;
    }

    if (xQueueSend(s_q_ntrip_caster, &f, 0) != pdTRUE) {
        caster_log_queue_drop();
        pool_release(f);
    }
}

int ntrip_caster_rover_count(void)
{
    int count = 0;
    if (s_rovers_mutex != NULL &&
        xSemaphoreTake(s_rovers_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < MAX_ROVERS; i++) {
            if (s_rovers[i].state == ROVER_STREAMING) count++;
        }
        xSemaphoreGive(s_rovers_mutex);
    }
    return count;
}

void ntrip_caster_get_diagnostics(ntrip_caster_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }

    *diag = s_diag;
    diag->queue_fill = s_q_ntrip_caster ? (uint32_t)uxQueueMessagesWaiting(s_q_ntrip_caster) : 0;
    diag->queue_depth = FRAME_QUEUE_DEPTH;
    diag->active_rovers = (uint32_t)ntrip_caster_rover_count();
}
