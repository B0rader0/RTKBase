#include "ntrip_client.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "mbedtls/base64.h"
#include "nvs_config.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "NTRIP_CLIENT";

#define NTRIP_UPLINK_COUNT        2
#define NTRIP_REQ_BUF_SIZE        512
#define NTRIP_RESP_BUF_SIZE       256
#define NTRIP_RECONNECT_MS_MIN    2000
#define NTRIP_RECONNECT_MS_MAX    60000
#define NTRIP_SEND_RETRY_MS       10
#define NTRIP_SEND_RETRY_LIMIT    20
#define NTRIP_CLIENT_DIAG_LOG_MASK 0x0F
#define NTRIP_TASK_STACK_SIZE     5120
#define NTRIP_TASK_PRIORITY       4
#define NTRIP_TASK_CORE           0

typedef struct {
    bool         enabled;
    char        *host;
    uint16_t     port;
    char        *mountpoint;
    char        *username;
    char        *password;
    QueueHandle_t queue;
    const char  *task_name;
    const char  *log_name;
} ntrip_uplink_t;

static ntrip_uplink_t s_uplinks[NTRIP_UPLINK_COUNT];
static ntrip_client_diag_t s_diag;

static int uplink_index(const ntrip_uplink_t *uplink)
{
    return (int)(uplink - s_uplinks);
}

static void uplink_set_connected(const ntrip_uplink_t *uplink, bool connected)
{
    int index = uplink_index(uplink);
    if (index < 0 || index >= NTRIP_UPLINK_COUNT) {
        return;
    }
    if (connected) {
        s_diag.connected_mask |= (1u << index);
    } else {
        s_diag.connected_mask &= ~(1u << index);
    }
}

static void uplink_log_queue_drop(const ntrip_uplink_t *uplink)
{
    int index = uplink_index(uplink);
    s_diag.queue_drops[index]++;
    if ((s_diag.queue_drops[index] & NTRIP_CLIENT_DIAG_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "[%s] Queue full, dropping frame count=%lu queue=%lu/%u",
                 uplink->log_name,
                 (unsigned long)s_diag.queue_drops[index],
                 (unsigned long)(uplink->queue ? uxQueueMessagesWaiting(uplink->queue) : 0),
                 (unsigned)FRAME_QUEUE_DEPTH);
    }
}

static void uplink_log_blocked(const ntrip_uplink_t *uplink, size_t remaining)
{
    int index = uplink_index(uplink);
    s_diag.send_block_events[index]++;
    if ((s_diag.send_block_events[index] & NTRIP_CLIENT_DIAG_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "[%s] send blocked, remaining=%uB count=%lu",
                 uplink->log_name, (unsigned)remaining,
                 (unsigned long)s_diag.send_block_events[index]);
    }
}

static void uplink_close_socket(const ntrip_uplink_t *uplink,
                                int *sock,
                                const char *reason,
                                int err)
{
    if (*sock < 0) {
        return;
    }

    if (err != 0) {
        ESP_LOGW(TAG, "[%s] reconnecting: %s errno=%d",
                 uplink->log_name, reason, err);
    } else {
        ESP_LOGW(TAG, "[%s] reconnecting: %s", uplink->log_name, reason);
    }

    close(*sock);
    *sock = -1;
    uplink_set_connected(uplink, false);
}

static void base64_encode_str(const char *input, char *output, size_t out_size)
{
    size_t written = 0;
    if (mbedtls_base64_encode((unsigned char *)output, out_size, &written,
                              (const unsigned char *)input, strlen(input)) != 0 ||
        written >= out_size) {
        output[0] = '\0';
        return;
    }
    output[written] = '\0';
}

static int open_connected_socket(const ntrip_uplink_t *uplink)
{
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res = NULL;
    char port_str[8];

    snprintf(port_str, sizeof(port_str), "%u", uplink->port);
    if (getaddrinfo(uplink->host, port_str, &hints, &res) != 0 || res == NULL) {
        ESP_LOGE(TAG, "[%s] DNS resolve failed for %s", uplink->log_name, uplink->host);
        return -1;
    }

    int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        freeaddrinfo(res);
        return -1;
    }

    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "[%s] connect() to %s:%u failed: errno=%d",
                 uplink->log_name, uplink->host, uplink->port, errno);
        freeaddrinfo(res);
        close(sock);
        return -1;
    }

    freeaddrinfo(res);
    return sock;
}

static bool socket_set_nonblocking(int sock)
{
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0) {
        ESP_LOGE(TAG, "[sock=%d] F_GETFL failed: errno=%d", sock, errno);
        return false;
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) != 0) {
        ESP_LOGE(TAG, "[sock=%d] F_SETFL O_NONBLOCK failed: errno=%d", sock, errno);
        return false;
    }
    return true;
}

static bool send_all(int sock, const char *data, size_t len)
{
    size_t offset = 0;
    while (offset < len) {
        int sent = send(sock, data + offset, len - offset, 0);
        if (sent <= 0) {
            return false;
        }
        offset += (size_t)sent;
    }
    return true;
}

static int recv_response(int sock, char *resp, size_t resp_size)
{
    int n = recv(sock, resp, resp_size - 1, 0);
    if (n <= 0) {
        return n;
    }
    resp[n] = '\0';
    return n;
}

static bool response_is_success(const char *resp)
{
    return strstr(resp, " 200 ") != NULL ||
           strstr(resp, " 200\r\n") != NULL ||
           strstr(resp, "ICY 200") != NULL;
}

static int ntrip_connect_v2(const ntrip_uplink_t *uplink)
{
    int sock = open_connected_socket(uplink);
    if (sock < 0) {
        return -1;
    }

    char credentials[128] = "";
    char b64[192] = "";
    char request[NTRIP_REQ_BUF_SIZE];

    if (uplink->username != NULL || uplink->password != NULL) {
        snprintf(credentials, sizeof(credentials), "%s:%s",
                 uplink->username ? uplink->username : "",
                 uplink->password ? uplink->password : "");
        base64_encode_str(credentials, b64, sizeof(b64));
    }

    int req_len;
    if (b64[0]) {
        req_len = snprintf(
            request, sizeof(request),
            "POST /%s HTTP/1.1\r\n"
            "Host: %s:%u\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: NTRIP RTKBase/1.0\r\n"
            "Authorization: Basic %s\r\n"
            "Content-Type: gnss/data\r\n"
            "\r\n",
            uplink->mountpoint, uplink->host, uplink->port, b64);
    } else {
        req_len = snprintf(
            request, sizeof(request),
            "POST /%s HTTP/1.1\r\n"
            "Host: %s:%u\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: NTRIP RTKBase/1.0\r\n"
            "Content-Type: gnss/data\r\n"
            "\r\n",
            uplink->mountpoint, uplink->host, uplink->port);
    }

    if (req_len <= 0 || (size_t)req_len >= sizeof(request) ||
        !send_all(sock, request, (size_t)req_len)) {
        close(sock);
        return -1;
    }

    char resp[NTRIP_RESP_BUF_SIZE];
    int n = recv_response(sock, resp, sizeof(resp));
    if (n <= 0) {
        ESP_LOGW(TAG, "[%s] No response to NTRIP v2 uplink request", uplink->log_name);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "[%s] NTRIP v2 response: %.80s", uplink->log_name, resp);
    if (!response_is_success(resp) || !socket_set_nonblocking(sock)) {
        close(sock);
        return -1;
    }

    return sock;
}

static int ntrip_connect_v1_source(const ntrip_uplink_t *uplink)
{
    if (uplink->password == NULL || uplink->password[0] == '\0') {
        return -1;
    }

    int sock = open_connected_socket(uplink);
    if (sock < 0) {
        return -1;
    }

    char request[NTRIP_REQ_BUF_SIZE];
    int req_len = snprintf(
        request, sizeof(request),
        "SOURCE %s /%s\r\n"
        "Source-Agent: NTRIP RTKBase/1.0\r\n"
        "\r\n",
        uplink->password, uplink->mountpoint);

    if (req_len <= 0 || (size_t)req_len >= sizeof(request) ||
        !send_all(sock, request, (size_t)req_len)) {
        close(sock);
        return -1;
    }

    char resp[NTRIP_RESP_BUF_SIZE];
    int n = recv_response(sock, resp, sizeof(resp));
    if (n <= 0) {
        ESP_LOGW(TAG, "[%s] No response to NTRIP v1 SOURCE request", uplink->log_name);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "[%s] NTRIP v1 response: %.80s", uplink->log_name, resp);
    if (!response_is_success(resp) || !socket_set_nonblocking(sock)) {
        close(sock);
        return -1;
    }

    return sock;
}

static int ntrip_connect(const ntrip_uplink_t *uplink)
{
    int sock = ntrip_connect_v2(uplink);
    if (sock >= 0) {
        ESP_LOGI(TAG, "[%s] Connected to %s/%s using NTRIP v2",
                 uplink->log_name, uplink->host, uplink->mountpoint);
        return sock;
    }

    sock = ntrip_connect_v1_source(uplink);
    if (sock >= 0) {
        ESP_LOGI(TAG, "[%s] Connected to %s/%s using SOURCE",
                 uplink->log_name, uplink->host, uplink->mountpoint);
        return sock;
    }

    ESP_LOGW(TAG, "[%s] Caster rejected both v2 POST and v1 SOURCE",
             uplink->log_name);
    return -1;
}

static void task_ntrip_uplink(void *arg)
{
    const ntrip_uplink_t *uplink = (const ntrip_uplink_t *)arg;
    int sock = -1;
    uint32_t reconnect_delay_ms = NTRIP_RECONNECT_MS_MIN;

    while (1) {
        if (sock < 0) {
            ESP_LOGI(TAG, "[%s] Connecting...", uplink->log_name);
            sock = ntrip_connect(uplink);
            if (sock < 0) {
                s_diag.reconnect_events[uplink_index(uplink)]++;
                vTaskDelay(pdMS_TO_TICKS(reconnect_delay_ms));
                reconnect_delay_ms = reconnect_delay_ms * 2 < NTRIP_RECONNECT_MS_MAX
                                   ? reconnect_delay_ms * 2
                                   : NTRIP_RECONNECT_MS_MAX;
                continue;
            }
            uplink_set_connected(uplink, true);
            reconnect_delay_ms = NTRIP_RECONNECT_MS_MIN;
        }

        pool_frame_t *f = NULL;
        if (xQueueReceive(uplink->queue, &f, pdMS_TO_TICKS(100)) == pdTRUE) {
            const uint8_t *ptr = f->data;
            size_t remaining = f->len;
            int retries = 0;

            while (remaining > 0) {
                int sent = send(sock, ptr, remaining, 0);
                if (sent > 0) {
                    ptr += sent;
                    remaining -= (size_t)sent;
                    retries = 0;
                    continue;
                }

                if (sent == 0) {
                    s_diag.send_fatal_errors[uplink_index(uplink)]++;
                    uplink_close_socket(uplink, &sock, "send returned 0", 0);
                    break;
                } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    if (retries == 0) {
                        uplink_log_blocked(uplink, remaining);
                    }
                    if (++retries > NTRIP_SEND_RETRY_LIMIT) {
                        s_diag.send_fatal_errors[uplink_index(uplink)]++;
                        uplink_close_socket(uplink, &sock,
                                            "send retry limit reached", 0);
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(NTRIP_SEND_RETRY_MS));
                    continue;
                } else {
                    s_diag.send_fatal_errors[uplink_index(uplink)]++;
                    uplink_close_socket(uplink, &sock, "send error", errno);
                    break;
                }
            }

            pool_release(f);
        }

        if (sock >= 0) {
            uint8_t tmp;
            int n = recv(sock, &tmp, sizeof(tmp), MSG_PEEK | MSG_DONTWAIT);
            if (n == 0) {
                uplink_close_socket(uplink, &sock, "remote caster closed connection", 0);
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                uplink_close_socket(uplink, &sock, "recv error", errno);
            }
        }
    }
}

static void ntrip_uplink_start(ntrip_uplink_t *uplink,
                               const char *task_name,
                               const char *log_name,
                               const char *host_key,
                               const char *port_key,
                               const char *mount_key,
                               const char *user_key,
                               const char *pass_key)
{
    memset(uplink, 0, sizeof(*uplink));
    uplink->task_name = task_name;
    uplink->log_name = log_name;
    cfg_get_str(host_key, &uplink->host);
    cfg_get_u16(port_key, &uplink->port);
    cfg_get_str(mount_key, &uplink->mountpoint);
    cfg_get_str(user_key, &uplink->username);
    cfg_get_str(pass_key, &uplink->password);

    uplink->queue = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(pool_frame_t *));
    if (uplink->queue == NULL) {
        ESP_LOGE(TAG, "[%s] Failed to create queue", uplink->log_name);
        return;
    }

    uplink->enabled = true;
    xTaskCreatePinnedToCore(task_ntrip_uplink, task_name, NTRIP_TASK_STACK_SIZE,
                            uplink, NTRIP_TASK_PRIORITY, NULL, NTRIP_TASK_CORE);
}

void ntrip_client_init(void)
{
    uint8_t enabled = 0;
    memset(&s_diag, 0, sizeof(s_diag));

    cfg_get_u8(KEY_CONFIG_NTRIP1_ACTIVE, &enabled);
    if (enabled) {
        ntrip_uplink_start(&s_uplinks[0], "ntrip_cli1", "ntrip1",
                           KEY_CONFIG_NTRIP1_HOST, KEY_CONFIG_NTRIP1_PORT,
                           KEY_CONFIG_NTRIP1_MOUNTPOINT, KEY_CONFIG_NTRIP1_USERNAME,
                           KEY_CONFIG_NTRIP1_PASSWORD);
    }

    cfg_get_u8(KEY_CONFIG_NTRIP2_ACTIVE, &enabled);
    if (enabled) {
        ntrip_uplink_start(&s_uplinks[1], "ntrip_cli2", "ntrip2",
                           KEY_CONFIG_NTRIP2_HOST, KEY_CONFIG_NTRIP2_PORT,
                           KEY_CONFIG_NTRIP2_MOUNTPOINT, KEY_CONFIG_NTRIP2_USERNAME,
                           KEY_CONFIG_NTRIP2_PASSWORD);
    }
}

int ntrip_client_active_count(void)
{
    int count = 0;
    for (int i = 0; i < NTRIP_UPLINK_COUNT; i++) {
        if (s_uplinks[i].queue != NULL) {
            count++;
        }
    }
    return count;
}

void ntrip_client_publish(pool_frame_t *f)
{
    for (int i = 0; i < NTRIP_UPLINK_COUNT; i++) {
        if (s_uplinks[i].queue == NULL) {
            continue;
        }

        if (xQueueSend(s_uplinks[i].queue, &f, 0) != pdTRUE) {
            uplink_log_queue_drop(&s_uplinks[i]);
            pool_release(f);
        }
    }
}

void ntrip_client_get_diagnostics(ntrip_client_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }

    *diag = s_diag;
    diag->queue_depth = FRAME_QUEUE_DEPTH;
    for (int i = 0; i < NTRIP_UPLINK_COUNT; i++) {
        diag->queue_fill[i] = s_uplinks[i].queue ? (uint32_t)uxQueueMessagesWaiting(s_uplinks[i].queue) : 0;
    }
}
