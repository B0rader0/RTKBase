#include "rtk_base.h"
#include "nvs_config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "mbedtls/base64.h"
#include <string.h>
#include <stdlib.h>             // malloc()
#include <stdio.h>
#include <errno.h>              // errno
#include <fcntl.h>
#include "ntrip_client.h"

static const char *TAG = "ntrip_client";


// ─── Base64 helper ────────────────────────────────────────────────────────────

static void base64_encode_str(const char *input, char *output, size_t out_size)
{
    size_t written = 0;
    mbedtls_base64_encode((unsigned char *)output, out_size, &written,
                          (const unsigned char *)input, strlen(input));
    output[written] = '\0';
}

// ─── Connect + NTRIP handshake ────────────────────────────────────────────────

static int ntrip_connect(const ntrip_client_cfg_t *cfg)
{
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res  = NULL;
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", cfg->port);

    if (getaddrinfo(cfg->host, port_str, &hints, &res) != 0 || !res) {
        ESP_LOGE(TAG, "[%s] DNS resolve failed for %s", cfg->task_name, cfg->host);
        return -1;
    }

    int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) { freeaddrinfo(res); return -1; }

    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "[%s] connect() to %s:%d failed",
                 cfg->task_name, cfg->host, cfg->port);
        freeaddrinfo(res);
        close(sock);
        return -1;
    }
    freeaddrinfo(res);

    // Build Basic Auth header
    char credentials[128], b64[192];
    snprintf(credentials, sizeof(credentials), "%s:%s", cfg->username, cfg->password);
    base64_encode_str(credentials, b64, sizeof(b64));

    char request[512];
    int req_len = snprintf(request, sizeof(request),
        "POST /%s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "User-Agent: NTRIP ESP32RTKBase/1.0\r\n"
        "Authorization: Basic %s\r\n"
        "Content-Type: gnss/data\r\n"
        "Connection: close\r\n"
        "\r\n",
        cfg->mountpoint, cfg->host, cfg->port, b64);

    if (send(sock, request, req_len, 0) < 0) {
        ESP_LOGE(TAG, "[%s] Failed to send NTRIP request", cfg->task_name);
        close(sock);
        return -1;
    }

    // Read and check the HTTP response
    char resp[256];
    int n = recv(sock, resp, sizeof(resp) - 1, 0);
    if (n <= 0) {
        ESP_LOGE(TAG, "[%s] No response from caster", cfg->task_name);
        close(sock);
        return -1;
    }
    resp[n] = '\0';
    ESP_LOGI(TAG, "[%s] Caster response: %.80s", cfg->task_name, resp);

    if (!strstr(resp, "200")) {
        ESP_LOGE(TAG, "[%s] Caster rejected connection", cfg->task_name);
        close(sock);
        return -1;
    }

    // Switch to non-blocking for the streaming phase
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "[%s] Connected → %s/%s", cfg->task_name, cfg->host, cfg->mountpoint);
    return sock;
}

// ─── NTRIP Client Task ────────────────────────────────────────────────────────
// One instance per caster.  arg must point to a ntrip_client_cfg_t.

void task_ntrip_client(void *arg)
{

    const ntrip_client_cfg_t *cfg = (const ntrip_client_cfg_t *)arg;

    int      sock               = -1;
    uint32_t reconnect_delay_ms = 2000;


    while (1) {
        // ── (Re)connect ───────────────────────────────────────────────────────
        if (sock < 0) {
            ESP_LOGI(TAG, "[%s] Connecting...", cfg->task_name);
            sock = ntrip_connect(cfg);
            if (sock < 0) {
                ESP_LOGW(TAG, "[%s] Retry in %lu ms",
                         cfg->task_name, (unsigned long)reconnect_delay_ms);
                vTaskDelay(pdMS_TO_TICKS(reconnect_delay_ms));
                reconnect_delay_ms = reconnect_delay_ms * 2 < 60000
                                   ? reconnect_delay_ms * 2 : 60000;
                continue;
            }
            reconnect_delay_ms = 2000;
        }

        // ── Dequeue and stream ────────────────────────────────────────────────
        pool_frame_t *f;
        if (xQueueReceive(cfg->queue, &f, pdMS_TO_TICKS(100)) == pdTRUE) {
            const uint8_t *ptr  = f->data;
            size_t remaining    = f->len;

            while (remaining > 0) {
                int sent = send(sock, ptr, remaining, 0);
                if (sent < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        vTaskDelay(pdMS_TO_TICKS(5));
                        continue;
                    }
                    ESP_LOGW(TAG, "[%s] send() error %d, reconnecting",
                             cfg->task_name, errno);
                    close(sock);
                    sock = -1;
                    break;
                }
                ptr       += sent;
                remaining -= sent;
            }
            pool_release(f);
        }

        // ── Detect caster-side close ──────────────────────────────────────────
        if (sock >= 0) {
            uint8_t tmp;
            if (recv(sock, &tmp, 1, MSG_DONTWAIT) == 0) {
                ESP_LOGW(TAG, "[%s] Caster closed connection", cfg->task_name);
                close(sock);
                sock = -1;
            }
        }
    }
}