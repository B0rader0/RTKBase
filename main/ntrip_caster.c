#include "rtk_base.h"
#include "nvs_config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include <mbedtls/base64.h>
#include <string.h>
#include <strings.h>            // strncasecmp(), strcasecmp() — POSIX
#include <stdio.h>
#include <ctype.h>
#include <errno.h>              // errno
#include <fcntl.h>
#include "nvs_config.h"

static const char *TAG = "NTRIP_CASTER";

// ─── Per-rover connection state ───────────────────────────────────────────────

typedef enum {
    ROVER_EMPTY,
    ROVER_HANDSHAKE,
    ROVER_STREAMING,
} rover_state_t;

typedef struct {
    int           fd;
    rover_state_t state;
    char          rx_buf[512];
    int           rx_pos;
    bool          is_ntrip_v2;
} rover_conn_t;

static rover_conn_t  s_rovers[4];
static SemaphoreHandle_t s_rovers_mutex;

// ─── Helpers ──────────────────────────────────────────────────────────────────

static const char *stristr(const char *hay, const char *needle)
{
    size_t nl = strlen(needle);
    for (; *hay; hay++) {
        if (strncasecmp(hay, needle, nl) == 0) return hay;
    }
    return NULL;
}

static bool auth_valid(const char *b64)
{
    char *pw, *usr;
    cfg_get_str(KEY_CONFIG_CASTER_PASSWORD, &pw);
    
    //??if (strlen(g_cfg.ntrip_srv_password) == 0) return true;
    if (pw == NULL) {
        return true;
    }

    if (strlen(pw) == 0) {
        free(pw);
        return true;
    }

    unsigned char decoded[128];
    size_t out_len = 0;
    if (mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &out_len,
                              (const unsigned char *)b64, strlen(b64)) != 0) {
        return false;
    }
    decoded[out_len] = '\0';
    
    cfg_get_str(KEY_CONFIG_CASTER_USERNAME, &usr);

    char expected[128];
    snprintf(expected, sizeof(expected), "%s:%s",
             usr, pw);
    return strcmp((char *)decoded, expected) == 0;
}

// ─── Sourcetable ─────────────────────────────────────────────────────────────

static void send_sourcetable(int fd, bool v2)
{
    char *mount_p;
    cfg_get_str(KEY_CONFIG_CASTER_MOUNTPOINT, &mount_p);
    char str_record[256];
    int str_len = snprintf(str_record, sizeof(str_record),
        "STR;%s;RTK Base;RTCM 3.2;"
        "1005(1),1074(1),1084(1),1094(1),1124(1);"
        "2;GPS+GLO+GAL+BDS;ESP32;DE;48.10;11.60;1;1;sNTRIP;none;N;N;0;\r\n"
        "ENDSOURCETABLE\r\n",
        mount_p);

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
    send(fd, header,     hdr_len, MSG_DONTWAIT);
    send(fd, str_record, str_len, MSG_DONTWAIT);
    free(mount_p);
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
        send(rover->fd, r, strlen(r), MSG_DONTWAIT);
        return false;
    }

    if (strcmp(path, "/") == 0) {
        send_sourcetable(rover->fd, rover->is_ntrip_v2);
        return false;
    }

    char expected[72];
    char *mount_p;

    cfg_get_str(KEY_CONFIG_CASTER_MOUNTPOINT, &mount_p);

    snprintf(expected, sizeof(expected), "/%s", mount_p);
    if (strcasecmp(path, expected) != 0) {
        const char *r = "HTTP/1.1 404 Not Found\r\n\r\n";
        send(rover->fd, r, strlen(r), MSG_DONTWAIT);
        ESP_LOGW(TAG, "Unknown mountpoint: %s", path);
        free(mount_p);
        return false;
    }

    // ── Auth ──────────────────────────────────────────────────────────────────
    char *pw;
    cfg_get_str(KEY_CONFIG_CASTER_PASSWORD, &pw);

    if (strlen(pw) > 0) {
        const char *auth_hdr = stristr(req, "Authorization: Basic ");
        if (!auth_hdr) {
            const char *r = "HTTP/1.1 401 Unauthorized\r\n"
                            "WWW-Authenticate: Basic realm=\"NTRIP\"\r\n\r\n";
            send(rover->fd, r, strlen(r), MSG_DONTWAIT);
            ESP_LOGW(TAG, "Rover provided no credentials");
            free(pw);
            free(mount_p);
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
            send(rover->fd, r, strlen(r), MSG_DONTWAIT);
            ESP_LOGW(TAG, "Rover failed authentication");
            return false;
        }
    }

    // ── Accept ────────────────────────────────────────────────────────────────
    if (rover->is_ntrip_v2) {
        const char *r = "HTTP/1.1 200 OK\r\n"
                        "Ntrip-Version: Ntrip/2.0\r\n"
                        "Content-Type: gnss/data\r\n\r\n";
        send(rover->fd, r, strlen(r), MSG_DONTWAIT);
    } else {
        const char *r = "ICY 200 OK\r\n\r\n";
        send(rover->fd, r, strlen(r), MSG_DONTWAIT);
    }

    ESP_LOGI(TAG, "Rover accepted on /%s (NTRIPv%d)", mount_p, rover->is_ntrip_v2 ? 2 : 1);
    free(pw);
    free(mount_p);

    return true;
} //handle_request

// ─── Close a rover slot ───────────────────────────────────────────────────────

static void close_rover(rover_conn_t *rover, int index)
{
    ESP_LOGI(TAG, "Rover %d disconnected", index);
    close(rover->fd);
    rover->fd     = -1;
    rover->state  = ROVER_EMPTY;
    rover->rx_pos = 0;
}

// ─── NTRIP Server Task ────────────────────────────────────────────────────────

void task_ntrip_server(void *arg)
{
    s_rovers_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < 4; i++) {
        s_rovers[i].fd    = -1;
        s_rovers[i].state = ROVER_EMPTY;
    }

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    uint16_t prt;
    cfg_get_u16(KEY_CONFIG_CASTER_PORT, &prt);
    
    struct sockaddr_in srv = {
        .sin_family      = AF_INET,
        .sin_port        = htons(prt),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_fd, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    
    listen(listen_fd, 4);

    char *mount_p;
    cfg_get_str(KEY_CONFIG_CASTER_MOUNTPOINT, &mount_p);
    ESP_LOGI(TAG, "NTRIP server listening on port %d, mountpoint /%s", prt, mount_p);
    free(mount_p);

    while (1) {
        // ── fd_set ────────────────────────────────────────────────────────────
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(listen_fd, &read_fds);
        int max_fd = listen_fd;
        for (int i = 0; i < 4; i++) {
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
                for (int i = 0; i < 4; i++) {
                    if (s_rovers[i].state == ROVER_EMPTY) {
                        int flag = 1;
                        setsockopt(new_fd, IPPROTO_TCP, TCP_NODELAY,
                                   &flag, sizeof(flag));
                        int fl = fcntl(new_fd, F_GETFL, 0);
                        fcntl(new_fd, F_SETFL, fl | O_NONBLOCK);

                        s_rovers[i].fd     = new_fd;
                        s_rovers[i].state  = ROVER_HANDSHAKE;
                        s_rovers[i].rx_pos = 0;
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
                    send(new_fd, busy, strlen(busy), 0);
                    close(new_fd);
                    ESP_LOGW(TAG, "Max rovers reached, connection refused");
                }
            }
        }

        // ── Service connected rovers ──────────────────────────────────────────
        for (int i = 0; i < 4; i++) {
            rover_conn_t *r = &s_rovers[i];
            if (r->state == ROVER_EMPTY) continue;

            if (FD_ISSET(r->fd, &read_fds)) {
                if (r->state == ROVER_HANDSHAKE) {
                    int n = recv(r->fd,
                                 r->rx_buf + r->rx_pos,
                                 sizeof(r->rx_buf) - r->rx_pos - 1, 0);
                    if (n <= 0) { close_rover(r, i); continue; }
                    r->rx_pos += n;
                    r->rx_buf[r->rx_pos] = '\0';
                    if (strstr(r->rx_buf, "\r\n\r\n")) {
                        if (handle_request(r)) {
                            r->state = ROVER_STREAMING;
                        } else {
                            close_rover(r, i);
                        }
                    } else if (r->rx_pos >= (int)sizeof(r->rx_buf) - 1) {
                        ESP_LOGW(TAG, "Rover %d request too large", i);
                        close_rover(r, i);
                    }
                } else if (r->state == ROVER_STREAMING) {
                    // Incoming GGA from rover (for VRS future use) — drain cleanly
                    uint8_t tmp[64];
                    int n = recv(r->fd, tmp, sizeof(tmp), MSG_DONTWAIT);
                    if (n == 0 || (n < 0 && errno != EAGAIN)) {
                        close_rover(r, i);
                    }
                }
            }
        }

        // ── Forward RTCM3 frames to streaming rovers ──────────────────────────
        pool_frame_t *f;
        while (xQueueReceive(q_ntrip_server, &f, 0) == pdTRUE) {
            xSemaphoreTake(s_rovers_mutex, portMAX_DELAY);
            for (int i = 0; i < 4; i++) {
                rover_conn_t *r = &s_rovers[i];
                if (r->state != ROVER_STREAMING) continue;

                const uint8_t *ptr = f->data;
                size_t remaining   = f->len;
                while (remaining > 0) {
                    int sent = send(r->fd, ptr, remaining, MSG_DONTWAIT);
                    if (sent < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            // Rover can't keep up — drop this frame for this rover
                            ESP_LOGW(TAG, "Rover %d slow, dropping frame", i);
                            break;
                        }
                        ESP_LOGW(TAG, "Rover %d send error, closing", i);
                        close_rover(r, i);
                        break;
                    }
                    ptr       += sent;
                    remaining -= sent;
                }
            }
            xSemaphoreGive(s_rovers_mutex);
            pool_release(f);    // done with this frame
        }
    }
}

// ─── Helper: count active streaming rovers ───────────────────────────────────

int ntrip_server_rover_count(void)
{
    int count = 0;
    if (xSemaphoreTake(s_rovers_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            if (s_rovers[i].state == ROVER_STREAMING) count++;
        }
        xSemaphoreGive(s_rovers_mutex);
    }
    return count;
}
