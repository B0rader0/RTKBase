#include "nvs_config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>              // errno
#include <fcntl.h>
#include <driver/uart.h>
#include "tcp_server.h"

static const char *TAG = "TCP_SERVER";

#define TCP_QUEUE_DEPTH 64
#define SOCKET_MAX_RETRIES 40
#define TCP_DIAG_LOG_MASK 0x0F

/**
 * @brief Indicates that the file descriptor represents an invalid (uninitialized or closed) socket
 *
 * Used in the TCP server structure `sock[]` which holds list of active clients we serve.
 */
#define INVALID_SOCK (-1)

/**
 * @brief Time in ms to wait before retrying a non-blocking socket send
 */
#define SOCKET_RETRY_MS 5

/**
 * @brief Time in ms to sleep when the TCP server loop had no work to do
 *
 * Keep this short so the server behaves like a transparent serial cable for
 * tools such as UPrecise, while still yielding CPU when the socket is idle.
 */
#define IDLE_POLL_MS 5

#define DELAY_TICKS_MS(ms) ((TickType_t) ((pdMS_TO_TICKS(ms) > 0) ? pdMS_TO_TICKS(ms) : 1))

static int client_socket = INVALID_SOCK;
static volatile bool disconnect_requested = false;
static char client_endpoint[64] = "";
static raw_frame_t pending_tx_frame;
static bool pending_tx_valid = false;
static size_t pending_tx_offset = 0;
static tcp_server_diag_t s_diag;

QueueHandle_t q_tcp_server   = NULL; // UART raw data → UPrecise TCP server
    
static bool accept_should_retry(int err);
static void task_tcp_server(void *pvParameters);

static void tcp_diag_log_drop(size_t dropped_len)
{
    s_diag.dropped_frames++;
    s_diag.dropped_bytes += (uint32_t)dropped_len;
    if ((s_diag.dropped_frames & TCP_DIAG_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "TCP raw stream drop: dropped=%uB total_drops=%lu queue=%lu/%u",
                 (unsigned)dropped_len,
                 (unsigned long)s_diag.dropped_frames,
                 (unsigned long)(q_tcp_server ? uxQueueMessagesWaiting(q_tcp_server) : 0),
                 (unsigned)TCP_QUEUE_DEPTH);
    }
}

static void tcp_diag_log_blocked(int sock, size_t remaining)
{
    s_diag.send_block_events++;
    if ((s_diag.send_block_events & TCP_DIAG_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "[sock=%d]: send blocked, remaining=%uB count=%lu",
                 sock, (unsigned)remaining,
                 (unsigned long)s_diag.send_block_events);
    }
}

static void tcp_diag_log_deferred(int sock, size_t remaining)
{
    s_diag.send_deferred_events++;
    if ((s_diag.send_deferred_events & TCP_DIAG_LOG_MASK) == 1) {
        ESP_LOGW(TAG, "[sock=%d]: send deferred, pending=%uB count=%lu",
                 sock, (unsigned)remaining,
                 (unsigned long)s_diag.send_deferred_events);
    }
}

static void tcp_server_drain_queue(void)
{
    raw_frame_t dropped_frame;

    if (q_tcp_server == NULL) {
        return;
    }

    while (xQueueReceive(q_tcp_server, &dropped_frame, 0) == pdTRUE) {
        tcp_diag_log_drop(dropped_frame.len);
    }
}

bool tcp_server_client_connected(void)
{
    return client_socket != INVALID_SOCK;
}

const char *tcp_server_client_endpoint(void)
{
    return client_endpoint;
}

void tcp_server_get_diagnostics(tcp_server_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }

    *diag = s_diag;
    diag->pending_bytes = pending_tx_valid ? (uint32_t)(pending_tx_frame.len - pending_tx_offset) : 0;
    diag->queue_fill = q_tcp_server ? (uint32_t)uxQueueMessagesWaiting(q_tcp_server) : 0;
    diag->queue_depth = TCP_QUEUE_DEPTH;
}

bool tcp_server_disconnect_client(void)
{
    int sock = client_socket;

    if (sock == INVALID_SOCK) {
        return false;
    }

    disconnect_requested = true;

    // Force the active session to break immediately so the TCP task will
    // observe the disconnect even if the request arrives between loop passes.
    shutdown(sock, SHUT_RDWR);

    return true;
}

/**
 * @brief Utility to log socket errors
 *
 * @param[in] sock Socket number
 * @param[in] err Socket errno
 * @param[in] message Message to print
 */
static void log_socket_error(const int sock, const int err, const char *message)
{
    ESP_LOGE(TAG, "[sock=%d]: %s\n"
                  "error=%d: %s", sock, message, err, strerror(err));
} // log_socket_error

/**
 * @brief Enqueue raw GNSS UART data for forwarding to the TCP client.
 *
 * If the TCP server is disabled or no client is connected, the frame is
 * discarded. If the queue is full, the oldest queued frame is dropped so the
 * most recent UART data is preserved.
 *
 * @param[in] frame Raw UART chunk to forward
 */
void post_raw_gnss_data_tcp(const raw_frame_t *frame)
{
    raw_frame_t dropped_frame;

    if(q_tcp_server == NULL) {
        return; // TCP server is not enabled, discard the data
    }

    if (client_socket == INVALID_SOCK) {
        return; // No clients connected, discard the data
    }

    // Prefer the newest UART data over stale queued data if the client falls behind.
    while (xQueueSend(q_tcp_server, frame, 0) != pdTRUE) {
        if (xQueueReceive(q_tcp_server, &dropped_frame, 0) != pdTRUE) {
            ESP_LOGW(TAG, "TCP server queue full, unable to drop stale data for %u-byte frame", (unsigned)frame->len);
            return;
        }
        tcp_diag_log_drop(dropped_frame.len);
    }

} // post_raw_gnss_data_tcp

/**
 * @brief Tries to receive data from specified sockets in a non-blocking way,
 *        i.e. returns immediately if no data.
 *
 * @param[in] sock Socket for reception
 * @param[out] data Data pointer to write the received data
 * @param[in] max_len Maximum size of the allocated space for receiving data
 * @return
 *          >0 : Size of received data
 *          =0 : No data available
 *          -1 : Error occurred during socket read operation
 *          -2 : Socket has been closed or disconnected
 */
static int try_receive(const int sock, char * data, size_t max_len)
{
    int len = recv(sock, data, max_len, 0);
    if (len == 0) {
        ESP_LOGI(TAG, "[sock=%d]: Connection closed by peer", sock);
        return -2;
    }
    if (len < 0) {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;   // Not an error
        }
        if (errno == ENOTCONN) {
            ESP_LOGW(TAG, "[sock=%d]: Connection closed", sock);
            return -2;  // Socket has been disconnected
        }
        log_socket_error(sock, errno, "Error occurred during receiving");
        return -1;
    }

    return len;
}  //try_receive

/**
 * @brief Sends the specified data to the socket with bounded retries for
 *        non-blocking sockets.
 *
 * @param[in] sock Socket to write data
 * @param[in] data Data to be written
 * @param[in] len Length of the data
 * @return
 *          1  : Full buffer sent
 *          0  : Socket temporarily blocked; retry later without disconnecting
 *          -1 : Fatal socket error
 */
static int socket_send(const int sock, const uint8_t *data, const size_t len, size_t *offset)
{
    int retries = 0;

    while (*offset < len) {
        int written = send(sock, data + *offset, len - *offset, 0);

        if (written > 0) {
            *offset += (size_t)written;
            retries = 0;
            continue;
        }

        if (written == 0) {
            ESP_LOGW(TAG, "[sock=%d]: send() returned 0", sock);
            s_diag.send_fatal_errors++;
            return -1;
        }

        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
            if (retries == 0) {
                tcp_diag_log_blocked(sock, len - *offset);
            }
            if (++retries > SOCKET_MAX_RETRIES) {
                tcp_diag_log_deferred(sock, len - *offset);
                return 0;
            }
            vTaskDelay(DELAY_TICKS_MS(SOCKET_RETRY_MS));
            continue;
        }

        log_socket_error(sock, errno, "Error occurred during sending");
        s_diag.send_fatal_errors++;
        return -1;
    }

    return 1;
} //socket_send

/**
 * @brief Returns the string representation of client's address (accepted on this server)
 */
static inline char* get_clients_address(struct sockaddr_storage *source_addr)
{
    static char address_str[128];
    char *res = NULL;
    // Convert ip address to string
    if (source_addr->ss_family == PF_INET) {
        res = inet_ntoa_r(((struct sockaddr_in *)source_addr)->sin_addr, address_str, sizeof(address_str) - 1);
    }

    if (!res) {
        address_str[0] = '\0'; // Returns empty string if conversion didn't succeed
    }
    return address_str;
}

static uint16_t get_client_port(const struct sockaddr_storage *source_addr)
{
    if (source_addr->ss_family == PF_INET) {
        return ntohs(((const struct sockaddr_in *)source_addr)->sin_port);
    }

    return 0;
}

static void close_socket_gracefully(const int sock)
{
    char discard[64];

    if (sock == INVALID_SOCK) {
        return;
    }

    shutdown(sock, SHUT_WR);

    while (recv(sock, discard, sizeof(discard), MSG_DONTWAIT) > 0) {
    }

    close(sock);
}

static void reject_pending_client(int listening_socket)
{
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    int pending_socket = accept(listening_socket, (struct sockaddr *)&source_addr, &addr_len);

    if (pending_socket < 0) {
        if (!accept_should_retry(errno)) {
            log_socket_error(listening_socket, errno, "Error when rejecting connection");
        }
        return;
    }

    ESP_LOGW(TAG, "[sock=%d]: Rejecting connection from IP:%s, client already connected",
             pending_socket, get_clients_address(&source_addr));
    close_socket_gracefully(pending_socket);
}

static bool accept_should_retry(int err)
{
    return err == EWOULDBLOCK || err == EAGAIN || err == EINTR ||
           err == EMFILE || err == ENFILE;
}

static void task_tcp_server(void *pvParameters)
{
    static char rx_buffer[128];
    int listening_socket = INVALID_SOCK;
    
    client_socket = INVALID_SOCK;
    disconnect_requested = false;
    client_endpoint[0] = '\0';
    pending_tx_valid = false;
    pending_tx_offset = 0;
    memset(&pending_tx_frame, 0, sizeof(pending_tx_frame));
    memset(&s_diag, 0, sizeof(s_diag));

    // Creating a listener socket
    listening_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (listening_socket < 0) {
        log_socket_error(listening_socket, errno, "Unable to create socket");
        goto error;
    }
    ESP_LOGI(TAG, "Listener socket created");

    int opt = 1;
    if (setsockopt(listening_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) != 0) {
        log_socket_error(listening_socket, errno, "Unable to set SO_REUSEADDR");
        goto error;
    }

    // Marking the socket as non-blocking
    int flags = fcntl(listening_socket, F_GETFL);
    if (flags == -1) {
        log_socket_error(listening_socket, errno, "Unable to get socket flags");
        goto error;
    }
    if (fcntl(listening_socket, F_SETFL, flags | O_NONBLOCK) == -1) {
        log_socket_error(listening_socket, errno, "Unable to set socket non blocking");
        goto error;
    }
    ESP_LOGI(TAG, "Socket marked as non blocking");


    uint16_t sin_p;
    cfg_get_u16(KEY_CONFIG_TCP_SERVER_PORT, &sin_p);

    struct sockaddr_in srv = {
        .sin_family      = AF_INET,
        .sin_port        = htons(sin_p),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    int err = bind(listening_socket, (struct sockaddr *)&srv, sizeof(srv));
    if (err != 0) {
        log_socket_error(listening_socket, errno, "Socket unable to bind");
        goto error;
    }
    ESP_LOGI(TAG, "Socket bound on %s:%d", inet_ntoa(srv.sin_addr), sin_p);
    
    // Single-client server; allow a small pending backlog for reconnect races.
    err = listen(listening_socket, 2);
    if (err != 0) {
        log_socket_error(listening_socket, errno, "Error occurred during listen");
        goto error;
    }
    ESP_LOGI(TAG, "Socket listening");

    uint8_t uart_port;
    cfg_get_u8(KEY_CONFIG_UART_NUM, &uart_port);

    q_tcp_server = xQueueCreate(TCP_QUEUE_DEPTH, sizeof(raw_frame_t));
    if (q_tcp_server == NULL) {
        ESP_LOGE(TAG, "Failed to create TCP server queue");
        goto error;
    }

    // Main loop for accepting a connection and serving the connected client
    while (1) {
        bool did_work = false;
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);

        if (disconnect_requested && client_socket != INVALID_SOCK) {
            int sock = client_socket;
            client_socket = INVALID_SOCK;
            disconnect_requested = false;
            tcp_server_drain_queue();
            pending_tx_valid = false;
            pending_tx_offset = 0;
            shutdown(sock, SHUT_RDWR);
            close(sock);
            client_endpoint[0] = '\0';
            ESP_LOGI(TAG, "[sock=%d]: Client disconnected by web request", sock);
            did_work = true;
        } else if (disconnect_requested) {
            disconnect_requested = false;
        }

        // Keep the server strictly single-client: reject any extra pending connection.
        if (client_socket != INVALID_SOCK) {
            reject_pending_client(listening_socket);
        }

        // We accept a new connection only if we do not already have an active client.
        if (client_socket == INVALID_SOCK) {
            // Try to accept a new connections
            client_socket = accept(listening_socket, (struct sockaddr *)&source_addr, &addr_len);

            if (client_socket < 0) {
                if (accept_should_retry(errno)) {
                    if (errno == EMFILE || errno == ENFILE) {
                        ESP_LOGW(TAG, "Socket limit reached while accepting client, will retry");
                    } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
                        ESP_LOGV(TAG, "No pending connections...");
                    }
                    // No pending connection or a transient resource shortage:
                    // continue serving and try again on a later iteration.
                } else {
                    log_socket_error(listening_socket, errno, "Error when accepting connection");
                    goto error;
                }
            } else {
                // We have a new client connected -> print it's address
                ESP_LOGI(TAG, "[sock=%d]: Connection accepted from IP:%s", client_socket, get_clients_address(&source_addr));
                snprintf(client_endpoint, sizeof(client_endpoint), "%s:%u",
                         get_clients_address(&source_addr),
                         (unsigned)get_client_port(&source_addr));
                did_work = true;

                int flag = 1;
                if (setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) != 0) {
                    log_socket_error(client_socket, errno, "Unable to set TCP_NODELAY");
                    close(client_socket);
                    client_socket = INVALID_SOCK;
                    pending_tx_valid = false;
                    pending_tx_offset = 0;
                    continue;
                }

                // ...and set the client's socket non-blocking
                flags = fcntl(client_socket, F_GETFL);
                if (flags == -1) {
                    log_socket_error(client_socket, errno, "Unable to get socket flags");
                    close(client_socket);
                    client_socket = INVALID_SOCK;
                    pending_tx_valid = false;
                    pending_tx_offset = 0;
                    continue;
                }
                if (fcntl(client_socket, F_SETFL, flags | O_NONBLOCK) == -1) {
                    log_socket_error(client_socket, errno, "Unable to set socket non blocking");
                    goto error;
                }
                ESP_LOGI(TAG, "[sock=%d]: Socket marked as non blocking", client_socket);
            }
        }

        // Serve the connected client.
        if (client_socket != INVALID_SOCK) {
            int len = try_receive(client_socket, rx_buffer, sizeof(rx_buffer));
            if (len < 0) {
                close(client_socket);
                client_socket = INVALID_SOCK;
                client_endpoint[0] = '\0';
                pending_tx_valid = false;
                pending_tx_offset = 0;
                did_work = true;
            } else if (len > 0) {
                // Prioritize command/response latency over stale background GNSS
                // output so UPrecise can identify the receiver promptly.
                tcp_server_drain_queue();
                // Delay should be negligible as the TCP client is expected to send
                // small config commands, not a continuous data stream.
                uart_write_bytes(uart_port, rx_buffer, len); // TODO: handle UART write errors and disconnect the client if needed
                did_work = true;
            }
        }

        // ── Drain queue and send to the connected client ──────────────────────
        static raw_frame_t rf;
        if (client_socket != INVALID_SOCK && pending_tx_valid) {
            did_work = true;
            int send_result = socket_send(client_socket,
                                          pending_tx_frame.data,
                                          pending_tx_frame.len,
                                          &pending_tx_offset);
            if (send_result < 0) {
                ESP_LOGW(TAG, "Client send error, closing socket");
                close(client_socket);
                client_socket = INVALID_SOCK;
                client_endpoint[0] = '\0';
                pending_tx_valid = false;
                pending_tx_offset = 0;
            } else if (send_result > 0) {
                pending_tx_valid = false;
                pending_tx_offset = 0;
            }
        }

        while (!pending_tx_valid && xQueueReceive(q_tcp_server, &rf, 0) == pdTRUE) {
            did_work = true;
            if (client_socket == INVALID_SOCK) {
                continue;
            }

            pending_tx_frame = rf;
            pending_tx_offset = 0;
            pending_tx_valid = true;

            int send_result = socket_send(client_socket,
                                          pending_tx_frame.data,
                                          pending_tx_frame.len,
                                          &pending_tx_offset);
            if (send_result < 0) {
                ESP_LOGW(TAG, "Client send error, closing socket");
                close(client_socket);
                client_socket = INVALID_SOCK;
                client_endpoint[0] = '\0';
                pending_tx_valid = false;
                pending_tx_offset = 0;
            } else if (send_result > 0) {
                pending_tx_valid = false;
                pending_tx_offset = 0;
            }
        }

        if (!did_work) {
            vTaskDelay(DELAY_TICKS_MS(IDLE_POLL_MS));
        }
    }

error:
    if (q_tcp_server != NULL) {
        vQueueDelete(q_tcp_server);
        q_tcp_server = NULL;
    }
    if (listening_socket != INVALID_SOCK) {
        close(listening_socket);
    }

    if (client_socket != INVALID_SOCK) {
        close(client_socket);
        client_socket = INVALID_SOCK;
    }
    client_endpoint[0] = '\0';
    pending_tx_valid = false;
    pending_tx_offset = 0;

    vTaskDelete(NULL);
}

void tcp_server_init(void)
{
    uint8_t enabled = 0;
    cfg_get_u8(KEY_CONFIG_TCP_SERVER_ACTIVE, &enabled);
    if (!enabled) {
        return;
    }

    xTaskCreatePinnedToCore(task_tcp_server, "tcp_srv", 4096, NULL, 4, NULL, 0);
}
