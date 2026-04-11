#include "rtk_base.h"
#include "nvs_config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <errno.h>              // errno
#include <fcntl.h>
#include <driver/uart.h>
#include "tcp_server.h"

static const char *TAG = "TCP_SERVER";

#define MAX_CLIENTS 4
#define TCP_QUEUE_DEPTH 8

/**
 * @brief Indicates that the file descriptor represents an invalid (uninitialized or closed) socket
 *
 * Used in the TCP server structure `sock[]` which holds list of active clients we serve.
 */
#define INVALID_SOCK (-1)

/**
 * @brief Time in ms to yield to all tasks when a non-blocking socket would block
 *
 * Non-blocking socket operations are typically executed in a separate task validating
 * the socket status. Whenever the socket returns `EAGAIN` (idle status, i.e. would block)
 * we have to yield to all tasks to prevent lower priority tasks from starving.
 */
#define YIELD_TO_ALL_MS 50

int client_sockets[MAX_CLIENTS];

QueueHandle_t q_tcp_server   = NULL; // UART raw data → UPrecise TCP server
    


/**
 * @brief Utility to log socket errors
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket number
 * @param[in] err Socket errno
 * @param[in] message Message to print
 */
static void log_socket_error(const char *tag, const int sock, const int err, const char *message)
{
    ESP_LOGE(tag, "[sock=%d]: %s\n"
                  "error=%d: %s", sock, message, err, strerror(err));
} // log_socket_error

/**
 * @brief To be called from the GNSS UART task whenever new data is received from the UART and needs to be sent to TCP clients.
 * Checks if the queue for TCP server is not NULL, i.e. the TCP server is enabled.
 * Then checks if there are any clients connected to the TCP server.
 * If clients are connected, copies the data to a pool frame and enqueues it to the TCP server queue for sending to clients.
 * The frame type is different as no reference counting is needed for the TCP server as it sends raw data and does not need to keep track of how many consumers are using the frame.
 *
 * @param[in] data pointer to the data received from UART
 * @param[in] len length of the data received from UART
 */
void post_raw_gnss_data_tcp(const raw_frame_t *frame)
{
    if(q_tcp_server == NULL) {
        return; // TCP server is not enabled, discard the data
    }

    // Check if there are any clients connected to the TCP server
    bool has_clients = false;
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        if (client_sockets[i] != INVALID_SOCK) {
            has_clients = true;
            break;
        }
    }

    if (!has_clients) {
        return; // No clients connected, discard the data
    }
    // Enqueue the data to the TCP server queue for sending to clients
    xQueueSend(q_tcp_server, frame, 0); // Non-blocking send, as we don't want to block the UART task if the queue is full

} // post_raw_gnss_data_tcp

/**
 * @brief Tries to receive data from specified sockets in a non-blocking way,
 *        i.e. returns immediately if no data.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket for reception
 * @param[out] data Data pointer to write the received data
 * @param[in] max_len Maximum size of the allocated space for receiving data
 * @return
 *          >0 : Size of received data
 *          =0 : No data available
 *          -1 : Error occurred during socket read operation
 *          -2 : Socket is not connected, to distinguish between an actual socket error and active disconnection
 */
static int try_receive(const char *tag, const int sock, char * data, size_t max_len)
{
    int len = recv(sock, data, max_len, 0);
    if (len < 0) {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;   // Not an error
        }
        if (errno == ENOTCONN) {
            ESP_LOGW(tag, "[sock=%d]: Connection closed", sock);
            return -2;  // Socket has been disconnected
        }
        log_socket_error(tag, sock, errno, "Error occurred during receiving");
        return -1;
    }

    return len;
}  //try_receive

/**
 * @brief Sends the specified data to the socket. This function blocks until all bytes got sent.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket to write data
 * @param[in] data Data to be written
 * @param[in] len Length of the data
 * @return
 *          >0 : Size the written data
 *          -1 : Error occurred during socket write operation
 */
static int socket_send(const char *tag, const int sock, const uint8_t *data, const size_t len)
{
    int to_write = len;
    while (to_write > 0) {
        int written = send(sock, data + (len - to_write), to_write, 0);
        if (written < 0 && errno != EINPROGRESS && errno != EAGAIN && errno != EWOULDBLOCK) {
            log_socket_error(tag, sock, errno, "Error occurred during sending");
            return -1;
        }
        to_write -= written;
    }
    return len;
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


void task_tcp_server(void *pvParameters)
{
    static char rx_buffer[128];
/* 
    struct addrinfo hints = { 
        .ai_socktype = SOCK_STREAM,
        .ai_family = AF_INET
    };
    */ 
    int listening_socket = INVALID_SOCK;
    
    // Prepare a list of file descriptors to hold client's sockets, mark all of them as invalid, i.e. available
    for (int i = 0; i < MAX_CLIENTS; ++i) {
        client_sockets[i] = INVALID_SOCK;
    }

    // Creating a listener socket
    listening_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (listening_socket < 0) {
        log_socket_error(TAG, listening_socket, errno, "Unable to create socket");
        goto error;
    }
    ESP_LOGI(TAG, "Listener socket created");

    // Marking the socket as non-blocking
    int flags = fcntl(listening_socket, F_GETFL);
    if (fcntl(listening_socket, F_SETFL, flags | O_NONBLOCK) == -1) {
        log_socket_error(TAG, listening_socket, errno, "Unable to set socket non blocking");
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
        log_socket_error(TAG, listening_socket, errno, "Socket unable to bind");
        goto error;
    }
    ESP_LOGI(TAG, "Socket bound on %s:%d", inet_ntoa(srv.sin_addr), sin_p);
    
    // Set queue (backlog) of pending connections to one (can be more)
    err = listen(listening_socket, 1);
    if (err != 0) {
        log_socket_error(TAG, listening_socket, errno, "Error occurred during listen");
        goto error;
    }
    ESP_LOGI(TAG, "Socket listening");

    uint8_t uart_port;
    cfg_get_u8(KEY_CONFIG_UART_NUM, &uart_port);

    q_tcp_server = xQueueCreate(TCP_QUEUE_DEPTH, sizeof(raw_frame_t));

    // Main loop for accepting new connections and serving all connected clients
    while (1) {
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);

        // Find a free socket
        int new_sock_index = 0;
        for (new_sock_index = 0; new_sock_index < MAX_CLIENTS; ++new_sock_index) {
            if (client_sockets[new_sock_index] == INVALID_SOCK) {
                break;
            }
        }

        // We accept a new connection only if we have a free socket
        if (new_sock_index < MAX_CLIENTS) {
            // Try to accept a new connections
            client_sockets[new_sock_index] = accept(listening_socket, (struct sockaddr *)&source_addr, &addr_len);

            if (client_sockets[new_sock_index] < 0) {
                if (errno == EWOULDBLOCK) { // The listener socket did not accepts any connection
                                            // continue to serve open connections and try to accept again upon the next iteration
                    ESP_LOGV(TAG, "No pending connections...");
                } else {
                    log_socket_error(TAG, listening_socket, errno, "Error when accepting connection");
                    goto error;
                }
            } else {
                // We have a new client connected -> print it's address
                ESP_LOGI(TAG, "[sock=%d]: Connection accepted from IP:%s", client_sockets[new_sock_index], get_clients_address(&source_addr));

                // ...and set the client's socket non-blocking
                flags = fcntl(client_sockets[new_sock_index], F_GETFL);
                if (fcntl(client_sockets[new_sock_index], F_SETFL, flags | O_NONBLOCK) == -1) {
                    log_socket_error(TAG, client_sockets[new_sock_index], errno, "Unable to set socket non blocking");
                    goto error;
                }
                ESP_LOGI(TAG, "[sock=%d]: Socket marked as non blocking", client_sockets[new_sock_index]);
            }
        }

        // We serve all the connected clients in this loop
        for (int i = 0; i < MAX_CLIENTS; ++i) {
            if (client_sockets[i] != INVALID_SOCK) {

                // This is an open socket -> try to serve it
                int len = try_receive(TAG, client_sockets[i], rx_buffer, sizeof(rx_buffer));
                if (len < 0) {
                    // Error occurred within this client's socket -> close and mark invalid
                    close(client_sockets[i]);
                    client_sockets[i] = INVALID_SOCK;
                } else if (len > 0) {
                    // Received some data -> send to UART in a blockig way. 
                    // Delay should be negligible as the TCP client is expected to send small config commands, not continuous data stream. 
                    uart_write_bytes(uart_port, rx_buffer, len); // TODO: handle UART write errors and disconnect the client if needed
                }
            } // one client's socket
        } // for all sockets

        // ── Drain queue and send to all connected clients ─────────────────────
        static raw_frame_t rf;
        while (xQueueReceive(q_tcp_server, &rf, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Sending data to clients: %d bytes", rf.len);
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (client_sockets[i] < 0) continue;
                //int sent = send(client_sockets[i], rf.data, rf.len, MSG_DONTWAIT);
                int sent = socket_send(TAG, client_sockets[i], rf.data, rf.len);
                if (sent < 0) {
                    ESP_LOGW(TAG, "Client %d send error, closing", i);
                    close(client_sockets[i]);
                    client_sockets[i] = INVALID_SOCK;
                }
            }
        }

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(YIELD_TO_ALL_MS));
    }

error:
    if (listening_socket != INVALID_SOCK) {
        close(listening_socket);
    }

    for (int i=0; i<MAX_CLIENTS; ++i) {
        if (client_sockets[i] != INVALID_SOCK) {
            close(client_sockets[i]);
        }
    }

    vTaskDelete(NULL);



/* 
    //------- originad old code
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));


    uint16_t sin_p;
    cfg_get_u16(KEY_CONFIG_TCP_SERVER_PORT, &sin_p);

    struct sockaddr_in srv = {
        .sin_family      = AF_INET,
        .sin_port        = htons(sin_p),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_fd, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    listen(listen_fd, MAX_CLIENTS);
    ESP_LOGI(TAG, "TCP server listening on port %d", sin_p);

    int clients[MAX_CLIENTS];
    for (int i = 0; i < MAX_CLIENTS; i++) clients[i] = -1;

    while (1) {
        // ── Build fd_set ──────────────────────────────────────────────────────
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(listen_fd, &read_fds);
        int max_fd = listen_fd;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i] >= 0) {
                FD_SET(clients[i], &read_fds);
                if (clients[i] > max_fd) max_fd = clients[i];
            }
        }

        struct timeval tv = { .tv_sec = 0, .tv_usec = 1000 };
        select(max_fd + 1, &read_fds, NULL, NULL, &tv);

        // ── Accept new connections ────────────────────────────────────────────
        if (FD_ISSET(listen_fd, &read_fds)) {
            struct sockaddr_in cli_addr;
            socklen_t cli_len = sizeof(cli_addr);
            int new_fd = accept(listen_fd, (struct sockaddr *)&cli_addr, &cli_len);
            if (new_fd >= 0) {
                int flag = 1;
                setsockopt(new_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
                bool accepted = false;
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (clients[i] < 0) {
                        clients[i] = new_fd;
                        char ip_str[16];
                        inet_ntop(AF_INET, &cli_addr.sin_addr, ip_str, sizeof(ip_str));
                        ESP_LOGI(TAG, "Client %d connected: %s", i, ip_str);
                        accepted = true;
                        break;
                    }
                }
                if (!accepted) {
                    ESP_LOGW(TAG, "Max clients reached, refusing");
                    close(new_fd);
                }
            }
        }

        // ── Detect disconnects (readable with 0 bytes) ────────────────────────
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i] >= 0 && FD_ISSET(clients[i], &read_fds)) {
                uint8_t tmp;
                int n = recv(clients[i], &tmp, 1, MSG_DONTWAIT);
                if (n == 0 || (n < 0 && errno != EAGAIN)) {
                    ESP_LOGI(TAG, "Client %d disconnected", i);
                    close(clients[i]);
                    clients[i] = -1;
                }
                // Hook: forward config commands from UPrecise to UART TX here.
            }
        }

        // ── Drain queue and send to all connected clients ─────────────────────
        pool_frame_t *f;
        fixme - memory for f needs to be allocated!!!!
        while (xQueueReceive(q_tcp_server, &f, 0) == pdTRUE) {
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (clients[i] < 0) continue;
                int sent = send(clients[i], f->data, f->len, MSG_DONTWAIT);
                if (sent < 0) {
                    ESP_LOGW(TAG, "Client %d send error, closing", i);
                    close(clients[i]);
                    clients[i] = -1;
                }
            }
            pool_release(f);    // done with this frame
        }
    }
 */
}
