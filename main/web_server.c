/*
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * Derived from the ESP32-XBee project:
 * https://github.com/nebkat/esp32-xbee
 *
 * Original work:
 * Copyright (c) 2019 Nebojsa Cvetkovic.
 *
 * Modified for RTKBase.
 */

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_system.h>
#include <wifi.h>
#include <cJSON.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdlib.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>
#include <mdns.h>
#include <log.h>
#include <core_dump.h>
#include <util.h>
#include <lwip/inet.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_wifi_ap_get_sta_list.h> 
#include <stream_stats.h>
#include <esp32/rom/crc.h>
#include <lwip/sockets.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <freertos/task.h>
#include "web_server.h"
#include "gnss_uart.h"
#include "ntrip_client.h"
#include "ntrip_caster.h"
#include "tcp_server.h"
#include "time_sync.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"                    // MACSTR/MAC2STR (GN)
#include "nvs_config.h"
#include <inttypes.h>   // at top of file (GN)

// Max length a file path can have on storage
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)
#define FILE_HASH_SUFFIX ".crc"

#define WWW_PARTITION_PATH "/www"
#define WWW_PARTITION_LABEL "www"
#define BUFFER_SIZE 2048
#define OTA_UPLOAD_BUF_SIZE 2048
#define RESTART_TASK_STACK_SIZE 4096
#define WEB_SERVER_MAX_OPEN_SOCKETS 3

static const char *TAG = "WEB";

static char *buffer;

enum auth_method {
    AUTH_METHOD_OPEN = 0,
    AUTH_METHOD_HOTSPOT = 1,
    AUTH_METHOD_BASIC = 2
};

static char *basic_authentication;
static enum auth_method auth_method;
static bool restart_scheduled;

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
        default:
        case ESP_RST_UNKNOWN:
            return "UNKNOWN";
        case ESP_RST_POWERON:
            return "POWERON";
        case ESP_RST_EXT:
            return "EXTERNAL";
        case ESP_RST_SW:
            return "SOFTWARE";
        case ESP_RST_PANIC:
            return "PANIC";
        case ESP_RST_INT_WDT:
            return "INTERRUPT_WATCHDOG";
        case ESP_RST_TASK_WDT:
            return "TASK_WATCHDOG";
        case ESP_RST_WDT:
            return "OTHER_WATCHDOG";
        case ESP_RST_DEEPSLEEP:
            return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:
            return "BROWNOUT";
        case ESP_RST_SDIO:
            return "SDIO";
    }
}

#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

static esp_err_t www_spiffs_init() {
    ESP_LOGI(TAG, "Initializing SPIFFS...");

    esp_vfs_spiffs_conf_t conf = {
            .base_path = WWW_PARTITION_PATH,
            .partition_label = WWW_PARTITION_LABEL,
            .max_files = 10,
            .format_if_mount_failed = false
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(WWW_PARTITION_LABEL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

   ESP_LOGI(TAG, "SPIFFS: total=%d bytes, used=%d bytes (%.1f%%)", 
             total, used, (used * 100.0) / total);
    
    DIR *dir = opendir(WWW_PARTITION_PATH);
    if (dir) {
        closedir(dir);
    } else {
        ESP_LOGE(TAG, "Failed to open directory %s", WWW_PARTITION_PATH);
    }
    
    return ESP_OK;
}

// Set HTTP response content type according to file extension
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".js")) {
        return httpd_resp_set_type(req, "application/javascript");
    } else if (IS_FILE_EXT(filename, ".css")) {
        return httpd_resp_set_type(req, "text/css");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest) {
        pathlen = MIN(pathlen, quest - uri);
    }
    const char *hash = strchr(uri, '#');
    if (hash) {
        pathlen = MIN(pathlen, hash - uri);
    }

    if (base_pathlen + pathlen + 1 > destsize) {
        // Full path string won't fit into destination buffer
        return NULL;
    }

    // Construct full path (base + path)
    strlcpy(dest, base_path, destsize);
    memcpy(dest + base_pathlen, uri, pathlen);
    dest[base_pathlen + pathlen] = '\0';

    // Return pointer to path, skipping the base
    return dest + base_pathlen;
}

static esp_err_t json_response(httpd_req_t *req, cJSON *root) {
    // Set mime type
    esp_err_t err = httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) return err;

    // Convert to string
    bool success = cJSON_PrintPreallocated(root, buffer, BUFFER_SIZE, false);
    cJSON_Delete(root);
    if (!success) {
        ESP_LOGE(TAG, "Not enough space in buffer to output JSON");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Not enough space in buffer to output JSON");
        return ESP_FAIL;
    }

    // Send as response
    err = httpd_resp_send(req, buffer, strlen(buffer));
    if (err != ESP_OK) return err;

    return ESP_OK;
}

static esp_err_t basic_auth(httpd_req_t *req) {
    int authorization_length = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
    if (authorization_length == 0) goto _auth_required;

    char *authorization_header = malloc(authorization_length);
    httpd_req_get_hdr_value_str(req, "Authorization", authorization_header, authorization_length);

    bool authenticated = strcasecmp(basic_authentication, authorization_header) == 0;
    free(authorization_header);

    if (authenticated) return ESP_OK;

    _auth_required:
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"ESP32 XBee Config\"");
    httpd_resp_set_status(req, "401"); // Unauthorized
    char *unauthorized = "401 Unauthorized - Incorrect or no password provided";
    httpd_resp_send(req, unauthorized, strlen(unauthorized));
    return ESP_FAIL;
}

static esp_err_t hotspot_auth(httpd_req_t *req) {
    int sock = httpd_req_to_sockfd(req);

    struct sockaddr_in6 client_addr;
    socklen_t socklen = sizeof(client_addr);
    getpeername(sock, (struct sockaddr *)&client_addr, &socklen);

    // TODO: Correctly read IPv4?
    // ERROR_ACTION(TAG, client_addr.sin6_family != AF_INET, goto _auth_error, "IPv6 connections not supported, IP family %d", client_addr.sin6_family);

    // wifi_sta_list_t *ap_sta_list = wifi_ap_sta_list();
    // esp_netif_sta_list_t esp_netif_ap_sta_list;
    // esp_netif_get_sta_list(ap_sta_list, &esp_netif_ap_sta_list);

    // // TODO: Correctly read IPv4?
    // for (int i = 0; i < esp_netif_ap_sta_list.num; i++) {
    //     if (esp_netif_ap_sta_list.sta[i].ip.addr == client_addr.sin6_addr.un.u32_addr[3]) return ESP_OK;
    // }

    // new IDF v5 style below (GN)
    wifi_sta_list_t wifi_sta_list = {0};
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&wifi_sta_list)); // MACs only

    wifi_sta_mac_ip_list_t mac_ip_list = {0};
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list_with_ip(&wifi_sta_list, &mac_ip_list)); // MAC + IPv4

    for (int i = 0; i < mac_ip_list.num; ++i)
    {
        const esp_netif_pair_mac_ip_t *e = &mac_ip_list.sta[i];
        ESP_LOGI(TAG, "STA " MACSTR "  IP: " IPSTR, MAC2STR(e->mac), IP2STR(&e->ip));
    }
    
    //_auth_error:
    httpd_resp_set_status(req, "401"); // Unauthorized
    char *unauthorized = "401 Unauthorized - Configured to only accept connections from hotspot devices";
    httpd_resp_send(req, unauthorized, strlen(unauthorized));
    return ESP_FAIL;
}

static esp_err_t check_auth(httpd_req_t *req) {
    if (auth_method == AUTH_METHOD_HOTSPOT) return hotspot_auth(req);
    if (auth_method == AUTH_METHOD_BASIC) return basic_auth(req);
    return ESP_OK;
}


// ─── Get log data as plain text ─────────────────────────────────────────────
// This is used by the web interface to show recent log messages, and can also be used to download the log as a file.
static esp_err_t log_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    char *log_buffer = malloc(LOG_SNAPSHOT_BUFFER_SIZE);
    if (log_buffer == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No log buffer");
        return ESP_FAIL;
    }

    size_t length = log_snapshot(log_buffer, LOG_SNAPSHOT_BUFFER_SIZE);
    esp_err_t err = httpd_resp_send(req, log_buffer, length);
    free(log_buffer);
    return err;
}

static void sanitize_filename_part(char *value, const char *fallback)
{
    bool had_output = false;

    for (char *p = value; *p != '\0'; p++) {
        char c = *p;
        bool allowed = (c >= 'A' && c <= 'Z') ||
                       (c >= 'a' && c <= 'z') ||
                       (c >= '0' && c <= '9') ||
                       c == '-' || c == '_' || c == '.';
        if (!allowed) {
            *p = '_';
        }
        if (*p != '_' && *p != '.') {
            had_output = true;
        }
    }

    if (!had_output && fallback != NULL) {
        strlcpy(value, fallback, strlen(fallback) + 1);
    }
}

static esp_err_t core_dump_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    size_t core_dump_size = core_dump_available();
    if (core_dump_size == 0) {
        httpd_resp_sendstr(req, "No core dump available");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/octet-stream");

    const esp_app_desc_t *app_desc = esp_app_get_description(); // new for IDF v5 

    char elf_sha256[7];
    esp_app_get_elf_sha256(elf_sha256, sizeof(elf_sha256)); // new for IDF v5 

    char app_version[33];
    strlcpy(app_version, app_desc->version, sizeof(app_version));
    sanitize_filename_part(app_version, "unknown");

    char station_name[33] = "rtkbase";
    char *configured_name = NULL;
    if (cfg_get_str(KEY_CONFIG_STATION_HOSTNAME, &configured_name) == ESP_OK &&
        configured_name != NULL && configured_name[0] != '\0') {
        strlcpy(station_name, configured_name, sizeof(station_name));
    }
    free(configured_name);
    sanitize_filename_part(station_name, "rtkbase");

    time_t t = time(NULL);
    char date[20] = "\0";
    if (t > 315360000l) strftime(date, sizeof(date), "_%F_%T", localtime(&t));
    sanitize_filename_part(date, NULL);

    char content_disposition[192];
    snprintf(content_disposition, sizeof(content_disposition),
            "attachment; filename=\"%s_%s_core_dump_%s%s.elf\"",
            station_name, app_version, elf_sha256, date);
    httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);

    for (size_t offset = 0; offset < core_dump_size; offset += BUFFER_SIZE) {
        size_t read = core_dump_size - offset;
        if (read > BUFFER_SIZE) read = BUFFER_SIZE;

        esp_err_t read_err = core_dump_read(offset, buffer, read);
        if (read_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read core dump at offset %u: %s",
                     (unsigned)offset, esp_err_to_name(read_err));
            return ESP_FAIL;
        }

        esp_err_t send_err = httpd_resp_send_chunk(req, buffer, read);
        if (send_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send core dump chunk at offset %u: %s",
                     (unsigned)offset, esp_err_to_name(send_err));
            return send_err;
        }
    }

    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

static esp_err_t heap_info_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_DEFAULT);

    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "total_free_bytes", info.total_free_bytes);
    cJSON_AddNumberToObject(root, "total_allocated_bytes", info.total_allocated_bytes);
    cJSON_AddNumberToObject(root, "largest_free_block", info.largest_free_block);
    cJSON_AddNumberToObject(root, "minimum_free_bytes", info.minimum_free_bytes);
    cJSON_AddNumberToObject(root, "allocated_blocks", info.allocated_blocks);
    cJSON_AddNumberToObject(root, "free_blocks", info.free_blocks);
    cJSON_AddNumberToObject(root, "total_blocks", info.total_blocks);

    return json_response(req, root);
}

static esp_err_t file_check_etag_hash(httpd_req_t *req, char *file_hash_path, char *etag, size_t etag_size) {
    struct stat file_hash_stat;
    if (stat(file_hash_path, &file_hash_stat) == -1) {
        // Hash file not created yet
        return ESP_ERR_NOT_FOUND;
    }

    FILE *fd_hash = fopen(file_hash_path, "r+");

    // Ensure hash file was opened
    ERROR_ACTION(TAG, fd_hash == NULL, return ESP_FAIL,
            "Could not open hash file %s (%lu bytes) for reading/updating: %d %s", file_hash_path,
            file_hash_stat.st_size, errno, strerror(errno));

    // Read existing hash
    uint32_t crc;
    int read = fread(&crc, sizeof(crc), 1, fd_hash);
    fclose(fd_hash);
    ERROR_ACTION(TAG, read != 1, return ESP_FAIL,
            "Could not read hash file %s: %d %s", file_hash_path,
            errno, strerror(errno));
   snprintf(etag, etag_size, "\"%08" PRIX32 "\"", crc);

    // Compare to header sent by client
    size_t if_none_match_length = httpd_req_get_hdr_value_len(req, "If-None-Match") + 1;
    if (if_none_match_length > 1) {
        char *if_none_match = malloc(if_none_match_length);
        httpd_req_get_hdr_value_str(req, "If-None-Match", if_none_match, if_none_match_length);

        bool header_match = strcmp(etag, if_none_match) == 0;
        
        // Matching ETag, return not modified
        if (header_match) {
            free(if_none_match); 
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "ETag for file %s sent by client does not match (%s != %s)", file_hash_path, etag, if_none_match);
            free(if_none_match); 
            return ESP_ERR_INVALID_CRC;
        }
    }

    return ESP_ERR_INVALID_ARG;
}

static esp_err_t file_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    char file_path[FILE_PATH_MAX - strlen(FILE_HASH_SUFFIX)];
    char file_hash_path[FILE_PATH_MAX];
    FILE *fd = NULL, *fd_hash = NULL;
    struct stat file_stat;

    // Extract filename from URL
    char *file_name = get_path_from_uri(file_path, WWW_PARTITION_PATH, req->uri, sizeof(file_path));
    ERROR_ACTION(TAG, file_name == NULL, {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }, "Filename too long")

    // If name has trailing '/', respond with index page
    if (file_name[strlen(file_name) - 1] == '/' && strlen(file_name) + strlen("index.html") < FILE_PATH_MAX) {
        strlcat(file_name, "index.html", FILE_PATH_MAX);

        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    }

    set_content_type_from_file(req, file_name);

    // Check if file exists
    ERROR_ACTION(TAG, stat(file_path, &file_stat) == -1, {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }, "Could not stat file %s", file_path)

    // Check file hash (if matches request, file is not modified)
    strlcpy(file_hash_path, file_path, sizeof(file_hash_path));
    strlcat(file_hash_path, FILE_HASH_SUFFIX, sizeof(file_hash_path));
    char etag[8 + 2 + 1] = ""; // Store CRC32, quotes and \0
    if (file_check_etag_hash(req, file_hash_path, etag, sizeof(etag)) == ESP_OK) {
        httpd_resp_set_status(req, "304 Not Modified");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    if (strlen(etag) > 0) httpd_resp_set_hdr(req, "ETag", etag);

    fd = fopen(file_path, "r");
    ERROR_ACTION(TAG, fd == NULL, {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Could not read file");
        return ESP_FAIL;
    }, "Could not read file %s", file_path)

    // Retrieve the pointer to scratch buffer for temporary storage
    size_t length;
    uint32_t crc = 0;
    do {
        // Read file in chunks into the scratch buffer
        length = fread(buffer, 1, BUFFER_SIZE, fd);

        // Send the buffer contents as HTTP response chunk
        if (httpd_resp_send_chunk(req, buffer, length) != ESP_OK) {
            ESP_LOGE(TAG, "Failed sending file %s", file_name);
            httpd_resp_sendstr_chunk(req, NULL);

            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");

            fclose(fd);
            return ESP_FAIL;
        }

        // Update checksum
        crc = crc32_le(crc, (const uint8_t *)buffer, length);
    } while (length != 0);

    // Close file after sending complete
    fclose(fd);

    // Store CRC hash
    fd_hash = fopen(file_hash_path, "w");
    if (fd_hash != NULL) {
        fwrite(&crc, sizeof(crc), 1, fd_hash);
        fclose(fd_hash);
    } else {
        ESP_LOGW(TAG, "Could not open hash file %s for writing: %d %s", file_hash_path, errno, strerror(errno));
    }

    return ESP_OK;
}

// Provides response to the GET request from the WEB interface
static esp_err_t config_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    cJSON *root = cJSON_CreateObject();

    const esp_app_desc_t *app_desc = esp_app_get_description(); // new for IDF v5 (GN)
    cJSON_AddStringToObject(root, "version", app_desc->version);

    cfg_to_json(root); //root is a pointer

    return json_response(req, root);
}

static void delayed_restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static esp_err_t schedule_restart(const char *task_name)
{
    if (restart_scheduled) {
        return ESP_OK;
    }

    BaseType_t task_ok = xTaskCreate(
            delayed_restart_task,
            task_name,
            RESTART_TASK_STACK_SIZE,
            NULL,
            TASK_PRIORITY_INTERFACE,
            NULL);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to schedule restart task %s", task_name);
        return ESP_FAIL;
    }

    restart_scheduled = true;
    return ESP_OK;
}

static esp_err_t config_post_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    int ret = httpd_req_recv(req, buffer, BUFFER_SIZE - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }

        return ESP_FAIL;
    }

    buffer[ret] = '\0';

    cJSON *root = cJSON_Parse(buffer);

    esp_err_t err = cfg_json_to_nvs(root);
    if (err != ESP_OK) {
        cJSON_Delete(root);

        ESP_LOGE(TAG, "Failed cfg_json_to_nvs in post handler: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to save config from JSON");
        return ESP_FAIL;
    }   
    
    cJSON_Delete(root);

    
    root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);

    esp_err_t response_err = json_response(req, root);
    if (response_err != ESP_OK) {
        return response_err;
    }

    if (schedule_restart("cfg_restart") != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;

}

static esp_err_t tcp_server_disconnect_post_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", tcp_server_disconnect_client());

    return json_response(req, root);
}

static esp_err_t ota_firmware_post_handler(httpd_req_t *req)
{
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    char *upload_buf = malloc(OTA_UPLOAD_BUF_SIZE);
    if (upload_buf == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No upload buffer");
        return ESP_FAIL;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        free(upload_buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    if (req->content_len <= 0 || req->content_len > update_partition->size) {
        free(upload_buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid firmware image size");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        free(upload_buf);
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int received = httpd_req_recv(req, upload_buf, MIN(remaining, OTA_UPLOAD_BUF_SIZE));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            err = ESP_FAIL;
            break;
        }

        err = esp_ota_write(ota_handle, upload_buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            break;
        }
        remaining -= received;
    }

    free(upload_buf);

    if (err == ESP_OK) {
        err = esp_ota_end(ota_handle);
    } else {
        esp_ota_abort(ota_handle);
    }

    if (err == ESP_OK) {
        err = esp_ota_set_boot_partition(update_partition);
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", err == ESP_OK);
    cJSON_AddStringToObject(root, "message", err == ESP_OK ? "Firmware uploaded. Restart to boot it." : esp_err_to_name(err));
    return json_response(req, root);
}

static esp_err_t ota_www_post_handler(httpd_req_t *req)
{
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    const esp_partition_t *www_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, WWW_PARTITION_LABEL);
    if (www_partition == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No www partition");
        return ESP_FAIL;
    }

    if (req->content_len != www_partition->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid www image size");
        return ESP_FAIL;
    }

    char *upload_buf = malloc(OTA_UPLOAD_BUF_SIZE);
    if (upload_buf == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No upload buffer");
        return ESP_FAIL;
    }

    esp_err_t err = esp_partition_erase_range(www_partition, 0, www_partition->size);
    int remaining = req->content_len;
    size_t offset = 0;

    while (err == ESP_OK && remaining > 0) {
        int received = httpd_req_recv(req, upload_buf, MIN(remaining, OTA_UPLOAD_BUF_SIZE));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            err = ESP_FAIL;
            break;
        }

        err = esp_partition_write(www_partition, offset, upload_buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_partition_write(www) failed: %s", esp_err_to_name(err));
            break;
        }
        offset += (size_t)received;
        remaining -= received;
    }

    free(upload_buf);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", err == ESP_OK);
    cJSON_AddStringToObject(root, "message", err == ESP_OK ? "Web image uploaded. Restart to remount it." : esp_err_to_name(err));
    return json_response(req, root);
}

static esp_err_t ota_restart_post_handler(httpd_req_t *req)
{
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    esp_err_t response_err = json_response(req, root);

    if (schedule_restart("ota_restart") != ESP_OK) {
        return ESP_FAIL;
    }

    return response_err;
}

static esp_err_t admin_restart_post_handler(httpd_req_t *req)
{
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Restart scheduled.");
    esp_err_t response_err = json_response(req, root);
    if (response_err != ESP_OK) {
        return response_err;
    }

    if (schedule_restart("admin_restart") != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t admin_factory_reset_post_handler(httpd_req_t *req)
{
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    esp_err_t erase_err = nvs_flash_erase();
    if (erase_err != ESP_OK) {
        ESP_LOGE(TAG, "Factory reset nvs_flash_erase failed: %s", esp_err_to_name(erase_err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Factory reset failed");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    cJSON_AddStringToObject(root, "message", "Factory reset scheduled.");
    esp_err_t response_err = json_response(req, root);
    if (response_err != ESP_OK) {
        return response_err;
    }

    if (schedule_restart("factory_reset") != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t gnss_command_post_handler(httpd_req_t *req)
{
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    if (tcp_server_client_connected()) {
        httpd_resp_set_status(req, "409 Conflict");
        cJSON *root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "success", false);
        cJSON_AddStringToObject(root, "message", "TCP client connected. Disconnect it before sending GNSS commands from the web UI.");
        return json_response(req, root);
    }

    if (req->content_len <= 0 || req->content_len >= BUFFER_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid command size");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int offset = 0;
    while (remaining > 0) {
        int received = httpd_req_recv(req, buffer + offset, remaining);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        offset += received;
        remaining -= received;
    }
    buffer[offset] = '\0';

    esp_err_t err = gnss_uart_send_command(buffer, (size_t)offset);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", err == ESP_OK);
    cJSON_AddStringToObject(root, "message", err == ESP_OK ? "Command sent." : esp_err_to_name(err));
    return json_response(req, root);
}

static esp_err_t gnss_log_get_handler(httpd_req_t *req)
{
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    char *log_buffer = malloc(GNSS_CONSOLE_SNAPSHOT_BUFFER_SIZE);
    if (log_buffer == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No GNSS console buffer");
        return ESP_FAIL;
    }

    size_t length = gnss_uart_console_snapshot(log_buffer, GNSS_CONSOLE_SNAPSHOT_BUFFER_SIZE);
    esp_err_t err = httpd_resp_send(req, log_buffer, length);
    free(log_buffer);
    return err;
}

static esp_err_t gnss_log_clear_post_handler(httpd_req_t *req)
{
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    gnss_uart_console_clear();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "success", true);
    return json_response(req, root);
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    cJSON *root = cJSON_CreateObject();

    // Uptime / reset
    uint64_t uptime_ms = (uint64_t)(esp_timer_get_time() / 1000);
    cJSON_AddNumberToObject(root, "uptime", (int)(uptime_ms / 1000));
    cJSON_AddNumberToObject(root, "uptime_ms", (double)uptime_ms);
    cJSON_AddStringToObject(root, "reset_reason", reset_reason_name(esp_reset_reason()));

    // Local wall time
    cJSON *time_json = cJSON_AddObjectToObject(root, "time");
    time_t now = time(NULL);
    bool time_valid = time_sync_time_valid();
    cJSON_AddBoolToObject(time_json, "valid", time_valid);
    cJSON_AddNumberToObject(time_json, "unix", (double)now);
    if (time_valid) {
        struct tm local_time;
        char time_text[32];
        localtime_r(&now, &local_time);
        strftime(time_text, sizeof(time_text), "%Y-%m-%d %H:%M:%S", &local_time);
        cJSON_AddStringToObject(time_json, "local", time_text);
    } else {
        cJSON_AddStringToObject(time_json, "local", "not synchronized");
    }

    // Heap
    cJSON *heap = cJSON_AddObjectToObject(root, "heap");
    cJSON_AddNumberToObject(heap, "total", heap_caps_get_total_size(MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(heap, "free", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(heap, "min_free", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(heap, "largest_free_block", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // Streams
    cJSON *streams = cJSON_AddObjectToObject(root, "streams");
    stream_stats_values_t values;
    for (stream_stats_handle_t stats = stream_stats_first(); stats != NULL; stats = stream_stats_next(stats)) {
        stream_stats_values(stats, &values);

        cJSON *stream = cJSON_AddObjectToObject(streams, values.name);
        cJSON *total = cJSON_AddObjectToObject(stream, "total");
        cJSON_AddNumberToObject(total, "in", values.total_in);
        cJSON_AddNumberToObject(total, "out", values.total_out);
        cJSON *rate = cJSON_AddObjectToObject(stream, "rate");
        cJSON_AddNumberToObject(rate, "in", values.rate_in);
        cJSON_AddNumberToObject(rate, "out", values.rate_out);
    }

    cJSON *tcp_server = cJSON_AddObjectToObject(root, "tcp_server");
    cJSON_AddBoolToObject(tcp_server, "connected", tcp_server_client_connected());
    cJSON_AddStringToObject(tcp_server, "endpoint", tcp_server_client_endpoint());
    tcp_server_diag_t tcp_diag = {0};
    tcp_server_get_diagnostics(&tcp_diag);
    cJSON *tcp_diag_json = cJSON_AddObjectToObject(tcp_server, "diag");
    cJSON_AddNumberToObject(tcp_diag_json, "dropped_frames", tcp_diag.dropped_frames);
    cJSON_AddNumberToObject(tcp_diag_json, "dropped_bytes", tcp_diag.dropped_bytes);
    cJSON_AddNumberToObject(tcp_diag_json, "send_block_events", tcp_diag.send_block_events);
    cJSON_AddNumberToObject(tcp_diag_json, "send_deferred_events", tcp_diag.send_deferred_events);
    cJSON_AddNumberToObject(tcp_diag_json, "send_fatal_errors", tcp_diag.send_fatal_errors);
    cJSON_AddNumberToObject(tcp_diag_json, "pending_bytes", tcp_diag.pending_bytes);
    cJSON_AddNumberToObject(tcp_diag_json, "queue_fill", tcp_diag.queue_fill);
    cJSON_AddNumberToObject(tcp_diag_json, "queue_depth", tcp_diag.queue_depth);

    ntrip_client_diag_t ntrip_client_diag = {0};
    ntrip_client_get_diagnostics(&ntrip_client_diag);
    cJSON *ntrip_clients = cJSON_AddObjectToObject(root, "ntrip_clients");
    for (int i = 0; i < 2; i++) {
        char name[8];
        snprintf(name, sizeof(name), "uplink%d", i + 1);
        cJSON *uplink = cJSON_AddObjectToObject(ntrip_clients, name);
        cJSON_AddBoolToObject(uplink, "connected", (ntrip_client_diag.connected_mask & (1u << i)) != 0);
        cJSON_AddNumberToObject(uplink, "queue_drops", ntrip_client_diag.queue_drops[i]);
        cJSON_AddNumberToObject(uplink, "queue_fill", ntrip_client_diag.queue_fill[i]);
        cJSON_AddNumberToObject(uplink, "queue_depth", ntrip_client_diag.queue_depth);
        cJSON_AddNumberToObject(uplink, "send_block_events", ntrip_client_diag.send_block_events[i]);
        cJSON_AddNumberToObject(uplink, "send_fatal_errors", ntrip_client_diag.send_fatal_errors[i]);
        cJSON_AddNumberToObject(uplink, "reconnect_events", ntrip_client_diag.reconnect_events[i]);
    }

    ntrip_caster_diag_t ntrip_caster_diag = {0};
    ntrip_caster_get_diagnostics(&ntrip_caster_diag);
    cJSON *ntrip_caster = cJSON_AddObjectToObject(root, "ntrip_caster");
    cJSON_AddNumberToObject(ntrip_caster, "queue_drops", ntrip_caster_diag.queue_drops);
    cJSON_AddNumberToObject(ntrip_caster, "queue_fill", ntrip_caster_diag.queue_fill);
    cJSON_AddNumberToObject(ntrip_caster, "queue_depth", ntrip_caster_diag.queue_depth);
    cJSON_AddNumberToObject(ntrip_caster, "rover_slow_drops", ntrip_caster_diag.rover_slow_drops);
    cJSON_AddNumberToObject(ntrip_caster, "rover_block_events", ntrip_caster_diag.rover_block_events);
    cJSON_AddNumberToObject(ntrip_caster, "rover_send_errors", ntrip_caster_diag.rover_send_errors);
    cJSON_AddNumberToObject(ntrip_caster, "active_rovers", ntrip_caster_diag.active_rovers);

    gnss_uart_diag_t gnss_diag = {0};
    gnss_uart_get_diagnostics(&gnss_diag);
    cJSON *gnss = cJSON_AddObjectToObject(root, "gnss");
    cJSON_AddNumberToObject(gnss, "rtcm_frames", gnss_diag.rtcm_frames);
    cJSON_AddNumberToObject(gnss, "last_rtcm_ms", (double)gnss_diag.last_rtcm_ms);
    cJSON_AddNumberToObject(gnss, "last_rtcm_age_ms",
                             gnss_diag.rtcm_frames > 0 &&
                                 gnss_diag.last_rtcm_ms > 0 &&
                                 uptime_ms >= gnss_diag.last_rtcm_ms
                                 ? (double)(uptime_ms - gnss_diag.last_rtcm_ms)
                                 : -1);
    cJSON_AddNumberToObject(gnss, "rtcm_bad_len_count", gnss_diag.rtcm_bad_len_count);
    cJSON_AddNumberToObject(gnss, "rtcm_crc_fail_count", gnss_diag.rtcm_crc_fail_count);

    const esp_app_desc_t *app_desc = esp_app_get_description();
    cJSON *firmware = cJSON_AddObjectToObject(root, "firmware");
    cJSON_AddStringToObject(firmware, "version", app_desc->version);

    // Sockets
    cJSON *sockets = cJSON_AddArrayToObject(root, "sockets");
    for (int s = LWIP_SOCKET_OFFSET; s < LWIP_SOCKET_OFFSET + CONFIG_LWIP_MAX_SOCKETS; s++) {
        int err;

        int socktype;
        socklen_t socktype_len = sizeof(socktype);
        err = getsockopt(s, SOL_SOCKET, SO_TYPE, &socktype, &socktype_len);
        if (err < 0) continue;

        cJSON *socket = cJSON_CreateObject();

        cJSON_AddStringToObject(socket, "type", SOCKTYPE_NAME(socktype));

        struct sockaddr_in6 addr;
        socklen_t socklen = sizeof(addr);

        err = getsockname(s, (struct sockaddr *)&addr, &socklen);
        if (err == 0) cJSON_AddStringToObject(socket, "local", sockaddrtostr((struct sockaddr *) &addr));

        err = getpeername(s, (struct sockaddr *)&addr, &socklen);
        if (err == 0) cJSON_AddStringToObject(socket, "peer", sockaddrtostr((struct sockaddr *) &addr));

        cJSON_AddItemToArray(sockets, socket);
    }

    // WiFi
    wifi_ap_status_t ap_status;
    wifi_sta_status_t sta_status;

    wifi_ap_status(&ap_status);
    wifi_sta_status(&sta_status);

    cJSON *wifi = cJSON_AddObjectToObject(root, "wifi");

    cJSON *ap = cJSON_AddObjectToObject(wifi, "ap");
    cJSON_AddBoolToObject(ap, "active", ap_status.active);
    if (ap_status.active) {
        cJSON_AddStringToObject(ap, "ssid", (char *) ap_status.ssid);
        cJSON_AddStringToObject(ap, "authmode", wifi_auth_mode_name(ap_status.authmode));
        cJSON_AddNumberToObject(ap, "devices", ap_status.devices);

        char ip[40];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ap_status.ip4_addr));
        cJSON_AddStringToObject(ap, "ip4", ip);
        snprintf(ip, sizeof(ip), IPV6STR, IPV62STR(ap_status.ip6_addr));
        cJSON_AddStringToObject(ap, "ip6", ip);
    }

    cJSON *sta = cJSON_AddObjectToObject(wifi, "sta");
    cJSON_AddBoolToObject(sta, "active", sta_status.active);
    if (sta_status.active) {
        cJSON_AddBoolToObject(sta, "connected", sta_status.connected);
        if (sta_status.connected) {
            cJSON_AddStringToObject(sta, "ssid", (char *) sta_status.ssid);
            cJSON_AddStringToObject(sta, "authmode", wifi_auth_mode_name(sta_status.authmode));
            cJSON_AddNumberToObject(sta, "rssi", sta_status.rssi);

            char ip[40];
            snprintf(ip, sizeof(ip), IPSTR, IP2STR(&sta_status.ip4_addr));
            cJSON_AddStringToObject(sta, "ip4", ip);
            snprintf(ip, sizeof(ip), IPV6STR, IPV62STR(sta_status.ip6_addr));
            cJSON_AddStringToObject(sta, "ip6", ip);
        }
    }

    return json_response(req, root);
}

static esp_err_t wifi_scan_get_handler(httpd_req_t *req) {
    if (check_auth(req) == ESP_FAIL) return ESP_FAIL;

    uint16_t ap_count;
    wifi_ap_record_t *ap_records =  wifi_scan(&ap_count);

    cJSON *root = cJSON_CreateArray();
    for (int i = 0; i < ap_count; i++) {
        wifi_ap_record_t *ap_record = &ap_records[i];
        cJSON *ap = cJSON_CreateObject();
        cJSON_AddItemToArray(root, ap);
        cJSON_AddStringToObject(ap, "ssid", (char *) ap_record->ssid);
        cJSON_AddNumberToObject(ap, "rssi", ap_record->rssi);
        cJSON_AddStringToObject(ap, "authmode", wifi_auth_mode_name(ap_record->authmode));
    }

    free(ap_records);

    return json_response(req, root);
}

static esp_err_t register_uri_handler(httpd_handle_t server, const char *path, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *r)) {
    httpd_uri_t uri_config_get = {
            .uri        = path,
            .method     = method,
            .handler    = handler
    };
    esp_err_t err = httpd_register_uri_handler(server, &uri_config_get);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register URI handler %s [%d]: %s", path, method, esp_err_to_name(err));
    }
    return err;
}

static httpd_handle_t web_server_start(void)
{
    cfg_get_u8(KEY_CONFIG_ADMIN_AUTH, (uint8_t*) &auth_method);
    if (auth_method == AUTH_METHOD_BASIC) {
        char *username, *password;
        cfg_get_str(KEY_CONFIG_ADMIN_USERNAME, &username);
        cfg_get_str(KEY_CONFIG_ADMIN_PASSWORD, &password);
        basic_authentication = http_auth_basic_header(username, password);
        free(username);
        free(password);
    }

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144;
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 18;
    config.max_open_sockets = WEB_SERVER_MAX_OPEN_SOCKETS;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 2;
    config.send_wait_timeout = 2;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        register_uri_handler(server, "/config", HTTP_GET, config_get_handler);
        register_uri_handler(server, "/config", HTTP_POST, config_post_handler);
        register_uri_handler(server, "/tcp_server/disconnect", HTTP_POST, tcp_server_disconnect_post_handler);
        register_uri_handler(server, "/ota/firmware", HTTP_POST, ota_firmware_post_handler);
        register_uri_handler(server, "/ota/www", HTTP_POST, ota_www_post_handler);
        register_uri_handler(server, "/ota/restart", HTTP_POST, ota_restart_post_handler);
        register_uri_handler(server, "/admin/restart", HTTP_POST, admin_restart_post_handler);
        register_uri_handler(server, "/admin/factory_reset", HTTP_POST, admin_factory_reset_post_handler);
        register_uri_handler(server, "/gnss/command", HTTP_POST, gnss_command_post_handler);
        register_uri_handler(server, "/gnss/log", HTTP_GET, gnss_log_get_handler);
        register_uri_handler(server, "/gnss/log/clear", HTTP_POST, gnss_log_clear_post_handler);
        register_uri_handler(server, "/status", HTTP_GET, status_get_handler);
        register_uri_handler(server, "/log", HTTP_GET, log_get_handler);
        register_uri_handler(server, "/core_dump", HTTP_GET, core_dump_get_handler);
        register_uri_handler(server, "/heap_info", HTTP_GET, heap_info_get_handler);
        register_uri_handler(server, "/wifi/scan", HTTP_GET, wifi_scan_get_handler);
        register_uri_handler(server, "/*", HTTP_GET, file_get_handler);
    }

    if (server == NULL) {
        ESP_LOGE(TAG, "Could not start server");
        return NULL;
    }

    buffer = malloc(BUFFER_SIZE);

    return server;
} //web_server_start

void web_server_init() {
    ESP_ERROR_CHECK(www_spiffs_init());
    web_server_start();
} //web_server_init
