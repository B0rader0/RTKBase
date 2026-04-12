/*
 *  
 */

#include <web_server.h>
#include <esp_sntp.h>
#include <core_dump.h>
#include <esp_ota_ops.h>
#include <stream_stats.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include "nvs_config.h"
#include "wifi.h"
#include "gnss_uart.h"
#include "reset_button.h"
#include "rtk_base.h"
#include "ntrip_client.h"

static const char *TAG = "MAIN";

// ─── Shared queue handles (NULL = service disabled) ───────────────────────────
QueueHandle_t q_ntrip_1      = NULL;
QueueHandle_t q_ntrip_2      = NULL;
QueueHandle_t q_ntrip_server = NULL;

// ─── Task forward declarations ────────────────────────────────────────────────
void task_frame_splitter(void *arg);
void task_tcp_server(void *arg);
void task_ntrip_client(void *arg);
void task_ntrip_server(void *arg);
void task_web_server(void *arg);

static char *reset_reason_name(esp_reset_reason_t reason);
 
void app_main()
{
    
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    //log_init();
        
    // BUG: (and hence commented out) because it is not yet known what to redirect to the UART and WebSocket 
    // as clients are not configured yet!
    // Redirect ESP-IDF logs to our custom log implementation, which forwards them to UART and WebSocket clients
    // This is a bug, because it is not yet known what to redirect to
    // esp_log_set_vprintf(log_vprintf);
    
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_log_level_set("system_api", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);
    esp_log_level_set("reset_button", ESP_LOG_DEBUG);
    
    core_dump_check();

    esp_reset_reason_t reset_reason = esp_reset_reason();

    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║ Reset reason: %-30s "                       "║", reset_reason_name(reset_reason));
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");

    
    // RTK Base specific code
    
    // Initialize NVS with default values if they don't exist already in NVS
    // When the values are later needed, they are read from NVS (where they shoud exist)!
    // Call first, even before the defauld event loop?
    nvs_cfg_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "esp_event_loop_create_default OK");

    // Registers a handler for the BOOT button for resetting the parameters to factory defaults. 
    reset_button_init();
    ESP_LOGI(TAG, "reset_button_init OK");
    
    stream_stats_init(); // yet another task - Find out what this code does.
    
    net_init();
    ESP_LOGI(TAG, "net_init OK");

    wifi_init();
    ESP_LOGI(TAG, "wifi_init OK");  //reached and stopped.
    
    //wait_for_ip(); // bug because it blocks until STA is connected, but the device may be configured to only use AP mode, in which case it will never connect to a STA network and hence never get an IP address. The web server should still be accessible in AP mode, so we should not block the main task waiting for an IP address. Instead, the tasks that require an IP address (e.g. socket server, NTRIP client/server) should check for connectivity before trying to use the network, and either wait for connectivity or exit gracefully if it is not available.
    //ESP_LOGI(TAG, "wait_for_ip OK");
     
    web_server_init(); //Does this start the service?
    ESP_LOGI(TAG, "web_server_init OK");

    // ── Frame pool ────────────────────────────────────────────────────────────
    pool_init();
   
    // ── Queues (only for enabled services) ───────────────────────────────────
    uint8_t enbld;

    
    cfg_get_u8(KEY_CONFIG_NTRIP1_ACTIVE, &enbld);
    if (enbld)
        q_ntrip_1 = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(pool_frame_t *));

    cfg_get_u8(KEY_CONFIG_NTRIP2_ACTIVE, &enbld);
    if (enbld)
        q_ntrip_2 = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(pool_frame_t *));

    cfg_get_u8(KEY_CONFIG_CASTER_ACTIVE, &enbld);
    if (enbld)
        q_ntrip_server = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(pool_frame_t *));
    
   
    // ── Tasks ─────────────────────────────────────────────────────────────────
    // Each task initializes its parameters from NVS and then starts running. 
    // The tasks are pinned to cores and prioritized as follows:
    //
    //  Task                        | Core | Pri | Stack  | Notes
    //  ----------------------- |------|-----|--------|-----------------------------
    //  gnss_reader             |  1   |  5  |  4096  | isolated to core 1
    //  tcp_server (UPrecise)   |      |  0  |  4     |  4096  | if enabled
    //  ntrip_client (×2)       |  0   |  4  |  5120  | if enabled
    //  ntrip_caster            |  0   |  4  |  5120  | if enabled
    //  web_server              |  0   |  3  | 16384  | large stack: page gen +
    //                          |      |     |        | field_select bufs + httpd

    xTaskCreatePinnedToCore(task_gnss_reader, // reads and perfomrs frame splitting
                            "gnss_reader",
                            4096, NULL, 5, NULL, 1);
    
    cfg_get_u8(KEY_CONFIG_TCP_SERVER_ACTIVE, &enbld);
    if (enbld)
        xTaskCreatePinnedToCore(task_tcp_server, "tcp_srv",
                                4096, NULL, 4, NULL, 0);

    // Using the ques to infer which services are enabled, as the tasks need to be created conditionally based on the configuration, which is read in the task itself after it is created.
    
    
                                // A parameter (1 or 2) is passed to the task to indicate which NTRIP client it is.
    // The task reads the correct configuration values from NVS and connects to the correct server.
    if (q_ntrip_1) {
        ntrip_client_cfg_t *cfg = malloc(sizeof(ntrip_client_cfg_t));
        cfg_get_str(KEY_CONFIG_NTRIP1_HOST, &cfg->host);
        cfg_get_u16(KEY_CONFIG_NTRIP1_PORT, &cfg->port);
        cfg_get_str(KEY_CONFIG_NTRIP1_MOUNTPOINT, &cfg->mountpoint);
        cfg_get_str(KEY_CONFIG_NTRIP1_USERNAME, &cfg->username);
        cfg_get_str(KEY_CONFIG_NTRIP1_PASSWORD, &cfg->password);
        xTaskCreatePinnedToCore(task_ntrip_client, "ntrip_cli1", 5120, cfg, 4, NULL, 0);
    }

    // A parameter (1 or 2) is passed to the task to indicate which NTRIP client it is.
    // The task reads the correct configuration values from NVS and connects to the correct server.
    if (q_ntrip_2) {
    ntrip_client_cfg_t *cfg = malloc(sizeof(ntrip_client_cfg_t));
        cfg_get_str(KEY_CONFIG_NTRIP2_HOST, &cfg->host);
        cfg_get_u16(KEY_CONFIG_NTRIP2_PORT, &cfg->port);
        cfg_get_str(KEY_CONFIG_NTRIP2_MOUNTPOINT, &cfg->mountpoint);
        cfg_get_str(KEY_CONFIG_NTRIP2_USERNAME, &cfg->username);
        cfg_get_str(KEY_CONFIG_NTRIP2_PASSWORD, &cfg->password);
        xTaskCreatePinnedToCore(task_ntrip_client, "ntrip_cli2", 5120, cfg, 4, NULL, 0);    }

    if (q_ntrip_server)
        xTaskCreatePinnedToCore(task_ntrip_server, "ntrip_srv", 5120, NULL, 4, NULL, 0);

    // Web server: 12288 bytes — cJSON heap alloc is lean but the blocking
    // wifi_manager_scan() call uses significant internal WiFi stack frames.
/* 
    // The original web server is already started.
    xTaskCreatePinnedToCore(task_web_server, "web_srv",
                            12288, NULL, 3, NULL, 0);

 */
    // ── Startup log ───────────────────────────────────────────────────────────
    
 
}  // app_main

// Helper function to convert reset reason enum to string for logging purposes
static char *reset_reason_name(esp_reset_reason_t reason) {
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
} // reset_reason_name