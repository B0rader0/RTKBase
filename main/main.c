/*
 *  
 */

#include <web_server.h>
#include <core_dump.h>
#include <log.h>
#include <stream_stats.h>
#include <freertos/FreeRTOS.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_log.h>
#include "nvs_config.h"
#include "wifi.h"
#include "gnss_uart.h"
#include "reset_button.h"
#include "frame_pool.h"
#include "ntrip_client.h"
#include "ntrip_caster.h"
#include "tcp_server.h"

static const char *TAG = "MAIN";

static char *reset_reason_name(esp_reset_reason_t reason);
static void ota_mark_running_app_valid(void);
 
void app_main()
{
    
    esp_log_level_set(TAG, ESP_LOG_INFO);
    log_init();
    
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

    gnss_uart_init();
    tcp_server_init();
    ntrip_client_init();
    ntrip_caster_init();

    ota_mark_running_app_valid();

    // Web server: 12288 bytes — cJSON heap alloc is lean but the blocking
    // wifi_manager_scan() call uses significant internal WiFi stack frames.
/* 
    // The original web server is already started.
    xTaskCreatePinnedToCore(task_web_server, "web_srv",
                            12288, NULL, 3, NULL, 0);

 */
    // ── Startup log ───────────────────────────────────────────────────────────
    
 
}  // app_main

static void ota_mark_running_app_valid(void)
{
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "OTA image marked valid");
        } else {
            ESP_LOGE(TAG, "Failed to mark OTA image valid: %s", esp_err_to_name(err));
        }
    }
#endif
}

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
