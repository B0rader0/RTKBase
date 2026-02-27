/*
 *  
 */

#include <web_server.h>
#include <log.h>
#include <interface/socket_client.h>
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
#include "config.h"
#include "wifi.h"
#include "interface/socket_server.h"
#include "gps_uart.h"
#include "interface/ntrip.h"
#include "tasks.h"
#include "reset_button.h"

static const char *TAG = "MAIN";

static char *reset_reason_name(esp_reset_reason_t reason);

static void sntp_time_set_handler(struct timeval *tv) {
    ESP_LOGI(TAG, "Synced time from SNTP");
}

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
    log_init();
        
    // BUG: (and hence commented out)
    // Bug because it is not yet known what to redirect to the UART and WebSocket clients are not configured yet!
    // Redirect ESP-IDF logs to our custom log implementation, which forwards them to UART and WebSocket clients
    // This is a bug, because it is not yet known what to redirect to
    // esp_log_set_vprintf(log_vprintf);
    
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_log_level_set("system_api", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);
    esp_log_level_set("reset_button", ESP_LOG_DEBUG);
    
    core_dump_check();

    
    // Registers a handler for the BOOT button for resetting the parameters to factory defaults. 
    // When the button held down for more than BTN_LONG_PRESS and then released, the NVM storage is 
    // cleared and the device restarts. 
    // The idea to use the built LED does not work because the LED is not controllable.
    reset_button_init();
 
    stream_stats_init();

    cfg_init();
    
    uart_init();

    esp_reset_reason_t reset_reason = esp_reset_reason();

    //uart_nmea("$PESP,INIT,START,%s,%s", app_desc->version, reset_reason_name(reset_reason));

    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║ Reset reason: %-30s "                       "║", reset_reason_name(reset_reason));
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");

    esp_event_loop_create_default();

    net_init();
    wifi_init();

    web_server_init();

    ntrip_caster_init();
    ntrip_server_init();
    ntrip_client_init();

    socket_server_init();
    socket_client_init();

    uart_nmea("$PESP,INIT,COMPLETE");

    wait_for_ip();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL); 
    esp_sntp_setservername(0, "pool.ntp.org"); 
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    sntp_set_time_sync_notification_cb(sntp_time_set_handler);
    esp_sntp_init(); 

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