/*
 * This file is part of the ESP32-XBee distribution (https://github.com/nebkat/esp32-xbee).
 * Copyright (c) 2019 Nebojsa Cvetkovic.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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

/* BAR - intend to use the official BOOT button on the board for factory reset, but it is not working reliably, so using GPIO0 instead. 
static void reset_button_task() {
    QueueHandle_t button_queue = reset_button_init(PIN_BIT(GPIO_NUM_0));
    gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);
    while (true) {
        button_event_t button_ev;
        if (xQueueReceive(button_queue, &button_ev, 1000 / portTICK_PERIOD_MS)) {
            if (button_ev.event == BUTTON_DOWN && button_ev.duration > 5000) {
                config_reset();
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                esp_restart();
            }
        }
    }
}
*/

static void sntp_time_set_handler(struct timeval *tv) {
    ESP_LOGI(TAG, "Synced time from SNTP");
}

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    
   // status_led_init(); 
  //  status_led_handle_t status_led = status_led_add(0xFFFFFF33, STATUS_LED_FADE, 250, 2500, 0);
     
    log_init();
        
    // BUG: (and hence commented out)
    // Bug because it is not yet known what to redirect to the UART and WebSocket clients are not configured yet!
    // Redirect ESP-IDF logs to our custom log implementation, which forwards them to UART and WebSocket clients
    // This is a bug, because it is not yet known what to redirect to
    //esp_log_set_vprintf(log_vprintf);
    
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

    config_init();
    uart_init();

    esp_reset_reason_t reset_reason = esp_reset_reason();

    //uart_nmea("$PESP,INIT,START,%s,%s", app_desc->version, reset_reason_name(reset_reason));

    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║ Reset reason: %-30s "                       "║", reset_reason_name(reset_reason));
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");

    esp_event_loop_create_default();

/*   
    vTaskDelay(pdMS_TO_TICKS(2500));
    status_led->interval = 100;
    status_led->duration = 1000;
    status_led->flashing_mode = STATUS_LED_BLINK;

    if (reset_reason != ESP_RST_POWERON && reset_reason != ESP_RST_SW && reset_reason != ESP_RST_WDT) {
        status_led->active = false;
        status_led_handle_t error_led = status_led_add(0xFF000033, STATUS_LED_BLINK, 50, 10000, 0);

        vTaskDelay(pdMS_TO_TICKS(10000));

        status_led_remove(error_led);
        status_led->active = true;
    }

*/
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
}