/*
 */

#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_err.h>
#include <esp_log.h>
#include <string.h>
#include <protocol/nmea.h>
#include <stream_stats.h>

#include "gps_uart.h"
#include "config.h"
#include "interface/socket_server.h"
#include "tasks.h"

static const char *TAG = "UART";

ESP_EVENT_DEFINE_BASE(UART_EVENT_READ);
ESP_EVENT_DEFINE_BASE(UART_EVENT_WRITE);

void uart_register_read_handler(esp_event_handler_t event_handler)
{
    ESP_ERROR_CHECK(esp_event_handler_register(UART_EVENT_READ, ESP_EVENT_ANY_ID, event_handler, NULL));
}

void uart_unregister_read_handler(esp_event_handler_t event_handler)
{
    ESP_ERROR_CHECK(esp_event_handler_unregister(UART_EVENT_READ, ESP_EVENT_ANY_ID, event_handler));
}

void uart_register_write_handler(esp_event_handler_t event_handler)
{
    ESP_ERROR_CHECK(esp_event_handler_register(UART_EVENT_WRITE, ESP_EVENT_ANY_ID, event_handler, NULL));
}

void uart_unregister_write_handler(esp_event_handler_t event_handler)
{
    ESP_ERROR_CHECK(esp_event_handler_unregister(UART_EVENT_WRITE, ESP_EVENT_ANY_ID, event_handler));
}

static uint8_t uart_port = 0;
static bool uart_log_forward = false;

static stream_stats_handle_t stream_stats;

static void uart_task(void *ctx);

esp_err_t uart_init()
{
    nvs_handle_t h_config;
    uart_config_t uart_config;
    config_item_value_t cfg_var;

    esp_err_t res = nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_config);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(res));
    }

    res = nvs_get_u8(h_config, KEY_CONFIG_UART_LOG_FORWARD, &cfg_var.uint8);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read UART log_forward config from NVS: %s", esp_err_to_name(res));
    }

    uart_log_forward = cfg_var.enabled;

    res = nvs_get_u8(h_config, KEY_CONFIG_UART_NUM, &cfg_var.uint8);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read UART port_number from NVS: %s", esp_err_to_name(res));
    }
    uart_port = cfg_var.uint8;

    // UART configuration structure. The values are populated from NVS later, after reading them from NVS.
    // Populating the uart_config structure with the configuration values read from NVS.
    uart_config.flags.allow_pd = 0; // Do not allow power down.
    uart_config.source_clk = UART_SCLK_APB; // Use APB clock as the source clock for UART. This is the default and recommended clock source for most applications.

    res = nvs_get_u32(h_config, KEY_CONFIG_UART_BAUD_RATE, &cfg_var.uint32);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read UART baud rate config from NVS: %s", esp_err_to_name(res));
    }
    uart_config.baud_rate = cfg_var.uint32;

    res = nvs_get_u8(h_config, KEY_CONFIG_UART_DATA_BITS, &cfg_var.uint8);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read UART data bits config from NVS: %s", esp_err_to_name(res));
    }
    uart_config.data_bits = cfg_var.uint8;
    
    res = nvs_get_u8(h_config, KEY_CONFIG_UART_PARITY, &cfg_var.uint8);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read UART parity config from NVS: %s", esp_err_to_name(res));
    }
    uart_config.parity = cfg_var.uint8;

    res = nvs_get_u8(h_config, KEY_CONFIG_UART_STOP_BITS, &cfg_var.uint8);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read UART stop bits config from NVS: %s", esp_err_to_name(res));
    }
    uart_config.stop_bits = cfg_var.uint8;

    // The flow control is set by bitwise ORing the RTS and CTS flow control values, which are defined in uart_hw_flowcontrol_t enum. If both are disabled, the flow control will be disabled. If only one of them is enabled, the flow control will be set to the corresponding value. If both are enabled, the flow control will be set to UART_HW_FLOWCTRL_CTS_RTS.
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; // default value, will be updated later based on the RTS and CTS flow control config values read from NVS.
    res = nvs_get_u8(h_config, KEY_CONFIG_UART_FLOW_CTRL_RTS, &cfg_var.uint8);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read UART RTS flow control config from NVS: %s", esp_err_to_name(res));
    }
    
    if (cfg_var.enabled) {
        uart_config.flow_ctrl = uart_config.flow_ctrl | UART_HW_FLOWCTRL_RTS;
    };

    res = nvs_get_u8(h_config, KEY_CONFIG_UART_FLOW_CTRL_CTS, &cfg_var.uint8);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read UART CTS flow control config from NVS: %s", esp_err_to_name(res));
    }

    if (cfg_var.enabled) {
        uart_config.flow_ctrl = uart_config.flow_ctrl | UART_HW_FLOWCTRL_CTS;
    };

    ESP_LOGI(TAG, "Configuring UART parameters: port=%d, baud_rate=%d, data_bits=%d, parity=%d, stop_bits=%d, flow_ctrl=%d", uart_port, uart_config.baud_rate, uart_config.data_bits, uart_config.parity, uart_config.stop_bits, uart_config.flow_ctrl);
    vTaskDelay(pdMS_TO_TICKS(500));

    res = uart_param_config(uart_port, &uart_config);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(res));
        nvs_close(h_config);
        return res;
    }

    ESP_LOGI(TAG, "AFTER Configuring UART parameters: port=%d, baud_rate=%d, data_bits=%d, parity=%d, stop_bits=%d, flow_ctrl=%d", uart_port, uart_config.baud_rate, uart_config.data_bits, uart_config.parity, uart_config.stop_bits, uart_config.flow_ctrl);
    vTaskDelay(pdMS_TO_TICKS(500));

    int pin_tx, pin_rx, pin_rts, pin_cts;
    res = nvs_get_u8(h_config, KEY_CONFIG_UART_TX_PIN, &cfg_var.uint8);
    if (res != ESP_OK)    {
        ESP_LOGE(TAG, "Failed to read UART TX pin config from NVS: %s", esp_err_to_name(res));
    }
    pin_tx = cfg_var.uint8;

    res = nvs_get_u8(h_config, KEY_CONFIG_UART_RX_PIN, &cfg_var.uint8);
    if (res != ESP_OK)    {
        ESP_LOGE(TAG, "Failed to read UART RX pin config from NVS: %s", esp_err_to_name(res));
    }
    pin_rx = cfg_var.uint8;

    res = nvs_get_u8(h_config, KEY_CONFIG_UART_RTS_PIN, &cfg_var.uint8);
    if (res != ESP_OK)    {
        ESP_LOGE(TAG, "Failed to read UART RTS pin config from NVS: %s", esp_err_to_name(res));
    }
    pin_rts = cfg_var.uint8;

    res = nvs_get_u8(h_config, KEY_CONFIG_UART_CTS_PIN, &cfg_var.uint8);
    if (res != ESP_OK)    {
        ESP_LOGE(TAG, "Failed to read UART CTS pin config from NVS: %s", esp_err_to_name(res));
    }
    pin_cts = cfg_var.uint8;

    res = uart_set_pin(uart_port, pin_tx, pin_rx, pin_rts, pin_cts);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(res));
        nvs_close(h_config);
        return res;
    }

    res = uart_driver_install(uart_port, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(res));
        nvs_close(h_config);
        return res;
    }

    stream_stats = stream_stats_new("uart");

    xTaskCreate(uart_task, "uart_task", 8192, NULL, TASK_PRIORITY_UART, NULL);

    nvs_close(h_config);

    ESP_LOGI(TAG, "UART initialized successfully on port %d", uart_port);

    return ESP_OK;
} // uart_init

static void uart_task(void *ctx)
{
    uint8_t buffer[UART_BUFFER_SIZE];

    while (true)
    {
        int32_t len = uart_read_bytes(uart_port, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
        if (len < 0)
        {
            ESP_LOGE(TAG, "Error reading from UART");
        }
        else if (len == 0)
        {
            continue;
        }

        stream_stats_increment(stream_stats, len, 0);

        esp_event_post(UART_EVENT_READ, len, &buffer, len, portMAX_DELAY);
    }
}

void uart_inject(void *buf, size_t len)
{
    esp_event_post(UART_EVENT_READ, len, buf, len, portMAX_DELAY);
}

int uart_log(char *buf, size_t len)
{
    if (!uart_log_forward)
        return 0;
    return uart_write(buf, len);
}

int uart_nmea(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    char *nmea;
    nmea_vasprintf(&nmea, fmt, args);
    int l = uart_write(nmea, strlen(nmea));
    free(nmea);

    va_end(args);

    return l;
}

int uart_write(char *buf, size_t len)
{
    if (len == 0)
        return 0;

    int written = uart_write_bytes(uart_port, buf, len);
    if (written < 0)
        return written;

    stream_stats_increment(stream_stats, 0, len);

    esp_event_post(UART_EVENT_WRITE, len, buf, len, portMAX_DELAY);

    return written;
}