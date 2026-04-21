#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <nvs_flash.h>
#include <cJSON.h>

// The original file containing the definitions is removed as this was the only content
#define TASK_PRIORITY_WIFI_STATUS 0
#define TASK_PRIORITY_STATS 0
#define TASK_PRIORITY_INTERFACE 5
#define TASK_PRIORITY_UART 10
#define TASK_PRIORITY_MAX 100


typedef enum
{
    TYPE_CFG_ITEM_BOOL = 0, 
    TYPE_CFG_ITEM_UINT8,
    TYPE_CFG_ITEM_UINT16,
    TYPE_CFG_ITEM_UINT32,
    TYPE_CFG_ITEM_STR,
    TYPE_CFG_ITEM_SECRET_STR, // not to send to the WEB UI1
    TYPE_CFG_ITEM_IP
} config_item_type_t;

typedef union
{
    bool     enabled;
    uint8_t  uint8;
    uint16_t uint16;
    uint32_t uint32;
    char *str;
} config_item_value_t;

#define CONFIG_PREFERENCES "config" // NVS namespace for configuration values

// Structure defining a configuration item as a key-value pair with type and default value
// The actual keys and defauld values are defined in an array in config.c
// The modified values are stored in NVS
typedef struct config_item
{
    char *key;
    config_item_type_t type;
    config_item_value_t def;
} config_item_t;

#define CFG_VALUE_UNCHANGED "\x1a\x1a\x1a\x1a\x1a\x1a\x1a\x1a"

// All keys of all properties that can be configured
// The strings need to correspond to the JSON keys used in the web interface
// Web server admin access - username, password and whether to use authentication
#define KEY_CONFIG_STATION_HOSTNAME "host_name"
#define KEY_CONFIG_ADMIN_USERNAME   "adm_user"
#define KEY_CONFIG_ADMIN_PASSWORD   "adm_pass"
#define KEY_CONFIG_ADMIN_AUTH       "adm_auth"
#define KEY_CONFIG_OTA_FIRMWARE_PATH "ota_fw_path"
#define KEY_CONFIG_OTA_WWW_PATH      "ota_www_path"

// NTRIP Client1 - parameters for the remote server to which this device will connect as a client and forward the RTCM data  you have an existing NTRIP caster/server infrastructure and want to integrate this RTK base station into it as a data source.
#define KEY_CONFIG_NTRIP1_ACTIVE     "ntrip1_active"
#define KEY_CONFIG_NTRIP1_HOST       "ntrip1_host"
#define KEY_CONFIG_NTRIP1_PORT       "ntrip1_port"
#define KEY_CONFIG_NTRIP1_MOUNTPOINT "ntrip1_mp"
#define KEY_CONFIG_NTRIP1_USERNAME   "ntrip1_user"
#define KEY_CONFIG_NTRIP1_PASSWORD   "ntrip1_pass"

// NTRIP Server2 - remote server to which this device will connect as a client and forward the RTCM data GPS via UART. This is typically used when you have an existing NTRIP caster/server infrastructure and want to integrate this RTK base station into it as a data source.
#define KEY_CONFIG_NTRIP2_ACTIVE     "ntrip2_active"
#define KEY_CONFIG_NTRIP2_HOST       "ntrip2_host"
#define KEY_CONFIG_NTRIP2_PORT       "ntrip2_port"
#define KEY_CONFIG_NTRIP2_MOUNTPOINT "ntrip2_mp"
#define KEY_CONFIG_NTRIP2_USERNAME   "ntrip2_user"
#define KEY_CONFIG_NTRIP2_PASSWORD   "ntrip2_pass"

// NTRIP Caster - this device will act as an NTRIP caster, allowing clients to connect to it and receive the RTCM data received from the GPS via UART. This is typically used when you want this RTK base station to serve as a data source for other devices (e.g. RTK rovers) that will connect to it directly as clients.
#define KEY_CONFIG_CASTER_ACTIVE          "caster_active"
#define KEY_CONFIG_CASTER_PORT            "caster_port"
#define KEY_CONFIG_CASTER_MOUNTPOINT      "caster_mp"
#define KEY_CONFIG_CASTER_USERNAME        "caster_user"
#define KEY_CONFIG_CASTER_PASSWORD        "caster_pass"

// TCP Server - to serve UPrecise connections. UPrecise uses TCP!
#define KEY_CONFIG_TCP_SERVER_ACTIVE      "tcp_srv_active"
#define KEY_CONFIG_TCP_SERVER_PORT        "tcp_srv_port"

// UART
#define KEY_CONFIG_UART_NUM               "uart_num"
#define KEY_CONFIG_UART_TX_PIN            "uart_tx_pin"
#define KEY_CONFIG_UART_RX_PIN            "uart_rx_pin"
#define KEY_CONFIG_UART_RTS_PIN           "uart_rts_pin"
#define KEY_CONFIG_UART_CTS_PIN           "uart_cts_pin"
#define KEY_CONFIG_UART_BAUD_RATE         "uart_baud_rate"
#define KEY_CONFIG_UART_DATA_BITS         "uart_data_bits"
#define KEY_CONFIG_UART_STOP_BITS         "uart_stop_bits"
#define KEY_CONFIG_UART_PARITY            "uart_parity"
#define KEY_CONFIG_UART_FLOW_CTRL_RTS     "uart_fc_rts"
#define KEY_CONFIG_UART_FLOW_CTRL_CTS     "uart_fc_cts"

// WiFi AP (access point)
#define KEY_CONFIG_WIFI_AP_ACTIVE         "w_ap_active"
#define KEY_CONFIG_WIFI_AP_SSID           "w_ap_ssid"
#define KEY_CONFIG_WIFI_AP_SSID_HIDDEN    "w_ap_ssid_hid"
#define KEY_CONFIG_WIFI_AP_AUTH_MODE      "w_ap_auth_mode"
#define KEY_CONFIG_WIFI_AP_PASSWORD       "w_ap_pass"
#define KEY_CONFIG_WIFI_AP_GATEWAY        "w_ap_gw"
#define KEY_CONFIG_WIFI_AP_SUBNET         "w_ap_subnet"

// WiFi STA (station)
#define KEY_CONFIG_WIFI_STA_ACTIVE        "w_sta_active"
#define KEY_CONFIG_WIFI_STA_SSID          "w_sta_ssid"
#define KEY_CONFIG_WIFI_STA_PASSWORD      "w_sta_pass"
#define KEY_CONFIG_WIFI_STA_SCAN_MODE_ALL "w_sta_scan_mode"
#define KEY_CONFIG_WIFI_STA_AP_FORWARD    "w_sta_ap_fwd"
#define KEY_CONFIG_WIFI_STA_STATIC        "w_sta_static"
#define KEY_CONFIG_WIFI_STA_IP            "w_sta_ip"
#define KEY_CONFIG_WIFI_STA_GATEWAY       "w_sta_gw"
#define KEY_CONFIG_WIFI_STA_SUBNET        "w_sta_subnet"
#define KEY_CONFIG_WIFI_STA_DNS_A         "w_sta_dns_a"
#define KEY_CONFIG_WIFI_STA_DNS_B         "w_sta_dns_b"

esp_err_t nvs_cfg_init();
esp_err_t cfg_to_json(cJSON *root);
esp_err_t cfg_json_to_nvs(cJSON *root);
esp_err_t cfg_get_str(const char* key, char** out_value);  
esp_err_t cfg_get_u8(const char* key, uint8_t* out_value);
esp_err_t cfg_get_u16(const char* key, uint16_t* out_value);
esp_err_t cfg_get_u32(const char* key, uint32_t* out_value);

void cfg_reset_restart();
void cfg_commit();
