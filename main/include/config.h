#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <nvs_flash.h>
#include <cJSON.h>

typedef enum
{
    TYPE_CFG_ITEM_BOOL = 0, 
    TYPE_CFG_ITEM_INT8,
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
    int8_t   int8;
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
#define KEY_CONFIG_ADMIN_USERNAME "adm_user"
#define KEY_CONFIG_ADMIN_PASSWORD "adm_pass"
#define KEY_CONFIG_ADMIN_AUTH     "adm_auth"

// Bluetooth
#define KEY_CONFIG_BLUETOOTH_ACTIVE "bt_active"
#define KEY_CONFIG_BLUETOOTH_DEVICE_NAME "bt_dev_name"
#define KEY_CONFIG_BLUETOOTH_DEVICE_DISCOVERABLE "bt_dev_vis"
#define KEY_CONFIG_BLUETOOTH_PIN_CODE "bt_pin_code"

// NTRIP Server
#define KEY_CONFIG_NTRIP_SERVER_ACTIVE "ntr_srv_active"
#define KEY_CONFIG_NTRIP_SERVER_HOST "ntr_srv_host"
#define KEY_CONFIG_NTRIP_SERVER_PORT "ntr_srv_port"
#define KEY_CONFIG_NTRIP_SERVER_MOUNTPOINT "ntr_srv_mp"
#define KEY_CONFIG_NTRIP_SERVER_USERNAME "ntr_srv_user"
#define KEY_CONFIG_NTRIP_SERVER_PASSWORD "ntr_srv_pass"

// NTRIP Client
#define KEY_CONFIG_NTRIP_CLIENT_ACTIVE "ntr_cli_active"
#define KEY_CONFIG_NTRIP_CLIENT_HOST "ntr_cli_host"
#define KEY_CONFIG_NTRIP_CLIENT_PORT "ntr_cli_port"
#define KEY_CONFIG_NTRIP_CLIENT_MOUNTPOINT "ntr_cli_mp"
#define KEY_CONFIG_NTRIP_CLIENT_USERNAME "ntr_cli_user"
#define KEY_CONFIG_NTRIP_CLIENT_PASSWORD "ntr_cli_pass"

// NTRIP Caster
#define KEY_CONFIG_NTRIP_CASTER_ACTIVE "ntr_cst_active"
#define KEY_CONFIG_NTRIP_CASTER_PORT "ntr_cst_port"
#define KEY_CONFIG_NTRIP_CASTER_MOUNTPOINT "ntr_cst_mp"
#define KEY_CONFIG_NTRIP_CASTER_USERNAME "ntr_cst_user"
#define KEY_CONFIG_NTRIP_CASTER_PASSWORD "ntr_cst_pass"

// Socket Server
#define KEY_CONFIG_SOCKET_SERVER_ACTIVE "sck_srv_active"
#define KEY_CONFIG_SOCKET_SERVER_TCP_PORT "sck_srv_t_port"
#define KEY_CONFIG_SOCKET_SERVER_UDP_PORT "sck_srv_u_port"

// Socket Client
#define KEY_CONFIG_SOCKET_CLIENT_ACTIVE "sck_cli_active"
#define KEY_CONFIG_SOCKET_CLIENT_HOST "sck_cli_host"
#define KEY_CONFIG_SOCKET_CLIENT_PORT "sck_cli_port"
#define KEY_CONFIG_SOCKET_CLIENT_TYPE_TCP_UDP "sck_cli_type"
#define KEY_CONFIG_SOCKET_CLIENT_CONNECT_MESSAGE "sck_cli_msg"

// UART
#define KEY_CONFIG_UART_NUM "uart_num"
#define KEY_CONFIG_UART_TX_PIN "uart_tx_pin"
#define KEY_CONFIG_UART_RX_PIN "uart_rx_pin"
#define KEY_CONFIG_UART_RTS_PIN "uart_rts_pin"
#define KEY_CONFIG_UART_CTS_PIN "uart_cts_pin"
#define KEY_CONFIG_UART_BAUD_RATE "uart_baud_rate"
#define KEY_CONFIG_UART_DATA_BITS "uart_data_bits"
#define KEY_CONFIG_UART_STOP_BITS "uart_stop_bits"
#define KEY_CONFIG_UART_PARITY "uart_parity"
#define KEY_CONFIG_UART_FLOW_CTRL_RTS "uart_fc_rts"
#define KEY_CONFIG_UART_FLOW_CTRL_CTS "uart_fc_cts"
#define KEY_CONFIG_UART_LOG_FORWARD "uart_log_fwd"

// WiFi AP (access point)
#define KEY_CONFIG_WIFI_AP_ACTIVE      "w_ap_active"
#define KEY_CONFIG_WIFI_AP_SSID        "w_ap_ssid"
#define KEY_CONFIG_WIFI_AP_SSID_HIDDEN "w_ap_ssid_hid"
#define KEY_CONFIG_WIFI_AP_AUTH_MODE   "w_ap_auth_mode"
#define KEY_CONFIG_WIFI_AP_PASSWORD    "w_ap_pass"
#define KEY_CONFIG_WIFI_AP_GATEWAY     "w_ap_gw"
#define KEY_CONFIG_WIFI_AP_SUBNET      "w_ap_subnet"

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

esp_err_t cfg_init();
esp_err_t cfg_to_json(cJSON *root);
esp_err_t cfg_json_to_nvs(cJSON *root);
esp_err_t cfg_get_str(const char* key, char** out_value);  
esp_err_t cfg_get_i8(const char* key, int8_t* out_value);
esp_err_t cfg_get_u8(const char* key, uint8_t* out_value);
esp_err_t cfg_get_u32(const char* key, uint32_t* out_value);

void cfg_reset_restart();
void cfg_commit();
