/*
 * Inspired by the ESP32-XBee distribution (https://github.com/nebkat/esp32-xbee).
 * Manages the confiuration of the RTK Base device.
 *
 */

#include <stdbool.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <string.h>
#include <errno.h>
#include <driver/uart.h>
#include <esp_wifi_types.h>
#include <driver/gpio.h>
#include <cJSON.h>
#include <esp_mac.h>

#include "nvs_config.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"

static const char *TAG = "CONFIG";

static bool json_get_bool_compat(cJSON *item, uint8_t *out_value)
{
    if (cJSON_IsBool(item)) {
        *out_value = cJSON_IsTrue(item) ? 1 : 0;
        return true;
    }

    if (cJSON_IsNumber(item)) {
        double number = cJSON_GetNumberValue(item);
        if (number == 0.0 || number == 1.0) {
            *out_value = (uint8_t)number;
            return true;
        }
        return false;
    }

    if (cJSON_IsString(item)) {
        const char *value = cJSON_GetStringValue(item);
        if (value == NULL) {
            return false;
        }
        if (strcmp(value, "0") == 0 || strcasecmp(value, "false") == 0) {
            *out_value = 0;
            return true;
        }
        if (strcmp(value, "1") == 0 || strcasecmp(value, "true") == 0) {
            *out_value = 1;
            return true;
        }
    }

    return false;
}

static bool json_get_u32_compat(cJSON *item, uint32_t max_value, uint32_t *out_value)
{
    if (cJSON_IsNumber(item)) {
        double number = cJSON_GetNumberValue(item);
        if (number < 0.0 || number > (double)max_value) {
            return false;
        }

        uint32_t value = (uint32_t)number;
        if ((double)value != number) {
            return false;
        }

        *out_value = value;
        return true;
    }

    if (cJSON_IsString(item)) {
        const char *value = cJSON_GetStringValue(item);
        char *end = NULL;
        unsigned long parsed;

        if (value == NULL || value[0] == '\0') {
            return false;
        }

        errno = 0;
        parsed = strtoul(value, &end, 10);
        if (errno != 0 || end == value || *end != '\0' || parsed > max_value) {
            return false;
        }

        *out_value = (uint32_t)parsed;
        return true;
    }

    return false;
}

static bool json_get_ip4_array(cJSON *item, uint32_t *out_value)
{
    if (!cJSON_IsArray(item) || cJSON_GetArraySize(item) != 4) {
        return false;
    }

    esp_ip4_addr_t ip;
    ip.addr = 0;

    for (int b = 0; b < 4; b++) {
        cJSON *part = cJSON_GetArrayItem(item, b);
        uint32_t octet = 0;
        if (!json_get_u32_compat(part, 255, &octet)) {
            return false;
        }
        ip.addr |= (octet << (b * 8));
    }

    *out_value = ip.addr;
    return true;
}

// Global handle to a NVS partition, used for reading and writing configuration values. I
// Risky as not clear when it is open and closed,
// Intend to remove it and open and close the partiion for each configuration operation
// in the corresppnding init functions (e.g. wifi_init) and the web server handlers for configuration changes.
// Rather use a string defining the name of the partition and open and close it for each operation,
// which is safer and more robust.

const config_item_t CONFIG_ITEMS[] = {
    
    {  // Admin access configuration
        .key = KEY_CONFIG_ADMIN_AUTH, // 0 = open, 1 = basic auth, 2 = hotspot auth - maybe BOOL is enough?
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = 0 // open by default
    },
    {
        .key = KEY_CONFIG_ADMIN_USERNAME,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_ADMIN_PASSWORD,
        .type = TYPE_CFG_ITEM_SECRET_STR,
        .def.str = ""
    },
     {
        .key = KEY_CONFIG_STATION_HOSTNAME,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = "RTK_"
    },
    {  // NTRIP Server 1
        .key = KEY_CONFIG_NTRIP1_ACTIVE,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_NTRIP1_HOST,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_NTRIP1_PORT,
        .type = TYPE_CFG_ITEM_UINT16,
        .def.uint16 = 2101
    },
    {
        .key = KEY_CONFIG_NTRIP1_MOUNTPOINT,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_NTRIP1_USERNAME,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_NTRIP1_PASSWORD,
        .type = TYPE_CFG_ITEM_SECRET_STR,
        .def.str = ""
    },
    {   // NTRIP Server 2
        .key = KEY_CONFIG_NTRIP2_ACTIVE,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_NTRIP2_HOST,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_NTRIP2_PORT,
        .type = TYPE_CFG_ITEM_UINT16,
        .def.uint16 = 2101
    },
    {
        .key = KEY_CONFIG_NTRIP2_MOUNTPOINT,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_NTRIP2_USERNAME,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_NTRIP2_PASSWORD,
        .type = TYPE_CFG_ITEM_SECRET_STR,
        .def.str = ""
    },
    {   //NTRIP Caster
        .key = KEY_CONFIG_CASTER_ACTIVE,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_CASTER_PORT,
        .type = TYPE_CFG_ITEM_UINT16,
        .def.uint16 = 2101
    },
    {
        .key = KEY_CONFIG_CASTER_MOUNTPOINT,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_CASTER_USERNAME,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_CASTER_PASSWORD,
        .type = TYPE_CFG_ITEM_SECRET_STR,
        .def.str = ""
    },

    // Configuraton for the local Socket server
    // to which UPrecise can cnnect
    // TODO: exchange configuration commands for the GPS module.
    {
        .key = KEY_CONFIG_TCP_SERVER_ACTIVE,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_TCP_SERVER_PORT,
        .type = TYPE_CFG_ITEM_UINT16,
        .def.uint16 = 5015
    },

    // UART
    // Do not use UART_NUM0 and GPIO_NUM_1 and GPIO_NUM_3 as default TX and RX pins, as they are used for logging.
    // Additionally, the board cannot be flashed if the GPS module is soldered to GPIO 1 and GPIO 3, as they are connected to the flash chip. So we use UART_NUM_1 and redefine the default pins to GPIO_NUM_16 and GPIO_NUM_17, which are not used for anything else and are not connected to the flash chip.
    // Using UART_NUM1 but the default pins need to be redefined as they are connected to flash
    {
        .key = KEY_CONFIG_UART_NUM,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = UART_NUM_1
    },
    {
        .key = KEY_CONFIG_UART_RX_PIN,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = GPIO_NUM_16
    },
    {
        .key = KEY_CONFIG_UART_TX_PIN,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = GPIO_NUM_17
    },
    {
        .key = KEY_CONFIG_UART_RTS_PIN,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = GPIO_NUM_14
    },
    {
        .key = KEY_CONFIG_UART_CTS_PIN,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = GPIO_NUM_33
    },
    {
        .key = KEY_CONFIG_UART_BAUD_RATE,
        .type = TYPE_CFG_ITEM_UINT32,
        .def.uint32 = 115200
    },
    {
        .key = KEY_CONFIG_UART_DATA_BITS,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = UART_DATA_8_BITS
    },
    {
        .key = KEY_CONFIG_UART_STOP_BITS,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = UART_STOP_BITS_1
    },
    {
        .key = KEY_CONFIG_UART_PARITY,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = UART_PARITY_DISABLE
    },
    {
        .key = KEY_CONFIG_UART_FLOW_CTRL_RTS,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_UART_FLOW_CTRL_CTS,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },

    // WiFi
    {// WiFi AP (access point)
        .key = KEY_CONFIG_WIFI_AP_ACTIVE,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = true
    },
    {
        .key = KEY_CONFIG_WIFI_AP_SSID,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = "RTK_"
    },
    {
        .key = KEY_CONFIG_WIFI_AP_SSID_HIDDEN,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_WIFI_AP_AUTH_MODE,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = WIFI_AUTH_OPEN
    },
    {
        .key = KEY_CONFIG_WIFI_AP_PASSWORD,
        .type = TYPE_CFG_ITEM_SECRET_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_WIFI_AP_GATEWAY,
        .type = TYPE_CFG_ITEM_IP,
        .def.uint32 = esp_netif_htonl(esp_netif_ip4_makeu32(192, 168, 4, 1)) 
    },
    {
        .key = KEY_CONFIG_WIFI_AP_SUBNET,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = 24
    },
    {// WiFi STA (station)
        .key = KEY_CONFIG_WIFI_STA_ACTIVE,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_WIFI_STA_SSID,
        .type = TYPE_CFG_ITEM_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_WIFI_STA_PASSWORD,
        .type = TYPE_CFG_ITEM_SECRET_STR,
        .def.str = ""
    },
    {
        .key = KEY_CONFIG_WIFI_STA_SCAN_MODE_ALL,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_WIFI_STA_AP_FORWARD,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_WIFI_STA_STATIC,
        .type = TYPE_CFG_ITEM_BOOL,
        .def.enabled = false
    },
    {
        .key = KEY_CONFIG_WIFI_STA_IP,
        .type = TYPE_CFG_ITEM_IP,
        .def.uint32 = esp_netif_htonl(esp_netif_ip4_makeu32(192, 168, 0, 100))
    },
    {
        .key = KEY_CONFIG_WIFI_STA_GATEWAY,
        .type = TYPE_CFG_ITEM_IP,
        .def.uint32 = esp_netif_htonl(esp_netif_ip4_makeu32(192, 168, 0, 1))
    },
    {
        .key = KEY_CONFIG_WIFI_STA_SUBNET,
        .type = TYPE_CFG_ITEM_UINT8,
        .def.uint8 = 24
    },
    {
        .key = KEY_CONFIG_WIFI_STA_DNS_A,
        .type = TYPE_CFG_ITEM_IP,
        .def.uint32 = esp_netif_htonl(esp_netif_ip4_makeu32(192, 168, 178, 1))
    },
    {
        .key = KEY_CONFIG_WIFI_STA_DNS_B,
        .type = TYPE_CFG_ITEM_IP,
        .def.uint32 = esp_netif_htonl(esp_netif_ip4_makeu32(8, 8, 8, 8))
    }
};

// Makes sure the config partition has valid settins.
// If not, the default values defined in the CONFIG_ITEMS are written to it.
// All subsequent initialization functions (e.g. wifi_init) will read the values from the config partition
// and apply them to the corresponding components (e.g. WiFi)
// esp_err_t config_init() {
esp_err_t nvs_cfg_init()
{
    nvs_handle_t h_config;       // Local handle to the NVS partition, used for reading and writing configuration values. It is opened and closed for each operation, which is safer and more robust than using a global handle.
    config_item_value_t cfg_var; // Local variable used for reading configuration values.
    size_t str_len_var;
    char str_buf [32];
    uint8_t mac_buff [6];

    esp_err_t res = nvs_flash_init(); // the default partition is used, which is defined in the partition table as "nvs" and has a size of 0.5MB, which should be enough for the configuration values.

    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        res = nvs_flash_init();
    }

    ESP_ERROR_CHECK(res);

    // A namespace is requred. It is differnet from the partition name.
    res = nvs_open(CONFIG_PREFERENCES, NVS_READWRITE, &h_config);
    ESP_ERROR_CHECK(res);

    esp_read_mac(mac_buff, ESP_MAC_EFUSE_FACTORY); // Get the MAC address of the WiFi STA interface
    //esp_wifi_get_mac(WIFI_IF_AP, mac_buff);
    sprintf(str_buf, "RTKBase_%02X%02X", mac_buff[4], mac_buff[5]);
    
    // Iterate over all config items and check if they exist in NVS. If not, write the default value to NVS.
    // No need of a separate function as it will be used only here.
    for (unsigned int i = 0; i < sizeof(CONFIG_ITEMS) / sizeof(config_item_t); i++) {

        switch (CONFIG_ITEMS[i].type){
            case TYPE_CFG_ITEM_UINT8:
            case TYPE_CFG_ITEM_BOOL:
                res = nvs_get_u8(h_config, CONFIG_ITEMS[i].key, &(cfg_var.uint8));
                if (res != ESP_OK)
                    nvs_set_u8(h_config, CONFIG_ITEMS[i].key, CONFIG_ITEMS[i].def.uint8);
                continue;
            case TYPE_CFG_ITEM_UINT16:
                res = nvs_get_u16(h_config, CONFIG_ITEMS[i].key, &(cfg_var.uint16));
                if (res != ESP_OK)
                    nvs_set_u16(h_config, CONFIG_ITEMS[i].key, CONFIG_ITEMS[i].def.uint16);
                continue;
            case TYPE_CFG_ITEM_UINT32:
            case TYPE_CFG_ITEM_IP: // IP addresses are stored as uint32 in NVS, so they can be read and written using the same functions as uint32
                res = nvs_get_u32(h_config, CONFIG_ITEMS[i].key, &(cfg_var.uint32));
                if (res != ESP_OK)
                    nvs_set_u32(h_config, CONFIG_ITEMS[i].key, CONFIG_ITEMS[i].def.uint32);
                continue;
            case TYPE_CFG_ITEM_STR:
            case TYPE_CFG_ITEM_SECRET_STR:
                res = nvs_get_str(h_config, CONFIG_ITEMS[i].key, NULL, &str_len_var); // Get the required buffer size for the string value
                if (res != ESP_OK) {
                    // If the string doesn't exist in NVS or is corrupted, set it to the default value.
                    if ((strcmp(CONFIG_ITEMS[i].key, KEY_CONFIG_WIFI_AP_SSID) == 0) || 
                        (strcmp(CONFIG_ITEMS[i].key, KEY_CONFIG_STATION_HOSTNAME) == 0)) {
                        nvs_set_str(h_config, CONFIG_ITEMS[i].key, str_buf); // Use the generated SSID as the default value for the WiFi AP SSID
                    } else {
                        nvs_set_str(h_config, CONFIG_ITEMS[i].key, CONFIG_ITEMS[i].def.str);
                    }
                }
                continue;
            default:
                ESP_LOGE(TAG, "Unknown config_item TYPE for key %s: %d", CONFIG_ITEMS[i].key, CONFIG_ITEMS[i].type);
            continue;
        } // switch
    } // for

    // Commiting the changes to NVS. This is required after writing any value to NVS, otherwise the changes will not be saved and will be lost after a restart.
    nvs_commit(h_config);

    nvs_close(h_config);

    return ESP_OK;

} // config_init()

void cfg_reset_restart()
{

        // 1. Erase NVS (using the default partition)
        // Note: nvs_flash_erase is safer for a full reset than just nvs_erase_all
        // nvs_flash_erase also deinitializes the flash, so no need to call nvs_flash_deinit before it,
        // and it is actually safer not to call it as it will affect other NVS handles that might be open.
        nvs_flash_erase();

        // 2. Restart
        esp_restart();
} // void config_restart()

// The config_to_json function is used to convert the configuration values stored in NVS
// to a JSON object that can be sent to the web interface for display and editing. 
// The function iterates over all config items, reads their values from NVS, and adds them to the JSON object. 
// For secret values, it hides the actual value and uses a placeholder instead.
esp_err_t cfg_to_json(cJSON *root)
{
    esp_err_t res;
    nvs_handle_t h_config;       // Local handle to the NVS partition, used for reading.
    //config_item_value_t cfg_var; // Local variable used for reading configuration values.
    //size_t str_len_var;
    config_item_value_t cfg_value = {0};

    size_t length = 0;
    char *str = NULL;

    esp_ip4_addr_t ip;

    // A namespace is requred. It is differnet from the partition name.
    res = nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_config);
    ESP_ERROR_CHECK(res);

    // Iterate over all config items and add them to the JSON object.
    for (unsigned int i = 0; i < sizeof(CONFIG_ITEMS) / sizeof(config_item_t); i++) {
        switch (CONFIG_ITEMS[i].type) {
            case TYPE_CFG_ITEM_SECRET_STR:
                // For secret strings, we don't send the actual value, just a placeholder.
                cJSON_AddStringToObject(root, CONFIG_ITEMS[i].key, CFG_VALUE_UNCHANGED);
                continue;
            case TYPE_CFG_ITEM_STR:
                // Get length
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(h_config, CONFIG_ITEMS[i].key, NULL, &length));
                str = malloc(length);
                // Get value
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_str(h_config, CONFIG_ITEMS[i].key, str, &length));
                cJSON_AddStringToObject(root, CONFIG_ITEMS[i].key, str);
                free(str);
                continue;
            case TYPE_CFG_ITEM_IP:
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u32(h_config, CONFIG_ITEMS[i].key, (uint32_t *)&ip.addr));
                cJSON *ip_parts = cJSON_AddArrayToObject(root, CONFIG_ITEMS[i].key);
                for (int b = 0; b < 4; b++) {
                    cJSON_AddItemToArray(ip_parts, cJSON_CreateNumber(esp_ip4_addr_get_byte(&ip, b)));
                }
                continue;
            case TYPE_CFG_ITEM_BOOL:
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, CONFIG_ITEMS[i].key, &cfg_value.uint8));
                cJSON_AddBoolToObject(root, CONFIG_ITEMS[i].key, cfg_value.enabled);
                continue;
            case TYPE_CFG_ITEM_UINT8:
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, CONFIG_ITEMS[i].key, &cfg_value.uint8));
                cJSON_AddNumberToObject(root, CONFIG_ITEMS[i].key, cfg_value.uint8);
                continue;
            case TYPE_CFG_ITEM_UINT16:
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u16(h_config, CONFIG_ITEMS[i].key, &cfg_value.uint16));
                cJSON_AddNumberToObject(root, CONFIG_ITEMS[i].key, cfg_value.uint16);
                continue;
            case TYPE_CFG_ITEM_UINT32:
                ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u32(h_config, CONFIG_ITEMS[i].key, &cfg_value.uint32));
                cJSON_AddNumberToObject(root, CONFIG_ITEMS[i].key, cfg_value.uint32);
                continue;
            default:
                ESP_LOGE(TAG, "Unknown config item TYPE for key %s: %d", CONFIG_ITEMS[i].key, CONFIG_ITEMS[i].type);
                continue;
        } // switch
    } // for

    nvs_close(h_config);
    return ESP_OK;
} // cfg_to_json

// Stores the configuration values from a JSON object to NVS.
// The function iterates over all config items, checks if they exist in the JSON object,
// and if they do, it updates the corresponding value in NVS.
// The magic value CONFIG_VALUE_UNCHANGED is used to indicate that a value should not be changed, 
// which is useful for secret values that are not sent to WEB interface for security reasons and 
// hence are not included in the JSON object sent from the web interface when a configuration change is made.
esp_err_t cfg_json_to_nvs(cJSON *root)
{
    esp_err_t res;
    nvs_handle_t h_config;
    cJSON *item = NULL;
    bool had_error = false;

    res = nvs_open(CONFIG_PREFERENCES, NVS_READWRITE, &h_config);
    ESP_ERROR_CHECK(res);

    // Iterate over all config items and check if they exist in the JSON object.
    for (unsigned int i = 0; i < sizeof(CONFIG_ITEMS) / sizeof(config_item_t); i++)
    {
        item = cJSON_GetObjectItem(root, CONFIG_ITEMS[i].key);
        if (item == NULL) {
            continue;
        }

        switch (CONFIG_ITEMS[i].type) {
            case TYPE_CFG_ITEM_STR: {
                if (!cJSON_IsString(item)) {
                    ESP_LOGE(TAG, "Invalid string value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                const char *value = cJSON_GetStringValue(item);
                if (value == NULL) {
                    ESP_LOGE(TAG, "Null string value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                res = nvs_set_str(h_config, CONFIG_ITEMS[i].key, value);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "nvs_set_str(%s) failed: %s",
                             CONFIG_ITEMS[i].key, esp_err_to_name(res));
                    had_error = true;
                }
                continue;
            }

            case TYPE_CFG_ITEM_SECRET_STR: {
                if (!cJSON_IsString(item)) {
                    ESP_LOGE(TAG, "Invalid secret string value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                const char *value = cJSON_GetStringValue(item);
                if (value == NULL) {
                    ESP_LOGE(TAG, "Null secret string value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                if (strcmp(value, CFG_VALUE_UNCHANGED) == 0) {
                    continue;
                }

                res = nvs_set_str(h_config, CONFIG_ITEMS[i].key, value);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "nvs_set_str(%s) failed: %s",
                             CONFIG_ITEMS[i].key, esp_err_to_name(res));
                    had_error = true;
                }
                continue;
            }

            case TYPE_CFG_ITEM_BOOL: {
                uint8_t value = 0;
                if (!json_get_bool_compat(item, &value)) {
                    ESP_LOGE(TAG, "Invalid bool value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                res = nvs_set_u8(h_config, CONFIG_ITEMS[i].key, value);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "nvs_set_u8(%s) failed: %s",
                             CONFIG_ITEMS[i].key, esp_err_to_name(res));
                    had_error = true;
                }
                continue;
            }

            case TYPE_CFG_ITEM_UINT8: {
                uint32_t value = 0;
                if (!json_get_u32_compat(item, UINT8_MAX, &value)) {
                    ESP_LOGE(TAG, "Invalid uint8 value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                res = nvs_set_u8(h_config, CONFIG_ITEMS[i].key, (uint8_t)value);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "nvs_set_u8(%s) failed: %s",
                             CONFIG_ITEMS[i].key, esp_err_to_name(res));
                    had_error = true;
                }
                continue;
            }

            case TYPE_CFG_ITEM_UINT16: {
                uint32_t value = 0;
                if (!json_get_u32_compat(item, UINT16_MAX, &value)) {
                    ESP_LOGE(TAG, "Invalid uint16 value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                res = nvs_set_u16(h_config, CONFIG_ITEMS[i].key, (uint16_t)value);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "nvs_set_u16(%s) failed: %s",
                             CONFIG_ITEMS[i].key, esp_err_to_name(res));
                    had_error = true;
                }
                continue;
            }

            case TYPE_CFG_ITEM_UINT32: {
                uint32_t value = 0;
                if (!json_get_u32_compat(item, UINT32_MAX, &value)) {
                    ESP_LOGE(TAG, "Invalid uint32 value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                res = nvs_set_u32(h_config, CONFIG_ITEMS[i].key, value);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "nvs_set_u32(%s) failed: %s",
                             CONFIG_ITEMS[i].key, esp_err_to_name(res));
                    had_error = true;
                }
                continue;
            }

            case TYPE_CFG_ITEM_IP: {
                uint32_t value = 0;
                if (!json_get_ip4_array(item, &value)) {
                    ESP_LOGE(TAG, "Invalid IPv4 array value for key %s", CONFIG_ITEMS[i].key);
                    had_error = true;
                    continue;
                }

                res = nvs_set_u32(h_config, CONFIG_ITEMS[i].key, value);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "nvs_set_u32(%s) failed: %s",
                             CONFIG_ITEMS[i].key, esp_err_to_name(res));
                    had_error = true;
                }
                continue;
            }
        }
    }

    if (!had_error) {
        res = nvs_commit(h_config);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(res));
            had_error = true;
        }
    }

    nvs_close(h_config);

    return had_error ? ESP_ERR_INVALID_ARG : ESP_OK;
} // cfg_json_to_nvs



// Reads the string value of a configuration item from NVS, 
// The needed memory is allocated here, because the caller does not know the length of the string in advance. 
// The caller should free the allocated memory after using it. 
esp_err_t cfg_get_str(const char* key, char** out_value)
{
    esp_err_t res;
    nvs_handle_t h_cfg;       // Local handle to the NVS partition, used for reading.
    size_t str_len = 0;
    
    res = nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_cfg);
    ESP_ERROR_CHECK(res);  // this should not fail as the partition should have been initialized in nvs_cfg_init()

    res = nvs_get_str(h_cfg, key, NULL, &str_len);
    if (res != ESP_OK) { 
        nvs_close(h_cfg);
        return res;
   }
        
    *out_value = malloc(str_len);

    res = nvs_get_str(h_cfg, key, *out_value, &str_len);
    if (res != ESP_OK) {
        free(*out_value);
        nvs_close(h_cfg);
        return res;
    }

    nvs_close(h_cfg);
    return ESP_OK;

} //cfg_get_str

esp_err_t cfg_get_u8(const char* key, uint8_t* out_value)
{
    esp_err_t res;
    nvs_handle_t h_cfg;       // Local handle to the NVS partition, used for reading.
    
    res = nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_cfg);
    ESP_ERROR_CHECK(res);  // this should not fail as the partition should have been initialized in cfg_init()

    res = nvs_get_u8(h_cfg, key, out_value);

    nvs_close(h_cfg);
    return res;
    
} //cfg_get_u8

esp_err_t cfg_get_u16(const char* key, uint16_t* out_value)
{
    esp_err_t res;
    nvs_handle_t h_cfg;       // Local handle to the NVS partition, used for reading.
    
    res = nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_cfg);
    ESP_ERROR_CHECK(res);  // this should not fail as the partition should have been initialized in cfg_init()

    res = nvs_get_u16(h_cfg, key, out_value);

    nvs_close(h_cfg);
    return res;
    
} //cfg_get_u16

esp_err_t cfg_get_u32(const char* key, uint32_t* out_value)
{
    nvs_handle_t h_cfg;       // Local handle to the NVS partition, used for reading.
    
    esp_err_t res = nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_cfg);
    ESP_ERROR_CHECK(res);  // this should not fail as the partition should have been initialized in nvs_cfg_init()

    res = nvs_get_u32(h_cfg, key, out_value);

    nvs_close(h_cfg);
    return res;
    
} //cfg_get_u32
