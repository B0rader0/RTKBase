/*
 * This file is based on ESP32-XBee distribution (https://github.com/nebkat/esp32-xbee).
 */

#include <freertos/FreeRTOS.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_mac.h>   
#include <string.h>
#include <mdns.h>
#include <math.h>
#include <driver/gpio.h>
#include <sys/param.h>
//#include <tasks.h>
#include <retry.h>
#include <freertos/event_groups.h>
#include <esp_netif_ip_addr.h>
#include <lwip/lwip_napt.h>
#include <string.h>
#include <esp_mac.h>
#include "wifi.h"
#include "nvs_config.h"
#include "gnss_uart.h"

static const char *TAG = "WIFI";

static EventGroupHandle_t wifi_event_group;
const int WIFI_STA_GOT_IPV4_BIT = BIT0;
const int WIFI_STA_GOT_IPV6_BIT = BIT1;
const int WIFI_AP_STA_CONNECTED_BIT = BIT2;

static TaskHandle_t sta_status_task = NULL;
static TaskHandle_t sta_reconnect_task = NULL;

static wifi_config_t config_ap;
static wifi_config_t config_sta;

static retry_delay_handle_t delay_handle;

static bool ap_active = false;
static bool sta_active = false;

static bool sta_connected;
static wifi_ap_record_t sta_ap_info;
static wifi_sta_list_t ap_sta_list;

static esp_netif_t *esp_netif_ap;
static esp_netif_t *esp_netif_sta;

static void wifi_sta_reconnect_task(void *ctx) {
    while (true) {
        int attempts = retry_delay(delay_handle);

        ESP_LOGI(TAG, "Station Reconnecting: %s, attempts: %d", config_sta.sta.ssid, attempts);

        esp_wifi_connect();

        // Wait for next disconnect event
        vTaskSuspend(NULL);
    }
}

static void handle_sta_start(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "WIFI_EVENT_STA_START");

    sta_active = true;

    esp_wifi_connect();
}

static void handle_sta_stop(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "WIFI_EVENT_STA_STOP");

    sta_active = false;
}

static void handle_sta_connected(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    const wifi_event_sta_connected_t *event = (const wifi_event_sta_connected_t *) event_data;

    ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED: ssid: %.*s", event->ssid_len, event->ssid);

    sta_connected = true;

    retry_reset(delay_handle);

    // Tracking status
    if (sta_status_task != NULL) vTaskResume(sta_status_task);

    // No longer attempting to reconnect
    if (sta_reconnect_task != NULL) vTaskSuspend(sta_reconnect_task);

}

static void handle_sta_disconnected(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    const wifi_event_sta_disconnected_t *event = (const wifi_event_sta_disconnected_t *) event_data;
    char *reason;
    switch (event->reason) {
        case WIFI_REASON_AUTH_EXPIRE:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_ASSOC_EXPIRE:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
            reason = "AUTH";
            break;
        case WIFI_REASON_NO_AP_FOUND:
            reason = "NOT_FOUND";
            break;
        default:
            reason = "UNKNOWN";
    }

    ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED: ssid: %.*s, reason: %d (%s)", event->ssid_len, event->ssid, event->reason, reason);

    sta_connected = false;

    // No longer tracking status
    if (sta_status_task != NULL) vTaskSuspend(sta_status_task);

    // Attempting to reconnect
    if (sta_reconnect_task != NULL) vTaskResume(sta_reconnect_task);

    xEventGroupClearBits(wifi_event_group, WIFI_STA_GOT_IPV4_BIT);
    xEventGroupClearBits(wifi_event_group, WIFI_STA_GOT_IPV6_BIT);
}

static void handle_sta_auth_mode_change(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    const wifi_event_sta_authmode_change_t *event = (const wifi_event_sta_authmode_change_t *) event_data;
    const char *old_auth_mode = wifi_auth_mode_name(event->old_mode);
    const char *new_auth_mode = wifi_auth_mode_name(event->new_mode);

    ESP_LOGI(TAG, "WIFI_EVENT_STA_AUTHMODE_CHANGE: old: %s, new: %s", old_auth_mode, new_auth_mode);
}

static void handle_ap_start(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "WIFI_EVENT_AP_START");

    // IP forwarding/NATP
    bool forward;
    cfg_get_u8(KEY_CONFIG_WIFI_STA_AP_FORWARD, (uint8_t*) &forward);
    if (forward) {
        esp_netif_ip_info_t ip_info_ap;
        esp_netif_get_ip_info(esp_netif_ap, &ip_info_ap);
        ip_napt_enable(ip_info_ap.ip.addr, 1);
    }

    ap_active = true;
}

static void handle_ap_stop(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "WIFI_EVENT_AP_STOP");

    ap_active = false;
}

static void handle_ap_sta_connected(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    const wifi_event_ap_staconnected_t *event = (const wifi_event_ap_staconnected_t *) event_data;

    ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED: mac: " MACSTR, MAC2STR(event->mac));

    xEventGroupSetBits(wifi_event_group, WIFI_AP_STA_CONNECTED_BIT);
}

static void handle_ap_sta_disconnected(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    const wifi_event_ap_stadisconnected_t *event = (const wifi_event_ap_stadisconnected_t *) event_data;

    ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED: mac: " MACSTR, MAC2STR(event->mac));

    wifi_ap_sta_list();
    if (ap_sta_list.num == 0) {
        xEventGroupClearBits(wifi_event_group, WIFI_AP_STA_CONNECTED_BIT);
    }
}

static void handle_sta_got_ip(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    const ip_event_got_ip_t *event = (const ip_event_got_ip_t *) event_data;

    // IP forwarding/NATP update AP DHCPS DNS info  
    bool forward;
    cfg_get_u8(KEY_CONFIG_WIFI_STA_AP_FORWARD, (uint8_t*) &forward);
    if (ap_active & forward) {
        esp_netif_dns_info_t dns_info_sta;
        ESP_ERROR_CHECK(esp_netif_get_dns_info(esp_netif_sta, ESP_NETIF_DNS_MAIN, &dns_info_sta));

        ESP_ERROR_CHECK(esp_netif_dhcps_stop(esp_netif_ap));
        ESP_ERROR_CHECK(esp_netif_set_dns_info(esp_netif_ap, ESP_NETIF_DNS_MAIN, &dns_info_sta));
        ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_netif_ap));
    }

    ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP: ip: " IPSTR "/%d, gw: " IPSTR,
            IP2STR(&event->ip_info.ip),
            ffs(~event->ip_info.netmask.addr) - 1,
            IP2STR(&event->ip_info.gw));
   
    xEventGroupSetBits(wifi_event_group, WIFI_STA_GOT_IPV4_BIT);
}

static void handle_sta_lost_ip(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "IP_EVENT_STA_LOST_IP");

    xEventGroupClearBits(wifi_event_group, WIFI_STA_GOT_IPV4_BIT);
}

static void handle_ap_sta_ip_assigned(void *esp_netif, esp_event_base_t base, int32_t event_id, void *event_data) {
    const ip_event_ap_staipassigned_t *event = (const ip_event_ap_staipassigned_t *) event_data;

    ESP_LOGI(TAG, "IP_EVENT_AP_STAIPASSIGNED: ip: " IPSTR, IP2STR(&event->ip));
}

void wait_for_ip() {
    xEventGroupWaitBits(wifi_event_group, WIFI_STA_GOT_IPV4_BIT, false, false, portMAX_DELAY);
}

void wait_for_network() {
    xEventGroupWaitBits(wifi_event_group, WIFI_STA_GOT_IPV4_BIT | WIFI_AP_STA_CONNECTED_BIT, false, false, portMAX_DELAY);
}

// Initialise WiFi according to the configurations (mode, credentials, IP, hostname).
// Blocks until network is up (STA connected / AP ready).
void net_init() {
    uint8_t bool_var; // Used for reading boolean config values from NVS, which are stored as uint8_t (0 or 1)

    ESP_ERROR_CHECK(esp_netif_init());

    // Soft AP (access point)
    cfg_get_u8(KEY_CONFIG_WIFI_AP_ACTIVE, &bool_var);
    if (bool_var) {
        ESP_LOGI(TAG, "Starting WiFi in AP mode, bool var is %d", bool_var);
        esp_netif_ap = esp_netif_create_default_wifi_ap();

        // IP configuration 
        esp_netif_ip_info_t ip_info_ap;
        cfg_get_u32(KEY_CONFIG_WIFI_AP_GATEWAY, (uint32_t*) &ip_info_ap.ip);
        ip_info_ap.gw = ip_info_ap.ip;

        uint8_t subnet = 24; // Default subnet
        ESP_ERROR_CHECK(cfg_get_u8(KEY_CONFIG_WIFI_AP_SUBNET, &subnet));
        
        ip_info_ap.netmask.addr = esp_netif_htonl(0xffffffffu << (32u - subnet));

        // IP forwarding/NATP
        cfg_get_u8(KEY_CONFIG_WIFI_STA_AP_FORWARD, &bool_var);
        if (bool_var) {
            uint8_t dhcps_offer = true;
            ESP_ERROR_CHECK(esp_netif_dhcps_option(esp_netif_ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_offer, 1));
        }

        ESP_ERROR_CHECK(esp_netif_dhcps_stop(esp_netif_ap));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_netif_ap, &ip_info_ap));
        ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_netif_ap));
    }

    // STA (station)
    cfg_get_u8(KEY_CONFIG_WIFI_STA_ACTIVE, &bool_var);
    if (bool_var) {
        esp_netif_ip_info_t ip_info_sta;
        
        esp_netif_sta = esp_netif_create_default_wifi_sta();

        char *hostname;
        cfg_get_str(KEY_CONFIG_STATION_HOSTNAME, &hostname);
        ESP_ERROR_CHECK(esp_netif_set_hostname(esp_netif_sta, hostname));
        free(hostname);

        // Static IP configuration
        cfg_get_u8(KEY_CONFIG_WIFI_STA_STATIC, (uint8_t*) &bool_var);
        if (bool_var) {
            cfg_get_u32(KEY_CONFIG_WIFI_STA_IP, (uint32_t*) &ip_info_sta.ip);
            cfg_get_u32(KEY_CONFIG_WIFI_STA_GATEWAY, (uint32_t*) &ip_info_sta.gw);
            uint8_t subnet = 24; // Default subnet
            ESP_ERROR_CHECK(cfg_get_u8(KEY_CONFIG_WIFI_STA_SUBNET, &subnet));
            ip_info_sta.netmask.addr = esp_netif_htonl(0xffffffffu << (32u - subnet));

            esp_netif_dns_info_t dns_info_sta_main, dns_info_sta_backup;
            cfg_get_u32(KEY_CONFIG_WIFI_STA_DNS_A, (uint32_t*) &dns_info_sta_main.ip.u_addr.ip4.addr);
            cfg_get_u32(KEY_CONFIG_WIFI_STA_DNS_B, (uint32_t*) &dns_info_sta_backup.ip.u_addr.ip4.addr);

            ESP_ERROR_CHECK(esp_netif_dhcpc_stop(esp_netif_sta));
            ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_netif_sta, &ip_info_sta));
            ESP_ERROR_CHECK(esp_netif_set_dns_info(esp_netif_sta, ESP_NETIF_DNS_MAIN, &dns_info_sta_main));
            ESP_ERROR_CHECK(esp_netif_set_dns_info(esp_netif_sta, ESP_NETIF_DNS_BACKUP, &dns_info_sta_backup));
            ESP_ERROR_CHECK(esp_netif_dhcpc_start(esp_netif_sta));
        }
    }
}

void wifi_init() {
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    wifi_event_group = xEventGroupCreate();

    // Listen for WiFi and IP events
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START, &handle_sta_start, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_STOP, &handle_sta_stop, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &handle_sta_connected, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &handle_sta_disconnected, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_AUTHMODE_CHANGE, &handle_sta_auth_mode_change, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, &handle_ap_start, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &handle_ap_stop, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &handle_ap_sta_connected, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &handle_ap_sta_disconnected, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &handle_sta_got_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &handle_sta_lost_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &handle_ap_sta_ip_assigned, NULL));

    // Reconnect delay timer
    delay_handle = retry_init(true, 5, 2000, 60000);

    bool sta_enable;
    cfg_get_u8(KEY_CONFIG_WIFI_STA_ACTIVE, (uint8_t*) &sta_enable);
    bool ap_enable;
    cfg_get_u8(KEY_CONFIG_WIFI_AP_ACTIVE, (uint8_t*) &ap_enable);

    // Configure and connect
    wifi_mode_t wifi_mode;
    if (sta_enable && ap_enable) {
        wifi_mode = WIFI_MODE_APSTA;
    } else if (ap_enable) {
        wifi_mode = WIFI_MODE_AP;
    } else if (sta_enable) {
        wifi_mode = WIFI_MODE_STA;
    } else { //The device needs to be in at least one mode to be useful, so we will default to AP mode if both are disabled.
        wifi_mode = WIFI_MODE_AP;
        ESP_LOGW(TAG, "Both STA and AP modes are disabled in configuration, defaulting to AP mode.");
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // SoftAP
    if (ap_enable) {
        esp_netif_ip_info_t ip_info_ap;
        esp_netif_get_ip_info(esp_netif_ap, &ip_info_ap);

        config_ap.ap.max_connection = 4;
        char *buff = NULL;
        config_ap.ap.ssid_len = 0; // If 0, the SSID is expected to be null-terminated string, and the length will be determined by strlen. This allows for flexibility in how the SSID is stored in the config (e.g. as a fixed-size string with null terminator, or as a separate string with length). If ssid_len is not 0, then the SSID will be read as a byte array of the specified length, which allows for SSIDs that may contain null bytes or are not null-terminated.
        cfg_get_str(KEY_CONFIG_WIFI_AP_SSID, &buff); //, &ap_ssid_len);
        strncpy((char *) config_ap.ap.ssid, buff, sizeof(config_ap.ap.ssid));
        free(buff);
       
        cfg_get_u8(KEY_CONFIG_WIFI_AP_SSID_HIDDEN, &config_ap.ap.ssid_hidden);
        size_t ap_password_len = sizeof(config_ap.ap.password);
        // fixme - a specific type needed cfg_get_str(KEY_CONFIG_WIFI_AP_PASSWORD, &config_ap.ap.password); //, &ap_password_len);
        ap_password_len--; // Remove null terminator from length
        cfg_get_u8(KEY_CONFIG_WIFI_AP_AUTH_MODE, (uint8_t*) &config_ap.ap.authmode);

        ESP_LOGI(TAG, "WIFI_AP_SSID: %s %s(%s)", config_ap.ap.ssid,
                config_ap.ap.ssid_hidden ? "(hidden) " : "",
                ap_password_len == 0 ? "open" : "with password");

        ESP_LOGI(TAG, "WIFI_AP_IP: ip: " IPSTR "/%d, gw: " IPSTR,
                IP2STR(&ip_info_ap.ip),
                ffs(~ip_info_ap.netmask.addr) - 1,
                IP2STR(&ip_info_ap.gw));

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config_ap));
        ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));

    }

    // STA
    if (sta_enable) {
        // Read SSID from config, it is assumed to be set. 
        char *buff = NULL;

        ESP_ERROR_CHECK(cfg_get_str(KEY_CONFIG_WIFI_STA_SSID, &buff)); //This should not fail as the config should have been initialized in cfg_init(), but if it does, we will treat it as if the SSID was not set and disable STA mode.
        strncpy((char *) config_sta.sta.ssid, buff, sizeof(config_sta.sta.ssid));   
        free(buff);

        ESP_ERROR_CHECK(cfg_get_str(KEY_CONFIG_WIFI_STA_PASSWORD, &buff)); //This should not fail as the config should have been initialized in cfg_init(), but if it does, we will treat it as if the SSID was not set and disable STA mode.
        strncpy((char *) config_sta.sta.password, buff, sizeof(config_sta.sta.password));   
        free(buff);
        
        cfg_get_u8(KEY_CONFIG_WIFI_STA_SCAN_MODE_ALL, (uint8_t*) &config_sta.sta.scan_method);
        
        ESP_LOGI(TAG, "WIFI_STA_CONNECTING: %s (%s), %s scan", config_sta.sta.ssid,
                strlen((char *) config_sta.sta.password) == 0 ? "open" : "with password",
                config_sta.sta.scan_method == WIFI_ALL_CHANNEL_SCAN ? "all channel" : "fast");

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config_sta));
        ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));

        // Reconnect when disconnected
        xTaskCreate(wifi_sta_reconnect_task, "wifi_sta_reconnect", 4096, NULL, TASK_PRIORITY_WIFI_STATUS, &sta_reconnect_task);
        vTaskSuspend(sta_reconnect_task);

    }

    ESP_ERROR_CHECK(esp_wifi_start());
}

wifi_sta_list_t *wifi_ap_sta_list() {
    esp_wifi_ap_get_sta_list(&ap_sta_list);
    return &ap_sta_list;
}

void wifi_ap_status(wifi_ap_status_t *status) {
    status->active = ap_active;
    if (!ap_active) return;

    memcpy(status->ssid, config_ap.ap.ssid, sizeof(config_ap.ap.ssid));
    status->authmode = config_ap.ap.authmode;

    wifi_ap_sta_list();
    status->devices = ap_sta_list.num;


    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_ap, &ip_info);
    status->ip4_addr = ip_info.ip;

    esp_netif_get_ip6_linklocal(esp_netif_ap, &status->ip6_addr);
}

void wifi_sta_status(wifi_sta_status_t *status) {
    status->active = sta_active;
    status->connected = sta_connected;
    if (!sta_connected) {
        memcpy(status->ssid, config_sta.sta.ssid, sizeof(config_sta.sta.ssid));
        return;
    }

    memcpy(status->ssid, sta_ap_info.ssid, sizeof(sta_ap_info.ssid));
    status->rssi = sta_ap_info.rssi;
    status->authmode = sta_ap_info.authmode;

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_sta, &ip_info);
    status->ip4_addr = ip_info.ip;

    esp_netif_get_ip6_linklocal(esp_netif_sta, &status->ip6_addr);
}

wifi_ap_record_t *wifi_scan(uint16_t *number) {
    wifi_mode_t wifi_mode;
    esp_wifi_get_mode(&wifi_mode);

    // Ensure STA is enabled
    if (wifi_mode != WIFI_MODE_APSTA && wifi_mode != WIFI_MODE_STA) {
        esp_wifi_set_mode(wifi_mode == WIFI_MODE_AP ? WIFI_MODE_APSTA : WIFI_MODE_STA);
    }

    wifi_scan_config_t wifi_scan_config = {
            .ssid = NULL,
            .bssid = NULL,
            .channel = 0,
            .show_hidden = 0
    };

    esp_wifi_scan_start(&wifi_scan_config, true);

    esp_wifi_scan_get_ap_num(number);
    if (*number <= 0) {
        return NULL;
    }

    wifi_ap_record_t *ap_records = (wifi_ap_record_t *) malloc(*number * sizeof(wifi_ap_record_t));
    esp_wifi_scan_get_ap_records(number, ap_records);

    return ap_records;
}

const char *wifi_auth_mode_name(wifi_auth_mode_t auth_mode) {
    switch (auth_mode) {
        case WIFI_AUTH_OPEN:
            return "OPEN";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA_PSK";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2_PSK";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA/2_PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE:
            return "WPA2_ENTERPRISE";
        default:
            return "Unknown";
    }
}
