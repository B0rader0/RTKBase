#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_wifi.h>

typedef struct wifi_ap_status {
    bool active;

    char ssid[33];
    wifi_auth_mode_t authmode;
    uint8_t devices;

    esp_ip4_addr_t ip4_addr;
    esp_ip6_addr_t ip6_addr;
} wifi_ap_status_t;

typedef struct wifi_sta_status {
    bool active;
    bool connected;

    char ssid[33];
    wifi_auth_mode_t authmode;
    int8_t rssi;

    esp_ip4_addr_t ip4_addr;
    esp_ip6_addr_t ip6_addr;
} wifi_sta_status_t;

typedef struct wifi_sta_diag {
    uint32_t connect_count;
    uint32_t disconnect_count;
    int32_t last_disconnect_reason;
    char last_disconnect_reason_name[32];
    int8_t last_rssi;
    uint8_t last_channel;
    uint8_t last_bssid[6];
    bool last_bssid_valid;
    uint64_t last_disconnect_ms;
} wifi_sta_diag_t;

void net_init();
void wifi_init();

wifi_ap_record_t * wifi_scan(uint16_t *number);

wifi_sta_list_t *wifi_ap_sta_list();

void wifi_ap_status(wifi_ap_status_t *status);
void wifi_sta_status(wifi_sta_status_t *status);
void wifi_sta_get_diagnostics(wifi_sta_diag_t *diag);

void wait_for_ip();
void wait_for_network();

const char *esp_netif_name(esp_netif_t *esp_netif);
const char * wifi_auth_mode_name(wifi_auth_mode_t auth_mode);

