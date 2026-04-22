#include "time_sync.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <esp_log.h>
#include <esp_sntp.h>

#include "nvs_config.h"

static const char *TAG = "TIME";

#define TIME_VALID_AFTER_UNIX 315360000L

static bool s_started;

static void time_sync_notification_cb(struct timeval *tv)
{
    (void)tv;

    time_t now = time(NULL);
    struct tm local_time;
    char text[32];

    localtime_r(&now, &local_time);
    strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%S", &local_time);
    ESP_LOGI(TAG, "Time synchronized: %s", text);
}

bool time_sync_time_valid(void)
{
    return time(NULL) > TIME_VALID_AFTER_UNIX;
}

void time_sync_init(void)
{
    char *timezone = NULL;

    if (cfg_get_str(KEY_CONFIG_TIMEZONE, &timezone) == ESP_OK &&
        timezone != NULL && timezone[0] != '\0') {
        setenv("TZ", timezone, 1);
    } else {
        setenv("TZ", "UTC0", 1);
    }

    free(timezone);
    tzset();
}

void time_sync_start(void)
{
    if (s_started) {
        return;
    }

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
    s_started = true;

    ESP_LOGI(TAG, "SNTP time synchronization started");
}
