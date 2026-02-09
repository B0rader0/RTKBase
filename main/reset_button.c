/*
*  Creates a handler for a  button for resetting the parameters to factory defaults. 
*  When thge button held down for more than BTN_LONG_PRESS and then released, the NVM storage is cleared
*  The idea to use the built LED does not work because the LED is not controllable.
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_pm.h>
#include <iot_button.h>
#include <esp_sleep.h>
#include <esp_idf_version.h>
#include <button_gpio.h>
#include <nvs_flash.h>  // For initializing and erasing the flash partition
//#include <nvs.h>        // For reading/writing actual data (keys and values)
#include "status_led.h"

/* 
*  Most development boards have "boot" button attached to GPIO0.
*  You can also change this to another pin.
*/
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32H2 || CONFIG_IDF_TARGET_ESP32C6
#define BOOT_BUTTON_NUM         9
#else
#define BOOT_BUTTON_NUM         0
#endif
#define BUTTON_ACTIVE_LEVEL     0
#define LONG_PRESS_MS          5000

// This is the callback function that will be called when the long press is released. It performs the factory reset and reboots the device.
static void button_event_reset_and_reboot(void *arg, void *data)
{
    
    // 1. Erase NVS (using the default partition)
    // Note: nvs_flash_erase is safer for a full reset than just nvs_erase_all
    nvs_flash_deinit();
    nvs_flash_erase_partition("nvs");
        
    // 2. Restart
    esp_restart();
}

//void button_init(uint32_t button_num)
void reset_button_init()
{
    
    button_config_t btn_cfg = {
        .long_press_time = LONG_PRESS_MS,
        .short_press_time = 10, // Not used, but set to a low value to avoid interference with long press
    };
    button_gpio_config_t gpio_cfg = {
        .gpio_num = BOOT_BUTTON_NUM,
        .active_level = BUTTON_ACTIVE_LEVEL,
        .enable_power_save = false,
    };

    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);

    ret = iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, NULL, button_event_reset_and_reboot, NULL);

    ESP_ERROR_CHECK(ret);
}

