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
#include "esp_timer.h"

#include "gps_uart.h"
#include "config.h"
#include "interface/socket_server.h"
#include "tasks.h"

static const char *TAG = "UART";

// The version command returns 120 bytes of data, so we set a larger buffer. The config com1/2/3 commands return 80 bytes of data, so the buffer size is also sufficient for them. If we receive more than 256 bytes in a single line, it will be truncated, but this should not happen with the expected commands.
#define MAX_LINE_LEN 250 // Maximum length of a line (NMEA sentence) that we expect to receive. The actual maximum length of an NMEA sentence is 82 characters, but we can set it to a higher value to be safe and to allow for any additional data that might be included in the line.
#define EVENT_QUEUE_SIZE   32  // Buffer for bursts of lines
#define UART_BUFFER_SIZE (1024) // Size of the UART driver's internal buffer for incoming data. This should be large enough to hold the expected bursts of data without overflowing. The actual size needed depends on the expected data rate and how quickly we process incoming lines.
 
#define PATTERN_CHR_NUM    (1)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/


// --- Event Definitions ---
ESP_EVENT_DEFINE_BASE(NTRIP_EVENTS);
enum {
    NTRIP_EVENT_LINE_RECEIVED 
};
 
typedef struct {
    char str[MAX_LINE_LEN];
    uint8_t len;
} uart_data_t;
 
// --- Globals ---
static QueueHandle_t uart_queue; // Queue to receive UART events from the ISR
static esp_event_loop_handle_t ntrip_event_loop; // Event loop for the received and stitched NTRIP messages
static uint8_t gps_port = 0;        // Default UART port, is set from NVS in uart_init().

static char ntrip_line_buffer[MAX_LINE_LEN]; // Buffer to accumulate incoming data until a full line is received
static int empty_pos = 0;     // Used to stitch DATA event to DET
    

// legacy - check if needed
static bool uart_log_forward = false;
//static stream_stats_handle_t stream_stats;

// To start listening:
void ntrip_handler_register(esp_event_handler_t event_handler) {
    esp_event_handler_register_with(ntrip_event_loop, NTRIP_EVENTS, NTRIP_EVENT_LINE_RECEIVED, event_handler, NULL);
}
  
void ntrip_handler_unregister(esp_event_handler_t event_handler) {
    esp_event_handler_unregister_with(ntrip_event_loop, NTRIP_EVENTS, NTRIP_EVENT_LINE_RECEIVED, event_handler);
}

// This is the "Pool" logic. Every registered task executes this.
void on_uart_received(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    uart_data_t* data = (uart_data_t*) event_data;
    const char* task_id = (const char*) handler_args;
 
    // Logic for each subscriber goes here
    ESP_LOGI(TAG, "Subscriber [%s] processing: %s", task_id, data->str);
}
 
// ---  The UART Processing Task ---
static void uart_event_reader_task(void *pvParameters)
{
    uart_event_t event;
    int bytes_read;
    
    // uart_read_bytes - portMAX_DELAY or 100 / portTICK_PERIOD_MS????
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t) portMAX_DELAY)) {

            switch (event.type) {
            case UART_DATA: 
                
                // Sore data at the beginning of the buffer
                empty_pos = uart_read_bytes(gps_port, ntrip_line_buffer, event.size, portMAX_DELAY);

                // never happnes, the problem is elsewhere.
                if (empty_pos != event.size) {
                    ESP_LOGW(TAG, "UART_DATA: event.size %d, empty_pos: %d", event.size, empty_pos);
                }

                // the below never happens, so the problem is somewhere else.
                if (empty_pos < 0 || empty_pos >= MAX_LINE_LEN) {
                    ESP_LOGW(TAG, "UART_DATA: empty_pos out of range: %d", empty_pos);
                    empty_pos = 0; // Reset the buffer position to the beginning of the buffer to avoid overflow
                }
                
                
                ntrip_line_buffer[empty_pos] = '\0'; // Null-terminate the buffer to make it a valid C string for debug purposes

                //ESP_LOGI(TAG, "UART_DATA: string: %s", ntrip_line_buffer);

                break;

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
            ESP_LOGE(TAG, "UART_BUFFER_FULL or OVERFLOW");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(gps_port);
                xQueueReset(uart_queue);
                empty_pos = 0; // Reset the buffer position to the beginning of the buffer
                break;
            //Event of UART RX break detected
            case UART_BREAK:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
                break;

            case UART_PATTERN_DET:
                
                int pattern_pos;
                
                while ((pattern_pos = uart_pattern_pop_pos(gps_port)) != -1) {
                    
                    // Do not wait!!!
                    bytes_read = uart_read_bytes(gps_port, ntrip_line_buffer + empty_pos, pattern_pos + PATTERN_CHR_NUM, portMAX_DELAY);//0); //100 / portTICK_PERIOD_MS);
                    
                    if (bytes_read != (pattern_pos + PATTERN_CHR_NUM)) {
                        ESP_LOGW(TAG, "ERROR!!!");
                    }

                    ntrip_line_buffer[empty_pos + pattern_pos + PATTERN_CHR_NUM] = '\0'; // Null-terminate the buffer to make it a valid C string
                    
                    // Filter invalid lines
                    if (strlen(ntrip_line_buffer) < 3) {
                        uart_flush_input(gps_port); // Flush the input buffer to clear any remaining data
                        empty_pos = 0; // Reset the buffer position to the beginning of the buffer
                        break;
                    }
                    
                    if (ntrip_line_buffer[strlen(ntrip_line_buffer) - 1] != '\n' || ntrip_line_buffer[strlen(ntrip_line_buffer) - 2] != '\r') {
                        uart_flush_input(gps_port); // Flush the input buffer to clear any remaining data
                        empty_pos = 0; // Reset the buffer position to the beginning of the buffer
                        break;
                    }
                    
                    empty_pos = 0; // Reset the buffer position to the beginning of the buffer for the next line. We have already read the line into the buffer, so we can reset the position for the next line.
                    
                    // send an extra byte, set to 0, to ensure that the string is null-terminated when received by the handlers.
                    esp_event_post_to(ntrip_event_loop, NTRIP_EVENTS, NTRIP_EVENT_LINE_RECEIVED, ntrip_line_buffer, strlen(ntrip_line_buffer) + 1, portMAX_DELAY);
                    
                } // while

                empty_pos = 0; //better?
                // do not flush uart here !!!        

                break;                
            // Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

/* void uart_register_read_handler(esp_event_handler_t event_handler)
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
 */

esp_err_t uart_init()
{
    nvs_handle_t h_config;
    uart_config_t uart_config;
    config_item_value_t cfg_var;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_open(CONFIG_PREFERENCES, NVS_READONLY, &h_config));

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_LOG_FORWARD, &cfg_var.uint8));
    uart_log_forward = cfg_var.enabled;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_NUM, &cfg_var.uint8));
    gps_port = cfg_var.uint8;

    // UART configuration structure. The values are populated from NVS later, after reading them from NVS.
    // Populating the uart_config structure with the configuration values read from NVS.
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u32(h_config, KEY_CONFIG_UART_BAUD_RATE, &cfg_var.uint32));
    uart_config.baud_rate = cfg_var.uint32;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_DATA_BITS, &cfg_var.uint8));
    uart_config.data_bits = cfg_var.uint8;
    
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_PARITY, &cfg_var.uint8));
    uart_config.parity = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_STOP_BITS, &cfg_var.uint8));
    uart_config.stop_bits = cfg_var.uint8;

    // The flow control is set by bitwise ORing the RTS and CTS flow control values, which are defined in uart_hw_flowcontrol_t enum. If both are disabled, the flow control will be disabled. If only one of them is enabled, the flow control will be set to the corresponding value. If both are enabled, the flow control will be set to UART_HW_FLOWCTRL_CTS_RTS.
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; // default value, will be updated later based on the RTS and CTS flow control config values read from NVS.
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_FLOW_CTRL_RTS, &cfg_var.uint8));
    if (cfg_var.enabled) {
        uart_config.flow_ctrl = uart_config.flow_ctrl | UART_HW_FLOWCTRL_RTS;
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_FLOW_CTRL_CTS, &cfg_var.uint8));
    if (cfg_var.enabled) {
        uart_config.flow_ctrl = uart_config.flow_ctrl | UART_HW_FLOWCTRL_CTS;
    };

    uart_config.flags.allow_pd = false; // Do not allow power down.
    uart_config.flags.backup_before_sleep = false; // Do not backup before sleep, as we do not allow power down.
    uart_config.source_clk = UART_SCLK_DEFAULT; // Use default source clock.
    
    // UART pin confguration.
    int pin_tx, pin_rx, pin_rts, pin_cts;
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_TX_PIN, &cfg_var.uint8));
    pin_tx = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_RX_PIN, &cfg_var.uint8));
    pin_rx = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_RTS_PIN, &cfg_var.uint8));
    pin_rts = cfg_var.uint8;

    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_get_u8(h_config, KEY_CONFIG_UART_CTS_PIN, &cfg_var.uint8));
    pin_cts = cfg_var.uint8;

    nvs_close(h_config);

    // A. Setup Event Loop with custom queue size for bursts
    // The received GSM messages will be posted to this event loop, 
    // and all the tasks that want to receive the messages will subscribe to this event loop.
    // This is created before the UART driver is set up, so that we can post events to it from the UART event reader task 
    // as soon as we start receiving data from the GPS.
    esp_event_loop_args_t loop_args = {
        .queue_size = EVENT_QUEUE_SIZE,
        .task_name = "gps_evt_loop_task",
        .task_priority = 15,
        .task_stack_size = 3072,
        .task_core_id = tskNO_AFFINITY
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &ntrip_event_loop));
 

    // Setup UART
    // uart_config is already populated with the values read from NVS. 
    // We just need to call the UART driver functions to apply the configuration and set up the UART.
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(gps_port, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_param_config(gps_port, &uart_config)); 
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_pin(gps_port, pin_tx, pin_rx, pin_rts, pin_cts));
   
    // D. Enable Pattern Detect for '\n' (ASCII 10)
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_enable_pattern_det_baud_intr(gps_port, '\n', PATTERN_CHR_NUM, 9, 0, 0)); // 9 What are the correct parameters?
    
    //Reset the pattern queue length to record at most 20 pattern positions.
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_pattern_queue_reset(gps_port, 20));//
    
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_reader_task, "uart_event_reader", 4096, NULL, 12, NULL);
    
    //stream_stats = stream_stats_new("uart");

    return ESP_OK;
} // uart_init