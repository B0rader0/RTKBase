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

#define MAX_LINE_LEN 102 // Maximum length of a line (NMEA sentence) that we expect to receive. The actual maximum length of an NMEA sentence is 82 characters, but we can set it to a higher value to be safe and to allow for any additional data that might be included in the line.
#define LINE_TIMEOUT_MS  200   // Time to wait for \n after receiving \r 
#define EVENT_QUEUE_SIZE   64  // Buffer for bursts of lines
 
#define PATTERN_CHR_NUM    (1)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define RD_BUF_SIZE (MAX_LINE_LEN)


// --- Event Definitions ---
ESP_EVENT_DEFINE_BASE(GPS_EVENTS);
enum {
    GPS_EVENT_LINE_RECEIVED 
};
 
typedef struct {
    char str[MAX_LINE_LEN];
    uint8_t len;
} uart_data_t;
 
// --- Globals ---
static QueueHandle_t uart_queue;
// not used for now static esp_timer_handle_t timeout_timer;
static esp_event_loop_handle_t gps_event_loop;
static uint8_t gps_port = 0; // Default UART port, can be set from NVS

// legacy - check if needed
static bool uart_log_forward = false;
//static stream_stats_handle_t stream_stats;

 
// --- 1. Timeout Cleanup ---
// If \n doesn't arrive shortly after \r, we ignore the corrupted data.
void line_timeout_callback(void* arg) {
    //uint8_t uart_port = *(uint8_t*)arg;
    
    //ESP_LOGW(TAG, "line_timeout_callback triggered. Flushing UART%d input buffer.", uart_port);
    uart_flush_input(gps_port);
    uart_pattern_queue_reset(gps_port, 20);
}
 
// --- 2. Subscriber Callback ---
// This is the "Pool" logic. Every registered task executes this.
void on_uart_received(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    uart_data_t* data = (uart_data_t*) event_data;
    const char* task_id = (const char*) handler_args;
 
    // Logic for each subscriber goes here
    ESP_LOGI(TAG, "Subscriber [%s] processing: %s", task_id, data->str);
}
 
// --- 3. The UART Processing Task ---
static void uart_event_reader_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    assert(dtmp);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", gps_port);
            switch (event.type) {
            //Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(gps_port, dtmp, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]:");
                //uart_write_bytes(gps_port, (const char*) dtmp, event.size); // why??
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(gps_port);
                xQueueReset(uart_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(gps_port);
                xQueueReset(uart_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(gps_port, &buffered_size);
                int pos = uart_pattern_pop_pos(gps_port);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(gps_port);
                } else {
                    uart_read_bytes(gps_port, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(gps_port, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "read data: %s", dtmp);
                    //ESP_LOGI(TAG, "read pat : %s", pat);
                }
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}


/* proposed by Gemini, commentedout
static void uart_event_reader(void *pvParameters) {
    uart_event_t event;
    uint8_t uart_port = *(uint8_t*)pvParameters; // The UART port number is passed as a parameter

    //ESP_LOGI(TAG, "UART Event Reader started on UART%d", uart_port);
   
    for (;;) {
        // Wait for UART Hardware Events
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {

            switch (event.type) {
               
                case UART_PATTERN_DET:
                    // Full CRLF found! Stop the "Ignore" timer.
                    esp_timer_stop(timeout_timer);
                   
                    int pos;
                    while ((pos = uart_pattern_pop_pos(uart_port)) != -1) {
                        uart_data_t msg = {0};
                       
                        // Read exactly up to the \n (pos + 1)
                        int read_len = uart_read_bytes(uart_port, (uint8_t*)msg.str, pos + 1, 0);
                       
                        if (read_len >= 2) {
                            // Strip \r\n and null-terminate
                            msg.str[read_len - 2] = '\0';
                            msg.len = read_len - 2;
                           
                            // Broadcast to all 6 tasks (Event Loop copies the 32 bytes)
                            esp_event_post(UART_EVENTS, UART_EVENT_LINE_RECEIVED, &msg, sizeof(msg), pdMS_TO_TICKS(10));
                        } 

                        // NULL terminate and set length (including \r\n)
                        if (read_len > 0) {
                            msg.str[read_len] = '\0';
                            msg.len = read_len;
                            // Broadcast to all 6 tasks (Event Loop copies the 32 bytes)
                            esp_event_post(UART_EVENTS, UART_EVENT_LINE_RECEIVED, &msg, sizeof(msg), pdMS_TO_TICKS(10));
                        }

                    }
                    break;
 
                case UART_DATA:
                    // Potential start of a line. Start the watchdog timer.
                    if (!esp_timer_is_active(timeout_timer)) {
                        esp_timer_start_once(timeout_timer, LINE_TIMEOUT_MS * 1000);
                    }
                    break;
 
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                case UART_FRAME_ERR:
                case UART_PARITY_ERR:
                    // Hardware-level corruption or overflow: Ignore and Reset
                    //ESP_LOGE(TAG, "UART Error/Overflow. Resetting buffer.");
                    esp_timer_stop(timeout_timer);
                    uart_flush_input(uart_port);
                    uart_pattern_queue_reset(uart_port, 20);
                    break;
 
                default:
                    break;
            }
        }
    }
} // uart_event_reader
 */

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
    esp_event_loop_args_t loop_args = {
        .queue_size = EVENT_QUEUE_SIZE,
        .task_name = "gps_evt_loop_task",
        .task_priority = 15,
        .task_stack_size = 3072,
        .task_core_id = tskNO_AFFINITY
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &gps_event_loop));
 
    /* 
    // Not working for the moment
    // B. Setup Timeout Timer
    const esp_timer_create_args_t timer_args = {
        .callback = &line_timeout_callback,
        .name = "gps_timeout"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timeout_timer));
  */

    // C. Setup UART
    // uart_config is already populated with the values read from NVS. 
    // We just need to call the UART driver functions to apply the configuration and set up the UART.
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(gps_port, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 40, &uart_queue, 0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_param_config(gps_port, &uart_config)); 
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_pin(gps_port, pin_tx, pin_rx, pin_rts, pin_cts));
   
    // D. Enable Pattern Detect for '\n' (ASCII 10)
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_enable_pattern_det_baud_intr(gps_port, '\n', 1, 9, 0, 0)); // What are the correct parameters?
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_rx_timeout(gps_port, 100)); //
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_pattern_queue_reset(gps_port, 20));//
 
    // E. Add Subscribers to the Pool
    /* esp_event_handler_instance_register(UART_EVENTS, UART_EVENT_LINE_RECEIVED, &on_uart_received, "DisplayTask", NULL);
    esp_event_handler_instance_register(UART_EVENTS, UART_EVENT_LINE_RECEIVED, &on_uart_received, "LoggerTask", NULL);
    esp_event_handler_instance_register(UART_EVENTS, UART_EVENT_LINE_RECEIVED, &on_uart_received, "SD_CardTask", NULL);
    esp_event_handler_instance_register(UART_EVENTS, UART_EVENT_LINE_RECEIVED, &on_uart_received, "MQTTTask", NULL);
    esp_event_handler_instance_register(UART_EVENTS, UART_EVENT_LINE_RECEIVED, &on_uart_received, "LogicTask", NULL);
    esp_event_handler_instance_register(UART_EVENTS, UART_EVENT_LINE_RECEIVED, &on_uart_received, "DebugTask", NULL);
  */
    // F. Start the Processing Task. It reads events from the UART driver.
    xTaskCreate(uart_event_reader_task, "uart_event_reader", 4096, &gps_port, 12, NULL);
    
    //stream_stats = stream_stats_new("uart");

    //xTaskCreate(uart_reader, "uart_reader", 8192, &uart_port, TASK_PRIORITY_UART, NULL);

    return ESP_OK;
} // uart_init


/* void uart_inject(void *buf, size_t len)
{
    esp_event_post(UART_EVENT_READ, len, buf, len, portMAX_DELAY);
}

int uart_log(uint8_t uart_port, char *buf, size_t len)
{
    if (!uart_log_forward)
        return 0;
    return uart_write(uart_port, buf, len);
}

int uart_write(uint8_t uart_port, char *buf, size_t len)
{
    if (len == 0)
        return 0;

    int written = uart_write_bytes(uart_port, buf, len);
    if (written < 0)
        return written;

    stream_stats_increment(stream_stats, 0, len);

    esp_event_post(UART_EVENT_WRITE, len, buf, len, portMAX_DELAY);

    return written;
} */