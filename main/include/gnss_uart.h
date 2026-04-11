#pragma once

//esp_err_t gnss_uart_init(); - initialzation happens in the reader task.

void task_gnss_reader(void *pvParams);

