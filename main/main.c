/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "mos_payload.h"
#include "stdio.h"
#include "stdlib.h"

#define RX_BUF_SIZE 1024
#define TX_DATA_MAX_SIZE 1024

Mos_Send_Payload_Object_t Send_Payload_Object = NULL;

#define UART0_TXD_PIN (GPIO_NUM_43)
#define UART0_RXD_PIN (GPIO_NUM_44)

#define UART1_TXD_PIN (GPIO_NUM_4)
#define UART1_RXD_PIN (GPIO_NUM_5)

#define UART2_TXD_PIN (GPIO_NUM_17)
#define UART2_RXD_PIN (GPIO_NUM_16)

void Mos_Send_Payload_Object_Init(Mos_Send_Payload_Object_t Send_Payload_Object)
{
    strcpy(Send_Payload_Object->method, "report");
    strcpy(Send_Payload_Object->ClientID , "Truck001");
    Send_Payload_Object->env_params.temperature = 27.5;
    Send_Payload_Object->env_params.humidity = 32;
    Send_Payload_Object->env_params.beidounum = 14;
    Send_Payload_Object->env_params.gpsnum = 15;
    Send_Payload_Object->env_params.lon = 130.16412;
    Send_Payload_Object->env_params.lat = 32.45423;
    Send_Payload_Object->env_params.speed = 35;
}

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // 开串口UART0
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART0_TXD_PIN, UART0_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 开串口UART1
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART1_TXD_PIN, UART1_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 开串口UART2
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART2_TXD_PIN, UART2_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int serialize_string_to_jsom(char *pstr, Mos_Send_Payload_Object_t Send_Payload_Object)
{
    //char str[TX_DATA_MAX_SIZE];
    sprintf(pstr,"{\"method\":\"%s\",\"clientID\":\"%s\",\"params\":{\"temperature\":%.1f, \
    \"humidity\":%d,\"beidounum\":%d,\"gpsnum\":%d,\"long\":%3.5f,\"lati\":%3.5f,\"speed\":%d}}",   \
    Send_Payload_Object->method, Send_Payload_Object->ClientID, Send_Payload_Object->env_params.temperature,    \
    Send_Payload_Object->env_params.humidity, Send_Payload_Object->env_params.beidounum,    \
    Send_Payload_Object->env_params.gpsnum, Send_Payload_Object->env_params.lon,    \
    Send_Payload_Object->env_params.lat, Send_Payload_Object->env_params.speed);
    return strlen(pstr);
}

/* 串口0发送数据 */
int uart0_sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

/* 串口1发送数据 */
int uart1_sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

/* 串口2发送数据 */
int uart2_sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

/* 串口0发送任务 */
static void uart0_tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "UART0_TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        uart0_sendData(TX_TASK_TAG, "Hello world abc");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

/* 串口1发送任务 */
static void uart1_tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "UART1_TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        uart1_sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

/* 串口2发送任务 */
static void uart2_tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "UART2_TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    static char json_string[TX_DATA_MAX_SIZE];
    Send_Payload_Object = (Mos_Send_Payload_Object_t)pvPortMalloc(sizeof(Mos_Send_Payload_Object));
    Mos_Send_Payload_Object_Init(Send_Payload_Object);

    while (1) {  
        Mos_Send_Payload_Object_Init(Send_Payload_Object);
        serialize_string_to_jsom(json_string, Send_Payload_Object);
        uart2_sendData(TX_TASK_TAG, json_string);
        vTaskDelay(8000 / portTICK_PERIOD_MS);
    }
}

/* 串口0接收任务 */
static void uart0_rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "UART0_RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* uart0_rx_data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, uart0_rx_data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            uart0_rx_data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, uart0_rx_data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(uart0_rx_data);
}

/* 串口1接收任务 */
static void uart1_rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "UART1_RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* uart1_rx_data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, uart1_rx_data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            uart1_rx_data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, uart1_rx_data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(uart1_rx_data);
}

/* 串口2接收任务 */
static void uart2_rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "UART2_RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* uart2_rx_data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, uart2_rx_data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            uart2_rx_data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, uart2_rx_data);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(uart2_rx_data);
}

void app_main(void)
{
    uart_init();
    xTaskCreate(uart0_rx_task, "uart0_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(uart0_tx_task, "uart0_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(uart1_rx_task, "uart1_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(uart1_tx_task, "uart1_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(uart2_rx_task, "uart2_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(uart2_tx_task, "uart2_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
