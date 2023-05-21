#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "mos_payload.h"
#include "nmea0183_parse.h"
#include "display_menu.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "lvgl/demos/lv_demos.h"
#include "lvgl_helpers.h"
#include "lvgl/src/hal/lv_hal_tick.h"

/*-------------------- DEFINE START --------------------*/

#define RX_BUF_SIZE 1024
#define TX_DATA_MAX_SIZE 1024

Mos_Send_Payload_Object_t Send_Payload_Object = NULL;

/* GPS_BD port */
#define UART0_TXD_PIN (GPIO_NUM_43)
#define UART0_RXD_PIN (GPIO_NUM_44)

/* 4G DTU port */
#define UART1_TXD_PIN (GPIO_NUM_4)
#define UART1_RXD_PIN (GPIO_NUM_5)

/* Zigbee port */
#define UART2_TXD_PIN (GPIO_NUM_17)
// #define UART2_RXD_PIN (GPIO_NUM_16)
#define UART2_RXD_PIN (GPIO_NUM_18)

/*--------------------- DEFINE END ---------------------*/

/*-------------------- GLOABLE VERIABLE START --------------------*/

static GNRMC GNRMC_Info;

int temperature = -999;
int humidity = -999;

char show_str[64];

extern lv_obj_t *cont_temperature;  // temperature cont容器
extern lv_obj_t *label_temperature; // temperature lable标签

extern lv_obj_t *cont_humidity;  // humidity cont容器
extern lv_obj_t *label_humidity; // humidity lable标签

extern lv_obj_t *cont_position;  // latitude cont容器
extern lv_obj_t *label_position; // latitude lable标签

extern lv_obj_t *cont_position_status;  // longitude cont容器
extern lv_obj_t *label_position_status; // longitude lable标签

/*--------------------- GLOABLE VERIABLE END ---------------------*/

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

/*  UART1、UART2 和 UART3 串口初始化*/
void uart_init(void) {
    const uart_config_t uart0_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // 开串口UART0
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart0_config);
    uart_set_pin(UART_NUM_0, UART0_TXD_PIN, UART0_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    const uart_config_t uart1_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // 开串口UART1
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart1_config);
    uart_set_pin(UART_NUM_1, UART1_TXD_PIN, UART1_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    const uart_config_t uart2_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // 开串口UART2
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart2_config);
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

    // static char json_string[TX_DATA_MAX_SIZE];
    // Send_Payload_Object = (Mos_Send_Payload_Object_t)pvPortMalloc(sizeof(Mos_Send_Payload_Object));
    // Mos_Send_Payload_Object_Init(Send_Payload_Object);

    while (1) {
        // Mos_Send_Payload_Object_Init(Send_Payload_Object);
        // serialize_string_to_jsom(json_string, Send_Payload_Object);
        uart1_sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

/* 串口2发送任务 */
static void uart2_tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "UART2_TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);    
    while (1) {  
        //uart2_sendData(TX_TASK_TAG, "Hello world");
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
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, uart0_rx_data);
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes", rxBytes);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
        NMEA0183_GNRMC_Analusis(&GNRMC_Info, (char *)uart0_rx_data);
        memset(uart0_rx_data, '\0', rxBytes);

        if(fabs(GNRMC_Info.Lati)<1e-6 && fabs(GNRMC_Info.Longi)<1e-6)
        {
            memset(show_str, '\0', sizeof(show_str));
            sprintf(show_str, "Positioning status : %s", "invalid");
            lv_label_set_text(label_position_status, show_str);

            memset(show_str, '\0', sizeof(show_str));
            sprintf(show_str, "Pos: %3.5f, %3.5f", -888.88888, -888.88888);
            lv_label_set_text(label_position, show_str);

        }
        else
        {
            memset(show_str, '\0', sizeof(show_str));
            sprintf(show_str, "Positioning status : %s", "valid");
            lv_label_set_text(label_position_status, show_str);

            memset(show_str, '\0', sizeof(show_str));
            sprintf(show_str, "Pos: %3.5f, %3.5f", GNRMC_Info.Longi, GNRMC_Info.Lati);
            lv_label_set_text(label_position, show_str);
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
        if(rxBytes >= 11)
        {
            temperature = (uart2_rx_data[4] - '0')* 10 + (uart2_rx_data[5] - '0');
            humidity = (uart2_rx_data[8] - '0')* 10 + (uart2_rx_data[9] - '0');

            if(temperature != 0 || humidity != 0)
            {
                memset(show_str, '\0', sizeof(show_str));
                sprintf(show_str, "Temperature : %2d °C", temperature);
                lv_label_set_text(label_temperature, show_str);

                memset(show_str, '\0', sizeof(show_str));
                sprintf(show_str, "Huminity : %2d %%", humidity);
                lv_label_set_text(label_humidity, show_str);
            }
        }
    }
    free(uart2_rx_data);
}

/* LVGL 时钟任务 */
static void lv_tick_task(void *arg)
{
   (void)arg;
   lv_tick_inc(10);
}

SemaphoreHandle_t xGuiSemaphore;// 创建一个GUI信号量

static void guiTask(void *pvParameter) {

     (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();// 创建GUI信号量

    lv_init();          //lvgl初始化

    lvgl_driver_init(); // 初始化液晶驱动
    
    // 初始化缓存
    // lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * 2, MALLOC_CAP_DMA);
    // lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * 2, MALLOC_CAP_DMA);
    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);

    // 添加并注册触摸驱动
    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LV_HOR_RES_MAX * LV_VER_RES_MAX); /*Initialize the display buffer*/

    static lv_disp_drv_t disp_drv;         /*A variable to hold the drivers. Must be static or global.*/
    lv_disp_drv_init(&disp_drv);           /*Basic initialization*/
    disp_drv.draw_buf = &draw_buf;         /*Set an initialized buffer*/
    disp_drv.flush_cb = disp_driver_flush; /*Set a flush callback to draw to the display*/
    disp_drv.hor_res = 320;                /*Set the horizontal resolution in pixels*/
    disp_drv.ver_res = 240;                /*Set the vertical resolution in pixels*/
    lv_disp_drv_register(&disp_drv);       /*Register the driver and save the created display objects*/
    
    /*触摸屏输入接口配置*/
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);

    /* 创建一个10ms定时器*/// 定期处理GUI回调
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10 * 1000));

    menu_init();

   while (1)
   {
      /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
      vTaskDelay(pdMS_TO_TICKS(10));

      /* Try to take the semaphore, call lvgl related function on success */
      if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
      {
         lv_task_handler();// 处理LVGL任务
         xSemaphoreGive(xGuiSemaphore);// 释放信号量
      }
   }

    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
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

    xTaskCreatePinnedToCore(guiTask, "gui", 1024 * 4, NULL, 1, NULL, 0);
}
