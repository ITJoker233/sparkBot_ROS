#ifndef _BSP_H_
#define _BSP_H_
#include "bsp_config.h"

#ifdef BSP_GPIO

void bsp_gpio_init(gpio_num_t gpio_num, gpio_mode_t gpio_mode, gpio_int_type_t gpio_intr_type, uint8_t gpio_pull_mode);
int  bsp_gpio_read(gpio_num_t gpio_num);

esp_err_t bsp_gpio_write(gpio_num_t gpio_num, uint32_t level);
#endif

#ifdef BSP_WS2812

typedef enum
{
    BSP_WS2812_LED_NONE = 0,
    BSP_WS2812_LED1,
    BSP_WS2812_LED2,
    BSP_WS2812_LED3,
    BSP_WS2812_LED4,
    BSP_WS2812_LED5,
    BSP_WS2812_LED6
} bsp_ws2812_led_e;

typedef enum
{
    BSP_WS2812_LED_OFF = 0,
    BSP_WS2812_LED_ON,
} bsp_ws2812_led_status_e;

typedef enum
{
    BSP_WS2812_MODE_SLEEP = 0,        // 休眠
    BSP_WS2812_MODE_CHARGING_BREATH,  // 充电中
    BSP_WS2812_MODE_CHARGING_FULL,    // 充电已充满
    BSP_WS2812_MODE_LOW_POWER,        // 低电量
    BSP_WS2812_MODE_MAX,
    BSP_WS2812_MODE_MANUAL,  // 手动控制
} bsp_ws2812_mode_e;

void bsp_ws2812_init(void);
void bsp_ws2812_mode_set(bsp_ws2812_mode_e mode);
void bsp_ws2812_led_set(bsp_ws2812_led_e led, bsp_ws2812_led_status_e status);

#endif

#ifdef BSP_ADC

void bsp_adc_init(adc_channel_t adc_chx, adc_atten_t adc_atten);

int bsp_adc_get_value(adc_channel_t adc_chx);

#endif

#ifdef BSP_UART

typedef struct bsp_uart_handle_t
{
    QueueHandle_t rx_event_queue;
    uint8_t       buffer[BSP_UART_BUFF_SIZE];
    void (*callback)(uint8_t *buffer, size_t buffer_size);
} bsp_uart_handle_t;

void bsp_uart_set_callback(int uart_num, void *callback);
void bsp_uart_init(int uart_num, int baudrate);

extern bsp_uart_handle_t bsp_uart0_handle;
#endif

typedef enum
{
    BSP_APP_MODE_UART_CONTROL,    // 串口控制
    BSP_APP_MODE_MANUAL_CONTROL,  // 手动控制
    BSP_APP_MODE_AUTO_TRACKING,   // 自动循迹
    BSP_APP_MODE_SLEEP            // 休眠
} bsp_app_run_mode_e;

typedef struct bsp_app_handle
{
    uint8_t app_run_mode;
    uint8_t app_run_target_mode;
    uint8_t app_run_mode_last;

    bsp_ws2812_mode_e ws2812_mode;
    bsp_ws2812_mode_e ws2812_target_mode;
    bsp_ws2812_mode_e ws2812_mode_last;
} bsp_app_handle_t;

extern bsp_app_handle_t bsp_app_handle;

void bsp_board_init(void);
void bsp_board_deinit(void);

#endif
