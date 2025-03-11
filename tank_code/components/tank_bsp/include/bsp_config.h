#ifndef _BSP_CONFIG_H_
#define _BSP_CONFIG_H_

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BSP_GPIO
#define BSP_WS2812
#define BSP_ADC
#define BSP_UART

// GPIO
#define BSP_GPIO_VBAT_MONITOR_PIN            (GPIO_NUM_9)
#define BSP_GPIO_VBAT_MONITOR_PIN_MODE       GPIO_MODE_INPUT
#define BSP_GPIO_VBAT_MONITOR_CHARGING_LEVEL 0

#define BSP_GPIO_KEY_PIN                     GPIO_NUM_8
#define BSP_GPIO_KEY_PIN_MODE                GPIO_MODE_INPUT

#define BSP_GPIO_POWER_PIN                   GPIO_NUM_2
#define BSP_GPIO_POWER_PIN_MODE              GPIO_MODE_OUTPUT

// WS2812
#define BSP_WS2812_GPIO_PIN                  (10)
#define BSP_WS2812_STRIPS_NUM                6

// ADC
#define BSP_GPIO_VBAT_ADC_CHAN               ADC_CHANNEL_3  // ADC - VBAT
#define BSP_GPIO_VBAT_ADC_ATTEN              ADC_ATTEN_DB_12

#define BSP_GPIO_IR_0_ADC_CHAN               ADC_CHANNEL_0  // 红外传感器0
#define BSP_GPIO_IR_0_ADC_ATTEN              ADC_ATTEN_DB_12
#define BSP_GPIO_IR_1_ADC_CHAN               ADC_CHANNEL_1  // 红外传感器1
#define BSP_GPIO_IR_1_ADC_ATTEN              ADC_ATTEN_DB_12

// UART
#define BSP_UART_BUFF_SIZE                   1024

#define PATTERN_CHR_NUM                      (3)  // 设置接收者接收的连续且相同的字符数，定义了收件箱模式

#define BSP_UART0_TXD_PIN                    (GPIO_NUM_21)
#define BSP_UART0_RXD_PIN                    (GPIO_NUM_20)
#define BSP_UART0_RTS_PIN                    (-1)
#define BSP_UART0_CTS_PIN                    (-1)

#define BSP_UART0                            (0)
#define BSP_UART0_BAUD_RATE                  (115200)
#define BSP_UART0_QUEUE_SIZE                 20

#endif