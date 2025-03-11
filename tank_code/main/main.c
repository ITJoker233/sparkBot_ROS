#include "main.h"
#include "bsp.h"
#include "bsp_config.h"
#ifdef BSP_UART
#include "drv_uart.h"
#endif
#include "drv_adc.h"
#include "drv_motor.h"
#include "drv_ir_senser.h"
#include "drv_power_manager.h"
#include "iot_button.h"

static const char *TAG = "app";

static void button_single_click_cb(void *arg, void *data)
{
    if (bsp_app_handle.app_run_target_mode == BSP_APP_MODE_AUTO_TRACKING)
    {
        drv_motor_action_control(0, 0);
        bsp_app_handle.app_run_target_mode = BSP_APP_MODE_UART_CONTROL;
    }
    // 按键单击
    ESP_LOGI(TAG, "single click:");
}

static void button_double_click_cb(void *arg, void *data)
{
    if (bsp_app_handle.app_run_target_mode == BSP_APP_MODE_AUTO_TRACKING)
    {
        drv_motor_action_control(0, 0);
        bsp_app_handle.app_run_target_mode = BSP_APP_MODE_UART_CONTROL;
    }
    // 按键双击
    ESP_LOGI(TAG, "double click:");
}
static void button_long_press_start_cb(void *arg, void *data)
{
    if (bsp_app_handle.app_run_target_mode == BSP_APP_MODE_AUTO_TRACKING)
    {
        drv_motor_action_control(0, 0);
        bsp_app_handle.app_run_target_mode = BSP_APP_MODE_UART_CONTROL;
    }
    // 按键长按开始
    ESP_LOGI(TAG, "long press start:");
}
static void button_long_press_up_cb(void *arg, void *data)
{
    // 按键长按抬起
    ESP_LOGI(TAG, "long press up:");
}

void app_button_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    button_config_t btn_config = {
        .type               = BUTTON_TYPE_GPIO,
        .long_press_time    = 1200,
        .short_press_time   = 400,
        .gpio_button_config = {
            .gpio_num     = BSP_GPIO_KEY_PIN,
            .active_level = 0,
        }};

    button_handle_t btn = iot_button_create(&btn_config);
    // iot_button_register_cb(btn, BUTTON_PRESS_DOWN, button_press_cb, NULL);
    // iot_button_register_cb(btn, BUTTON_PRESS_UP, button_press_up_cb, NULL);
    iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, button_double_click_cb, NULL);
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, button_long_press_start_cb, NULL);
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, button_long_press_up_cb, NULL);
    ESP_LOGI(TAG, "drv_button_task init success!");
}

void app_task(void *pvParameters)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    for (;;)
    {
        drv_adc_run();
        drv_power_manager_run();

        if (bsp_app_handle.app_run_target_mode != bsp_app_handle.app_run_mode)
        {
            bsp_app_handle.app_run_mode_last = bsp_app_handle.app_run_mode;
            bsp_app_handle.app_run_mode      = bsp_app_handle.app_run_target_mode;
        }
        switch (bsp_app_handle.app_run_mode)
        {
        case BSP_APP_MODE_UART_CONTROL: {
            break;
        }
        case BSP_APP_MODE_MANUAL_CONTROL: {
            break;
        }
        case BSP_APP_MODE_AUTO_TRACKING: {
            drv_ir_senser_line_tracking();  // 红外线性循迹
            break;
        }
        case BSP_APP_MODE_SLEEP: {
            drv_power_manager_handle.set_sleep_flag = 0x01;
            bsp_app_handle.ws2812_target_mode       = 0;
            break;
        }
        default:
            bsp_app_handle.app_run_target_mode = BSP_APP_MODE_UART_CONTROL;
            break;
        }

        if (bsp_app_handle.ws2812_target_mode != bsp_app_handle.ws2812_mode)
        {
            bsp_app_handle.ws2812_mode_last = bsp_app_handle.ws2812_mode;
            bsp_app_handle.ws2812_mode      = bsp_app_handle.ws2812_target_mode;
        }
        bsp_ws2812_mode_set(bsp_app_handle.ws2812_mode);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Tank Init...");
    bsp_board_init();  // 板载初始化
    memset(&bsp_app_handle, 0, sizeof(bsp_app_handle));
    bsp_app_handle.app_run_target_mode = BSP_APP_MODE_UART_CONTROL;
    bsp_app_handle.ws2812_target_mode  = 0;

    ESP_LOGI(TAG, "Tank Running...");

    drv_power_manager_init();  // 电源管理
    app_button_init();         // 按键初始化

    drv_motor_task_init();  // 电机任务初始化
#ifdef BSP_UART
    drv_uart0_task_init();  // uart0任务初始化
#endif

    xTaskCreate(app_task, "app_task", UART0_TASK_STACK_SIZE, NULL, 10, NULL);
}
