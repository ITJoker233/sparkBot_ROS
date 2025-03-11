#include "drv_power_manager.h"
#include "drv_adc.h"

static const char *TAG = "drv_power_manager";

drv_power_manager_handle_t drv_power_manager_handle;

// 检测是否低电量
bool drv_power_manager_check_low_power(void)
{
    if (drv_power_manager_handle.battery_percent <= DRV_POWER_MANAGER_LOW_POWER_PERCENT && drv_power_manager_handle.battery_percent >= 0)
    {
        return true;
    }
    return false;
}
// 检测是否空闲
bool drv_power_manager_check_idle(void)
{
    // 不在充电并且头部没有放上来
    if ((bsp_gpio_read(BSP_GPIO_VBAT_MONITOR_PIN) != BSP_GPIO_VBAT_MONITOR_CHARGING_LEVEL &&
         (bsp_gpio_read(BSP_UART0_RXD_PIN) == 0) && (bsp_app_handle.app_run_mode == BSP_APP_MODE_UART_CONTROL)))
    {
        return true;
    }
    return false;
}

void drv_power_manager_run(void)
{

    drv_power_manager_handle.battery_voltage = drv_adc_handle.battery_adc_value;
    drv_power_manager_handle.battery_percent = (int)((drv_power_manager_handle.battery_voltage - 3300) / (4200 - 3300) * 100.0) % 100;

    if (drv_power_manager_handle.battery_percent_last != drv_power_manager_handle.battery_percent)
    {
        drv_power_manager_handle.battery_percent_last = drv_power_manager_handle.battery_percent;
    }
    // 灯光效果，根据充电状态和电量判断
    if (bsp_gpio_read(BSP_GPIO_VBAT_MONITOR_PIN) == BSP_GPIO_VBAT_MONITOR_CHARGING_LEVEL)
    {
        if (drv_power_manager_handle.battery_percent < 100)
        {
            bsp_app_handle.ws2812_target_mode = BSP_WS2812_MODE_CHARGING_BREATH;
            bsp_app_handle.ws2812_mode        = bsp_app_handle.ws2812_target_mode;
        }
        else
        {
            // 电池充满
            bsp_app_handle.ws2812_target_mode = BSP_WS2812_MODE_CHARGING_FULL;
            bsp_app_handle.ws2812_mode        = bsp_app_handle.ws2812_target_mode;
        }
    }
    else
    {
        if (drv_power_manager_check_low_power() == true)  // 电池低电量
        {
            drv_power_manager_handle.low_power_flag = 0x01;
            bsp_app_handle.ws2812_target_mode       = BSP_WS2812_MODE_LOW_POWER;
            bsp_app_handle.ws2812_mode              = bsp_app_handle.ws2812_target_mode;
        }
        else
        {
            drv_power_manager_handle.low_power_flag = 0x00;
            bsp_app_handle.ws2812_target_mode       = bsp_app_handle.ws2812_mode_last;
        }

        //  电池低电量时 或 空闲状态
        if ((drv_power_manager_handle.low_power_flag > 0) || drv_power_manager_check_idle())
        {
            drv_power_manager_handle.idle_timer_cnt++;
        }
        else
        {
            drv_power_manager_handle.idle_timer_cnt = 0;
        }

        if (drv_power_manager_handle.idle_timer_cnt >= 500 || drv_power_manager_handle.set_sleep_flag != 0x00)  // 待机超过5s
        {
            drv_power_manager_handle.idle_timer_cnt = 0;
            bsp_board_deinit();
        }
    }
}

void drv_power_manager_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    memset(&drv_power_manager_handle, 0, sizeof(drv_power_manager_handle));
    drv_power_manager_handle.battery_percent_last = 75;
    ESP_LOGI(TAG, "drv_power_manager init success!");
}