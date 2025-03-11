#include "drv_ir_senser.h"
#include "drv_adc.h"
#include "drv_motor.h"

void drv_ir_senser_line_tracking(void)
{
    if (drv_adc_handle.ir_0_adc_value < 3000 && drv_adc_handle.ir_1_adc_value < 3000)
    {
        drv_motor_set_speed(DRV_MOTOR_SPEED_DEFAULT, DRV_MOTOR_SPEED_DEFAULT);
    }
    if (drv_adc_handle.ir_0_adc_value > 3000)
    {
        drv_motor_set_speed(-DRV_MOTOR_SPEED_DEFAULT, DRV_MOTOR_SPEED_DEFAULT);
    }
    if (drv_adc_handle.ir_1_adc_value > 3000)
    {
        drv_motor_set_speed(DRV_MOTOR_SPEED_DEFAULT, -DRV_MOTOR_SPEED_DEFAULT);
    }
    if (drv_adc_handle.ir_0_adc_value > 3000 && drv_adc_handle.ir_1_adc_value > 3000)
    {
        drv_motor_set_speed(DRV_MOTOR_SPEED_DEFAULT, DRV_MOTOR_SPEED_DEFAULT);
    }
    if (bsp_gpio_read(BSP_GPIO_VBAT_MONITOR_PIN) == BSP_GPIO_VBAT_MONITOR_CHARGING_LEVEL)  // 判断是否充电，如果gpio为低则充电
    {
        bsp_app_handle.app_run_mode = bsp_app_handle.app_run_mode_last;
        drv_motor_set_speed(0, 0);
    }
}
