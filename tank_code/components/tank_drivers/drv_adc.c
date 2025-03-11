#include "drv_adc.h"

drv_adc_handle_t drv_adc_handle;

void drv_adc_run(void)
{
    drv_adc_handle.battery_adc_value = bsp_adc_get_value(BSP_GPIO_VBAT_ADC_CHAN);
    drv_adc_handle.ir_0_adc_value    = bsp_adc_get_value(BSP_GPIO_IR_0_ADC_CHAN);
    drv_adc_handle.ir_1_adc_value    = bsp_adc_get_value(BSP_GPIO_IR_1_ADC_CHAN);
}
