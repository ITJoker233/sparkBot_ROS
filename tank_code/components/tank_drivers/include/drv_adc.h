#ifndef _DRV_ADC_H_
#define _DRV_ADC_H_
#include "bsp.h"
#include "bsp_config.h"

typedef struct drv_adc_handle_t
{
    int ir_0_adc_value;
    int ir_1_adc_value;
    int battery_adc_value;
} drv_adc_handle_t;

extern drv_adc_handle_t drv_adc_handle;

void drv_adc_run(void);

#endif