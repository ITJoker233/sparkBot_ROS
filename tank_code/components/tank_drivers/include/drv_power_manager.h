#ifndef _DRV_POWER_MANAGER_H_
#define _DRV_POWER_MANAGER_H_
#include "bsp.h"
#include "bsp_config.h"
typedef struct drv_power_manager_handle_t
{
    float    battery_voltage;
    uint32_t idle_timer_cnt;
    int      battery_percent;
    int      battery_percent_last;

    uint8_t low_power_flag;  // 低电量标志
    uint8_t set_sleep_flag;  // 进入休眠标志
} drv_power_manager_handle_t;

#define DRV_POWER_MANAGER_LOW_POWER_PERCENT 20

extern drv_power_manager_handle_t drv_power_manager_handle;

void drv_power_manager_run(void);
void drv_power_manager_init(void);
#endif