#ifndef _DRV_UART_H_
#define _DRV_UART_H_
#include "bsp.h"
#include "bsp_config.h"

#define UART0_TASK_STACK_SIZE 2048
void drv_uart0_task_init(void);

#endif