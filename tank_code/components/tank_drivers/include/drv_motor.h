#ifndef _DRV_MOTOR_H_
#define _DRV_MOTOR_H_
#include "bsp.h"
#include "bsp_config.h"
#include "driver/ledc.h"

#define DRV_MOTOR_TASK_STACK_SIZE 2048

#define DRV_MOTOR_SPEED_MAX       100
#define DRV_MOTOR_SPEED_80        80
#define DRV_MOTOR_SPEED_60        60
#define DRV_MOTOR_SPEED_MIN       0
#define DRV_MOTOR_SPEED_DEFAULT   DRV_MOTOR_SPEED_60

#define LEDC_TIMER                LEDC_TIMER_0
#define LEDC_MODE                 LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES             LEDC_TIMER_13_BIT  // Set duty resolution to 13 bits
#define LEDC_FREQUENCY            (4000)             // Frequency in Hertz. Set frequency at 4 kHz

#define LEDC_CHANNEL_COUNT        (4)
#define LEDC_M1_CHANNEL_A         LEDC_CHANNEL_0
#define LEDC_M1_CHANNEL_B         LEDC_CHANNEL_1
#define LEDC_M2_CHANNEL_A         LEDC_CHANNEL_2
#define LEDC_M2_CHANNEL_B         LEDC_CHANNEL_3

#define LEDC_M1_CHANNEL_A_IO      (4)
#define LEDC_M1_CHANNEL_B_IO      (5)
#define LEDC_M2_CHANNEL_A_IO      (7)
#define LEDC_M2_CHANNEL_B_IO      (6)

typedef struct drv_motor_handle_t
{
    float m1_coefficient;
    float m2_coefficient;

    int current_speed_x;
    int current_speed_y;
    int target_speed_left;
    int target_speed_right;
} drv_motor_handle_t;

extern drv_motor_handle_t drv_motor_handle;

void drv_motor_set_speed(int left_speed, int right_speed);
void drv_motor_action_control(float x, float y);
void drv_motor_task_init(void);

#endif