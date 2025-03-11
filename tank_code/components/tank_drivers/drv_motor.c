#include "drv_motor.h"
static const char *TAG = "drv_motor";

drv_motor_handle_t drv_motor_handle;

void drv_motor_set_speed(int left_speed, int right_speed)
{
    drv_motor_handle.target_speed_left  = left_speed;
    drv_motor_handle.target_speed_right = right_speed;
}

void drv_motor_action_control(float x, float y)
{
    float base_speed  = y * DRV_MOTOR_SPEED_MAX;
    float turn_adjust = x * DRV_MOTOR_SPEED_MAX;

    int left_speed  = (int)(base_speed + turn_adjust);
    int right_speed = (int)(base_speed - turn_adjust);

    if (left_speed > DRV_MOTOR_SPEED_MAX)
    {
        left_speed = DRV_MOTOR_SPEED_MAX;
    }
    if (right_speed > DRV_MOTOR_SPEED_MAX)
    {
        right_speed = DRV_MOTOR_SPEED_MAX;
    }
    drv_motor_set_speed(left_speed, right_speed);
}

static void drv_motor_set_left_speed(int speed)
{
    if (speed >= 0)
    {
        uint32_t m1a_duty = (uint32_t)((speed * drv_motor_handle.m1_coefficient * 8192) / 100);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M1_CHANNEL_A, m1a_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M1_CHANNEL_A));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M1_CHANNEL_B, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M1_CHANNEL_B));
    }
    else
    {
        uint32_t m1b_duty = (uint32_t)((-speed * drv_motor_handle.m1_coefficient * 8192) / 100);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M1_CHANNEL_A, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M1_CHANNEL_A));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M1_CHANNEL_B, m1b_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M1_CHANNEL_B));
    }
}

static void drv_motor_set_right_speed(int speed)
{
    if (speed >= 0)
    {
        uint32_t m2a_duty = (uint32_t)((speed * drv_motor_handle.m2_coefficient * 8192) / 100);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M2_CHANNEL_A, m2a_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M2_CHANNEL_A));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M2_CHANNEL_B, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M2_CHANNEL_B));
    }
    else
    {
        uint32_t m2b_duty = (uint32_t)((-speed * drv_motor_handle.m2_coefficient * 8192) / 100);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M2_CHANNEL_A, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M2_CHANNEL_A));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_M2_CHANNEL_B, m2b_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_M2_CHANNEL_B));
    }
}

void drv_motor_task(void *pvParameters)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    for (;;)
    {
        if (drv_motor_handle.target_speed_left != drv_motor_handle.current_speed_x)
        {
            int step_x = (drv_motor_handle.target_speed_left - drv_motor_handle.current_speed_x) / 10;  // 10次调整，每次20ms，共200ms
            if (step_x == 0)                                                                            // 每次最少调整1，防止调整数为0永远等于不了期望速度
            {
                if (drv_motor_handle.target_speed_left > drv_motor_handle.current_speed_x)
                    step_x = 1;
                else
                    step_x = -1;
            }
            drv_motor_handle.current_speed_x += step_x;
            drv_motor_set_left_speed(drv_motor_handle.current_speed_x);
        }

        if (drv_motor_handle.target_speed_right != drv_motor_handle.current_speed_y)
        {
            int step_y = (drv_motor_handle.target_speed_right - drv_motor_handle.current_speed_y) / 10;  // 10次调整，每次20ms，共200ms
            if (step_y == 0)
            {
                if (drv_motor_handle.target_speed_right > drv_motor_handle.current_speed_y)
                    step_y = 1;
                else
                    step_y = -1;
            }
            drv_motor_handle.current_speed_y += step_y;
            drv_motor_set_right_speed(drv_motor_handle.current_speed_y);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void drv_motor_task_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    memset(&drv_motor_handle, 0, sizeof(drv_motor_handle));

    drv_motor_handle.m1_coefficient = 1.0f;
    drv_motor_handle.m2_coefficient = 1.0f;

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg         = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Array of channel configurations for easy iteration
    const uint8_t motor_ledc_channel[LEDC_CHANNEL_COUNT] = {LEDC_M1_CHANNEL_A, LEDC_M1_CHANNEL_B, LEDC_M2_CHANNEL_A, LEDC_M2_CHANNEL_B};
    const int32_t ledc_channel_pins[LEDC_CHANNEL_COUNT]  = {LEDC_M1_CHANNEL_A_IO, LEDC_M1_CHANNEL_B_IO, LEDC_M2_CHANNEL_A_IO, LEDC_M2_CHANNEL_B_IO};
    for (int i = 0; i < LEDC_CHANNEL_COUNT; i++)
    {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_MODE,
            .channel    = motor_ledc_channel[i],  // Assuming channel values increment for each new channel
            .timer_sel  = LEDC_TIMER,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = ledc_channel_pins[i],
            .duty       = 0,  // Set duty to 0%
            .hpoint     = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    xTaskCreate(drv_motor_task, "drv_motor_task", DRV_MOTOR_TASK_STACK_SIZE, NULL, 10, NULL);
    ESP_LOGI(TAG, "drv_motor_task init success!");
}
