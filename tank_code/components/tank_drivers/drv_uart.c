#include "drv_uart.h"
#include "drv_motor.h"

static const char *TAG = "drv_urat";

void bsp_uart0_rx_task(void *pvParameters)
{
    int recv_size = 0;

    uart_event_t uart0_event;
    esp_log_level_set(TAG, ESP_LOG_INFO);
    for (;;)
    {
        if (xQueueReceive(bsp_uart0_handle.rx_event_queue, (void *)&uart0_event, (TickType_t)portMAX_DELAY * 5))
        {
            bzero(bsp_uart0_handle.buffer, sizeof(bsp_uart0_handle.buffer));  // 清空接收缓存
            switch (uart0_event.type)
            {
            case UART_DATA:  // UART接收数据的事件
                recv_size = uart_read_bytes(BSP_UART0, bsp_uart0_handle.buffer, uart0_event.size, portMAX_DELAY);
                bsp_uart0_handle.callback(bsp_uart0_handle.buffer, recv_size);
                break;
            case UART_FIFO_OVF:  // 检测到硬件 FIFO 溢出事件
                uart_flush_input(BSP_UART0);
                xQueueReset(bsp_uart0_handle.rx_event_queue);
                break;
            case UART_BUFFER_FULL:  // UART RX 缓冲器满事件
                uart_flush_input(BSP_UART0);
                xQueueReset(bsp_uart0_handle.rx_event_queue);
                break;
            case UART_BREAK:  // UART 空闲中断事件
                break;
            case UART_PARITY_ERR:  // UART奇偶校验错误事件
                break;
            case UART_FRAME_ERR:  // UART 帧错误事件
                break;
            case UART_PATTERN_DET:  // UART 模式检测事件
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

// 从字符串中提取数字
int extractNumber(const char *str)
{
    int num = 0;
    // ESP_LOGI(TAG, "Extracting number from: %s", str);
    sscanf(str, "%d", &num);
    if (num < 30 && num > 0)
    {
        num += 40;
    }
    if (num < 0 && num > -30)
    {
        num -= 40;
    }
    // ESP_LOGI(TAG, "Extracted number: %d", num);
    return num;
}

// 提取 x 和 y 的值并存入数组
int extractXY(const char *input, int x[], int y[])
{
    int         count = 0;
    const char *ptr   = input;
    while (*ptr)
    {
        if (*ptr == 'l')
        {
            ptr++;
            int currentX = extractNumber(ptr);
            if (currentX < -100 || currentX > 100)
            {
                continue;  // x 超出范围，跳过这组
            }
            while (*ptr)
            {
                if (*ptr == 'r')
                {
                    ptr++;
                    int currentY = extractNumber(ptr);
                    if (currentY < -100 || currentY > 100)
                    {
                        break;  // y 超出范围，跳过这组
                    }
                    x[count] = currentX;
                    y[count] = currentY;
                    count++;
                    break;
                }
                else
                {
                    ptr++;
                }
            }
        }
        else
        {
            ptr++;
        }
    }
    return count;
}

#define MAX_PAIRS 15  // llm最多几个命令

void drv_uart0_callback(uint8_t *buffer, size_t buffer_size)
{
    static int   uart_read_count    = 0;
    static bool  uart_read_web_flag = false;
    static float x_value            = 0.0f;
    static float y_value            = 0.0f;
    static int   x_llm[MAX_PAIRS];
    static int   y_llm[MAX_PAIRS];
    static int   llm_pair_count = 0;
    static int   llm_index      = 0;
    if (buffer_size > 0)
    {
        buffer[buffer_size] = '\0';
        char command        = 0;
        ESP_LOGI(TAG, "Recv DATA: %s", (char *)buffer);
        sscanf((const char *)buffer, "%c", &command);
        ESP_LOGI(TAG, "Recv command: %c", command);
        switch (command)
        {
        case 'l': {
            if (bsp_app_handle.app_run_mode != BSP_APP_MODE_MANUAL_CONTROL)
            {
                bsp_app_handle.app_run_mode = BSP_APP_MODE_UART_CONTROL;

                llm_pair_count = extractXY((const char *)(buffer + 1), x_llm, y_llm);
                if (llm_pair_count)
                {
                    llm_index       = 0;
                    uart_read_count = 0;
                    ESP_LOGI(TAG, "llm_pair_count: %d\n", llm_pair_count);
                }
            }
            break;
        }
        case 'x': {
            if (bsp_app_handle.app_run_mode != BSP_APP_MODE_MANUAL_CONTROL)
            {
                bsp_app_handle.app_run_mode = BSP_APP_MODE_UART_CONTROL;

                if (sscanf((const char *)buffer, "x%f y%f", &x_value, &y_value) == 2)
                {
                    uart_read_count    = 0;
                    uart_read_web_flag = true;
                    ESP_LOGI(TAG, "Parsed x: %.2f, y: %.2f\n", x_value, y_value);
                    drv_motor_action_control(x_value, y_value);
                }
            }
            break;
        }
        case 'w': {
            // 灯光模式
            int value = 0;
            if (sscanf((const char *)(buffer + 1), "%d", &value) == 1)
            {
            }
            break;
        }
        case 'c': {
            // 红外循迹
            if (bsp_app_handle.app_run_mode != BSP_APP_MODE_MANUAL_CONTROL)
            {
                bsp_app_handle.app_run_mode = BSP_APP_MODE_AUTO_TRACKING;
            }
            break;
        }
        case 'd': {
            // 跳舞
            break;
        }
        default: {
            ESP_LOGW(TAG, "Unknown command: %c", command);
            break;
        }
        }
        if (uart_read_web_flag == true)
        {
            uart_read_count++;  // 累计没有读取到数据的次数，直到25*20ms后（0.5s），自动停止
            if (uart_read_count > 25)
            {
                uart_read_count    = 0;
                uart_read_web_flag = false;
                drv_motor_action_control(0, 0);
            }
        }
        else if (llm_index < llm_pair_count)
        {
            uart_read_count++;  // 累计没有读取到数据的次数，直到40*20ms后（0.8s），自动停止
            if (uart_read_count > 40)
            {
                ESP_LOGI(TAG, "llm index: %d", llm_index);
                uart_read_count = 0;
                ESP_LOGI(TAG, "X = %d, Y = %d", x_llm[llm_index], y_llm[llm_index]);
                drv_motor_set_speed(x_llm[llm_index], y_llm[llm_index]);
                llm_index++;
            }
            if (llm_index == llm_pair_count)  // llm对话命令执行结束
            {
                uart_read_web_flag = true;
            }
        }
    }
}

void drv_uart0_task_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    bsp_uart_set_callback(BSP_UART0, drv_uart0_callback);  // 设置串口0接收回调
    xTaskCreate(bsp_uart0_rx_task, "uart0_rx_task", UART0_TASK_STACK_SIZE, NULL, 10, NULL);
    ESP_LOGI(TAG, "drv_uart0_task init success!");
}
