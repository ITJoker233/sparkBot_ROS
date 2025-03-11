#include "drv_uart.h"

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

void drv_uart0_callback(uint8_t *buffer, size_t buffer_size)
{
    if (buffer_size > 0)
    {
    }
}

void drv_uart0_task_init(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    bsp_uart_set_callback(BSP_UART0, drv_uart0_callback);  // 设置串口0接收回调
    xTaskCreate(bsp_uart0_rx_task, "uart0_rx_task", UART0_TASK_STACK_SIZE, NULL, 10, NULL);
    ESP_LOGI(TAG, "drv_uart0_task init success!");
}
