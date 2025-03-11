#include "include/bsp_config.h"
#include "include/bsp.h"
#include "led_indicator.h"
#include "led_strip.h"

static const char *TAG = "BSP";

bsp_app_handle_t bsp_app_handle;

#ifdef BSP_GPIO

void bsp_gpio_init(gpio_num_t gpio_num, gpio_mode_t gpio_mode, gpio_int_type_t gpio_intr_type, uint8_t gpio_pull_mode)
{
    gpio_pullup_t   pullup;
    gpio_pulldown_t pulldown;

    switch (gpio_pull_mode)
    {
    case 0:  // 无
        pullup   = GPIO_PULLUP_DISABLE;
        pulldown = GPIO_PULLDOWN_DISABLE;
        break;
    case 1:  // 上拉
        pullup   = GPIO_PULLUP_ENABLE;
        pulldown = GPIO_PULLDOWN_DISABLE;
        break;
    case 2:  // 下拉
        pullup   = GPIO_PULLUP_DISABLE;
        pulldown = GPIO_PULLDOWN_ENABLE;
        break;
    default:
        pullup   = GPIO_PULLUP_DISABLE;
        pulldown = GPIO_PULLDOWN_DISABLE;
        break;
    }
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode         = gpio_mode,
        .pull_up_en   = pullup,
        .pull_down_en = pulldown,
        .intr_type    = gpio_intr_type};
    gpio_config(&gpio_cfg);
}

int bsp_gpio_read(gpio_num_t gpio_num)
{
    return gpio_get_level(gpio_num);
}

esp_err_t bsp_gpio_write(gpio_num_t gpio_num, uint32_t level)
{
    return gpio_set_level(gpio_num, level);
}
#endif

#ifdef BSP_WS2812

led_indicator_handle_t bsp_led_handle = NULL;

static const blink_step_t bsp_ws2812_mode_charging_blink[] = {
    {LED_BLINK_HSV, SET_IHSV(MAX_INDEX, 0, 0, 0), 0},
    {LED_BLINK_HSV, SET_IHSV(4, 100, 255, 0), 0},
    {LED_BLINK_BREATHE, INSERT_INDEX(4, LED_STATE_OFF), 500},
    {LED_BLINK_BREATHE, INSERT_INDEX(4, LED_STATE_50_PERCENT), 500},
    {LED_BLINK_BREATHE, INSERT_INDEX(4, LED_STATE_OFF), 500},
    {LED_BLINK_LOOP, 0, 0},
};

static const blink_step_t bsp_ws2812_mode_charging_full[] = {
    {LED_BLINK_HSV, SET_IHSV(MAX_INDEX, 0, 0, 0), 0},
    {LED_BLINK_HSV, SET_IHSV(4, 100, 255, 0), 0},
    {LED_BLINK_LOOP, 0, 0},
};

static const blink_step_t bsp_ws2812_mode_low_power[] = {
    {LED_BLINK_HSV, SET_IHSV(MAX_INDEX, 0, 0, 0), 50},
    {LED_BLINK_HSV, SET_IHSV(4, 0, 255, LED_STATE_25_PERCENT), 100},
    {LED_BLINK_HSV, SET_IHSV(4, 0, 255, LED_STATE_OFF), 50},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t bsp_ws2812_mode_sleep_blink[] = {
    {LED_BLINK_HSV, SET_IHSV(MAX_INDEX, 0, 0, 127), 200},
    {LED_BLINK_BRIGHTNESS, INSERT_INDEX(MAX_INDEX, LED_STATE_OFF), 5000},
    {LED_BLINK_LOOP, 0, 0},
};

blink_step_t const *bsp_led_mode[] = {
    [BSP_WS2812_MODE_SLEEP]           = bsp_ws2812_mode_sleep_blink,
    [BSP_WS2812_MODE_CHARGING_BREATH] = bsp_ws2812_mode_charging_blink,
    [BSP_WS2812_MODE_CHARGING_FULL]   = bsp_ws2812_mode_charging_full,
    [BSP_WS2812_MODE_LOW_POWER]       = bsp_ws2812_mode_low_power,
    [BSP_WS2812_MODE_MAX]             = NULL,
};

void bsp_ws2812_init(void)
{
    // LED strip backend configuration: RMT
    led_strip_spi_config_t spi_config = {
        .clk_src        = SPI_CLK_SRC_DEFAULT,  // different clock source can lead to different power consumption
        .spi_bus        = SPI2_HOST,            // RMT counter clock frequency
        .flags.with_dma = true,                 // DMA feature is available on ESP target like ESP32-S3
    };

    led_strip_config_t strip_config = {
        .strip_gpio_num   = BSP_WS2812_GPIO_PIN,    // The GPIO that connected to the LED strip's data line
        .max_leds         = BSP_WS2812_STRIPS_NUM,  // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,   // Pixel format of your LED strip
        .led_model        = LED_MODEL_WS2812,       // LED strip model
        .flags.invert_out = false,                  // whether to invert the output signal
    };

    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg     = strip_config,
        .led_strip_driver  = LED_STRIP_SPI,
        .led_strip_spi_cfg = spi_config,
    };

    const led_indicator_config_t config = {
        .mode                        = LED_STRIPS_MODE,
        .led_indicator_strips_config = &strips_config,
        .blink_lists                 = bsp_led_mode,
        .blink_list_num              = BSP_WS2812_MODE_MAX,
    };

    bsp_led_handle = led_indicator_create(&config);
    assert(bsp_led_handle != NULL);
}

void bsp_ws2812_mode_set(bsp_ws2812_mode_e mode)
{
    if (BSP_WS2812_MODE_MANUAL != mode)
    {
        led_indicator_preempt_start(bsp_led_handle, mode);
    }
    else
    {
        led_indicator_preempt_stop(bsp_led_handle, BSP_WS2812_MODE_SLEEP);
    }
}

void bsp_ws2812_led_set(bsp_ws2812_led_e led, bsp_ws2812_led_status_e status)
{
    if (led == BSP_WS2812_LED_NONE)
    {
        return;
    }

    if (bsp_app_handle.ws2812_target_mode != BSP_WS2812_MODE_MANUAL)
    {
        bsp_app_handle.ws2812_target_mode = BSP_WS2812_MODE_MANUAL;
        led_indicator_preempt_stop(bsp_led_handle, BSP_WS2812_MODE_SLEEP);
    }
    if (bsp_led_handle != NULL)
    {
        // led_indicator_set_hsv
        for (uint8_t i = 0; i < BSP_WS2812_STRIPS_NUM; i++)
        {
            if ((i + 1) == led)
            {
            }
            else
            {
            }
        }
    }
}

#endif

#ifdef BSP_ADC

#define BSP_ADC_MAX_CHN            10
#define BSP_ADC_SMOOTH_WINDOW_SIZE 10

adc_oneshot_unit_handle_t adc1_handle;

static int bsp_adc_smoothing_filter_samples[BSP_ADC_MAX_CHN][BSP_ADC_SMOOTH_WINDOW_SIZE] = {0};

bool bsp_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle     = NULL;
    esp_err_t         ret        = ESP_FAIL;
    bool              calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id  = unit,
            .chan     = channel,
            .atten    = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id  = unit,
            .atten    = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void bsp_adc_init(adc_channel_t adc_chx, adc_atten_t adc_atten)
{
    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = adc_atten,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_chx, &config));
    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chx_handle = NULL;

    bsp_adc_calibration_init(ADC_UNIT_1, adc_chx, adc_atten, &adc1_cali_chx_handle);
}

// 平滑滤波
static int bsp_adc_smoothing_filter(adc_channel_t adc_chx, int new_sample)
{
    static uint8_t adc_index[BSP_ADC_MAX_CHN] = {};
    static int     adc_sum[BSP_ADC_MAX_CHN]   = {};

    int index = adc_index[adc_chx];

    adc_sum[index] -= bsp_adc_smoothing_filter_samples[adc_chx][index];
    bsp_adc_smoothing_filter_samples[adc_chx][index] = new_sample;
    adc_sum[index] += new_sample;
    adc_index[adc_chx] = (adc_index[adc_chx] + 1) % BSP_ADC_SMOOTH_WINDOW_SIZE;

    return (adc_sum[index] / BSP_ADC_SMOOTH_WINDOW_SIZE);
}

int bsp_adc_get_value(adc_channel_t adc_chx)
{
    int adc_value;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_chx, &adc_value));
    adc_value = bsp_adc_smoothing_filter(adc_chx, adc_value);
    return adc_value;
}
#endif

#ifdef BSP_UART

bsp_uart_handle_t bsp_uart0_handle;

void bsp_uart_set_callback(int uart_num, void *callback)
{
    switch (uart_num)
    {
    case 0: {
        bsp_uart0_handle.callback = callback;
        break;
    }
    default: {
        bsp_uart0_handle.callback = callback;
        break;
    }
    }
}

void bsp_uart_init(int uart_num, int baudrate)
{
    uart_config_t uart_config = {
        .baud_rate  = baudrate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    switch (uart_num)
    {
    case 0: {
        // 串口0
        ESP_ERROR_CHECK(uart_driver_install(uart_num, BSP_UART_BUFF_SIZE, BSP_UART_BUFF_SIZE, BSP_UART0_QUEUE_SIZE, &bsp_uart0_handle.rx_event_queue, intr_alloc_flags));
        ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(uart_num, BSP_UART0_TXD_PIN, BSP_UART0_RXD_PIN, BSP_UART0_RTS_PIN, BSP_UART0_CTS_PIN));
        uart_pattern_queue_reset(uart_num, BSP_UART0_QUEUE_SIZE);

        break;
    }
    default: {
        ESP_ERROR_CHECK(uart_driver_install(uart_num, BSP_UART_BUFF_SIZE, BSP_UART_BUFF_SIZE, BSP_UART0_QUEUE_SIZE, &bsp_uart0_handle.rx_event_queue, intr_alloc_flags));
        ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(uart_num, BSP_UART0_TXD_PIN, BSP_UART0_RXD_PIN, BSP_UART0_RTS_PIN, BSP_UART0_CTS_PIN));
        uart_pattern_queue_reset(uart_num, BSP_UART0_QUEUE_SIZE);
        break;
    }
    }
}

#endif

void bsp_board_init(void)
{
#ifdef BSP_GPIO
    bsp_gpio_init(BSP_GPIO_VBAT_MONITOR_PIN, BSP_GPIO_VBAT_MONITOR_PIN_MODE, GPIO_INTR_DISABLE, 1);  // 输入上拉
    bsp_gpio_init(BSP_GPIO_KEY_PIN, BSP_GPIO_KEY_PIN_MODE, GPIO_INTR_DISABLE, 1);                    // 输入上拉
    bsp_gpio_init(BSP_GPIO_POWER_PIN, BSP_GPIO_POWER_PIN_MODE, GPIO_INTR_DISABLE, 0);                // 输出

    bsp_gpio_write(BSP_GPIO_POWER_PIN, 1);  // 打开电源
#endif

#ifdef BSP_WS2812
    bsp_ws2812_init();
#endif

#ifdef BSP_ADC
    bsp_adc_init(BSP_GPIO_VBAT_ADC_CHAN, BSP_GPIO_VBAT_ADC_ATTEN);
    bsp_adc_init(BSP_GPIO_IR_0_ADC_CHAN, BSP_GPIO_IR_0_ADC_ATTEN);
    bsp_adc_init(BSP_GPIO_IR_1_ADC_CHAN, BSP_GPIO_IR_1_ADC_ATTEN);
#endif

#ifdef BSP_UART
    bsp_uart_init(BSP_UART0, BSP_UART0_BAUD_RATE);
#endif
}

void bsp_board_deinit(void)
{
    bsp_gpio_init(BSP_UART0_RXD_PIN, GPIO_MODE_OUTPUT, GPIO_INTR_DISABLE, 2);  // 输出下拉
    bsp_gpio_write(BSP_UART0_RXD_PIN, 0);
    bsp_gpio_write(BSP_GPIO_POWER_PIN, 0);  // 关闭电源
}
