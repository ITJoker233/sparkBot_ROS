idf_component_register(SRCS "drv_ir_senser.c" "drv_motor.c" "drv_uart.c" "drv_power_manager.c" "drv_adc.c"
                    REQUIRES tank_bsp
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_adc driver)
