#ifndef DS18B20_H
    #define DS18B20_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    //-----------------------------
    #define DS18B20_GPIO      GPIOB
    #define DS18B20_GPIO_BUS  RCC_AHBENR_GPIOBEN
    #define DS18B20_PIN       GPIO_PIN_4
    #define DS18B20_TIM       TIM3
    #define DS18B20_TIM_BUS   RCC_APB1ENR_TIM3EN
    #define DS18B20_IRQ       TIM3_IRQn
    //-----------------------
    void  DS18B20_Init(void);
    void  DS18B20_Convert(void* param);
    float DS18B20_Temperature(void);
#endif
