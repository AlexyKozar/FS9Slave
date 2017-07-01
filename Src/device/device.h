#ifndef DEVICE_H
    #define DEVICE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "../fs9slave/fs9slave.h"
    //-----------------------
    #define MAX_SIZE_DS_IO 16
    //------------------------
    #define DEV_IO_INPUT  0x00
    #define DEV_IO_OUTPUT 0x01
    //-----------------------------
    #define GPIO_PORT_ADDRESS GPIOC
    #define GPIO_PIN_ADDR_0   GPIO_IDR_14
    #define GPIO_PIN_ADDR_1   GPIO_IDR_15
    #define DEV_ADDR_MASK     (uint16_t)(GPIO_PIN_ADDR_0 | GPIO_PIN_ADDR_0)
    //------------
    struct IO_Type
    {
        GPIO_TypeDef* gpio;
        uint16_t      io[MAX_SIZE_DS_IO];
        uint8_t       size;
    };
    //--------------------------------------------------------
    void    DEV_Create(GPIO_TypeDef* gpio, uint16_t pin_addr);
    void    DEV_Init(struct IO_Type* inputs, struct IO_Type* outputs);
    uint8_t DEV_Address(void);
#endif
