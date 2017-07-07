#ifndef DEVICE_H
    #define DEVICE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    #include "../fs9slave/fs9slave.h"
    #include "../ain/ain.h"
    //-----------------------
    #define MAX_SIZE_DS_IO 16
    //------------------------
    #define DEV_IO_INPUT  0x00
    #define DEV_IO_OUTPUT 0x01
    //------------
    struct IO_Type
    {
        GPIO_TypeDef* gpio;
        uint16_t      io[MAX_SIZE_DS_IO];
        uint8_t       size;
    };
    //---------------------------------------------------------
    void    DEV_Create(GPIO_TypeDef* gpio, uint16_t addr_pins);
    void    DEV_Init(struct IO_Type* inputs, struct IO_Type* outputs);
    uint8_t DEV_Address(void);
    bool    DEV_Request(struct FS9Packet_t* source, struct FS9Packet_t* dest);
    bool    DEV_Driver(uint8_t cmd, struct FS9Packet_t* packet);
    uint8_t DEV_Checksum(struct FS9Packet_t* packet, uint8_t size);
#endif
