#ifndef DEVICE_H
    #define DEVICE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    #include "../fs9slave/fs9slave.h"
    //---------------------------
    #define MAX_SIZE_DS_INPUT  12
    #define MAX_SIZE_DS_OUTPUT 8
    //------------------------
    #define DEV_IO_INPUT  0x00
    #define DEV_IO_OUTPUT 0x01
    //-------------------------
    #define IN_DIR_DIRECT  0x00
    #define IN_DIR_INVERSE 0x01
    //---------------------
    #define IN_MODE_AC 0x00
    #define IN_MODE_DC 0x01
    //---------------
    struct INPUT_Type
    {
        uint16_t In;      // the input
        uint8_t  Nac;     // the period count (one period is 10ms)
        uint8_t  Dac;     // the discret filter set AC (5..40)
        uint8_t  SGac;    // the impulse length (2..8)
        uint8_t  NSac;    // the noise length (0..2)
        uint8_t  In_mode; // the input mode AC or DC
        uint8_t  In_dir;  // the input direct
    };
    //--------------------
    struct PORT_Input_Type
    {
        GPIO_TypeDef*     gpio;
        struct INPUT_Type inputs[MAX_SIZE_DS_INPUT];
        uint8_t           size;
    };
    //---------------------
    struct PORT_Output_Type
    {
        GPIO_TypeDef* gpio;
        uint16_t      outputs[MAX_SIZE_DS_OUTPUT];
        uint8_t       size;
    };
    //---------------------------------------------------------
    void    DEV_Create(GPIO_TypeDef* gpio, uint16_t addr_pins);
    void    DEV_Init(struct PORT_Input_Type* inputs, struct PORT_Output_Type* outputs);
    uint8_t DEV_Address(void);
    bool    DEV_Request(struct FS9Packet_t* source, struct FS9Packet_t* dest);
    bool    DEV_Driver(uint8_t cmd, struct FS9Packet_t* packet);
    uint8_t DEV_Checksum(struct FS9Packet_t* packet, uint8_t size);
#endif
