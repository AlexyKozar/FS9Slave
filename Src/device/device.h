#ifndef DEVICE_H
    #define DEVICE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx_hal.h"
    #include "stm32f0xx_hal_gpio.h"
    #include "fs9slave/fs9slave.h"
    #include "registers/registers.h"
    //---------------------------
    #define MAX_SIZE_DS_INPUT  12
    #define MAX_SIZE_DS_OUTPUT 8
    //------------------------
    #define DEV_IO_INPUT  0x00
    #define DEV_IO_OUTPUT 0x01
    //-------------------------
    #define IN_DIR_INVERSE 0x00
    #define IN_DIR_DIRECT  0x01
    //---------------------
    #define IN_MODE_AC 0x00
    #define IN_MODE_DC 0x01
    //-------------------
    struct INPUT_Set_Type
    {
        uint8_t mode;
        uint8_t Nac;
        uint8_t Dac;
        float   SGac;
        float   NSac;
        uint8_t Ndc;
        uint8_t Ddc;
        uint8_t P0dc;
        uint8_t P1dc;
    };
    //---------------
    struct INPUT_Type
    {
        uint16_t              in;      // the input
        struct INPUT_Set_Type set;
        uint8_t               dir;  // the input direct
        uint32_t              buf;  // the input buffer for filter
        uint8_t               state; // the state input
    };
    //--------------------
    struct PORT_Input_Type
    {
        GPIO_TypeDef*     gpio;
        struct INPUT_Type in_arr[MAX_SIZE_DS_INPUT];
        uint8_t           size;
    };
    //---------------------
    struct PORT_Output_Type
    {
        GPIO_TypeDef* gpio;
        uint16_t      out_arr[MAX_SIZE_DS_OUTPUT];
        uint8_t       size;
    };
    //---------------------------------------------------------
    void    DEV_Create(GPIO_TypeDef* gpio, uint16_t addr_pins);
    void    DEV_Init(struct PORT_Input_Type* inputs, struct PORT_Output_Type* outputs);
    uint8_t DEV_Address(void);
    bool    DEV_Request(struct FS9Packet_t* source, struct FS9Packet_t* dest);
    bool    DEV_Driver(uint8_t cmd, struct FS9Packet_t* packet);
    uint8_t DEV_Checksum(struct FS9Packet_t* packet, uint8_t size);
    void    DEV_Input_Scan(void);
    void    DEV_Input_Set_Default(void);
    bool    DEV_Input_Ready(void);
    void    DEV_Input_Filter(void);
    bool    DEV_Input_Change_Channel(void);
#endif
