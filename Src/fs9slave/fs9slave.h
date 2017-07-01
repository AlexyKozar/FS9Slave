#ifndef FS9SLAVE_H
    #define FS9SLAVE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx.h"
    #include "cmd/cmd.h"
    //---------------------------
    #define FS9_UART       USART1
    #define FS9_IRQHandler USART1_IRQHandler
    //------------------------------------
    #define CMD_MASK      (uint16_t)0x0100
    #define CMD_ADDR_MASK (uint8_t)0xC0
    #define CMD_CODE_MASK (uint8_t)0x3F
    //------------------------
    #define MAX_SIZE_BUF_RX 20
    #define MAX_SIZE_BUF_TX 20
    //----------------------------------
    extern volatile bool FS9_Data_Ready;
    //-----------------------------
    uint8_t FS9_read(uint8_t* buf);
    bool    FS9_write(uint8_t* buf, uint8_t size_t);
#endif
