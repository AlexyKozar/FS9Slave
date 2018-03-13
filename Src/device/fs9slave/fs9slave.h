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
    //------------------------
    #define MAX_SIZE_BUF_RX 20
    #define MAX_SIZE_BUF_TX 20
    //--------------------------
    #define MAX_SIZE_PACK_BUF 20
    //--------------
    #define ACK 0x06 // подтверждение
    #define NAK 0x15 // отказ
    //-------------------------
    typedef struct _FS9Packet_t
    {
        uint8_t buffer[MAX_SIZE_PACK_BUF];
        uint8_t size;
    } FS9Packet_t;
    //---------------------------------
    bool FS9_read(FS9Packet_t* packet);
    bool FS9_write(FS9Packet_t* packet);
    bool FS9_Is_Ready(void);
#endif
