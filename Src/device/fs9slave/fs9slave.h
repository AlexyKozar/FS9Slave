#ifndef FS9SLAVE_H
    #define FS9SLAVE_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include <string.h>
    #include "stm32f0xx.h"
    #include "cmd/cmd.h"
    //--------------------------
    #define BUFFER_MAX_SIZE 0x14 // max size rx and tx buffer
    #define ACK             0x06
    #define NAK             0x15
    /*!
     * /brief struct buffer type
     */
    typedef struct _FS9Buffer_t
    {
        uint16_t data[BUFFER_MAX_SIZE]; // messages buffer
        uint8_t  cmd_code; // command code
        cmd_t    cmd; // command data
        uint16_t size; // data buffer size
        uint16_t index; // current data index
    } FS9Buffer_t;    
    /*!
     * /brief Protocol initialize
     * /param address The device address
     */
    void FS9Slave_Init(uint8_t address);
    /*!
     * /brief Data is ready
     * /return true if data is ready
     */
    bool FS9Slave_IsReady(void);
    /*!
     * /brief Read a data from receiver buffer
     * /param dest Destination buffer
     * /param source Source buffer
     * /return true if data is read from receiver buffer
     */
    bool FS9Slave_Read(FS9Buffer_t* dest);
    /*!
     * /brief Send a data to transmission buffer
     * /param packet Data packet for transmission
     */
     void FS9Slave_Write(FS9Buffer_t* packet);
#endif
