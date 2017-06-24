#ifndef CMD_LIST_H
    #define CMD_LIST_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    //---------------------------------------------
    /** @brief This file for access to command list
        @param index command ans is command's    
     */
    //----------------------------------------
    /** @brief This variables is command types
     */
    #define READ_DSInCh              (uint8_t)0x00 // the command read a discret input chanels
    #define READ_DSOutCh             (uint8_t)0x01 // the command read a discret output chanels
    #define READ_AIn                 (uint8_t)0x02 // the command read an analog inputs
    #define READ_DSExInCh            (uint8_t)0x03 // the command read the extend discret input chanels
    #define READ_DSExOutCh           (uint8_t)0x04 // the command read the extend discret output chanals
    #define WRITE_REG_ExOutCh        (uint8_t)0x05 // the command write a register extend discret output chanels
    #define SET_LEVEL_LOW_DSOut_Ch0  (uint8_t)0x06 // the command set on discret out low value chanel 0
    #define SET_LEVEL_LOW_DSOut_Ch1  (uint8_t)0x07 // the command set on discret out low value chanel 1
    #define SET_LEVEL_LOW_DSOut_Ch2  (uint8_t)0x08 // the command set on discret out low value chanel 2
    #define SET_LEVEL_LOW_DSOut_Ch3  (uint8_t)0x09 // the command set on discret out low value chanel 3
    #define SET_LEVEL_LOW_DSOut_Ch4  (uint8_t)0x0A // the command set on discret out low value chanel 4
    #define SET_LEVEL_LOW_DSOut_Ch5  (uint8_t)0x0B // the command set on discret out low value chanel 5
    #define SET_LEVEL_LOW_DSOut_Ch6  (uint8_t)0x0С // the command set on discret out low value chanel 6
    #define SET_LEVEL_LOW_DSOut_Ch7  (uint8_t)0x0D // the command set on discret out low value chanel 7
    #define SET_LEVEL_HIGH_DSOut_Ch0 (uint8_t)0x0E // the command set on discret out high value chanel 0
    #define SET_LEVEL_HIGH_DSOut_Ch1 (uint8_t)0x0F // the command set on discret out high value chanel 1
    #define SET_LEVEL_HIGH_DSOut_Ch2 (uint8_t)0x10 // the command set on discret out high value chanel 2
    #define SET_LEVEL_HIGH_DSOut_Ch3 (uint8_t)0x11 // the command set on discret out high value chanel 3
    #define SET_LEVEL_HIGH_DSOut_Ch4 (uint8_t)0x12 // the command set on discret out high value chanel 4
    #define SET_LEVEL_HIGH_DSOut_Ch5 (uint8_t)0x13 // the command set on discret out high value chanel 5
    #define SET_LEVEL_HIGH_DSOut_Ch6 (uint8_t)0x14 // the command set on discret out high value chanel 6
    #define SET_LEVEL_HIGH_DSOut_Ch7 (uint8_t)0x15 // the command set on discret out high value chanel 7
    #define READ_CNF                 (uint8_t)0x16 // the command read a device configuration
    #define WRITE_CNF                (uint8_t)0x17 // the command write a device configuration
    #define READ_ID                  (uint8_t)0x18 // the command read a device ID
    #define READ_SET_DSDIn           (uint8_t)0x19 // the command read the changed set input discret chanel
    #define RESERVE                  (uint8_t)0x1A // the command reserve
    //-------------------------------
    #define MAX_CMD_NUM (uint8_t)0x40
    //--------------------------------------
    /** @brief Description structure command
     */
    //----------
    struct cmd_t
    {
        uint8_t n; // the count bytes in command
        uint8_t m; // the count bytes in response
        uint8_t type; // the type command
        bool    is_ack; // need ack
    };
    //-----------------------------------
    /** @brief get configuration a command
        @param a code command
        @return struct with parameters a command (return the type RESERVE if command is not find in list or it RESERVE type)
     */
    struct cmd_t CMD_get(uint8_t code);
#endif
