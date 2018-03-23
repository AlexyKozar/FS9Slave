#ifndef CMD_LIST_H
    #define CMD_LIST_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    //------------------------------------
    #define CMD_MASK      (uint16_t)0x0100
    #define CMD_CODE_MASK (uint16_t)0x003F
    //-------------------------------
    #define MAX_CMD_NUM (uint8_t)0x40
    //--------------------------------------
    /*! @brief Description structure command
     */
    //-------------------
    typedef struct _cmd_t
    {
        uint8_t n; // the count bytes in command
        uint8_t m; // the count bytes in response
        bool    is_ack; // need ack
    } cmd_t;
    //-----------------------------------
    /*! @brief get configuration a command
        @param a code command
        @return struct with parameters a command (return the type RESERVE if command is not find in list or it RESERVE type)
     */
    cmd_t CMD_get(uint8_t code);
#endif
