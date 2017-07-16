#ifndef AIN_H
    #define AIN_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f030x6.h"
    //--------------------------
    #define MAX_NUM_CHANNELS   4
    #define MAX_NUM_CONVERSION 10
    //-----------------------------------------------------------
    #define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
    //-------------------
    void  AIN_Init(void);
    bool  AIN_Is_Ready(void);
    float AIN_Get_Temperature(void);
    float AIN_Get_Channel_1(void);
    float AIN_Get_Channel_2(void);
#endif
