#ifndef AIN_H
    #define AIN_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f030x6.h"
    //-----------------------------------------------------------
    #define TEMP30_CAL_ADDR  ((uint16_t*) ((uint32_t)0x1FFFF7B8))
    #define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t)0x1FFFF7BA))
    //------------------------------
    void     AIN_Init(uint8_t addr);
    bool     AIN_Is_Ready(void);
    int32_t  AIN_Get_Temperature(void);
    uint16_t AIN_Get_Channel_1(void);
    uint16_t AIN_Get_Channel_2(void);
    uint16_t AIN_Get_VDDA(void);
#endif
