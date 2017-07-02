#ifndef AIN_H
    #define AIN_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include "stm32f0xx.h"
    //----------------------------------------
    #define AIN_IRQHandler DMA1_Ch1_IRQHandler
    //--------------------------
    #define MAX_NUM_CHANNEL    3
    #define MAX_NUM_CONVERSION 10
    //------------------
    void AIN_Init(void);
    bool AIN_Is_Ready(void);
    void AIN_Read(uint16_t* buf);
#endif
