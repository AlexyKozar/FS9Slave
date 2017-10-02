#ifndef I2C_H
    #define I2C_H
    //-----------------
    #include <stdint.h>
    #include <stdlib.h>
    #include <stdbool.h>
    #include "stm32f030x6.h"
    //---------------------
    void I2C_EE_Init(void);
    void I2C_EE_WriteBytes(uint8_t addr, uint8_t* data, uint8_t size);
#endif
