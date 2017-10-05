#ifndef I2C_H
    #define I2C_H
    //-----------------
    #include <stdint.h>
    #include <stdlib.h>
    #include <stdbool.h>
    #include "stm32f030x6.h"
    //----------------------
    #define I2C_BUS     I2C1
    #define I2C_SUCCESS true
    #define I2C_ERROR   false
    //---------------------
    void I2C_EE_Init(void);
    bool I2C_EE_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t size);
    bool I2C_EE_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t size);
#endif
