#ifndef FLASH_H
    #define FLASH_H
    //-----------------
    #include <stdint.h>
    #include <stdbool.h>
    #include <stdlib.h>
    #include "stm32f030x6.h"
    //--------------------------------------
    #define FLASH_KEY_1 (uint32_t)0x45670123
    #define FLASH_KEY_2 (uint32_t)0xCDEF89AB
    //---------------------------------------------
    #define FLASH_BASE_ADDRESS (uint32_t)0x08007C00
    #define FLASH_END_ADDRESS  (uint32_t)0x08007FFF
    #define FLASH_CELL_EMPTY   (uint32_t)0xFFFFFFFF
    //------------------------
    void     FLASH_Init(void);
    bool     FLASH_Unlock(void);
    bool     FLASH_Write(uint32_t addr, uint32_t data);
    bool     FLASH_Erase(uint32_t addr);
    uint32_t FLASH_Read(uint32_t addr);
    void     FLASH_Lock(void);
    bool     FLASH_WriteBlock(uint8_t* data, uint8_t size);
    bool     FLASH_AddressSearch(void);
#endif
