#include "flash.h"
//---------------------
bool FLASH_Unlock(void)
{
    while((FLASH->SR & FLASH_SR_BSY) != 0);
    
    if((FLASH->CR & FLASH_CR_LOCK) != 0)
    {
        FLASH->KEYR = FLASH_KEY_1;
        FLASH->KEYR = FLASH_KEY_2;
        
        return true;
    }
    
    return false;
}
//--------------------------------------------
bool FLASH_Write(uint32_t addr, uint32_t data)
{
    FLASH->CR |= FLASH_CR_PG;
    
    (*(__IO uint16_t*)addr) = data;
    
    while((FLASH->SR & FLASH_SR_BSY) != 0);
    
    addr += 2;
    data >>= 16;
    
    (*(__IO uint16_t*)addr) = data;
    
    while((FLASH->SR & FLASH_SR_BSY) != 0);
    
    if((FLASH->SR & FLASH_SR_EOP) != 0)
    {
        FLASH->SR = FLASH_SR_EOP;
    }
    else
    {   
        return false;
    }
    
    FLASH->CR &= ~FLASH_CR_PG;
    
    return true;
}
//-----------------------------
bool FLASH_Erase(uint32_t addr)
{
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT;
    
    while((FLASH->SR & FLASH_SR_BSY) != 0);
    
    if((FLASH->SR & FLASH_SR_EOP) != 0)
    {
        FLASH->SR = FLASH_SR_EOP;
    }
    else
    {
        return false;
    }
    
    FLASH->CR &= ~FLASH_CR_PER; /* (7) */
    
    return true;
}
//--------------------------------
uint32_t FLASH_Read(uint32_t addr)
{
    return (*(__IO uint32_t*)addr);
}
//-------------------
void FLASH_Lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}
