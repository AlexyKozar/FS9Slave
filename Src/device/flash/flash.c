#include "flash.h"
//--------------------------------
uint32_t _addr = FLASH_CELL_EMPTY; // the current position for saved
uint32_t _last = FLASH_CELL_EMPTY; // the last write for read
//-------------------
void FLASH_Init(void)
{
    if(FLASH_AddressSearch() == false)
    {
        FLASH_Unlock();
        FLASH_Erase(FLASH_BASE_ADDRESS);
        FLASH_Lock();
    }
}
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
//------------------------------------------------
bool FLASH_WriteBlock(uint8_t* data, uint8_t size)
{
    if(data == NULL || size == 0)
    {
        return false;
    }
    
    if(FLASH_Unlock() == false)
    {
        return false;
    }
    
    if(_addr == FLASH_CELL_EMPTY)
    {
        _addr = FLASH_BASE_ADDRESS;
    }
    else
    {
        if((_addr + size/2 + 1) >= FLASH_END_ADDRESS)
        {
            FLASH_Erase(FLASH_BASE_ADDRESS);
            
            _addr = FLASH_BASE_ADDRESS;
        }
    }
    
    uint32_t cell = 0;
    
    for(uint16_t i = 0, j = 0; i < size; ++i)
    {
        if(i == 0)
        {
            uint16_t count = size/4; // aligment on byte
            
            if(size%4)
            {
                count++;
            }
            
            cell = count | (uint32_t)data[i] << 8;
            
            j = 2;
            
            continue;
        }
        
        cell |= (uint32_t)data[i] << j++*8;
        
        if(j%4 == 0 || i == (size - 1))
        {   
            FLASH_Write(_addr, cell);
            
            _addr += 4;
            j      = 0;
            cell   = 0;
        }
    }
    
    FLASH_Lock();
    
    return true;
}
//----------------------------
bool FLASH_AddressSearch(void)
{
    uint32_t addr = FLASH_BASE_ADDRESS;
    uint32_t cell = FLASH_Read(addr);
    
    if(cell == FLASH_CELL_EMPTY) // memory cell is empty
    {
        _addr = FLASH_BASE_ADDRESS;
        
        return true;
    }
    
    while(addr < FLASH_END_ADDRESS && cell != FLASH_CELL_EMPTY)
    {
        _last = addr;
        addr += (cell&0x000000FF)*4; // address next cell
        
        cell = FLASH_Read(addr); // a new value from cell memory
    }
    
    if(addr == FLASH_END_ADDRESS)
    {
        return false;
    }
    
    _addr = addr;
    
    return true;
}
