#include "device.h"
//---------------------
uint8_t devAddr = 0xFF;
//-----------------------------
void dev_set_addr(uint8_t addr)
{
    devAddr = addr;
}
//------------------------
uint8_t dev_get_addr(void)
{
    return devAddr;
}
