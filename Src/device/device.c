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
//--------------------------------------
uint8_t dev_get_size_packet(uint8_t cmd)
{
    uint8_t size = 0;
    
    struct cmd_t tcmd = CMD_get(cmd);
    
    if(tcmd.type != RESERVE)
        size = tcmd.n;
    
    return size;
}
