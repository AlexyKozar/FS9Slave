#include "cmd.h"
//----------------------------------
struct cmd_t cmd_list[MAX_CMD_NUM] =
{
    { 2, 4, false },
    { 2, 2, false },
    { 2, 17, false },
    { 2, 4, false },
    { 2, 4, false },
    { 5, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 2, 2, true },
    { 3, 10, false },
    { 11, 2, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 2, 5, false },
    { 2, 4, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false },
    { 0, 0, false }
};
//--------------------------------
struct cmd_t CMD_get(uint8_t code)
{
    struct cmd_t cmd = { 0, 0, false };
    
    if(code > 0x3F)
        return cmd;
        
    return cmd_list[code];
}
