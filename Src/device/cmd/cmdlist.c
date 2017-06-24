#include "cmdlist.h"
//----------------------------------
struct cmd_t cmd_list[MAX_CMD_NUM] =
{
    { 2, 4, READ_DSInCh, false },
    { 2, 2, READ_DSOutCh, false },
    { 2, 17, READ_AIn, false },
    { 2, 4, READ_DSExInCh, false },
    { 2, 4, READ_DSExOutCh, false },
    { 5, 2, WRITE_REG_ExOutCh, true },
    { 2, 2, SET_LEVEL_LOW_DSOut_Ch0, true },
    { 2, 2, SET_LEVEL_LOW_DSOut_Ch1, true },
    { 2, 2, SET_LEVEL_LOW_DSOut_Ch2, true },
    { 2, 2, SET_LEVEL_LOW_DSOut_Ch3, true },
    { 2, 2, SET_LEVEL_LOW_DSOut_Ch4, true },
    { 2, 2, SET_LEVEL_LOW_DSOut_Ch5, true },
    { 2, 2, SET_LEVEL_LOW_DSOut_Ch6, true },
    { 2, 2, SET_LEVEL_LOW_DSOut_Ch7, true },
    { 2, 2, SET_LEVEL_HIGH_DSOut_Ch0, true },
    { 2, 2, SET_LEVEL_HIGH_DSOut_Ch1, true },
    { 2, 2, SET_LEVEL_HIGH_DSOut_Ch2, true },
    { 2, 2, SET_LEVEL_HIGH_DSOut_Ch3, true },
    { 2, 2, SET_LEVEL_HIGH_DSOut_Ch4, true },
    { 2, 2, SET_LEVEL_HIGH_DSOut_Ch5, true },
    { 2, 2, SET_LEVEL_HIGH_DSOut_Ch6, true },
    { 2, 2, SET_LEVEL_HIGH_DSOut_Ch7, true },
    { 3, 10, READ_CNF, false },
    { 11, 2, WRITE_CNF, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 2, 5, READ_ID, false },
    { 2, 4, READ_SET_DSDIn, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false },
    { 0, 0, RESERVE, false }
};
//--------------------------------
struct cmd_t CMD_get(uint8_t code)
{
    struct cmd_t cmd = { 0, 0, RESERVE, false };
    
    if(code > 0x3F)
        return cmd;
        
    return cmd_list[code];
}
