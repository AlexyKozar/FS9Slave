#include "cmd.h"
//---------------------------
cmd_t cmd_list[MAX_CMD_NUM] =
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
    { 2, 9, false }, // a device data (id, number, lot, firmware variant and firmware date - 3 bytes)
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
    { 2, 2, true }, // запись в память (тест eeprom)
    { 2, 7, false }, // получение счетчика ошибок
    { 5, 2, true }, // установка параметров фильтрации (кол-во периодов, дискретность, сигнал в мс)
    { 6, 2, true } // настройка входа по его номеру
};
//-------------------------
cmd_t CMD_get(uint8_t code)
{
    cmd_t cmd = { 0, 0, false };
    
    if(code > 0x3F)
        return cmd;
        
    return cmd_list[code];
}
