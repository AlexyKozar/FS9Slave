#include "cmd.h"
//---------------------------
cmd_t cmd_list[MAX_CMD_NUM] =
{
    { 2, 4, false },     // 0x00
    { 2, 2, false },     // 0x01
    { 2, 17, false },    // 0x02
    { 2, 4, false },     // 0x03
    { 2, 4, false },     // 0x04
    { 5, 2, true },      // 0x05
    { 2, 2, true },      // 0x06
    { 2, 2, true },      // 0x07
    { 2, 2, true },      // 0x08
    { 2, 2, true },      // 0x09
    { 2, 2, true },      // 0x0A
    { 2, 2, true },      // 0x0B
    { 2, 2, true },      // 0x0C
    { 2, 2, true },      // 0x0D
    { 2, 2, true },      // 0x0E
    { 2, 2, true },      // 0x0F
    { 2, 2, true },      // 0x10
    { 2, 2, true },      // 0x11
    { 2, 2, false },     // 0x12
    { 2, 2, true },      // 0x13
    { 2, 2, true },      // 0x14
    { 2, 2, true },      // 0x15
    { 3, 10, false },    // 0x16
    { 11, 2, false },    // 0x17
    { 2, 2, false },     // 0x18 - чтение байта конфигурации искробезопасных входов
    { 3, 2, false },     // 0x19 - запись байта конфигурации искробезопасных входов
    { 0, 0, false },     // 0x1A
    { 0, 0, false },     // 0x1B
    { 0, 0, false },     // 0x1C
    { 2, 17, false },    // 0x1D - чтение отладочной информации (счетчиков ошибок)
    { 2, 9, false },     // 0x1E - a device data (id, number, lot, firmware variant and firmware date - 3 bytes)
    { 2, 4, false },     // 0x1F - чтение времени срабатывания выделенного входного дискретного канала
    { 0, 0, false },     // 0x20
    { 0, 0, false },     // 0x21
    { 0, 0, false },     // 0x22
    { 0, 0, false },     // 0x23
    { 0, 0, false },     // 0x24
    { 0, 0, false },     // 0x25
    { 0, 0, false },     // 0x26
    { 0, 0, false },     // 0x27
    { 0, 0, false },     // 0x28
    { 0, 0, false },     // 0x29
    { 0, 0, false },     // 0x2A
    { 0, 0, false },     // 0x2B
    { 0, 0, false },     // 0x2C
    { 0, 0, false },     // 0x2D
    { 0, 0, false },     // 0x2E
    { 0, 0, false },     // 0x2F
    { 0, 0, false },     // 0x30
    { 0, 0, false },     // 0x31
    { 0, 0, false },     // 0x32
    { 0, 0, false },     // 0x33
    { 0, 0, false },     // 0x34
    { 0, 0, false },     // 0x35
    { 0, 0, false },     // 0x36
    { 0, 0, false },     // 0x37
    { 0, 0, false },     // 0x38
    { 0, 0, false },     // 0x39
    { 0, 0, false },     // 0x3A
    { 2, 2, true },      // 0x3B - чтение из памяти (тест eeprom)
    { 2, 2, true },      // 0x3C - запись в память (тест eeprom)
    { 0, 0, false },     // 0x3D
    { 5, 2, true },      // 0x3E - установка параметров фильтрации (кол-во периодов, дискретность, сигнал в мс)
    { 6, 2, true }       // 0x3F - настройка входа по его номеру
};
//-------------------------
cmd_t CMD_get(uint8_t code)
{
    cmd_t cmd = { 0, 0, false };
    
    if(code > 0x3F)
        return cmd;
    
    cmd = cmd_list[code];
        
    return cmd;
}
