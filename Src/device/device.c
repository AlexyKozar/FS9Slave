#include "device.h"
//---------------------------------------
void IO_Clock_Enable(GPIO_TypeDef* gpio);
void IO_Init(io_t io, uint8_t io_dir);
void IO_Set(output_t output, bool state);
void CHANNEL_Out_Set(uint8_t index, bool state);
void TIM_Scan_Init(void);
void TIM_Scan_Update(void);
void TIM_INT_Init(void);
void TIM_INT_Start(void);
void PWROK_Init(void);
void CRASH_Init(void);
float Get_Temp(uint16_t val, uint8_t in_num);
float UAIN_to_TResistance(uint16_t val, uint8_t in_num); // преобразование напряжения в сопротивление температуры
void  blink2Hz(GPIO_TypeDef* gpio, uint16_t pin); // мигание с частотой 2Гц (для МИК-01)
void  crash(GPIO_TypeDef* gpio, uint16_t pin); // для обработки аварийной ситуации (нет запросов от ЦП 5 сек)
//----------------------
PORT_Input_Type*  io_in;
PORT_Output_Type* io_out;
PWROK_Type        pwr_ok = { false, false, 0, false, false };
//---------------------
uint8_t devAddr = 0xFF;
//-------------------------
bool Input_Changed = false;
//--------------------
bool is_crash = false; // авария - отключение выхода (происходит в случае отсутствия запросов от ЦП 5 сек)
output_t out_crash; // выход аварийной сигнализации
//---------------------------------------
uint16_t AIN_TEMP[MAX_SIZE_AIN_TEMP][3] = 
{
    { 0, 13, 18 }, // сопротивление термодатчика, напряжение AIN1, напряжение AIN2 (напряжения умножены на 10000)
    { 1, 170, 170 },
    { 2, 330, 340 },
    { 10, 1650, 1650 },
    { 20, 3280, 3270 },
    { 37, 6080, 6070 },
    { 48, 8010, 7980 },
    { 66, 10800, 10780 },
    { 80, 13230, 13190 },
    { 92, 15080, 15040 },
    { 100, 16470, 16420 },
    { 115, 18850, 18790 },
    { 127, 20800, 20700 },
    { 166, 27200, 27100 },
    { 178, 29200, 29100 },
    { 187, 30800, 30700 },
    { 189, 31000, 30900 },
    { 195, 32200, 32100 },
    { 200, 32900, 32900 }
};
//-----------------------------------------------------
void DEV_Create(GPIO_TypeDef* gpio, uint16_t addr_pins)
{
    io_t pin = { gpio, addr_pins };
    
    IO_Init(pin, DEV_IO_INPUT);
    
    devAddr = (uint8_t)((gpio->IDR & addr_pins) >> 14); // set the address device
}
//---------------------------------------------------------------
void DEV_Init(PORT_Input_Type* inputs, PORT_Output_Type* outputs)
{
    io_in  = inputs;
    io_out = outputs;
    
    for(uint8_t i = 0; i < io_in->size; ++i)
    {
        IO_Init(io_in->list[i].pin, DEV_IO_INPUT);
        io_in->list[i].pin.num = i;
    }
    
    for(uint8_t i = 0; i < io_out->size; ++i)
    {
        IO_Init(io_out->list[i].pin, DEV_IO_OUTPUT);
        io_out->list[i].pin.num = i;
        io_out->list[i].state   = OUTPUT_STATE_OFF;
        
        IO_Set(io_out->list[i], false); // выключить выход - состояние по умолчанию
    }
    
    io_t pin_int = { GPIO_INT, GPIO_INT_PIN };
    IO_Init(pin_int, DEV_IO_OUTPUT); // вывод INT как выход
    
    GPIO_INT->BSRR |= GPIO_INT_SET; // включить выход INT (default state)
    
    if(devAddr == 0x00)
    {
        PWROK_Init();
    }
    
    DEV_Input_Set_Default();
    
    TIM_Scan_Init();
    TIM_INT_Init();
    
    AIN_Init();
    
    CRASH_Init();
}
//--------------------------------------
void IO_Clock_Enable(GPIO_TypeDef* gpio)
{
    if(gpio == GPIOA)
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    else if(gpio == GPIOB)
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    else if(gpio == GPIOC)
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
}
//------------------------------------
void IO_Init(io_t pin, uint8_t io_dir)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    IO_Clock_Enable(pin.gpio);
    
    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin  = pin.pin;
    GPIO_InitStruct.Mode = (io_dir == 0x01)?GPIO_MODE_OUTPUT_PP:GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(pin.gpio, &GPIO_InitStruct);
}
//--------------------------------------
void IO_Set(output_t output, bool state)
{
    if(output.level == true)
    {
        if(state == true)
        {
            output.pin.gpio->ODR |= output.pin.pin;
        }
        else
        {
            output.pin.gpio->ODR &= ~output.pin.pin;
        }
    }
    else if(output.level == false)
    {
        if(state == true)
        {
            output.pin.gpio->ODR &= ~output.pin.pin;
        }
        else
        {
            output.pin.gpio->ODR |= output.pin.pin;
        }
    }
}
//---------------------------------------------
void CHANNEL_Out_Set(uint8_t index, bool state)
{
    if(index < io_out->size)
    {
        output_t out = io_out->list[index];
        
        IO_Set(out, state);
    }
}
//----------------------
void TIM_Scan_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    
    TIM16->PSC   = F_CPU/1000000UL - 1;
    
    TIM_Scan_Update();
    
    TIM16->CR1  |= TIM_CR1_ARPE;
    TIM16->DIER |= TIM_DIER_UIE;
    TIM16->CR1  |= TIM_CR1_CEN;
    
    NVIC_EnableIRQ(TIM16_IRQn);
}
//------------------------
void TIM_Scan_Update(void)
{
    TIM16->ARR   = 10000/io_in->set.Ndiscret - 1;
    TIM16->DIER &= ~TIM_DIER_UIE;
    TIM16->EGR  |= TIM_EGR_UG;
    TIM16->DIER |= TIM_DIER_UIE;
}
//---------------------
void TIM_INT_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    
    TIM17->PSC   = F_CPU/1000000UL - 1;
    TIM17->ARR   = 5000; // 5 ms сигнал INT
    TIM17->DIER |= TIM_DIER_UIE;
    TIM17->CR1  |= TIM_CR1_OPM;
    
    NVIC_EnableIRQ(TIM17_IRQn);
}
//----------------------
void TIM_INT_Start(void)
{
    GPIO_INT->BSRR |= GPIO_INT_RESET;
    TIM17->CR1 |= TIM_CR1_CEN;
}
//-------------------
void PWROK_Init(void)
{
    IO_Clock_Enable(GPIO_PWROK);
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM14_STOP;

    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI2;
    
    EXTI->IMR  |= GPIO_PWROK_PIN;
    EXTI->RTSR |= GPIO_PWROK_PIN;
    
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    
    TIM14->PSC = F_CPU/1000L - 1;
    TIM14->ARR = 11 - 1;
    
    TIM14->CR1  |= TIM_CR1_ARPE;
    TIM14->EGR  |= TIM_EGR_UG;
    TIM14->SR   &= ~TIM_SR_UIF;
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1  |= TIM_CR1_CEN;
    
    NVIC_EnableIRQ(TIM14_IRQn);
}
//-------------------
void CRASH_Init(void)
{
    // настройка аварийного выхода
    out_crash       = io_out->list[2];
    out_crash.param = EVENT_Create(5000, true, crash, out_crash.pin.gpio, out_crash.pin.pin, 0xFF);
    IO_Set(out_crash, true); // включаем аварийный выход - только для теста
}
//-----------------------
uint8_t DEV_Address(void)
{
    return devAddr;
}
//------------------------------------------------------
bool DEV_Request(FS9Packet_t* source, FS9Packet_t* dest)
{
    is_crash = true; // получили запрос от ЦП - сброс аварийной ситуации
    
    uint8_t addr = ((source->buffer[0]&CMD_ADDR_MASK) >> 6);
    
    if(addr != devAddr)
        return false;
    
    uint8_t cmd = source->buffer[0]&CMD_CODE_MASK; // get the command for device
    
    cmd_t tcmd = CMD_get(cmd); // verification command
    
    if(tcmd.n == 0 || tcmd.m == 0)
        return false; // command is not valid
    
    uint8_t checksum = DEV_Checksum(source, source->size - 1);
    
    if(checksum != source->buffer[source->size - 1])
        return false;
    
    FS9Packet_t packet; // пакет данных (т.е. чистые данные без контрольной суммы и команды)
    
    packet.size = source->size - 2; // размер пакета данных (минус команда и контрольная сумма)
    
    for(uint8_t i = 0; i < packet.size; ++i) // заполнение данными пакета
    {
        packet.buffer[i] = source->buffer[i + 1];
    }
    
    if(!DEV_Driver(cmd, &packet, dest))
        return false;
    
    if(tcmd.n > 0 && tcmd.m > 0)
    {
        if(tcmd.is_ack)
        {
            dest->buffer[dest->size++] = ACK;
        }
        
        // append checksum for packet
        checksum = DEV_Checksum(dest, dest->size);
        dest->buffer[dest->size++] = checksum;
    }
    else
        return false;
    
    return true;
}
//------------------------------------------------------------------
bool DEV_Driver(uint8_t cmd, FS9Packet_t* data, FS9Packet_t* packet)
{
    uint8_t bit_count = 0; // счетчик бит (позиция канала в байте)
    
    int32_t   temp;
    uint16_t  ain1;
    uint16_t  ain2;
    uint8_t   byte;
    uint8_t   state;
    uint8_t   n_out;
    output_t* out = NULL;
    
    switch(cmd)
    {
        case 0x00: // чтение дискретных каналов входов
            for(uint8_t i = 0; i < io_in->size; ++i)
            {
                if(bit_count == 8)
                {
                    bit_count = 0;
                    packet->size++;
                    packet->buffer[packet->size] = 0x00;
                }
                
                uint8_t channel_state = 0x00;
                
                if(io_in->list[i].state == true && io_in->list[i].error == false)
                {
                    channel_state = 0x01;
                }
                else if(io_in->list[i].error == true)
                {
                    channel_state = 0x02;
                }
                
                packet->buffer[packet->size] |= channel_state << bit_count;
                
                bit_count += 2;
            }
            
            packet->size = 3;
        break;
            
        case 0x01: // чтение дискретных каналов выходов
            packet->buffer[0] = 0x00;
        
            for(uint8_t i = 0; i < io_out->size; ++i)
            {
                io_t pin = io_out->list[i].pin;
                
                if((pin.gpio->ODR & pin.pin) == pin.pin)
                {
                    packet->buffer[0] |= 0x01 << i;
                }
            }
            
            packet->size = 1;
        break;
            
        case 0x02: // чтение аналоговых величин 1..4
            while(AIN_Is_Ready() == false); // ожидание готовности результатов
            
            temp = AIN_Get_Temperature();
            ain1 = AIN_Get_Channel_1();
            ain2 = AIN_Get_Channel_2();
            
            union float_t t;
            
            if(devAddr == 0)
            {
                uint16_t vdda = AIN_Get_VDDA();
                
                t.number = ((float)(ain1/4095.0f*vdda/1000.0f));
            }
            else if(devAddr == 1)
            {
                uint16_t temp = ((float)ain1/4095)*AIN_TEMP[MAX_SIZE_AIN_TEMP - 1][1];
                t.number = Get_Temp(temp, 1);
            }
            
            packet->buffer[0] = t.byte[0];
            packet->buffer[1] = t.byte[1];
            packet->buffer[2] = t.byte[2];
            packet->buffer[3] = t.byte[3];
        
            if(devAddr == 0)
            {
                uint16_t vdda = AIN_Get_VDDA();
                t.number = ((float)ain2/4095)*vdda/1000.0f; // шунт 0.1 Ом, усиление ОУ 20, делитель 2
                //t.number /= 0.1f;                         // т.е. значение пропорционально току
            }
            else if(devAddr == 1)
            {
                uint16_t temp = ((float)ain2/4095)*AIN_TEMP[MAX_SIZE_AIN_TEMP - 1][2];
                t.number = Get_Temp(temp, 2);
            }
            
            packet->buffer[4] = t.byte[0];
            packet->buffer[5] = t.byte[1];
            packet->buffer[6] = t.byte[2];
            packet->buffer[7] = t.byte[3];
        
            t.number = temp/1000.0f;
        
            packet->buffer[8]  = t.byte[0];
            packet->buffer[9]  = t.byte[1];
            packet->buffer[10] = t.byte[2];
            packet->buffer[11] = t.byte[3];
        
            packet->buffer[12] = 0x00;
            packet->buffer[13] = 0x00;
            packet->buffer[14] = 0x00;
            packet->buffer[15] = 0x00;
        
            packet->size = 16;
        break;
            
        case 0x03: // чтение регистра расширения дискретных каналов входов
            for(uint8_t i = 0; i < io_in->size; ++i)
            {
                if(bit_count == 8)
                {
                    bit_count = 0;
                    packet->buffer[++packet->size] = 0x00;
                }
                
                uint8_t channel_state = (io_in->list[i].state == true)?0x01:0x00;
                
                packet->buffer[packet->size] |= channel_state << bit_count++;
            }
            
            packet->size = 3;
        break;
            
        case 0x04: // чтение регистра расширения дискретных каналов выходов
            for(uint8_t i = 0; i < io_out->size; ++i)
            {
                out = &io_out->list[i];
                
                if(bit_count == 8)
                {
                    bit_count = 0;
                    packet->buffer[++packet->size] = 0x00;
                }
                
                uint8_t channel_state = 0x00;
                
                if(out->state == OUTPUT_STATE_ON)
                {
                    channel_state = 0x01;
                }
                else if(out->state == OUTPUT_STATE_FREQ_2HZ)
                {
                    channel_state = 0x02;
                }
                else if(out->state == OUTPUT_STATE_RESERVE)
                {
                    channel_state = 0x03;
                }
                
                packet->buffer[packet->size] |= channel_state << bit_count;
                
                bit_count += 2;
            }
                
            packet->size = 3;
        break;
            
        case 0x05: // запись регистра расширения дискретных каналов выходов
            for(uint8_t i = 0; i < data->size; ++i)
            {
                byte = data->buffer[i]; // текущий байт
                
                for(uint8_t j = 0; j < 8; j += 2) // 8 бит по 2 на описание каждого канала
                {
                    state = (byte >> j)&0x03; // состояние текущего канал
                    n_out = i*4 + j/2; // порядковый номер канала
                    out   = &io_out->list[n_out];
                    
                    switch(state)
                    {
                        case OUTPUT_STATE_OFF: // отключение выхода
                            if(out->state == OUTPUT_STATE_OFF)
                            {
                                break;
                            }
                            else if(out->state == OUTPUT_STATE_FREQ_2HZ || out->state == OUTPUT_STATE_RESERVE)
                            {
                                EVENT_Kill(out->param);
                            }
                            
                            out->state = OUTPUT_STATE_OFF;
                            
                            IO_Set(*out, false);
                        break;
                        
                        case OUTPUT_STATE_ON: // включение выхода
                            if(out->state == OUTPUT_STATE_ON)
                            {
                                break;
                            }
                            else if(out->state == OUTPUT_STATE_FREQ_2HZ || out->state == OUTPUT_STATE_RESERVE)
                            {
                                EVENT_Kill(out->param);
                                IO_Set(*out, false);
                            }
                            
                            out->state = OUTPUT_STATE_ON;
                            IO_Set(*out, true);
                        break;
                        
                        case OUTPUT_STATE_FREQ_2HZ: // включение выхода с альтернативной функцией
                        case OUTPUT_STATE_RESERVE:
                            if(out->state == OUTPUT_STATE_FREQ_2HZ || out->state == OUTPUT_STATE_RESERVE)
                            {
                                break;
                            }
                            else if(out->state == OUTPUT_STATE_ON)
                            {
                                IO_Set(*out, false);
                            }
                            
                            out->state = OUTPUT_STATE_FREQ_2HZ;
                            
                            out->param = EVENT_Create(1000, true, blink2Hz, out->pin.gpio, out->pin.pin, 0xFF);
                        break;
                    }
                }
            }
        break;
        
        case 0x06: // установка значения 0 на выходе канала 0
            CHANNEL_Out_Set(0, false);
        break;
            
        case 0x07: // установка значения 0 на выходе канала 1
            CHANNEL_Out_Set(1, false);
        break;
            
        case 0x08: // установка значения 0 на выходе канала 2
            CHANNEL_Out_Set(2, false);
        break;
            
        case 0x09: // установка значения 0 на выходе канала 3
            CHANNEL_Out_Set(3, false);
        break;
            
        case 0x0A: // установка значения 0 на выходе канала 4
            CHANNEL_Out_Set(4, false);
        break;
            
        case 0x0B: // установка значения 0 на выходе канала 5
            CHANNEL_Out_Set(5, false);
        break;
            
        case 0x0C: // установка значения 0 на выходе канала 6
            CHANNEL_Out_Set(6, false);
        break;
            
        case 0x0D: // установка значения 0 на выходе канала 7
            CHANNEL_Out_Set(7, false);
        break;
            
        case 0x0E: // установка значения 1 на выходе канала 0
            CHANNEL_Out_Set(0, true);
        break;
            
        case 0x0F: // установка значения 1 на выходе канала 1
            CHANNEL_Out_Set(1, true);
        break;
            
        case 0x10: // установка значения 1 на выходе канала 2
            CHANNEL_Out_Set(2, true);
        break;
            
        case 0x11: // установка значения 1 на выходе канала 3
            CHANNEL_Out_Set(3, true);
        break;
            
        case 0x12: // установка значения 1 на выходе канала 4
            CHANNEL_Out_Set(4, true);
        break;
            
        case 0x13: // установка значения 1 на выходе канала 5
            CHANNEL_Out_Set(5, true);
        break;
            
        case 0x14: // установка значения 1 на выходе канала 6
            CHANNEL_Out_Set(6, true);
        break;
            
        case 0x15: // установка значения 1 на выходе канала 7
            CHANNEL_Out_Set(7, true);
        break;
            
        case 0x1F: // чтение времени срабатывания выделенного входного дискретного канала
            if(devAddr == 0x00) // если модуль МДВВ-01
            {
                if(pwr_ok.dsdin_lev_changed)
                {
                    if(pwr_ok.dsdin_level)
                        packet->buffer[0] = DSDIN_TRIGGER_ON_1;
                    else
                        packet->buffer[0] = DSDIN_TRIGGER_ON_0;
                }
                else
                    packet->buffer[0] = DSDIN_TRIGGER_OFF;
                
                union
                {
                    uint16_t time;
                    uint8_t  buffer[2];
                } dsdin_time;
                
                dsdin_time.time = pwr_ok.dsdin_time;
                
                // установка для теста - должно вернуть 0xA13C011E
                //packet->buffer[0] = DSDIN_TRIGGER_ON_0;
                //dsdin_time.time = 316;
                
                packet->buffer[1] = dsdin_time.buffer[0];
                packet->buffer[2] = dsdin_time.buffer[1];
            }
            else // иначе, если другие модули
            {
                packet->buffer[0] = DSDIN_FUNCTION_NOT_SUPPORT;
                packet->buffer[1] = 0x00;
                packet->buffer[2] = 0x00;
            }
            
            packet->size = 3;
        break;
            
        case 0x3E: // изменение параметров фильтрации
            if(data->size == 3)
            {
                io_in->set.Nperiod  = data->buffer[0]; // количество периодов фильтрации
                io_in->set.Ndiscret = data->buffer[1]; // количество выборок на период
                io_in->set.SGac     = data->buffer[2]; // длительность сигнала считаемая, что сигнал валидный
                
                TIM_Scan_Update();
            }
        break;
            
        case 0x3F: // изменение входа
            if(data->size == 4)
            {
                uint8_t in_num = data->buffer[0]; // номер настраиваемого входа
                io_in->list[in_num].mode  = data->buffer[1]; // режим работы входа AC или DC
              
                io_in->list[in_num].duration = data->buffer[2]; // длительность периода
                
                if(io_in->list[in_num].mode == IN_MODE_AC)
                {
                    io_in->list[in_num].fault = data->buffer[3]; // погрешность допускаемая за один период - в процентах
                }
                else
                {
                    io_in->set.P0dc = io_in->set.P1dc = data->buffer[3];
                }
            }
        break;
        
        default: 
            return false;
    };
    
    return true;
}
//-----------------------------------------------------
uint8_t DEV_Checksum(FS9Packet_t* packet, uint8_t size)
{
    uint8_t checksum = 0;
    
    for(uint8_t i = 0; i < size; ++i)
    {
        checksum += packet->buffer[i];
    }
    
    checksum += size;
    checksum ^= 0xFF;
    
    return checksum;
}
//-----------------------
void DEV_Input_Scan(void)
{
    if(pwr_ok.is_dsdin == false)
    {
        for(uint8_t i = 0; i < io_in->size; ++i)
        {
            DEV_Input_Filter(i);
        }
    }
    else
    {
        DEV_Input_Filter(4);
    }
    
    if(Input_Changed)
    {
        TIM_INT_Start();
    }
}
//----------------------------------
void DEV_Input_Filter(uint8_t index)
{
    input_t* input = &io_in->list[index];
    
    bool in_state  = input->pin.gpio->IDR & input->pin.pin;
    bool act_level = !input->state; // ожидаемый уровень (при выключенном входе - лог "1", при включенном лог "0")
    
    // если уровень на входе равен ожидаемому уровню (обратное значение от состояния входа)
    if(in_state == act_level && input->filter.is_capture == false)
    {
        input->filter.is_capture = true; // захватываем вход
        
        if(in_state == true)
            input->filter.c_lev_1++;
        else
            input->filter.c_lev_0++;
        
        input->filter.c_clock++;
    }
    else if(input->filter.is_capture == true) // если вход захвачен, то набираем данные для фильтрации
    {
        if(in_state == true)
            input->filter.c_lev_1++;
        else
            input->filter.c_lev_0++;
        
        input->filter.c_clock++;
        
        if(input->filter.c_clock >= input->duration) // набрали данные на очередной период
        {
            input->filter.c_period++;
            
            if(input->mode == IN_MODE_AC) // режим входа АС
            {
                // если ожидаемый уровень лог "1" и количество лог "0" равно нулю, то это либо постоянный
                // сигнал, либо сигнал с частотой меньше установленно, т.е. это ошибка
                if(act_level == true && input->filter.c_lev_0 == 0)
                {
                    input->filter.c_error++;

                    input->filter.c_clock = 0;
                    input->filter.c_lev_0 = 0;
                    input->filter.c_lev_1 = 0;

                    return;
                }
                
                // если ожидаемый уровень лог "1" и длительность сигнала меньше пороговой, то это ошибка
                if(act_level == true && input->filter.c_lev_1 < io_in->set.SGac)
                {
                    input->filter.c_error++;

                    input->filter.c_clock = 0;
                    input->filter.c_lev_0 = 0;
                    input->filter.c_lev_1 = 0;

                    return;
                }
                
                // если ожидаемый уровень лог "0" (снятие сигнала со входа) и длительность лог "1"
                // больше нуля, то значит это не снятие сигнала и нет смысла дальше анализировать
                if(act_level == false && input->filter.c_lev_1 > 0)
                {
                    input->filter.c_clock    = 0;
                    input->filter.c_error    = 0;
                    input->filter.c_lev_0    = 0;
                    input->filter.c_lev_1    = 0;
                    input->filter.c_period   = 0;
                    input->filter.c_state    = 0;
                    input->filter.is_capture = false;
                    
                    return;
                }
                
                uint16_t tdur = (act_level == true)?(input->filter.c_lev_0 + input->filter.c_lev_1):
                                                     input->filter.c_lev_0;
                
                uint16_t tfault     = input->duration*input->fault/100;
                uint16_t trange_beg = input->duration - tfault;
                uint16_t trange_end = input->duration + tfault;
                
                if(tdur >= trange_beg && tdur <= trange_end)
                {
                    input->filter.c_state++;
                }
                else
                    input->filter.c_error++;
            }
            else if(input->mode == IN_MODE_DC) // режим входа DC
            {
                uint16_t tfault_lev = (act_level == true)?input->duration*io_in->set.P1dc:
                                                          input->duration*io_in->set.P0dc;
                uint16_t tdur = (act_level == true)?input->filter.c_lev_1:input->filter.c_lev_0;
                
                if(tdur >= tfault_lev)
                    input->filter.c_state++;
            }
            
            input->filter.c_clock = 0;
            input->filter.c_lev_0 = 0;
            input->filter.c_lev_1 = 0;
        }
        
        if(input->filter.c_period >= io_in->set.Nperiod) // конец фильтрации - принятие решения
        {
            if(input->filter.c_state >= (io_in->set.Nperiod - 1))
            {
                input->error  = false;
                input->state  = act_level;
                Input_Changed = true;
                
                if(pwr_ok.is_dsdin == true)
                {
                    pwr_ok.dsdin_level       = input->state;
                    pwr_ok.dsdin_lev_changed = true;
                    pwr_ok.dsdin_time        = TIM14->CNT;
                    
                    TIM14->CR1 &= ~TIM_CR1_CEN;
                }
            }
            else if(input->filter.c_error >= io_in->set.Nperiod)
                input->error = true;
            
            input->filter.c_clock    = 0;
            input->filter.c_error    = 0;
            input->filter.c_lev_0    = 0;
            input->filter.c_lev_1    = 0;
            input->filter.c_period   = 0;
            input->filter.c_state    = 0;
            input->filter.is_capture = false;
        }
    }
}
//------------------------------
void DEV_Input_Set_Default(void)
{
    io_in->set.Ndiscret = 10;
    io_in->set.Nperiod  = 3;
    io_in->set.SGac     = 5;
    io_in->set.P0dc     = 50;
    io_in->set.P1dc     = 50;
    
    for(uint8_t i = 0; i < io_in->size; ++i)
    {
        io_in->list[i].mode              = IN_MODE_AC;
        io_in->list[i].fault             = 10;
        io_in->list[i].state             = false;
        io_in->list[i].error             = false;
        io_in->list[i].duration          = 10;
        io_in->list[i].filter.c_clock    = 0;
        io_in->list[i].filter.c_period   = 0;
        io_in->list[i].filter.c_state    = 0;
        io_in->list[i].filter.c_error    = 0;
        io_in->list[i].filter.is_capture = false;
    }
}
//----------------------------------
bool DEV_Input_Changed_Channel(void)
{
    return Input_Changed;
}
//-------------------------
void TIM16_IRQHandler(void)
{
    if((TIM16->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        DEV_Input_Scan();
        
        TIM16->SR &= ~TIM_SR_UIF;
    }
}
//-------------------------
void TIM17_IRQHandler(void)
{
    if((TIM17->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        GPIO_INT->BSRR |= GPIO_INT_SET; // отключить выход INT
        
        Input_Changed = false;
        
        TIM17->SR &= ~TIM_SR_UIF;
    }
}
//------------------------------------------
float Get_Temp(uint16_t val, uint8_t in_num)
{
    float Rt = UAIN_to_TResistance(val, in_num);
    
    if(Rt == -100.0f)
        return Rt;
    
    float Pt100 = 3383.8098f - 8658.0088f*sqrtf(0.1758481f - 0.000231f*Rt);
    
    return Pt100;
}
//-----------------------------------------------------
float UAIN_to_TResistance(uint16_t val, uint8_t in_num)
{
    // значение val приходит умноженное на 10 (значение ацп целочисленное, т.е. ацп*1000), т.к. в таблице значения
    // хранятся умноженные на 10000 для приведения к целочисленному виду
    float res_beg = 0;
    float res_end = 0;
    float ain_beg = 0;
    float ain_end = 0;
    
    for(uint8_t i = 1; i < 19; ++i)
    {
        if(AIN_TEMP[i - 1][in_num] == val)
            return AIN_TEMP[i - 1][0];
        else if(val > AIN_TEMP[i - 1][in_num] && val < AIN_TEMP[i][in_num])
        {
            res_beg = AIN_TEMP[i - 1][0];
            res_end = AIN_TEMP[i][0];
            ain_beg = AIN_TEMP[i - 1][in_num];
            ain_end = AIN_TEMP[i][in_num];
            
            break;
        }
    }
    
    if(res_beg == 0 && res_end == 0)
        return -100.0f;
    
    float Rt = res_beg + ((val - ain_beg)/(ain_end - ain_beg))*(res_end - res_beg);
    
    return Rt;
}
//----------------------------
void EXTI4_15_IRQHandler(void)
{
    if(EXTI->PR & GPIO_PWROK_PIN)
    {
        EXTI->PR |= GPIO_PWROK_PIN;
        
        pwr_ok.is_pwrok = true;
        
        if(pwr_ok.is_dsdin == true)
        {
            pwr_ok.is_dsdin          = false;
            pwr_ok.dsdin_level       = false;
            pwr_ok.dsdin_lev_changed = false;
            pwr_ok.dsdin_time        = 0;
            
            TIM14->ARR  = 11 - 1;
            TIM14->CR1 &= ~TIM_CR1_OPM;
        }
        
        TIM14->EGR |= TIM_EGR_UG;
    }
}
//-------------------------
void TIM14_IRQHandler(void)
{
    if(TIM14->SR & TIM_SR_UIF)
    {
        TIM14->SR &= ~TIM_SR_UIF;
        
        if(pwr_ok.is_pwrok == false && pwr_ok.is_dsdin == false) // активация алгоритма обработки
        {                                                        // отключения внешнего питания
            pwr_ok.is_dsdin = true;
            
            TIM14->ARR   = 500 - 1; // запуск таймера на 500мс
            TIM14->CR1  |= TIM_CR1_OPM; // в режиме одного импульса
            TIM14->DIER &= TIM_DIER_UIE;
            TIM14->EGR  |= TIM_EGR_UG;
            TIM14->SR   &= ~TIM_SR_UIF;
            TIM14->DIER |= TIM_DIER_UIE;
        }
        else
        {
            pwr_ok.is_pwrok = false;
        }
    }
}
//---------------------------------------------
void blink2Hz(GPIO_TypeDef* gpio, uint16_t pin)
{
    if((gpio->ODR & pin) == pin)
    {
        gpio->ODR &= ~pin;
    }
    else
    {
        gpio->ODR |= pin;
    }
}
//------------------------------------------
void crash(GPIO_TypeDef* gpio, uint16_t pin)
{
    if(is_crash == true) // запрос пришел
    {
        is_crash = false;
    }
    else // запроса нет - отключаем выход
    {
        IO_Set(out_crash, false);
    }
}
