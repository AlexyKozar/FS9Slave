#include "device.h"
//---------------------------------------
void IO_Clock_Enable(GPIO_TypeDef* gpio);
void IO_Init(GPIO_TypeDef* gpio, uint16_t io, uint8_t io_dir);
void TIM_Scan_Init(void);
void TIM_Scan_Update(void);
void TIM_INT_Init(void);
void TIM_INT_Start(void);
float Get_Temp(uint16_t val, uint8_t in_num);
float UAIN_to_TResistance(uint16_t val, uint8_t in_num); // преобразование напряжения в сопротивление температуры
//---------------------------------
struct PORT_Input_Type*  io_inputs;
struct PORT_Output_Type* io_outputs;
//---------------------
uint8_t devAddr = 0xFF;
//-------------------------
bool Input_Changed = false;
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
    IO_Clock_Enable(gpio);
    IO_Init(gpio, addr_pins, DEV_IO_INPUT);
    
    devAddr = (uint8_t)((gpio->IDR & addr_pins) >> 14); // set the address device
}
//-----------------------------------------------------------------------------
void DEV_Init(struct PORT_Input_Type* inputs, struct PORT_Output_Type* outputs)
{
    io_inputs  = inputs;
    io_outputs = outputs;
    
    uint16_t in  = 0x0000;
    uint16_t out = 0x0000;
    
    for(uint8_t i = 0; i < io_inputs->size; ++i)
    {
        in |= io_inputs->in_arr[i].pin;
    }
    
    for(uint8_t i = 0; i < io_outputs->size; ++i)
    {
        out |= io_outputs->out_arr[i];
    }
    
    IO_Clock_Enable(io_inputs->gpio);
    IO_Clock_Enable(io_outputs->gpio);
    IO_Clock_Enable(GPIOB); // подключение тактирования для вывода INT
    
    IO_Init(io_inputs->gpio, in, DEV_IO_INPUT);
    IO_Init(io_outputs->gpio, out, DEV_IO_OUTPUT);
    IO_Init(GPIO_INT, GPIO_INT_PIN, DEV_IO_OUTPUT); // вывод INT как выход
    
    GPIO_INT->BSRR |= GPIO_INT_SET; // включить выход INT (default state)
    
    DEV_Input_Set_Default();
    
    TIM_Scan_Init();
    TIM_INT_Init();
    
    AIN_Init();
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
//-----------------------------------------------------------
void IO_Init(GPIO_TypeDef* gpio, uint16_t io, uint8_t io_dir)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin  = io;
    GPIO_InitStruct.Mode = (io_dir == 0x01)?GPIO_MODE_OUTPUT_PP:GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);
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
    TIM16->ARR   = 10000/io_inputs->in_set.Dac - 1;
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
//-----------------------
uint8_t DEV_Address(void)
{
    return devAddr;
}
//--------------------------------------------------------------------
bool DEV_Request(struct FS9Packet_t* source, struct FS9Packet_t* dest)
{
    uint8_t addr = ((source->buffer[0]&CMD_ADDR_MASK) >> 6);
    
    if(addr != devAddr)
        return false;
    
    uint8_t cmd = source->buffer[0]&CMD_CODE_MASK; // get the command for device
    
    struct cmd_t tcmd = CMD_get(cmd); // verification command
    
    if(tcmd.n == 0 || tcmd.m == 0)
        return false; // command is not valid
    
    uint8_t checksum = DEV_Checksum(source, source->size - 1);
    
    if(checksum != source->buffer[source->size - 1])
        return false;
    
    struct FS9Packet_t packet;
    
    packet.size = source->size - 2;
    
    for(uint8_t i = 0; i < packet.size; ++i)
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
//--------------------------------------------------------------------------------
bool DEV_Driver(uint8_t cmd, struct FS9Packet_t* data, struct FS9Packet_t* packet)
{
    uint8_t bit_count = 0; // счетчик бит (позиция канала в байте)
    
    int32_t  temp;
    uint16_t ain1;
    uint16_t ain2;
    
    switch(cmd)
    {
        case 0x00: // чтение дискретных каналов входов
            for(uint8_t i = 0; i < io_inputs->size; ++i)
            {
                if(bit_count == 8)
                {
                    bit_count = 0;
                    packet->size++;
                    packet->buffer[packet->size] = 0x00;
                }
                
                uint8_t channel_state = 0x00;
                
                if(io_inputs->in_arr[i].state == true && io_inputs->in_arr[i].error == false)
                {
                    channel_state = 0x01;
                }
                else if(io_inputs->in_arr[i].error == true)
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
        
            for(uint8_t i = 0; i < io_outputs->size; ++i)
            {
                uint16_t pin = io_outputs->out_arr[i];
                
                if((io_outputs->gpio->ODR & pin) == pin)
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
                float    k    = 5.0f/(vdda/1000.0f);
                
                t.number = ((float)(ain1/4095*vdda/1000.0f))*k;
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
                t.number = ((float)ain2/4095)*vdda/1000.0f;
                t.number /= 0.1f;
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
        
        case 0x06: // установка значения 0 на выходе канала 0
            if(io_outputs->size > 0)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[0])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[0];
                }
            }
        break;
            
        case 0x07: // установка значения 0 на выходе канала 1
            if(io_outputs->size > 1)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[1])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[1];
                }
            }
        break;
            
        case 0x08: // установка значения 0 на выходе канала 2
            if(io_outputs->size > 2)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[2])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[2];
                }
            }
        break;
            
        case 0x09: // установка значения 0 на выходе канала 3
            if(io_outputs->size > 3)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[3])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[3];
                }
            }
        break;
            
        case 0x0A: // установка значения 0 на выходе канала 4
            if(io_outputs->size > 4)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[4])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[4];
                }
            }
            break;
            
        case 0x0B: // установка значения 0 на выходе канала 5
            if(io_outputs->size > 5)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[5])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[5];
                }
            }
        break;
            
        case 0x0C: // установка значения 0 на выходе канала 6
            if(io_outputs->size > 6)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[6])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[6];
                }
            }
        break;
            
        case 0x0D: // установка значения 0 на выходе канала 7
            if(io_outputs->size > 7)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[7])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[7];
                }
            }
        break;
            
        case 0x0E: // установка значения 1 на выходе канала 0
            if(io_outputs->size > 0)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[0]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[0];
                }
            }
        break;
            
        case 0x0F: // установка значения 1 на выходе канала 1
            if(io_outputs->size > 1)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[1]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[1];
                }
            }
        break;
            
        case 0x10: // установка значения 1 на выходе канала 2
            if(io_outputs->size > 2)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[2]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[2];
                }
            }
        break;
            
        case 0x11: // установка значения 1 на выходе канала 3
            if(io_outputs->size > 3)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[3]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[3];
                }
            }
        break;
            
        case 0x12: // установка значения 1 на выходе канала 4
            if(io_outputs->size > 4)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[4]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[4];
                }
            }
        break;
            
        case 0x13: // установка значения 1 на выходе канала 5
            if(io_outputs->size > 5)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[5]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[5];
                }
            }
        break;
            
        case 0x14: // установка значения 1 на выходе канала 6
            if(io_outputs->size > 6)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[6]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[6];
                }
            }
        break;
            
        case 0x15: // установка значения 1 на выходе канала 7
            if(io_outputs->size > 7)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[7]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[7];
                }
            }
        break;
            
        case 0x3D: // изменение длительности сигнала
            if(data->size == 1)
            {
                io_inputs->in_set.Sdur = data->buffer[0];
            }
        break;
            
        case 0x3E: // изменение параметров фильтрации
            if(data->size == 3)
            {
                io_inputs->in_set.Nac = data->buffer[0]; // количество периодов фильтрации
                io_inputs->in_set.Dac = data->buffer[1]; // количество выборок на период
                io_inputs->in_set.SGac = data->buffer[2]; // длительность сигнала считаемая, что сигнал валидный
            }
        break;
            
        case 0x3F: // изменение входа
            if(data->size == 3)
            {
                uint8_t in_num = data->buffer[0]; // номер настраиваемого входа
                io_inputs->in_arr[in_num].mode  = data->buffer[1]; // режим работы входа AC или DC
                
                if(io_inputs->in_arr[in_num].mode == IN_MODE_AC)
                {
                    io_inputs->in_arr[in_num].fault = data->buffer[2]; // погрешность допускаемая за один период - в процентах
                }
                else
                {
                    io_inputs->in_set.P0dc = io_inputs->in_set.P1dc = data->buffer[2];
                }
            }
        break;
        
        default: 
            return false;
    };
    
    return true;
}
//------------------------------------------------------------
uint8_t DEV_Checksum(struct FS9Packet_t* packet, uint8_t size)
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
    for(uint8_t i = 0; i < io_inputs->size; ++i)
    {
        DEV_Input_Filter(i);
    }
    
    if(Input_Changed)
    {
        TIM_INT_Start();
    }
}
//----------------------------------
void DEV_Input_Filter(uint8_t index)
{
    struct INPUT_Type* input = &io_inputs->in_arr[index];
    
    bool in_state  = io_inputs->gpio->IDR & input->pin;
    bool act_level = !input->state; // ожидаемый уровень (при выключенном входе - лог "1", при включенном лог "0")
    
    // счетчик импульсов на окно - введен для работы со стендом, т.к. частота сигнала 50Гц вместо 100Гц
    uint8_t imp_count = io_inputs->in_set.Sdur/10*io_inputs->in_set.Dac;
    
    if(input->filter.is_capture == false && in_state == act_level) // если захвата не было и на входе ожидаемый уровень
    {
        input->filter.c_clock++;
        input->filter.is_capture = true;
        
        if(act_level == true)
            input->filter.c_high_lev++;
        else
            input->filter.c_low_lev++;
    }
    else if(input->filter.is_capture == true) // захват произведен - накопление данных
    {
        if(in_state == true)
            input->filter.c_high_lev++;
        else if(in_state == false)
            input->filter.c_low_lev++;
        
        input->filter.c_clock++;
        
        if(input->filter.c_clock >= imp_count) // счетчик тактирования равен длительности сигнала
        {
            uint8_t count = (act_level == true)?input->filter.c_high_lev:input->filter.c_low_lev;
            bool is_valid = false;
            
            if(input->mode == IN_MODE_AC)
            {
                uint8_t tcount = (act_level == true)?io_inputs->in_set.SGac:(imp_count - io_inputs->in_set.SGac);
                
                if(count >= tcount)
                {
                    is_valid = true;
                }
                else
                {
                    is_valid = false;
                }
            }
            else if(input->mode == IN_MODE_DC)
            {
                uint8_t tcount = (act_level == true)?io_inputs->in_set.P1dc:io_inputs->in_set.P0dc;
                
                if(count >= tcount)
                {
                    is_valid = true;
                }
                else
                {
                    is_valid = false;
                }
            }
            
            if(is_valid == true)
            {
                // расчет полученной длительности сигнала
                uint8_t duration;
                
                if(input->mode == IN_MODE_AC)
                {
                    duration = (act_level == true)?input->filter.c_high_lev + input->filter.c_low_lev:
                                                   input->filter.c_low_lev;
                }
                else
                {
                    duration = (act_level == true)?input->filter.c_high_lev:input->filter.c_low_lev;
                }
                
                // расчет пределов погрешности
                is_valid = false;
                
                if(input->mode == IN_MODE_AC)
                {
                    uint8_t dur_fault_beg = imp_count - (imp_count*input->fault)/100;
                    uint8_t dur_fault_end = imp_count + (imp_count*input->fault)/100;
                            
                    if(duration >= dur_fault_beg && duration <= dur_fault_end) // проверка сигнала на вхождение в пределы
                        is_valid = true;
                }
                else
                {
                    uint8_t fault = (act_level == true)?(duration*io_inputs->in_set.P1dc)/100:
                                                        (duration*io_inputs->in_set.P0dc)/100;
                    
                    if(duration >= fault) // проверка сигнала на вхождение в предел погрешности
                        is_valid = true;
                }
                
                if(is_valid == true) // проверка сигнала на вхождение в пределы
                {
                    input->filter.c_state++;
                }
                else
                {
                    input->filter.с_error++; // иначе ошибка канала в текущем периоде
                }
                
                input->filter.c_period++;
                
                input->filter.c_clock    = 0;
                input->filter.c_high_lev = 0;
                input->filter.c_low_lev  = 0;
            }
        }
        
        if(input->filter.c_period >= io_inputs->in_set.Nac) // количество прошедших периодов равно установленному
        {
            if(input->filter.c_state >= (io_inputs->in_set.Nac - 1)) // количество изменений состояний входа за
            {                                                        // прошедшие периоды равно (-1 - первый период не
                input->state = act_level;                            // считаем (списываем на помеху)
                
                if(Input_Changed == false) // выставляем флаг изменения на входах
                    Input_Changed = true;
            }
            else if(input->filter.с_error >= (io_inputs->in_set.Nac - 1)) // иначе, если счетчик ошибок равен
            {                                                              // количеству периодов - 1, то
                input->error = true;                                       // ошибка канала
                
                if(Input_Changed == false) // выставляем флаг изменения на входах
                    Input_Changed = true;
            }
            
            input->filter.c_clock    = 0;
            input->filter.с_error    = 0;
            input->filter.c_high_lev = 0;
            input->filter.c_low_lev  = 0;
            input->filter.c_period   = 0;
            input->filter.c_state    = 0;
            input->filter.is_capture = false;
        }
    }
}
//------------------------------
void DEV_Input_Set_Default(void)
{
    io_inputs->in_set.Nac  = 3;
    io_inputs->in_set.Dac  = 10;
    io_inputs->in_set.NSac = 4;
    io_inputs->in_set.SGac = 5;
    io_inputs->in_set.Sdur = 10;
    io_inputs->in_set.Ndc  = 3;
    io_inputs->in_set.Ddc  = 10;
    io_inputs->in_set.P0dc = 80;
    io_inputs->in_set.P1dc = 80;
    
    for(uint8_t i = 0; i < io_inputs->size; ++i)
    {
        io_inputs->in_arr[i].mode              = IN_MODE_AC;
        io_inputs->in_arr[i].fault             = 10;
        io_inputs->in_arr[i].state             = false;
        io_inputs->in_arr[i].error             = false;
        io_inputs->in_arr[i].filter.c_clock    = 0;
        io_inputs->in_arr[i].filter.c_period   = 0;
        io_inputs->in_arr[i].filter.c_state    = 0;
        io_inputs->in_arr[i].filter.с_error    = 0;
        io_inputs->in_arr[i].filter.is_capture = false;
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
