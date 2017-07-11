#include "device.h"
//---------------------------------------
void IO_Clock_Enable(GPIO_TypeDef* gpio);
void IO_Init(GPIO_TypeDef* gpio, uint16_t io, uint8_t io_dir);
//---------------------------------
struct PORT_Input_Type*  io_inputs;
struct PORT_Output_Type* io_outputs;
//---------------------
uint8_t devAddr = 0xFF;
//--------------------
uint8_t bit_count = 0; // the bit counter
//-----------------------
bool Input_Ready = false;
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
    
    IO_Init(io_inputs->gpio, in, DEV_IO_INPUT);
    IO_Init(io_outputs->gpio, out, DEV_IO_OUTPUT);
    
    DEV_Input_Set_Default();
    
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    
    TIM16->PSC   = 48 - 1;
    TIM16->ARR   = 1000 - 1;
    TIM16->DIER |= TIM_DIER_UIE;
    TIM16->CR1  |= TIM_CR1_CEN;
    
    NVIC_EnableIRQ(TIM16_IRQn);
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
    
    if(!DEV_Driver(cmd, dest))
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
//------------------------------------------------------
bool DEV_Driver(uint8_t cmd, struct FS9Packet_t* packet)
{
    switch(cmd)
    {
        case 0x06: // set level low on channel 0
            if(io_outputs->size > 0)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[0])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[0];
                }
            }
        break;
            
        case 0x07: // set level low on channel 1
            if(io_outputs->size > 1)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[1])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[1];
                }
            }
        break;
            
        case 0x08: // set level low on channel 2
            if(io_outputs->size > 2)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[2])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[2];
                }
            }
        break;
            
        case 0x09: // set level low on channel 3
            if(io_outputs->size > 3)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[3])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[3];
                }
            }
        break;
            
        case 0x0A: // set level low on channel 4
            if(io_outputs->size > 4)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[4])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[4];
                }
            }
            break;
            
        case 0x0B: // set level low on channel 5
            if(io_outputs->size > 5)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[5])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[5];
                }
            }
        break;
            
        case 0x0C: // set level low on channel 6
            if(io_outputs->size > 6)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[6])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[6];
                }
            }
        break;
            
        case 0x0D: // set level low on channel 7
            if(io_outputs->size > 7)
            {
                if(io_outputs->gpio->ODR & io_outputs->out_arr[7])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->out_arr[7];
                }
            }
        break;
            
        case 0x0E: // set level high on channel 0
            if(io_outputs->size > 0)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[0]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[0];
                }
            }
        break;
            
        case 0x0F: // set level high on channel 1
            if(io_outputs->size > 1)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[1]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[1];
                }
            }
        break;
            
        case 0x10: // set level high on channel 2
            if(io_outputs->size > 2)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[2]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[2];
                }
            }
        break;
            
        case 0x11: // set level high on channel 3
            if(io_outputs->size > 3)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[3]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[3];
                }
            }
        break;
            
        case 0x12: // set level high on channel 4
            if(io_outputs->size > 4)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[4]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[4];
                }
            }
        break;
            
        case 0x13: // set level high on channel 5
            if(io_outputs->size > 5)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[5]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[5];
                }
            }
        break;
            
        case 0x14: // set level high on channel 6
            if(io_outputs->size > 6)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[6]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[6];
                }
            }
        break;
            
        case 0x15: // set level high on channel 7
            if(io_outputs->size > 7)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->out_arr[7]))
                {
                    io_outputs->gpio->ODR |= io_outputs->out_arr[7];
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
    struct INPUT_Type* input = &io_inputs->in_arr[0]; // текущий вход
    bool is_active = (io_inputs->gpio->IDR & input->pin)?true:false; // текущее состоние входа
    bool lev_active = !input->state; // уровень который будет считаться активным
    
    if(input->is_capture == false && is_active == true) // если вход не захвачен
    {
        // устанавливаем флаг захвата и инкрементируем счетчики тактов и импульсов
        input->is_capture = true;
        input->clock++;
        input->impulse++;
    }
    else if(input->is_capture == true)
    {
        if(is_active == true)
        {
            input->impulse++;
        }
        else
        {
            if(input->impulse >= io_inputs->in_set.SGac)
            {
                input->state_period++;
                input->impulse = 0;
            }
        }
        
        if(input->clock >= io_inputs->in_set.Dac)
        {
            input->period++;
            input->clock = 0;
        }
        else
            input->clock++;
        
        if(input->period >= io_inputs->in_set.Nac)
        {
            if(input->impulse >= io_inputs->in_set.SGac)
            {
                input->state_period++;
            }
            
            if(input->state_period >= (input->period - 1))
            {
                input->state = true;
            }
            
            input->clock        = 0;
            input->impulse      = 0;
            input->is_capture   = false;
            input->period       = 0;
            input->state_period = 0;
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
    
    for(uint8_t i = 0; i < io_inputs->size; ++i)
    {
        io_inputs->in_arr[i].clock        = 0;
        io_inputs->in_arr[i].impulse      = 0;
        io_inputs->in_arr[i].period       = 0;
        io_inputs->in_arr[i].state_period = 0;
        io_inputs->in_arr[i].is_capture   = false;
    }
}
//------------------------
bool DEV_Input_Ready(void)
{
    return Input_Ready;
}
//-------------------------
void DEV_Input_Filter(void)
{
    
}
//---------------------------------
bool DEV_Input_Change_Channel(void)
{
    bool state = false;
    
    return state;
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
