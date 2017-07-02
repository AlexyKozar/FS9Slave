#include "device.h"
//---------------------------------------
void IO_Clock_Enable(GPIO_TypeDef* gpio);
void IO_Init(GPIO_TypeDef* gpio, uint16_t io, uint8_t io_dir);
//------------------------
struct IO_Type* io_inputs;
struct IO_Type* io_outputs;
//---------------------
uint8_t devAddr = 0xFF;
//----------------------------------------------------
void DEV_Create(GPIO_TypeDef* gpio, uint16_t pin_addr)
{
    IO_Clock_Enable(gpio);
    IO_Init(gpio, pin_addr, DEV_IO_INPUT);
    
    devAddr = (uint8_t)(gpio->IDR & pin_addr) >> 14; // set the address device
}
//------------------------------------------------------------
void DEV_Init(struct IO_Type* inputs, struct IO_Type* outputs)
{
    io_inputs  = inputs;
    io_outputs = outputs;
    
    uint16_t in  = 0x0000;
    uint16_t out = 0x0000;
    
    for(uint8_t i = 0; i < io_inputs->size; ++i)
    {
        in |= io_inputs->io[i];
    }
    
    for(uint8_t i = 0; i < io_outputs->size; ++i)
    {
        out |= io_outputs->io[i];
    }
    
    IO_Clock_Enable(io_inputs->gpio);
    IO_Clock_Enable(io_outputs->gpio);
    
    IO_Init(io_inputs->gpio, in, DEV_IO_INPUT);
    IO_Init(io_outputs->gpio, out, DEV_IO_OUTPUT);
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
    
    if(tcmd.n == 0)
        return false; // command is not valid
    
    uint8_t checksum = DEV_Checksum(source, source->size - 1);
    
    if(checksum != source->buffer[source->size - 1])
        return false;
    
    if(!DEV_Driver(cmd, dest))
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
                if(io_outputs->gpio->ODR & io_outputs->io[0])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->io[0];
                }
            }
        break;
            
        case 0x07: // set level low on channel 1
            if(io_outputs->size > 1)
            {
                if(io_outputs->gpio->ODR & io_outputs->io[1])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->io[1];
                }
            }
        break;
            
        case 0x08: // set level low on channel 2
            if(io_outputs->size > 2)
            {
                if(io_outputs->gpio->ODR & io_outputs->io[2])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->io[2];
                }
            }
        break;
            
        case 0x09: // set level low on channel 3
            if(io_outputs->size > 3)
            {
                if(io_outputs->gpio->ODR & io_outputs->io[3])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->io[3];
                }
            }
        break;
            
        case 0x0A: // set level low on channel 4
            if(io_outputs->size > 4)
            {
                if(io_outputs->gpio->ODR & io_outputs->io[4])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->io[4];
                }
            }
            break;
            
        case 0x0B: // set level low on channel 5
            if(io_outputs->size > 5)
            {
                if(io_outputs->gpio->ODR & io_outputs->io[5])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->io[5];
                }
            }
        break;
            
        case 0x0C: // set level low on channel 6
            if(io_outputs->size > 6)
            {
                if(io_outputs->gpio->ODR & io_outputs->io[6])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->io[6];
                }
            }
        break;
            
        case 0x0D: // set level low on channel 7
            if(io_outputs->size > 7)
            {
                if(io_outputs->gpio->ODR & io_outputs->io[7])
                {
                    io_outputs->gpio->ODR &= ~io_outputs->io[7];
                }
            }
        break;
            
        case 0x0E: // set level high on channel 0
            if(io_outputs->size > 0)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->io[0]))
                {
                    io_outputs->gpio->ODR |= io_outputs->io[0];
                }
            }
        break;
            
        case 0x0F: // set level high on channel 1
            if(io_outputs->size > 1)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->io[1]))
                {
                    io_outputs->gpio->ODR |= io_outputs->io[1];
                }
            }
        break;
            
        case 0x10: // set level high on channel 2
            if(io_outputs->size > 2)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->io[2]))
                {
                    io_outputs->gpio->ODR |= io_outputs->io[2];
                }
            }
        break;
            
        case 0x11: // set level high on channel 3
            if(io_outputs->size > 3)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->io[3]))
                {
                    io_outputs->gpio->ODR |= io_outputs->io[3];
                }
            }
        break;
            
        case 0x12: // set level high on channel 4
            if(io_outputs->size > 4)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->io[4]))
                {
                    io_outputs->gpio->ODR |= io_outputs->io[4];
                }
            }
        break;
            
        case 0x13: // set level high on channel 5
            if(io_outputs->size > 5)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->io[5]))
                {
                    io_outputs->gpio->ODR |= io_outputs->io[5];
                }
            }
        break;
            
        case 0x14: // set level high on channel 6
            if(io_outputs->size > 6)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->io[6]))
                {
                    io_outputs->gpio->ODR |= io_outputs->io[6];
                }
            }
        break;
            
        case 0x15: // set level high on channel 7
            if(io_outputs->size > 7)
            {
                if(!(io_outputs->gpio->ODR & io_outputs->io[7]))
                {
                    io_outputs->gpio->ODR |= io_outputs->io[7];
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
