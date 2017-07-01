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
