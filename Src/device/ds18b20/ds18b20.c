#include "ds18b20.h"
//----------------------
#define CMD_CONVERT 0x44
#define CMD_READ    0xBE
#define CMD_SKIP    0xCC
//----------------------
void     DQ_input(void);
void     DQ_output(void);
void     DQ_set(void);
void     DQ_reset(void);
uint16_t DQ_read(void);
bool     DQ_read_bit(void);
bool     DQ_state(void);
void     DQ_send(uint8_t cmd);
void     DQ_send_zero(void);
void     DQ_send_one(void);
bool     init(void); // reset and presence (true - if is valid)
void     delay_us(uint32_t time);
//--------------------------
bool     is_process = false;
uint16_t temp       = 0xFFFF;
//---------------------
void DS18B20_Init(void)
{
    // enable clock timer and port
    RCC->AHBENR |= DS18B20_GPIO_BUS;
    
    if(DS18B20_TIM == TIM3 || DS18B20_TIM == TIM14)
    {
        RCC->APB1ENR |= DS18B20_TIM_BUS;
    }
    else
    {
        RCC->APB2ENR |= DS18B20_TIM_BUS;
    }
    
    DQ_output();
    DQ_set();
    
    DS18B20_TIM->PSC   = 6 - 1; // 125ns
    DS18B20_TIM->CR1  |= TIM_CR1_ARPE;
    DS18B20_TIM->CR1  |= TIM_CR1_OPM;
    DS18B20_TIM->DIER &= ~TIM_DIER_UIE;
    
    NVIC_EnableIRQ(DS18B20_IRQ);
}
//-------------------------------
void DS18B20_Convert(void* param)
{
    
    if(init()) // receive the impulse presence
    {
        DQ_send(CMD_SKIP);
        
        if(is_process == false)
        {
            is_process = true;
            
            DQ_send(CMD_CONVERT);
            
            EVENT_Create(750, false, DS18B20_Convert, NULL, 0xFF);
        }
        else
        {
            is_process = false;
            
            DQ_send(CMD_READ);
            temp = DQ_read();
            
            EVENT_Create(5000, false, DS18B20_Convert, NULL, 0xFF);
            
            //init(); // reset (we read two bytes of nine bytes)
        }
    }
}
//-----------------------------
float DS18B20_Temperature(void)
{
    float t = 0.0f;
    
    if(temp == 0xFFFF) // read error
    {
        return 126.0f;
    }
    else if(temp == 0x5005) // sensor error
    {
        return 127.0f;
    }
    else
    {
        bool    sign    = temp&0xF800; // temperature sign (true - minus)
        uint8_t fract   = temp&0x000F; // fractional part
        uint8_t integer = (temp&0x07F0) >> 4; // integer part
        
        if(sign) // negative value temperature
        {
            fract   = 0 - fract;
            integer = 0 - integer;
        }
        
        t = integer + ((float)(fract >> 4));
    }
    
    return t;
}
//-----------------
void DQ_input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Pin   = DS18B20_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    
    HAL_GPIO_Init(DS18B20_GPIO, &GPIO_InitStruct);
}
//------------------
void DQ_output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Pin   = DS18B20_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    
    HAL_GPIO_Init(DS18B20_GPIO, &GPIO_InitStruct);
}
//---------------
void DQ_set(void)
{
    DS18B20_GPIO->BSRR |= DS18B20_PIN;
}
//-----------------
void DQ_reset(void)
{
    DS18B20_GPIO->BSRR |= DS18B20_PIN << 16;
}
//--------------------
uint16_t DQ_read(void)
{
    uint16_t data = 0x0000;
    
    for(uint8_t i = 0; i < 16; ++i)
    {
        if(DQ_read_bit() == true)
        {
            data |= 1 << i;
        }
        else
        {
            data |= 0 << i;
        }
    }
    
    return data;
}
//--------------------
bool DQ_read_bit(void)
{
    DQ_output();
    DQ_reset();
    delay_us(1);
    DQ_set();
    DQ_input();
    delay_us(7);
    
    bool state = DQ_state();
    
    delay_us(45);
    
    return state;
}
//-----------------
bool DQ_state(void)
{
    return (DS18B20_GPIO->IDR & DS18B20_PIN);
}
//-----------------------
void DQ_send(uint8_t cmd)
{
    DQ_output();
    
    for(uint8_t i = 0; i < 8; ++i)
    {
        uint8_t bit = (cmd >> i)&0x01;
        
        if(bit == 0)
        {
            DQ_send_zero();
        }
        else
        {
            DQ_send_one();
        }
    }
}
//---------------------
void DQ_send_zero(void)
{
    DQ_reset();   
    delay_us(80);
    DQ_set();
    delay_us(5);
}
//--------------------
void DQ_send_one(void)
{
    DQ_reset();    
    delay_us(15);
    DQ_set();
    delay_us(75);
}
//-------------
bool init(void)
{
    DQ_output(); // pin to out
    DQ_reset(); // set the low level
    delay_us(500); // the impulse reset
    DQ_set();
    DQ_input(); // pin to input
    delay_us(60); // wait impulse presence
    
    bool state = !DQ_state();
    
    delay_us(400); // end time slot
    
    return state;
}
//--------------------------
void delay_us(uint32_t time)
{
    DS18B20_TIM->ARR  = (time << 3) - 1; // multiplied by 8
    DS18B20_TIM->EGR |= TIM_EGR_UG;
    DS18B20_TIM->SR  &= ~TIM_SR_UIF;
    DS18B20_TIM->CR1 |= TIM_CR1_CEN;
    
    while(!(DS18B20_TIM->SR & TIM_SR_UIF));
}
