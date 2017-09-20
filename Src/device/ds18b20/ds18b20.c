#include "ds18b20.h"
//---------------------
#define STATE_NONE 0x00
#define STATE_INIT 0x01
#define STATE_SKIP 0x02
#define STATE_CONV 0x04
#define STATE_READ 0x08
//------------------------
#define MODE_NONE     0x00
#define MODE_RESET    0x01
#define MODE_PRESENCE 0x02
#define MODE_CMD      0x04
#define MODE_CMD_ZERO 0x08
#define MODE_CMD_ONE  0x10
//-------------------
#define CMD_CONV 0x44
#define CMD_READ 0xBE
#define CMD_SKIP 0xCC
//---------------------
typedef struct _state_t
{
    uint8_t state;
    bool    is_ok;
} state_t;
//---------------------------
typedef struct _ds18b20_cmd_t
{
    uint8_t cmd;
    uint8_t bit_count;
    bool    is_bit;
} ds18b20_cmd_t;
//-----------------------
void DQ_in_setting(void);
void DQ_out_setting(void);
void DQ_in_falling(void);
void DQ_in_rising(void);
void DQ_out_set(void);
void DQ_out_reset(void);
void DQ_write_cmd(uint8_t byte);
void DQ_write_zero(void);
void DQ_write_one(void);
void TIM_set_capture(void);
void TIM_set_opm(uint16_t length);
void reset(void);
//--------------------------------------------
ds18b20_cmd_t cmd        = { 0xFF, 0, false };
state_t       state      = { STATE_NONE, false };
uint16_t      is_capture = false;
uint16_t      capture_1  = 0x0000;
uint16_t      capture_2  = 0x0000;
uint16_t      period     = 0x0000;
uint16_t      pause      = 0x0000;
uint8_t       mode       = MODE_NONE;
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
    
    // general settings a timer
    DS18B20_TIM->PSC  = 48 - 1;
    DS18B20_TIM->CR1 |= TIM_CR1_ARPE;
    
    // specific settings for input capture mode
    DS18B20_TIM->CCER  &= ~TIM_CCER_CC1E; // disable input capture channel
    DS18B20_TIM->CCMR1 |= TIM_CCMR1_CC1S_0; // link to input TI1
    DS18B20_TIM->CCMR1 &= ~TIM_CCMR1_IC1F; // disable filter
    DS18B20_TIM->CCER  &= ~TIM_CCER_CC1NP;
    DS18B20_TIM->CCMR1 &= ~TIM_CCMR1_IC1PSC; // disable input prescaler
    
    NVIC_EnableIRQ(DS18B20_IRQ);
    
    DQ_out_setting();
    DQ_out_set();
}
//------------------------
void DS18B20_Convert(void)
{    
    reset();
}
//--------------------------
void DS18B20_INTERRUPT(void)
{
    if((DS18B20_TIM->SR & TIM_SR_UIF) == TIM_SR_UIF)
    {
        if(mode == MODE_RESET)
        {
            DQ_in_setting();
            DQ_in_falling();
            TIM_set_capture();
        }
        else if(mode == MODE_PRESENCE)
        {
            DQ_write_cmd(CMD_SKIP);
        }
        else if((mode & MODE_CMD) == MODE_CMD)
        {   
            if((mode & MODE_CMD_ZERO) == MODE_CMD_ZERO)
            {
                if(cmd.is_bit == false)
                {
                    cmd.is_bit = true;
                    DQ_out_set();
                    TIM_set_opm(5);
                }
                else
                {
                    cmd.is_bit = false;
                    mode &= ~MODE_CMD_ZERO;
                }
            }
            else if((mode & MODE_CMD_ONE) == MODE_CMD_ONE)
            {
                if(cmd.is_bit == false)
                {
                    cmd.is_bit = true;
                    DQ_out_set();
                    TIM_set_opm(45);
                }
                else
                {
                    cmd.is_bit = false;
                    mode &= ~MODE_CMD_ONE;
                }
            }
            
            if(cmd.is_bit == false)
            {
                DQ_write_cmd(0xFF);
            }
        }
        
        DS18B20_TIM->SR &= ~TIM_SR_UIF;
    }
    
    if((DS18B20_TIM->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF)
    {
        if(is_capture == false)
        {
            is_capture = true;
            capture_1  = DS18B20_TIM->CCR1;
        }
        else
        {
            capture_2 = DS18B20_TIM->CCR1;
            
            period = (capture_1 > capture_2)?((0xFFFF - capture_1) + capture_2):
                                              (capture_2 - capture_1);
            
            capture_1 = capture_2;
        }
        
        if(mode == MODE_RESET)
        {
            pause = capture_1;
            
            if(pause >= 15 && pause <= 60)
            {
                mode = MODE_PRESENCE;
                DQ_in_rising();
            }
            else
            {
                mode        = MODE_NONE;
                TIM_set_opm(460);
            }
        }
        else if(mode == MODE_PRESENCE)
        {
            if(period >= 60 && period <= 240)
            {
                uint16_t time = 460 - pause - period;
                
                DQ_out_set();
                DQ_out_setting();
                TIM_set_opm(time);
            }
            else
            {
                mode        = MODE_NONE;
                TIM_set_opm(460);
            }
        }
        
        DS18B20_TIM->SR &= ~TIM_SR_CC1IF;
    }
}
//----------------------
void DQ_in_setting(void)
{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    
//    GPIO_InitStruct.Pin   = DS18B20_PIN;
//    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Pull  = GPIO_NOPULL;
//    
//    HAL_GPIO_Init(DS18B20_GPIO, &GPIO_InitStruct);
    
    GPIOB->MODER   &= ~GPIO_MODER_MODER4; // reset set DQ
    GPIOB->MODER   |= GPIO_MODER_MODER4_1; // alternate push-pull
    GPIOB->OTYPER  &= ~GPIO_OTYPER_OT_4;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4; // high speed
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR4;
    GPIOB->AFR[0]  |= 0x01 << GPIO_AFRL_AFSEL4_Pos; // set alternate function (AF1)
}
//-----------------------
void DQ_out_setting(void)
{
//    GPIO_InitTypeDef GPIO_InitStruct;
//    
//    GPIO_InitStruct.Pin   = DS18B20_PIN;
//    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Pull  = GPIO_NOPULL;
//    
//    HAL_GPIO_Init(DS18B20_GPIO, &GPIO_InitStruct);
    
    // settings output DQ
    GPIOB->MODER   &= ~GPIO_MODER_MODER4;
    GPIOB->MODER   |= GPIO_MODER_MODER4_0; // output push-pull
    GPIOB->OTYPER  &= ~GPIO_OTYPER_OT_4;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4; // high speed
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR4;
}
//----------------------
void DQ_in_falling(void)
{
    DS18B20_TIM->CCER |= TIM_CCER_CC1P;
}
//---------------------
void DQ_in_rising(void)
{
    DS18B20_TIM->CCER &= ~TIM_CCER_CC1P;
}
//-------------------
void DQ_out_set(void)
{
    DS18B20_GPIO->BSRR |= DS18B20_PIN;
}
//---------------------
void DQ_out_reset(void)
{
    DS18B20_GPIO->BSRR |= DS18B20_PIN << 16;
}
//-----------------------------
void DQ_write_cmd(uint8_t byte)
{
    if(cmd.bit_count == 8)
    {
        cmd.cmd       = 0xFF;
        cmd.bit_count = 0;
        
        return;
    }
    
    if(byte != 0xFF)
    {
        cmd.cmd = byte;
        mode    = MODE_CMD;
    }
    
    uint8_t bit = (cmd.cmd >> cmd.bit_count++)&0x01;
    
    if(bit == 0x00)
    {
        DQ_write_zero();
    }
    else if(bit == 0x01)
    {
        DQ_write_one();
    }
}
//----------------------
void DQ_write_zero(void)
{
    mode |= MODE_CMD_ZERO;
    
    DQ_out_reset();
    TIM_set_opm(80);
}
//---------------------
void DQ_write_one(void)
{
    mode |= MODE_CMD_ONE;
    
    DQ_out_reset();
    TIM_set_opm(5);
}
//------------------------
void TIM_set_capture(void)
{
    DS18B20_TIM->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC1IE);
    DS18B20_TIM->ARR   = 0xFFFF - 1;
    DS18B20_TIM->CR1  &= ~TIM_CR1_OPM;
    DS18B20_TIM->EGR  |= TIM_EGR_UG;
    DS18B20_TIM->SR   &= ~(TIM_SR_UIF | TIM_SR_CC1IF);
    DS18B20_TIM->CCER |= TIM_CCER_CC1E;
    DS18B20_TIM->DIER |= TIM_DIER_CC1IE;
    DS18B20_TIM->CR1  |= TIM_CR1_CEN;
}
//-------------------------------
void TIM_set_opm(uint16_t length)
{
    DS18B20_TIM->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC1IE);
    DS18B20_TIM->CCER |= TIM_CCER_CC1E;
    DS18B20_TIM->ARR   = length - 1;
    DS18B20_TIM->CR1  |= TIM_CR1_OPM;
    DS18B20_TIM->EGR  |= TIM_EGR_UG;
    DS18B20_TIM->SR   &= ~TIM_SR_UIF;
    DS18B20_TIM->DIER |= TIM_DIER_UIE;
    DS18B20_TIM->CR1  |= TIM_CR1_CEN;
}
//--------------
void reset(void)
{
    mode = MODE_RESET;
    
    DQ_out_setting();
    DQ_out_reset();
    
    TIM_set_opm(500);
}
