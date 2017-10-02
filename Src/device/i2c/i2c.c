#include "i2c.h"
//--------------------
uint8_t* _data = NULL;
uint8_t  bytes = 0;
//--------------------
void I2C_EE_Init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    
    // set pins SCL and SDA alternate function open drain
    GPIOB->MODER   &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->MODER   |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOB->OTYPER  |= (GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9); // max speed
    GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
    GPIOB->AFR[1]  |= ((0x01 << GPIO_AFRH_AFSEL8_Pos) | (0x01 << GPIO_AFRH_AFSEL9_Pos));
    
    // set i2c
    I2C1->CR1     &= ~I2C_CR1_PE; // disabling the peripheral
    I2C1->CR1     &= ~I2C_CR1_ANFOFF; // analog noise filter disabled
    I2C1->CR1     &= ~I2C_CR1_DNF; // digital filter disabled
    I2C1->TIMINGR  = 0x00B0DBFF; // set timing i2c (100kHz - standard mode)
    I2C1->CR1     |= I2C_CR1_NOSTRETCH; // clock stretching disabled
    I2C1->CR1     |= I2C_CR1_ADDRIE | I2C_CR1_TXIE;
    I2C1->CR1     |= I2C_CR1_PE; // enabling the peripheral
    
    NVIC_EnableIRQ(I2C1_IRQn);
}
//---------------------------------------------------------------
void I2C_EE_WriteBytes(uint8_t addr, uint8_t* data, uint8_t size)
{
    if(size == 0 || data == NULL)
    {
        return;
    }
    
    _data = data;
    bytes = 0;
    
    I2C1->CR2 &= ~I2C_CR2_ADD10; // 7 bit mode
    I2C1->CR2 &= I2C_CR2_RD_WRN; // write mode
    I2C1->CR2 |= addr; // set slave address
    I2C1->CR2 |= size << I2C_CR2_NBYTES_Pos; // size array data
    I2C1->CR2 |= I2C_CR2_AUTOEND; // automatic end mode
    I2C1->CR2 |= I2C_CR2_START; // start enable
}
//------------------------
void I2C1_IRQHandler(void)
{
    uint32_t state = I2C1->ISR;
    
    if((state & I2C_ISR_TXIS) == I2C_ISR_TXIS)
    {
        if((I2C1->CR2 & I2C_CR2_RD_WRN) == false)
        {
            I2C1->TXDR = _data[bytes++];
        }
    }
}
