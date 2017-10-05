#include "i2c.h"
//---------------------
void I2C_EE_Stop(void);
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
    I2C_BUS->CR1 &= ~I2C_CR1_PE; // disabling the peripheral
    
    while((I2C_BUS->CR1 & I2C_CR1_PE) == I2C_CR1_PE); // wait disabled peripheral
    
    I2C_BUS->TIMINGR  = 0x10805E89; // set timing i2c (100kHz - standard mode)
    I2C_BUS->CR1     |= I2C_CR1_PE; // enabling the peripheral
    
    while((I2C_BUS->CR1 & I2C_CR1_PE) != I2C_CR1_PE); // wait enabled peripheral
}
//-------------------------------------------------------------------------------------
bool I2C_EE_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t size)
{
    uint8_t bytes = 0;
    
    I2C_BUS->CR2 &= ~I2C_CR2_ADD10; // 7 bit mode
    I2C_BUS->CR2 |= 1 << I2C_CR2_NBYTES_Pos | (dev_addr & 0xFE);
    I2C_BUS->CR2 |= I2C_CR2_START;
    
    while((I2C_BUS->ISR & I2C_ISR_BUSY) != I2C_ISR_BUSY);
    
    while(((I2C_BUS->ISR & I2C_ISR_TC) != I2C_ISR_TC) && 
          ((I2C_BUS->ISR & I2C_ISR_NACKF) != I2C_ISR_NACKF) &&
          ((I2C_BUS->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY))    
    {
        if(I2C_BUS->ISR & I2C_ISR_TXIS)
        {
            I2C_BUS->TXDR = reg_addr;    // send register address
        }
    }
    
    I2C_BUS->CR2 |= (uint32_t)size << I2C_CR2_NBYTES_Pos | (dev_addr & 0xFE);
    I2C_BUS->CR2 |= I2C_CR2_START;
    
    while((I2C_BUS->ISR & I2C_ISR_BUSY) != I2C_ISR_BUSY);
    
    while(((I2C_BUS->ISR & I2C_ISR_TC) != I2C_ISR_TC) && 
          ((I2C_BUS->ISR & I2C_ISR_NACKF) != I2C_ISR_NACKF) &&
          ((I2C_BUS->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY))
    {
        if((I2C_BUS->ISR & I2C_ISR_TXIS) == I2C_ISR_TXIS)
        {
            I2C_BUS->TXDR = *data++;
            bytes++;
        }
    }
    
    I2C_EE_Stop();
    
    if(bytes == size)
    {
        return I2C_SUCCESS;
    }
    
    return I2C_ERROR;
}
//------------------------------------------------------------------------------------
bool I2C_EE_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t size)
{
    uint8_t bytes = 0;
    
    I2C_BUS->CR2 &= ~I2C_CR2_ADD10; // 7 bit mode
    I2C_BUS->CR2 |= 1 << I2C_CR2_NBYTES_Pos | (dev_addr & 0xFE);
    I2C_BUS->CR2 |= I2C_CR2_START;
    
    while((I2C_BUS->ISR & I2C_ISR_BUSY) != I2C_ISR_BUSY);
    
    while(((I2C_BUS->ISR & I2C_ISR_TC) != I2C_ISR_TC) && 
          ((I2C_BUS->ISR & I2C_ISR_NACKF) != I2C_ISR_NACKF) &&
          ((I2C_BUS->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY))    
    {
        if(I2C_BUS->ISR & I2C_ISR_TXIS)
        {
            I2C_BUS->TXDR = reg_addr; // send register address
        }
    }
    
    I2C_BUS->CR2 |= I2C_CR2_RD_WRN | (uint32_t)size << I2C_CR2_NBYTES_Pos | 
                    (dev_addr & 0xFE);
    I2C_BUS->CR2 |= I2C_CR2_START;
    
    while((I2C_BUS->ISR & I2C_ISR_BUSY) != I2C_ISR_BUSY);
    
    while(((I2C_BUS->ISR & I2C_ISR_TC) != I2C_ISR_TC) && 
          ((I2C_BUS->ISR & I2C_ISR_NACKF) != I2C_ISR_NACKF) &&
          ((I2C_BUS->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY))
    {
        if((I2C_BUS->ISR & I2C_ISR_RXNE) == I2C_ISR_RXNE)
        {
            *data++ = I2C_BUS->RXDR;
            bytes++;
        }
    }
    
    I2C_EE_Stop();
    
    if(bytes == size)
    {
        return I2C_SUCCESS;
    }
    
    return I2C_ERROR;
}
//--------------------
void I2C_EE_Stop(void)
{
    I2C_BUS->CR2 |= I2C_CR2_STOP;				// Выдать стоп на шину
	while(I2C_BUS->ISR & I2C_ISR_BUSY) {};		// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C_BUS->ICR |= I2C_ICR_STOPCF;		// STOP флаг
	I2C_BUS->ICR |= I2C_ICR_NACKCF;		// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if(I2C_BUS->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C_BUS->ICR |= I2C_ICR_ARLOCF;
		I2C_BUS->ICR |= I2C_ICR_BERRCF;
	}
}
