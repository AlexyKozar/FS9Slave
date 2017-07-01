/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
volatile bool is_packet = false;
volatile uint8_t size_packet = 0;
volatile bool is_data = false;
volatile uint16_t GPIO_INT = GPIO_PIN_5; // for send change signal to master
volatile uint32_t T_adc = 0;
volatile uint8_t  T_adc_count = 0;
volatile bool     adc_is_ready = false;

volatile uint16_t AIN_channels[3] = { 0, 0, 0 };
volatile uint16_t AIN_TEMP = 0;
volatile uint16_t AIN1 = 0;
volatile uint16_t AIN2 = 0;

volatile uint8_t AIN_ch_cur = 0;
volatile uint8_t AIN_CON_count = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/*void USART1_IRQHandler(void)
{
    uint32_t status = USART1->ISR;
    
    if(status & USART_ISR_RXNE)
    {
        uint16_t byte9bit = USART1->RDR;

        if(byte9bit & 0x0100)
        {
            uint8_t byte = (uint8_t)(byte9bit & 0xFF);
            uint8_t addr = (byte & 0xC0) >> 6; // get a device address
            
            if(addr == dev_get_addr())
            {
                uint8_t cmd = byte & 0x3F; // get a command
                
                size_packet = dev_get_size_packet(cmd); // get a size packet
                
                if(size_packet > 0) // cmd is valid
                {
                    is_data = true; // a begin packet is find
                    rx_buf_push(byte); // first byte a packet
                }
            }
        }
        else if(is_data)
        {
            rx_buf_push((uint8_t)(byte9bit & 0xFF));
            
            if(rx_buf_size() == size_packet)
            {
                is_packet   = true;
                is_data     = false;
                size_packet = 0;
            }
        }
        
        USART1->RQR |= USART_RQR_RXFRQ;
    }
    
    if(status & USART_ISR_TXE)
    {
       if(!tx_buf_is_empty())
        {
            USART1->TDR = tx_buf_pop();
        }
        else
        {
            USART1->CR1 &= ~(USART_CR1_TE | USART_CR1_TXEIE);
            USART1->ISR &= ~USART_ISR_TXE;
        }
    }
    
    if(status & USART_ISR_ORE)
    {
        USART1->ICR |= USART_ICR_ORECF;
    }
}*/

/* USER CODE BEGIN 1 */
void TIM14_IRQHandler(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_INT, GPIO_PIN_RESET);

    TIM16->CR1 |= TIM_CR1_CEN;
    
    TIM14->SR &= ~TIM_SR_UIF;
}

void TIM16_IRQHandler(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_INT, GPIO_PIN_SET);
    
    TIM16->SR &= ~TIM_SR_UIF;
}

void ADC1_IRQHandler(void)
{
    if((ADC1->ISR & ADC_ISR_EOC) == ADC_ISR_EOC)
    {   
        /*T_adc += ADC1->DR;
        
        T_adc_count++;
        
        if(T_adc_count >= 10)
            adc_is_ready = true;*/

        AIN_channels[AIN_ch_cur++] = ADC1->DR;
        
        if(AIN_ch_cur == 3)
            AIN_ch_cur = 0;
        
        AIN_CON_count++;
        
        if(AIN_CON_count == 10)
        {
            AIN_ch_cur = 0;
            AIN_CON_count = 0;
            adc_is_ready = true;
        }
        
        ADC1->ISR |= ADC_ISR_EOC;
    }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
