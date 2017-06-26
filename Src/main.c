/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Temperature sensor calibration value address */
    #define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void init_adc(void);
int32_t cpu_temperature(uint16_t t_adc);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int32_t temp = 0;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
    init_adc();
    
    USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
    
    ADC1->CR |= ADC_CR_ADSTART; // adc start conversion
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    
    HAL_GPIO_WritePin(GPIOB, GPIO_INT, GPIO_PIN_SET); // set high level on INT pin
    
    TIM14->PSC   = 48000 - 1; // each 1ms
    TIM14->ARR   = 100 - 1; // reload 100ms
    TIM14->CR1  |= TIM_CR1_ARPE;
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1  |= TIM_CR1_CEN;
    
    TIM16->PSC   = 480 - 1; // each 100us
    TIM16->ARR   = 10 - 1; // reload 1ms
    TIM16->DIER |= TIM_DIER_UIE;
    
    NVIC_EnableIRQ(TIM14_IRQn);
    NVIC_EnableIRQ(TIM16_IRQn);
    
    // get a device address
    uint8_t addr = (uint8_t)((GPIOC->IDR & 0xC000) >> 14);
    // set a device address
    dev_set_addr(addr);
    
    uint16_t output[8]; // output chanels
    uint8_t  output_count = 0; 
    
    if(addr == 0x00) // device-01
    {
        output[0] = GPIO_PIN_10; // chanel 1
        output[1] = GPIO_PIN_15; // chanel 2
        output[2] = GPIO_PIN_14; // chanel 3
        output[3] = GPIO_PIN_13; // chanel 4
        output[4] = GPIO_PIN_12; // chanel 5
        output[5] = GPIO_PIN_11; // chanel 6
        
        output_count = 6;
    }
    else if(addr == 0x01) // device-02
    {
        output[0] = GPIO_PIN_11; // chanel 1
        output[1] = GPIO_PIN_12; // chanel 2
        output[2] = GPIO_PIN_13; // chanel 3
        output[3] = GPIO_PIN_14; // chanel 4
        output[4] = GPIO_PIN_15; // chanel 5
        output[5] = GPIO_PIN_10; // chanel 6
        output[6] = GPIO_PIN_2; // chanel 7
        
        output_count = 7;
    }
    
    dev_set_output(output, output_count);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    if(is_packet)
    {
        if(!rx_buf_is_empty())
        {
            uint8_t size = rx_buf_size();
            struct packet_t packet = { {0}, size };
            
            for(uint8_t i = 0; i < size; i++)
            {
                packet.array[i] = rx_buf_pop();
            }
            
            dev_get_packet(&packet);
            
            if(packet.size != 0)
            {
                for(uint8_t i = 0; i < packet.size; i++)
                {
                    tx_buf_push(packet.array[i]);
                }
                
                USART1->CR1 |= USART_CR1_TE | USART_CR1_TXEIE;
                USART1->ISR |= USART_ISR_TXE;
            }
        }
        
        is_packet = false;
    }
    
    if(adc_is_ready)
    {
        // data temperature is ready
        temp = cpu_temperature(T_adc/10);
        
        T_adc = 0;
        T_adc_count = 0;
        adc_is_ready = false;
    }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    GPIO_InitTypeDef GPIO_InitINT;
    
    GPIO_InitINT.Pin = GPIO_INT;
    GPIO_InitINT.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitINT.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitINT);
}

/* USER CODE BEGIN 4 */
void init_adc(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADCEN;
    
    ADC1->CR &= ~ADC_CR_ADEN; // disable adc for calibration
    ADC1->CR |= ADC_CR_ADCAL; // start calibration adc
    
    while((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL); // wait end calibration adc
    
    ADC1->CFGR1 &= ~ADC_CFGR1_RES; // resolution 12 bit
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; // right-aligned
    //ADC1->ISR   |= ADC_ISR_ADRDY;
    ADC1->CR    |= ADC_CR_ADEN; // adc enable
    
    while((ADC1->ISR & ADC_ISR_ADRDY) != ADC_ISR_ADRDY); // wait ready adc
    
    ADC1->CHSELR |= ADC_CHSELR_CHSEL16/* | ADC_CHSELR_CHSEL17*/; // selection temperature sensor channel
    ADC1->SMPR   |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; // set sampling time 239.5 ADC clock cycles
    ADC1->CFGR1  |= ADC_CFGR1_CONT; // continuous conversion mode
    ADC1->IER    |= ADC_IER_EOCIE; // generate interrupt EOC
    ADC->CCR     |= ADC_CCR_TSEN; // enable temperature sensor
    //ADC->CCR     |= ADC_CCR_VREFEN;
    
    NVIC_EnableIRQ(ADC1_IRQn);
}

int32_t cpu_temperature(uint16_t t_adc)
{
    int32_t temperature = 0;
    
    temperature = (*TEMP30_CAL_ADDR - t_adc)/5 + 30;
    
    return temperature;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
