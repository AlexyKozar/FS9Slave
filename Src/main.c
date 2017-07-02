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
int32_t cpu_temperature(uint16_t t_adc);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    AIN_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
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
  
    // Create device
    DEV_Create(GPIOB, GPIO_PIN_14 | GPIO_PIN_15);
    // Get address device
    uint8_t addr = DEV_Address();
    
    // Declaration struct inputs and outputs
    struct IO_Type in; // the input channels
    struct IO_Type out; // the output channels
    
    in.gpio  = GPIOA;
    out.gpio = GPIOB;
    
    in.io[0] = GPIO_PIN_0; // input channel 1
    in.io[1] = GPIO_PIN_1; // input channel 2
    in.io[2] = GPIO_PIN_2; // input channel 3
    in.io[3] = GPIO_PIN_3; // input channel 4
    in.io[4] = GPIO_PIN_4; // input channel 5
    in.io[5] = GPIO_PIN_5; // input channel 6
    in.io[6] = GPIO_PIN_6; // input channel 7
    in.io[7] = GPIO_PIN_7; // input channel 8
    in.io[8] = GPIO_PIN_8; // input channel 9
    in.io[9] = GPIO_PIN_9; // input channel 10

    if(addr == 0x00) // device-01
    {
        in.io[10] = GPIO_PIN_10; // input channel 11
        in.io[11] = GPIO_PIN_11; // input channel 12
        
        out.io[0] = GPIO_PIN_10; // output chanel 1
        out.io[1] = GPIO_PIN_15; // output chanel 2
        out.io[2] = GPIO_PIN_14; // output chanel 3
        out.io[3] = GPIO_PIN_13; // output chanel 4
        out.io[4] = GPIO_PIN_12; // output chanel 5
        out.io[5] = GPIO_PIN_11; // output chanel 6
        
        in.size  = 12;
        out.size = 6;
    }
    else if(addr == 0x01) // device-02
    {
        out.io[0] = GPIO_PIN_11; // output chanel 1
        out.io[1] = GPIO_PIN_12; // output chanel 2
        out.io[2] = GPIO_PIN_13; // output chanel 3
        out.io[3] = GPIO_PIN_14; // output chanel 4
        out.io[4] = GPIO_PIN_15; // output chanel 5
        out.io[5] = GPIO_PIN_10; // output chanel 6
        out.io[6] = GPIO_PIN_2; // output chanel 7
        
        in.size  = 10;
        out.size = 7;
    }
    
    DEV_Init(&in, &out);
    
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      
    if(FS9_Is_Ready())
    {
        struct FS9Packet_t packet_source;
        struct FS9Packet_t packet_dest;
        
        if(FS9_read(&packet_source))
            DEV_Request(&packet_source, &packet_dest);
    }
    
    /*if((DMA1->ISR & DMA_ISR_TCIF1) == DMA_ISR_TCIF1)
    {
        AIN1 = AIN_channels[0];
        AIN2 = AIN_channels[1];
        AIN_TEMP = AIN_channels[2];
        
        DMA1->IFCR |=DMA_IFCR_CTCIF1;
    }*/
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
