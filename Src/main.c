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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//---------------------------------------------------------------------------
void IODevice_Init(uint8_t addr, PORT_Input_Type* in, PORT_Output_Type* out);
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
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
    
    // Create device
    DEV_Create(GPIOC, GPIO_PIN_14 | GPIO_PIN_15);
    // Get address device
    uint8_t addr = DEV_Address();
    
    // Declaration struct inputs and outputs
    PORT_Input_Type  input; // the input channels
    PORT_Output_Type output; // the output channels
    
    EVENT_Init();
    IODevice_Init(addr, &input, &output);
    DEV_Init(&input, &output);

    if(addr == 0x00)
    {
        DEV_Crash_Init();
        DEV_PWROK_Init();
    }
    
    AIN_Init(addr);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      
    if(FS9_Is_Ready())
    {
        FS9Packet_t packet_source = { 0, 0 };
        FS9Packet_t packet_dest = { 0, 0 };
        
        if(FS9_read(&packet_source))
            DEV_Request(&packet_source, &packet_dest);
        
        if(packet_dest.size > 0)
        {
            FS9_write(&packet_dest);
        }
    }
    
    // обработка события
    EVENT_Execute();
    // конец обработки события
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
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    
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
    
    USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
}

/* USER CODE BEGIN 4 */
void IODevice_Init(uint8_t addr, PORT_Input_Type* in, PORT_Output_Type* out)
{
    in->list[0].pin.gpio = GPIOA;
    in->list[1].pin.gpio = GPIOA;
    in->list[2].pin.gpio = GPIOA;
    in->list[3].pin.gpio = GPIOA;
    in->list[4].pin.gpio = GPIOA;
    in->list[5].pin.gpio = GPIOA;
    in->list[6].pin.gpio = GPIOA;
    in->list[7].pin.gpio = GPIOA;
    in->list[8].pin.gpio = GPIOA;
    in->list[9].pin.gpio = GPIOA;
    
    in->list[0].pin.io = GPIO_PIN_0; // input channel 1
    in->list[1].pin.io = GPIO_PIN_1; // input channel 2
    in->list[2].pin.io = GPIO_PIN_2; // input channel 3
    in->list[3].pin.io = GPIO_PIN_3; // input channel 4
    in->list[4].pin.io = GPIO_PIN_4; // input channel 5
    in->list[5].pin.io = GPIO_PIN_5; // input channel 6
    in->list[6].pin.io = GPIO_PIN_6; // input channel 7
    in->list[7].pin.io = GPIO_PIN_7; // input channel 8
    in->list[8].pin.io = GPIO_PIN_8; // input channel 9
    in->list[9].pin.io = GPIO_PIN_9; // input channel 10
    
    if(addr == 0x00) // МДВВ-01
    {
        in->list[10].pin.gpio = GPIOA;
        in->list[11].pin.gpio = GPIOA;
        
        in->list[10].pin.io = GPIO_PIN_10; // input channel 11
        in->list[11].pin.io = GPIO_PIN_11; // input channel 12
        
        out->list[0].pin.gpio = GPIOB;
        out->list[1].pin.gpio = GPIOB;
        out->list[2].pin.gpio = GPIOB;
        out->list[3].pin.gpio = GPIOB;
        out->list[4].pin.gpio = GPIOB;
        out->list[5].pin.gpio = GPIOB;
        
        out->list[0].pin.io = GPIO_PIN_10; // output chanel 1
        out->list[1].pin.io = GPIO_PIN_15; // output chanel 2
        out->list[2].pin.io = GPIO_PIN_14; // output chanel 3
        out->list[3].pin.io = GPIO_PIN_13; // output chanel 4
        out->list[4].pin.io = GPIO_PIN_12; // output chanel 5
        out->list[5].pin.io = GPIO_PIN_11; // output chanel 6
        
        out->list[0].level = true;
        out->list[1].level = true;
        out->list[2].level = true;
        out->list[3].level = true;
        out->list[4].level = true;
        out->list[5].level = true;
        
        in->size  = 12;
        out->size = 6;
    }
    else if(addr == 0x01) // МДВВ-02
    {
        out->list[0].pin.gpio = GPIOB;
        out->list[1].pin.gpio = GPIOB;
        out->list[2].pin.gpio = GPIOB;
        out->list[3].pin.gpio = GPIOB;
        out->list[4].pin.gpio = GPIOB;
        out->list[5].pin.gpio = GPIOB;
        out->list[6].pin.gpio = GPIOB;
        
        out->list[0].pin.io = GPIO_PIN_11; // output chanel 1
        out->list[1].pin.io = GPIO_PIN_12; // output chanel 2
        out->list[2].pin.io = GPIO_PIN_13; // output chanel 3
        out->list[3].pin.io = GPIO_PIN_14; // output chanel 4
        out->list[4].pin.io = GPIO_PIN_15; // output chanel 5
        out->list[5].pin.io = GPIO_PIN_10; // output chanel 6
        out->list[6].pin.io = GPIO_PIN_2; // output chanel 7
        
        out->list[0].level = true;
        out->list[1].level = true;
        out->list[2].level = true;
        out->list[3].level = true;
        out->list[4].level = true;
        out->list[5].level = true;
        out->list[6].level = true;
        
        in->size  = 10;
        out->size = 7;
    }
    else if(addr == 0x02) // МИК-01V1
    {
        in->list[10].pin.gpio = GPIOA;
        in->list[11].pin.gpio = GPIOA;
        
        in->list[10].pin.io = GPIO_PIN_10; // input channel 11 (scan1)
        in->list[11].pin.io = GPIO_PIN_11; // input channel 12 (scan2)
        
        out->list[0].pin.gpio  = GPIOB;
        out->list[1].pin.gpio  = GPIOB;
        out->list[2].pin.gpio  = GPIOB;
        out->list[3].pin.gpio  = GPIOB;
        out->list[4].pin.gpio  = GPIOB;
        out->list[5].pin.gpio  = GPIOB;
        out->list[6].pin.gpio  = GPIOB;
        out->list[7].pin.gpio  = GPIOB;
        out->list[8].pin.gpio  = GPIOB;
        out->list[9].pin.gpio  = GPIOF;
        out->list[10].pin.gpio = GPIOF;
        out->list[11].pin.gpio = GPIOB;
        
        out->list[0].pin.io  = GPIO_PIN_14;
        out->list[1].pin.io  = GPIO_PIN_13;
        out->list[2].pin.io  = GPIO_PIN_12;
        out->list[3].pin.io  = GPIO_PIN_11;
        out->list[4].pin.io  = GPIO_PIN_10;
        out->list[5].pin.io  = GPIO_PIN_2;
        out->list[6].pin.io  = GPIO_PIN_1;
        out->list[7].pin.io  = GPIO_PIN_0;
        out->list[8].pin.io  = GPIO_PIN_15;
        out->list[9].pin.io  = GPIO_PIN_6;
        out->list[10].pin.io = GPIO_PIN_7;
        out->list[11].pin.io = GPIO_PIN_4;
        
        out->list[0].level  = false;
        out->list[1].level  = false;
        out->list[2].level  = false;
        out->list[3].level  = false;
        out->list[4].level  = false;
        out->list[5].level  = false;
        out->list[6].level  = false;
        out->list[7].level  = false;
        out->list[8].level  = false;
        out->list[9].level  = false;
        out->list[10].level = false;
        out->list[11].level = false;
        
        in->size  = 12;
        out->size = 12;
    }
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
