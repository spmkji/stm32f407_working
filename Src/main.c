/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef		GPIO_InitStruct;
TIM_HandleTypeDef 	htim2;
TIM_HandleTypeDef 	htim3;

UART_HandleTypeDef	huart2;
UART_HandleTypeDef	huart3;


#define	led_time	24		// 20 x 25 = 500ms

uint8_t				SysTickBuffer;
uint8_t				SubBuffer;

uint8_t 				RecvOn;
uint8_t 				Usart2RxBuf[50];
uint8_t 				Usart2TxBuf[50];
uint8_t 				Usart2RecvTimer;		/* Reset Timer */
uint8_t 				Usart2RecvIndex;		/* Reset Index */


uint8_t 				RecvOn3;
uint8_t 				Usart3RxBuf[50];
uint8_t 				Usart3TxBuf[50];
uint8_t 				Usart3RecvTimer;		/* Reset Timer */
uint8_t 				Usart3RecvIndex;		/* Reset Index */
 
uint8_t				TaskSwitchBuffer;

uint8_t				led_count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void Init_Timer2(void);
static void Init_Timer3(void);
static void Init_Usart2(void);
static void Init_Usart3(void);
void MainFunction(void);
void Usart2Recv(void);
void Usart3Recv(void);
uint8_t CheckTwoString(uint8_t* str1, uint8_t* str2, uint8_t cnt);
unsigned char CopyStingToPointer(unsigned char *Destination, unsigned char *Source);
void ClearMemory(uint8_t* str, uint8_t cnt);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
	
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  // github_test

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
	Init_Timer2();
	Init_Timer3();
	Init_Usart2();
	Init_Usart3();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	MainFunction();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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

  /* GPIO Ports Clock Enable */
 	 __HAL_RCC_GPIOD_CLK_ENABLE();

 	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
 	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
}

/* USER CODE BEGIN 4 */
void MainFunction(void)
{
	if(SysTickBuffer == 0x01)
	{
		switch(TaskSwitchBuffer)
		{
			case 0:			// start
				if(led_count++ > led_time){
					led_count = 0;
					//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
				}
				break;
			case 1:			// 4ms
				break;
			case 2:			// 8ms
				break;
			case 3:			// 12ms
				break;
			case 4:			// 16ms
				Usart2Recv();
				Usart3Recv();				
				break;
			default:
				TaskSwitchBuffer= 0x00;
				break;
		}

		SysTickBuffer = 0;
		
		TaskSwitchBuffer++;
		if(TaskSwitchBuffer > 0x04)
		{
			TaskSwitchBuffer= 0x00;
		}
	}
}

void Usart2Recv(void)
{
	uint8_t cnt;

	/* Check Received Or Not */
	if(RecvOn == 1)
	{
		Usart2RecvTimer++;
		if(Usart2RecvTimer > 6)	/* 0.14sec */
		{
			RecvOn= 0;				/* Stop Timer */
			Usart2RecvTimer= 0;		/* Reset Timer */
			Usart2RecvIndex= 0;		/* Reset Index */

			if(CheckTwoString(&Usart2RxBuf[0], "Embedded Lab", 12))
			{
				cnt= CopyStingToPointer(Usart2TxBuf, "Embedded Lab Good !$");
				HAL_UART_Transmit(&huart2, Usart2TxBuf, cnt, 10);
				ClearMemory(&Usart2RxBuf[0], 50);
			}
			else
			{
				cnt= CopyStingToPointer(Usart2TxBuf, "Pleas check again !$");
				HAL_UART_Transmit(&huart2, Usart2TxBuf, cnt, 10);
			}
		}
	}
}

void Usart3Recv(void)
{
	uint8_t cnt;

	/* Check Received Or Not */
	if(RecvOn3 == 1)
	{
		Usart3RecvTimer++;
		if(Usart3RecvTimer > 6)	/* 0.14sec */
		{
			RecvOn3= 0;				/* Stop Timer */
			Usart3RecvTimer= 0;		/* Reset Timer */
			Usart3RecvIndex= 0;		/* Reset Index */

			if(CheckTwoString(&Usart3RxBuf[0], "on", 2))
			{
				cnt= CopyStingToPointer(Usart2TxBuf, "LED on$");
				HAL_UART_Transmit(&huart2, Usart2TxBuf, cnt, 10);
				ClearMemory(&Usart3RxBuf[0], 50);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
			}
			
			if(CheckTwoString(&Usart3RxBuf[0], "off", 3))
			{
				cnt= CopyStingToPointer(Usart2TxBuf, "LED off$");
				HAL_UART_Transmit(&huart2, Usart2TxBuf, cnt, 10);
				ClearMemory(&Usart3RxBuf[0], 50);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);				
			}
			
		}
	}
}


static void Init_Timer2(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();

	htim2.Instance = TIM2;
	htim2.Init.Period = 39;
	htim2.Init.Prescaler = 8400;
	htim2.Init.ClockDivision = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	HAL_TIM_Base_Init(&htim2);

	if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}

	/* Set Interrupt Group Priority */ 
	HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);
	/* Enable the TIM3 global Interrupt */
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

static void Init_Timer3(void)
{
	__HAL_RCC_TIM3_CLK_ENABLE();

	htim3.Instance = TIM3;
	htim3.Init.Period = 999;
	htim3.Init.Prescaler = 8400;
	htim3.Init.ClockDivision = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

	if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}

	/* Set Interrupt Group Priority */ 
	HAL_NVIC_SetPriority(TIM3_IRQn, 4, 0);
	/* Enable the TIM3 global Interrupt */
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}
static void Init_Usart2(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
#if 0
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    	GPIO_InitStruct.Pull = GPIO_PULLUP;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	huart2.Instance = USART2;
  	huart2.Init.BaudRate = 115200;
  	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_NVIC_SetPriority(USART2_IRQn, 0xf, 1);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

static void Init_Usart3(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();
#if 0
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    	GPIO_InitStruct.Pull = GPIO_PULLUP;
    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	huart3.Instance = USART3;
  	huart3.Init.BaudRate = 9600;
  	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_NVIC_SetPriority(USART3_IRQn, 0xf, 1);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}


uint8_t CheckTwoString(uint8_t* str1, uint8_t* str2, uint8_t cnt)
{
	uint8_t ret, i;

	for(i=0; i<cnt; i++)
	{
		if(*str1 != *str2)
		{
			ret = 0;	// Not match
			break;
		}
		else{
			str1++;
			str2++;
		}
		ret = 1;
	}

	return ret;
}

unsigned char CopyStingToPointer(unsigned char *Destination, unsigned char *Source)
{
	unsigned char cnt;

	for(cnt=0; cnt<100; cnt++)
	{
		*Destination= *Source;
		Destination++;
		Source++;

		if(*Source == '$')
		{
			cnt++;
			break;
		}
	}
	return cnt;
}
void ClearMemory(uint8_t* str, uint8_t cnt)
{
	uint8_t i;

	for(i=0; i<cnt; i++)
	{
		*str= 0;
		str++;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
