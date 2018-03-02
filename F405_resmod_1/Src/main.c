/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"


/* USER CODE BEGIN Includes */
#include "sine_table.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int index_of_sine_array_bulat=0;
int index_I_dt =0;

int32_t I_dt[10000];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Delay_few_nano();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */



  HAL_GPIO_WritePin(AD5541__CS_GPIO_Port, AD5541__CS_Pin, GPIO_PIN_SET);
  LL_SPI_Enable(SPI3);//dac spi enable

  LL_SPI_Enable(SPI2);//I_adcc spi enable

  //pga currente gain 64
  HAL_GPIO_WritePin(GPIOB, I_PGA_G3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, I_PGA_G2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, I_PGA_G1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, I_PGA_G0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, I_PGA_G4_Pin, GPIO_PIN_RESET);

  //pga voltage gain 32
  HAL_GPIO_WritePin(GPIOC, V_PGA_G3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, V_PGA_G2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, V_PGA_G1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, V_PGA_G0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, V_PGA_G4_Pin, GPIO_PIN_RESET);


  HAL_TIM_Base_Start_IT(&htim3);

  HAL_Delay(100);
  Delay_few_nano();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

	 RCC_OscInitTypeDef RCC_OscInitStruct;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;

	    //*Configure the main internal regulator output voltage



	   // *Initializes the CPU, AHB and APB busses clocks

	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = 16;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL.PLLM = 8;
	  RCC_OscInitStruct.PLL.PLLN = 80;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = 4;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	 //   *Initializes the CPU, AHB and APB busses clocks

	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	    /**Configure the main internal regulator output voltage
	    */
	  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  //__HAL_RCC_PWR_CLK_ENABLE();

	  //__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	  //  *Configure the Systick interrupt time

	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	 //   *Configure the Systick

	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	 //  SysTick_IRQn interrupt configuration
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  LL_SPI_InitTypeDef SPI_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  
  /**SPI2 GPIO Configuration  
  PC2   ------> SPI2_MISO
  PB10   ------> SPI2_SCK 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection =LL_SPI_FULL_DUPLEX;
  //SPI_InitStruct.TransferDirection = LL_SPI_SIMPLEX_RX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  LL_SPI_InitTypeDef SPI_InitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  
  /**SPI3 GPIO Configuration  
  PC10   ------> SPI3_SCK
  PC12   ------> SPI3_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  //GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI3, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 800;
 // htim3.Init.Period = 4000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, V_PGA_G4_Pin|V_PGA_G3_Pin|V_PGA_G2_Pin|I_PGA_G1_Pin 
                          |I_PGA_G2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, I_ADC_CNV_Pin|I_PGA_G0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AD5541__CS_GPIO_Port, AD5541__CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, I_PGA_G3_Pin|I_PGA_G4_Pin|DAC__LDAC_Pin|V_PGA_G0_Pin 
                          |V_PGA_G1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : V_PGA_G4_Pin V_PGA_G3_Pin V_PGA_G2_Pin I_PGA_G1_Pin 
                           I_PGA_G2_Pin */
  GPIO_InitStruct.Pin = V_PGA_G4_Pin|V_PGA_G3_Pin|V_PGA_G2_Pin|I_PGA_G1_Pin 
                          |I_PGA_G2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : I_ADC_BUSYF_Pin */
  GPIO_InitStruct.Pin = I_ADC_BUSYF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I_ADC_BUSYF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I_ADC_CNV_Pin I_PGA_G0_Pin */
  GPIO_InitStruct.Pin = I_ADC_CNV_Pin|I_PGA_G0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AD5541__CS_Pin */
  GPIO_InitStruct.Pin = AD5541__CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AD5541__CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I_PGA_G3_Pin I_PGA_G4_Pin DAC__LDAC_Pin V_PGA_G0_Pin 
                           V_PGA_G1_Pin */
  GPIO_InitStruct.Pin = I_PGA_G3_Pin|I_PGA_G4_Pin|DAC__LDAC_Pin|V_PGA_G0_Pin 
                          |V_PGA_G1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim3)
{
    if (htim3->Instance==TIM3) //check if the interrupt comes from TIM2
    {
    	uint16_t  hundred_nano_delay =0;
    	uint16_t temp1=0;
    	int16_t I_temp_dt;

    	//dac
    	HAL_GPIO_WritePin(GPIOA, AD5541__CS_Pin, GPIO_PIN_RESET);
    	asm("NOP");
    	LL_SPI_TransmitData16(SPI3, sine_in_flash_array_bulat[index_of_sine_array_bulat] );
    	index_of_sine_array_bulat++;

    	while (LL_SPI_IsActiveFlag_BSY(SPI3) )
    	{    	}
    	asm("NOP");
    	HAL_GPIO_WritePin(GPIOA, AD5541__CS_Pin, GPIO_PIN_SET);

    	if (index_of_sine_array_bulat == 100)
    	    index_of_sine_array_bulat = 0;


   	//adc
    	//first CNV pulse for both I & V   --- I first
    	HAL_GPIO_WritePin(GPIOA,I_ADC_CNV_Pin , GPIO_PIN_SET);
    	//create delay 3us conversion time or polling BUSY pin
    	asm("NOP");
    	asm("NOP"); //tCNVH of page 4 -min 20ns  (each NO is 12.5 for 80Mhz clock, so total 25ns delay)

    	while(HAL_GPIO_ReadPin(GPIOC, I_ADC_BUSYF_Pin))	//conv done ?
    	{}
    	asm("NOP"); //dtsheet 5ns max

    	HAL_GPIO_WritePin(GPIOA,I_ADC_CNV_Pin , GPIO_PIN_RESET);
    	while (LL_SPI_IsActiveFlag_RXNE(SPI2))
    	{
    		(void)LL_SPI_ReceiveData16(SPI2);      // flush any FIFO content
    	}


    	LL_SPI_TransmitData16(SPI2, 0xFFff);   // send dummy byte
    	while (LL_SPI_IsActiveFlag_BSY(SPI2) )
    	{
    	}
    	asm("NOP");
    	while (!LL_SPI_IsActiveFlag_RXNE(SPI2))
    	{

    	}



    	while (!LL_SPI_IsActiveFlag_TXE(SPI2))
    	{
    		;
    	}
    	I_temp_dt =LL_SPI_ReceiveData16(SPI2);;
    	I_dt[index_I_dt++] = I_temp_dt;
    	if (index_I_dt == 10000)
    	{
    		index_I_dt = 0;
    	}

    	temp1=9;
    }
}



void Delay_few_nano()
{
	int counter_nano = 0;
	while (counter_nano < 1)
	{
		counter_nano++;

	}


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
