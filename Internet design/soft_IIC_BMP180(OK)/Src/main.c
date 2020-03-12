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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "I2C.h"
#include "bmp180.h"
#include "delay.h"
#include "stdint.h"
#include "dht11.h"
#include "I2C_2.h"
#include "GY30.h"
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
/***********机智云**********/	
#include "hal_key.h"
#include "gizwits_product.h"
#include "common.h"	
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
/***********机智云**********/	
DHT11_Data_TypeDef dht11;	float GY_30_value;
	uint8_t UV_index;
//	uint8_t ID = 0;
  uint16_t ADC_Value[12];
	uint32_t ADC_UV;
extern dataPoint_t  tDataPoint; 
//TIM_HandleTypeDef htim2;
#define GPIO_KEY_NUM 2 ///< Defines the total number of key member
keyTypedef_t singleKey[GPIO_KEY_NUM]; ///< Defines a single key member array pointer
keysTypedef_t keys;   
static void MX_NVIC_Init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
int test_temp=18;
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
* key1 short press handle
* @param none
* @return none
*/
void key1ShortPress(void)
{
    GIZWITS_LOG("KEY1 PRESS ,Production Mode\n");
    gizwitsSetMode(WIFI_PRODUCTION_TEST);
}

/**
* key1 long press handle
* @param none
* @return none
*/
void key1LongPress(void)
{
    GIZWITS_LOG("KEY1 PRESS LONG ,Wifi Reset\n");
    gizwitsSetMode(WIFI_RESET_MODE);

}

/**
* key2 short press handle
* @param none
* @return none
*/
void key2ShortPress(void)
{
    GIZWITS_LOG("KEY2 PRESS ,Soft AP mode\n");
    #if !MODULE_TYPE
    gizwitsSetMode(WIFI_SOFTAP_MODE);
    #endif
}

/**
* key2 long press handle
* @param none
* @return none
*/
void key2LongPress(void)
{
    //AirLink mode
    GIZWITS_LOG("KEY2 PRESS LONG ,AirLink mode\n");
    #if !MODULE_TYPE
    gizwitsSetMode(WIFI_AIRLINK_MODE);
    #endif
}

/**
* Key init function
* @param none
* @return none
*/
void keyInit(void)
{
    singleKey[0] = keyInitOne(NULL, KEY1_GPIO_Port, KEY1_Pin, key1ShortPress, key1LongPress);
    singleKey[1] = keyInitOne(NULL, KEY2_GPIO_Port, KEY2_Pin, key2ShortPress, key2LongPress);
//		singleKey[1] = keyInitOne(KEY2_GPIO_Port, KEY2_Pin,key2ShortPress, key2LongPress);
    keys.singleKey = (keyTypedef_t *)&singleKey;
    keyParaInit(&keys); 
}

void userHandle(void)
{
 /*
    currentDataPoint.valueUltraviolet_index = ;//Add Sensor Data Collection
    currentDataPoint.valuetemp_value = ;//Add Sensor Data Collection
    currentDataPoint.valuehumidity_value = ;//Add Sensor Data Collection
    currentDataPoint.valuelight_intensity = ;//Add Sensor Data Collection
    currentDataPoint.valueair_pressure = ;//Add Sensor Data Collection

    */
		
		static uint32_t cnt = 0;
		
		if((gizGetTimerCount()-cnt) > 2000) //每隔2000ms读取一次
		{
//    currentDataPoint.valueUltraviolet_index = ADC_UV;//Add Sensor Data Collection
		currentDataPoint.valueUltraviolet_index = 7;//Add Sensor Data Collection	
    currentDataPoint.valuetemp_value = dht11.temperature;//Add Sensor Data Collection
    currentDataPoint.valuehumidity_value = dht11.humidity;//Add Sensor Data Collection
    currentDataPoint.valuelight_intensity = (int)GY_30_value;//Add Sensor Data Collection
    currentDataPoint.valueair_pressure = bmp180.p+43000;//Add Sensor Data Collection
			cnt = gizGetTimerCount();
		}

}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	/***********机智云**********/	
	MX_NVIC_Init();


  userInit();
  gizwitsInit();
  keyInit();
	
	
	timerInit();
  uartInit();
  GIZWITS_LOG("MCU Init Success \n");
	/***********机智云**********/	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  HAL_Init();
	MX_TIM1_Init();
	
	
	DHT11_Init(); // DHT11初始化（单总线）
	GY30_Int(); // GY30初始化（IIC）
	BMP_Init(); // BMP180初始化（IIC）
	BMP_ReadCalibrationData();	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 12); // 开启DMA读取四路ADC
	HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_7, GPIO_PIN_SET);
	
	
	while(1)
	{
		 userHandle();
     gizwitsHandle((dataPoint_t *)&currentDataPoint);
		
		
		
    /* （DHT11）读取温湿度信息 */
		DHT11_Read_TempAndHumidity(&dht11);
		
		
		/* （BMP180）读取大气压以及对应海拔 */
//		ID = BMP_ReadOneByte(0xd0);
		BMP_UncompemstatedToTrue();
		
		/* （adc）读取四路adc值 */
		ADC_UV = 0;
		for(uint8_t i = 0 ;i < 12;)
		{
			ADC_UV += ADC_Value[i++];
			i++;
			i++;
			i++;
		}
		/**ADC1 GPIO Configuration    
    PA0     ------> ADC1_IN5
    PA1     ------> ADC1_IN6
    PB0     ------> ADC1_IN15
    PB1     ------> ADC1_IN16 
    */
		/* （GY_30）读取光强 */
		GY_30_value = GY30_GetLight();
		
//		printf(" ADC_功耗 value = %1.3fmA \r\n", (ADC_UV/3.0f));//ADC1 PA0
		/* 通过采集到的电压值计算紫外线等级 */
		ADC_UV = ADC_UV*3300/4096;
		
		if(ADC_UV<227) UV_index = 0;
		else if(ADC_UV<318) UV_index = 1;
		else if(ADC_UV<408) UV_index = 2;
		else if(ADC_UV<503) UV_index = 3;
		else if(ADC_UV<606) UV_index = 4;
		else if(ADC_UV<696) UV_index = 5;
		else if(ADC_UV<795) UV_index = 6;
		else if(ADC_UV<881) UV_index = 7;
		else if(ADC_UV<976) UV_index = 8;
		else if(ADC_UV<1079) UV_index = 9;
		else if(ADC_UV<1170) UV_index = 10; 
		else UV_index = 11; 
		
		
//		printf("temp = %.2f ; humi = %.2f\r\n",dht11.temperature,dht11.humidity); //打印温湿度信息
//		
//		printf("Pressure = %ldPa\t   Altitude = %.5fm\r\n", bmp180.p+43000,bmp180.altitude/10); // 打印大气压强以及对应的海拔

//		printf("ADC_光强_value = %1.3f LUX \r\n", GY_30_value); // 打印光强
//		
//		printf(" ADC_紫外线_index = %d \r\n", UV_index); // 打印紫外线等级

		
//		HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_5, GPIO_PIN_SET);

//		HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_5, GPIO_PIN_SET);//提示系统正在运行	
//		myDelay(500);
//		HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_5, GPIO_PIN_RESET);
//		myDelay(500);
//		myDelay(10000);
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2,(uint8_t*)&ch, 1, 0xFFFF);

  return ch;
}
/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim==&htim2)
//	{
//			keyHandle((keysTypedef_t *)&keys);
//			gizTimerMs();
//	}
//}
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
