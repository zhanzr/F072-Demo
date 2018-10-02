
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#ifdef __cplusplus
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include "LiquidCrystal.h"

using namespace std;
#else
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#endif

#include "gyro.h"
#include "l3gd20.h"
#include "retarget_io_drv.h"
//using namespace std;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
__IO int16_t g_adc_buf[ADC_CHAN_NO];
__IO int16_t g_mems_buf[MEMS_CHAN_NO];
__IO uint8_t g_mems_id;
//Re-implement any functions that require re-implementation.


/**
* @brief  MEMS Test.
* @param  None
* @retval None
*/
static void MEMS_Test(void)
{
  float Buffer[6] = {0};
  uint8_t Xval, Yval = 0;
  
  /* Demo Gyroscope */
  if(BSP_GYRO_Init() != GYRO_OK)
  {
    Error_Handler();
  }

    /* Read Gyro Angular data */
    BSP_GYRO_GetXYZ(Buffer);
    
    /* Update autoreload and capture compare registers value */
    Xval = abs((int8_t)(Buffer[0]));
    Yval = abs((int8_t)(Buffer[1]));
    
    if(Xval > Yval)
    {
      if(Buffer[0] > 5000.0f)
      {
        /* Insert 250ms delay */ 
        HAL_Delay(250);
      }
      
      if(Buffer[0] < -5000.0f)
      {
        /* Insert 250ms delay */ 
        HAL_Delay(250);
      }
    }
    else
    {
      if(Buffer[1] > 5000.0f)
      {
        /* Insert 250ms delay */ 
        HAL_Delay(250);
      }
      
      if(Buffer[1] < -5000.0f)
      {

        /* Insert 250ms delay */ 
        HAL_Delay(250);
      }
    }

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int8_t mems_tmp;
	int32_t JTemp;	
	uint32_t VRef;	
	uint32_t VBat;	
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
  MX_SPI2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	MEMS_Test();
	
	printf("F072 Discovery Test CM0 CPUID:%08X, @ %u Hz\n %u %u %u\n",
		SCB->CPUID,
		SystemCoreClock,
		T30_VAL_3300,
		T110_VAL_3300,
		VREF_VAL_3300
		);
		
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)g_adc_buf, ADC_CHAN_NO);	
#ifdef __cplusplus
	LiquidCrystal lcd;
	
	lcd.Display(0, 0, (uint8_t*)__TIME__);	
	lcd.Display(1, 0, (uint8_t*)__DATE__);
#endif	

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
  {		
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		HAL_Delay(2000);
		
		GYRO_IO_Read((uint8_t*)&mems_tmp, L3GD20_OUT_TEMP_ADDR, 1);
			
		JTemp = ((80*g_adc_buf[0]*VDD_MV)/3300 + 30*T110_VAL_3300 - 110*T30_VAL_3300)/(T110_VAL_3300-T30_VAL_3300);
		VRef = ADC_2_MV(g_adc_buf[1]);
		VBat = 2 * ADC_2_MV(g_adc_buf[2]);
		
		printf("%02X %d %d %d %d\n",
		g_mems_id, mems_tmp, JTemp, VRef, VBat);
				
		GYRO_IO_Read((uint8_t*)g_mems_buf, L3GD20_OUT_X_L_ADDR, MEMS_CHAN_NO*2);
		printf("X: %d, Y: %d, Z: %d\n", 
		g_mems_buf[0],
		g_mems_buf[1],
		g_mems_buf[2]
		);
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* USER CODE BEGIN 4 */

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
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
