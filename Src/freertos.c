/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
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

extern __IO uint16_t g_adc_buf[ADC_CHAN_NO];
extern __IO uint8_t g_mems_id;
extern __IO int16_t g_mems_buf[MEMS_CHAN_NO];

extern xSemaphoreHandle notification_semaphore;
extern xQueueHandle Queue_id;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityBelowNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	int8_t mems_tmp;
	int32_t JTemp;	
	uint32_t VRef;	
	uint32_t tmpTicks;	
	
  float Buffer[6] = {0};  
	
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(Queue_id, &tmpTicks, portMAX_DELAY);
		
		printf("%s %u %u\n", 
		osKernelSystemId,
		tmpTicks,
		g_adc_buf[0]
		);

		GYRO_IO_Read((uint8_t*)&mems_tmp, L3GD20_OUT_TEMP_ADDR, 1);
			
		JTemp = ((80*g_adc_buf[0]*VDD_MV)/3300 + 30*T110_VAL_3300 - 110*T30_VAL_3300)/(T110_VAL_3300-T30_VAL_3300);
		VRef = ADC_2_MV(g_adc_buf[1]);
		
		printf("%02X %d %d %d %d\n",
		g_mems_id, mems_tmp, JTemp, VRef, 2 * ADC_2_MV(g_adc_buf[2]));
				
		GYRO_IO_Read((uint8_t*)g_mems_buf, L3GD20_OUT_X_L_ADDR, MEMS_CHAN_NO*2);
		printf("X: %d, Y: %d, Z: %d\n", 
		g_mems_buf[0],
		g_mems_buf[1],
		g_mems_buf[2]
		);
		
		/* Read Gyro Angular data */
		BSP_GYRO_GetXYZ(Buffer);
		printf("%.3f %.3f %.3f\n",
		Buffer[0], Buffer[1], Buffer[2]
		);
	
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		
		xSemaphoreTake(notification_semaphore, portMAX_DELAY);		
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
//		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);

		xSemaphoreGive(notification_semaphore);		
		
		uint32_t tmpTick = osKernelSysTick();
		xQueueSend(Queue_id, &tmpTick, 0);

    osDelay(1000);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
