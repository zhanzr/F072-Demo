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
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

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
TaskHandle_t g_task01_handle;
TaskHandle_t g_task02_handle;
SemaphoreHandle_t g_noti_sema;
QueueHandle_t g_queue;
TimerHandle_t g_timer;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Determine whether we are in thread mode or handler mode. */
static int inHandlerMode (void)
{
  return __get_IPSR() != 0;
}

uint32_t getKernelSysTick(void)
{
  if (inHandlerMode()) {
    return xTaskGetTickCountFromISR();
  }
  else {
    return xTaskGetTickCount();
  }
}

extern void xPortSysTickHandler(void);

void freertos_tick_handler(void)
{
	#if (INCLUDE_xTaskGetSchedulerState  == 1 )
		if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
		{
	#endif  /* INCLUDE_xTaskGetSchedulerState */  
			xPortSysTickHandler();
	#if (INCLUDE_xTaskGetSchedulerState  == 1 )
		}
	#endif  /* INCLUDE_xTaskGetSchedulerState */  
}

void TimerCallback( xTimerHandle pxtimer )
{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
}
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
	xTaskCreate((TaskFunction_t)StartDefaultTask,
							(const portCHAR *)"defaultTask",
							512,
							NULL,
							2,
							&g_task01_handle);

  /* definition and creation of myTask02 */
	xTaskCreate((TaskFunction_t)StartTask02,
							(const portCHAR *)"myTask02",
							64,
							NULL,
							3,
							&g_task02_handle);
							
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
/* Create one Software Timer.*/
	g_timer = xTimerCreate("Timer", 
							200/ portTICK_PERIOD_MS,
							pdTRUE,
							0,
							TimerCallback);
	xTimerStart( g_timer, 0);

	/* Create the notification semaphore and set the initial state. */
	vSemaphoreCreateBinary(g_noti_sema);
	vQueueAddToRegistry(g_noti_sema, "Notification Semaphore");
	xSemaphoreTake(g_noti_sema, 0);

	/* Create a queue*/
	g_queue = xQueueCreate(2, sizeof(uint32_t));
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
	uint32_t tmpTicks;	
	
  float Buffer[6] = {0};  
	
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(g_queue, &tmpTicks, portMAX_DELAY);
		
		printf("%s %u %u\n", 
		tskKERNEL_VERSION_NUMBER,
		tmpTicks,
		g_adc_buf[0]
		);

		GYRO_IO_Read((uint8_t*)&mems_tmp, L3GD20_OUT_TEMP_ADDR, 1);
			
		JTemp = ((80*g_adc_buf[0]*VDD_MV)/3300 + 30*T110_VAL_3300 - 110*T30_VAL_3300)/(T110_VAL_3300-T30_VAL_3300);
		
		printf("%02X %d %d %d %d\n",
		g_mems_id, mems_tmp, JTemp, ADC_2_MV(g_adc_buf[1]), 2 * ADC_2_MV(g_adc_buf[2]));
				
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
		
		xSemaphoreTake(g_noti_sema, portMAX_DELAY);	

		char tmpBuf[1024];
		vTaskList(tmpBuf);
		printf(tmpBuf);
		
		printf("Total Heap:%u\n", 
		configTOTAL_HEAP_SIZE);
//		vTaskGetRunTimeStats(tmpBuf);
//		printf(tmpBuf);		

		printf("\n");
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

		xSemaphoreGive(g_noti_sema);		
		
		uint32_t tmpTick = getKernelSysTick();
		xQueueSend(g_queue, &tmpTick, 0);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
