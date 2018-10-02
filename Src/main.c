
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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "retarget_io_drv.h"
#include "asm_prototype.h"

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
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#endif

#include "gyro.h"
#include "l3gd20.h"

#define TEMP_REFRESH_PERIOD   1000    /* Internal temperature refresh period */
#define MAX_CONVERTED_VALUE   4095    /* Max converted value */
#define	T30_VAL_3300	*(uint16_t*)(0x1FFFF7B8)
#define	T110_VAL_3300	*(uint16_t*)(0x1FFFF7C2)
#define	VREF_VAL_3300	*(uint16_t*)(0x1FFFF7BA)
#define VREF                  VDD_MV

#define SPIx_TIMEOUT_MAX                      ((uint32_t)0x1000)

/* Read/Write command */
#define READWRITE_CMD                         ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD                      ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                            ((uint8_t)0x00)

/* Chip Select macro definition */
#define GYRO_CS_LOW()       HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()      HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  GYRO SPI Interface pins
  */
#define GYRO_CS_GPIO_PORT            GPIOC                       /* GPIOC */
#define GYRO_CS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define GYRO_CS_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOC_CLK_DISABLE()
#define GYRO_CS_PIN                  GPIO_PIN_0                  /* PC.00 */

#define GYRO_INT_GPIO_PORT               GPIOC                       /* GPIOC */
#define GYRO_INT_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define GYRO_INT_GPIO_CLK_DISABLE()      __HAL_RCC_GPIOC_CLK_DISABLE()
#define GYRO_INT1_PIN                    GPIO_PIN_1                  /* PC.01 */
#define GYRO_INT1_EXTI_IRQn              EXTI0_1_IRQn 
#define GYRO_INT2_PIN                    GPIO_PIN_2                  /* PC.02 */
#define GYRO_INT2_EXTI_IRQn              EXTI2_3_IRQn 

#define ABS(x)         (x < 0) ? (-x) : x

typedef enum 
{
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
} 
GYRO_StatusTypeDef;

extern ADC_HandleTypeDef hadc;
extern SPI_HandleTypeDef hspi2;
extern GYRO_DrvTypeDef L3gd20Drv;

static GYRO_DrvTypeDef *GyroscopeDrv;

/******************************* SPI Routines**********************************/
/**
  * @brief SPI1 Bus initialization
  * @retval None
  */

/**
  * @brief SPI1 error treatment function
  * @retval None
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI comunication BUS */
  HAL_SPI_DeInit(&hspi2);
  
  /* Re- Initiaize the SPI comunication BUS */
  MX_SPI2_Init();
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
static uint8_t SPIx_WriteRead(uint8_t Byte)
{

  uint8_t receivedbyte = 0;
  
  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, SPIx_TIMEOUT_MAX) != HAL_OK)
  {
    SPIx_Error();
  }
  
  return receivedbyte;
}

/**
  * @}
  */ 

/** @addtogroup STM32F072B_DISCOVERY_LINK_Operations_Functions
  * @{
  */ 

/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK GYRO *****************************/
/**
  * @brief  Configures the GYRO SPI interface.
  * @retval None
  */
void GYRO_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure the Gyroscope Control pins ------------------------------------------*/
  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */  
  GYRO_CS_GPIO_CLK_ENABLE();  
  GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GYRO_CS_HIGH();

  /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
  GYRO_INT_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_INT1_PIN | GYRO_INT2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull= GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GPIO_InitStructure);
  
  MX_SPI2_Init();
}

/**
  * @brief  Writes one byte to the GYRO.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the GYRO.
  * @param  WriteAddr : GYRO's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void GYRO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit: 
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(WriteAddr);
  
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GYRO_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the GYROSCOPE.
  * @param  pBuffer : pointer to the buffer that receives the data read from the GYROSCOPE.
  * @param  ReadAddr : GYROSCOPE's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the GYROSCOPE.
  * @retval None
  */
void GYRO_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to GYROSCOPE (Slave device) */
    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GYRO_CS_HIGH();
}  

uint8_t mems_id;
uint8_t BSP_GYRO_Init(void)
{  
  uint8_t ret = GYRO_ERROR;
  uint16_t ctrl = 0x0000;
  GYRO_InitTypeDef L3GD20_InitStructure;
  GYRO_FilterConfigTypeDef L3GD20_FilterStructure={0,0};

	mems_id = L3gd20Drv.ReadID();
	
  if((mems_id == I_AM_L3GD20) || (mems_id == I_AM_L3GD20_TR))
  {
    /* Initialize the gyroscope driver structure */
    GyroscopeDrv = &L3gd20Drv;

    /* Configure Mems : data rate, power mode, full scale and axes */
    L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
    L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
    L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
    L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
    L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
    L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
    L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; 
	
    /* Configure MEMS: data rate, power mode, full scale and axes */
    ctrl = (uint16_t) (L3GD20_InitStructure.Power_Mode | L3GD20_InitStructure.Output_DataRate | \
                      L3GD20_InitStructure.Axes_Enable | L3GD20_InitStructure.Band_Width);
	
    ctrl |= (uint16_t) ((L3GD20_InitStructure.BlockData_Update | L3GD20_InitStructure.Endianness | \
                        L3GD20_InitStructure.Full_Scale) << 8);

    /* L3gd20 Init */	 
    GyroscopeDrv->Init(ctrl);
  
    L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
    L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	
    ctrl = (uint8_t) ((L3GD20_FilterStructure.HighPassFilter_Mode_Selection |\
                       L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency));		
	
    GyroscopeDrv->FilterConfig(ctrl) ;
  
    GyroscopeDrv->FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
	
    ret = GYRO_OK;
  }
  else
  {
    ret = GYRO_ERROR;
  }
  
  return ret;
}

/**
  * @brief  Read ID of Gyroscope component
  * @retval ID
  */
uint8_t BSP_GYRO_ReadID(void)
{
  uint8_t id = 0x00;

  if(GyroscopeDrv->ReadID != NULL)
  {
    id = GyroscopeDrv->ReadID();
  }  
  return id;
}

/**
  * @brief  Reboot memory content of GYRO
  * @retval None
  */
void BSP_GYRO_Reset(void)
{
  if(GyroscopeDrv->Reset != NULL)
  {
    GyroscopeDrv->Reset();
  }  
}

/**
  * @brief  Configure INT1 interrupt
  * @param  pIntConfig: pointer to a L3GD20_InterruptConfig_TypeDef 
  *         structure that contains the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */
void BSP_GYRO_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfig)
{  
uint16_t interruptconfig = 0x0000;

  if(GyroscopeDrv->ConfigIT != NULL)
  {
    /* Configure latch Interrupt request and axe interrupts */                   
    interruptconfig |= ((uint8_t)(pIntConfig->Latch_Request| \
                                  pIntConfig->Interrupt_Axes) << 8);
                   
    interruptconfig |= (uint8_t)(pIntConfig->Interrupt_ActiveEdge);
 
	GyroscopeDrv->ConfigIT(interruptconfig);
  }  
}

/**
  * @brief  Enable INT1 or INT2 interrupt
  * @param  IntPin: Interrupt pin 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  * @retval None
  */
void BSP_GYRO_EnableIT(uint8_t IntPin)
{  
  if(GyroscopeDrv->EnableIT != NULL)
  {
	GyroscopeDrv->EnableIT(IntPin);
  }  
}

/**
  * @brief  Disable INT1 or INT2 interrupt
  * @param  IntPin: Interrupt pin 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  * @retval None
  */
void BSP_GYRO_DisableIT(uint8_t IntPin)
{  
  if(GyroscopeDrv->DisableIT != NULL)
  {
    GyroscopeDrv->DisableIT(IntPin);
  }  
}

/**
  * @brief  Get XYZ angular acceleration
  * @param pfData: pointer on floating array         
  * @retval None
  */
void BSP_GYRO_GetXYZ(float* pfData)
{
  if(GyroscopeDrv->GetXYZ!= NULL)
  {
	GyroscopeDrv->GetXYZ(pfData);
  }  
}

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
    Xval = ABS((int8_t)(Buffer[0]));
    Yval = ABS((int8_t)(Buffer[1]));
    
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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
__IO int16_t g_adc_buf[ADC_CHAN_NO];
__IO int16_t g_mems_buf[MEMS_CHAN_NO];

//Re-implement any functions that require re-implementation.

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
	uint32_t tmpTick;	
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
/* USER CODE BEGIN 1 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Test Addition/Mulitiplication Cycles
		#define	TEST_ADD_MUL_NUM	500000
			//If the muliplication takes similar cycles, it is a single cycle multiplication implementation
			tmpTick = HAL_GetTick();
			for(uint32_t i=0; i<TEST_ADD_MUL_NUM; ++i)
			{
				uint32_t tn = 101;
				asm_simple_add(tn, 456);
			}
			tmpTick = HAL_GetTick()-tmpTick;
			printf("A:%u\n", tmpTick);	
			
			tmpTick = HAL_GetTick();
			for(uint32_t i=0; i<TEST_ADD_MUL_NUM; ++i)
			{		
				uint32_t tn = 101;
				asm_simple_mul(tn, 456);
			}
			tmpTick = HAL_GetTick()-tmpTick;
			printf("M:%u\n", tmpTick);	
		
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
		mems_id, mems_tmp, JTemp, VRef, VBat);
				
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/SYS_TICK_HZ);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
