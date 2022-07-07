/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "mpu9250.h"
#include "packet.h"
#include "MadgwickAHRS.h"
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
 I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId uwbTaskHandle;
osThreadId imuTaskHandle;
osThreadId comTaskHandle;
osThreadId motionTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void UWBTask(void const * argument);
void IMUTask(void const * argument);
void COMTask(void const * argument);
void MOTIONTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define GPIO_Pin_SET  	1
#define GPIO_Pin_RESET  0

/*##packet Init##*/
structLocation location;
structRingoMode ringomode;

/*##IMU Init##*/
MPU9250_t mpu9250;

uint8_t isDeviceConnected = 0;

HAL_StatusTypeDef whoAmI_Check(MPU9250_t *mpu9250);

/*##UWB Init##*/


uint8_t rx_data;
uart_t 	uart;
uint8_t UWB_Tx_init = 0x0D;
uint8_t UWB_Tx_lec[] = "lec\n";

// added by commidi
uint8_t g_strReceive[MAX_BUFFER_SIZE];
uint8_t g_strCommand[MAX_BUFFER_SIZE];
unsigned int g_udCount = 0;
unsigned char g_ucChkCommand = 0;

void BusControlLoop( void );
void init_uart(uart_t* u);
void push(uart_t*, uint8_t);
uint8_t pop(uart_t*);
uint8_t isEmpty(uart_t*);
int ParseUWBStr(uint8_t *strInput, structUWB *pUwb);

int _write(int file, char *ptr, int len)
{
	for(int i=0; i<len; i++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  int tick_count = 0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uwbTask */
  osThreadDef(uwbTask, UWBTask, osPriorityIdle, 0, 1024);
  uwbTaskHandle = osThreadCreate(osThread(uwbTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, IMUTask, osPriorityIdle, 0, 512);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of comTask */
  osThreadDef(comTask, COMTask, osPriorityHigh, 0, 512);
  comTaskHandle = osThreadCreate(osThread(comTask), NULL);

  /* definition and creation of motionTask */
  osThreadDef(motionTask, MOTIONTask, osPriorityHigh, 0, 512);
  motionTaskHandle = osThreadCreate(osThread(motionTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  printf("%d\n", tick_count++);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
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
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_NFC_Pin|LED_UWB_Pin|LED_IMU_Pin|LED_MDR_Pin
                          |LED_MDL_Pin|LED_COM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : JSLEFT_SWF_Pin JSLEFT_SWB_Pin JSLEFT_SWH_Pin JSLEFT_SPD_Pin */
  GPIO_InitStruct.Pin = JSLEFT_SWF_Pin|JSLEFT_SWB_Pin|JSLEFT_SWH_Pin|JSLEFT_SPD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : JSRIGHT_SWF_Pin JSRIGHT_SWB_Pin JSRIGHT_SWH_Pin JSRIGHT_SPD_Pin */
  GPIO_InitStruct.Pin = JSRIGHT_SWF_Pin|JSRIGHT_SWB_Pin|JSRIGHT_SWH_Pin|JSRIGHT_SPD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_NFC_Pin LED_UWB_Pin LED_IMU_Pin LED_MDR_Pin
                           LED_MDL_Pin LED_COM_Pin */
  GPIO_InitStruct.Pin = LED_NFC_Pin|LED_UWB_Pin|LED_IMU_Pin|LED_MDR_Pin
                          |LED_MDL_Pin|LED_COM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    push(&uart, rx_data);
    HAL_UART_Receive_IT(&huart3, &rx_data, 1);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UWBTask */
/**
* @brief Function implementing the uwbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UWBTask */
void UWBTask(void const * argument)
{
  /* USER CODE BEGIN UWBTask */
	uint8_t data;
	char l_cStatusFlag = STREAM_STATUS_READY;
	char l_cHeader[4] = { 0, };
	char l_cEnder[2] = { 0, };
	unsigned int l_udChkTimeout = 0;
	structUWB l_struUwbInfo;

	init_uart(&uart);
	HAL_UART_Receive_IT(&huart3, &rx_data, 1);

	HAL_UART_Transmit_IT(&huart3, &UWB_Tx_init, sizeof(UWB_Tx_init));
	HAL_Delay(1);
	HAL_UART_Transmit_IT(&huart3, &UWB_Tx_init, sizeof(UWB_Tx_init));
	HAL_Delay(2000);

  /* Infinite loop */
  for(;;)
  {
	  if (isEmpty(&uart) == 0)
	  {
		  data = pop(&uart);

		  l_cHeader[3] = l_cHeader[2];
		  l_cHeader[2] = l_cHeader[1];
		  l_cHeader[1] = l_cHeader[0];
		  l_cHeader[0] = data;

		  l_cEnder[1] = l_cEnder[0];
		  l_cEnder[0] = data;

		  if ((l_cHeader[3] == 'D') && (l_cHeader[2] == 'I') && (l_cHeader[1] == 'S') && (l_cHeader[0] == 'T'))
		  {
			  // ?��?�� 조건..
			  g_udCount = 0;
			  g_strReceive[0] = l_cHeader[0]; g_udCount++;
			  g_strReceive[1] = l_cHeader[1]; g_udCount++;
			  g_strReceive[2] = l_cHeader[2]; g_udCount++;
			  g_strReceive[3] = l_cHeader[3]; g_udCount++;

			  l_udChkTimeout = 0 ;

			  l_cStatusFlag = STREAM_STATUS_START;

			  //Debug_Serial.println("Start..........................");
		  }
		  else if ((l_cHeader[1] == 0x0D) && (l_cHeader[0] == 0x0A))
		  {
			  //// ?��조건..
			  //Debug_Serial.println("End..........................");
			  if (l_cStatusFlag == STREAM_STATUS_START)
			  {
					  g_strReceive[g_udCount] = '\0';
					  memcpy(g_strCommand, g_strReceive, g_udCount + 1);

					  //Debug_Serial.println(g_strCommand);

					  if(ParseUWBStr(g_strCommand, &l_struUwbInfo) != 0)
					  {
						  location.positionX = l_struUwbInfo.l_fPosX * 100;
						  location.positionY = l_struUwbInfo.l_fPosY * 100;
						  location.pqf = l_struUwbInfo.l_udPosProb;
					  }

					  HAL_GPIO_WritePin(LED_COM_GPIO_Port, LED_COM_Pin, GPIO_PIN_RESET);
					  g_ucChkCommand = 1;

					  g_udCount = 0;
					  g_strReceive[g_udCount] = '\0';
			  }
			  else
			  {
				  g_udCount = 0;
				  g_strReceive[g_udCount] = '\0';
			  }
			  l_cStatusFlag = STREAM_STATUS_READY;
		  }
		  else
		  {
			  if (l_cStatusFlag == STREAM_STATUS_START)
			  {
				  g_strReceive[g_udCount] = data;
				  g_udCount++;

				  if (g_udCount > MAX_BUFFER_SIZE)
				  {
					  g_strReceive[g_udCount] = '\0';
					  l_cStatusFlag = STREAM_STATUS_READY;
					  g_udCount = 0;
				  }
				  // 버퍼?��..?��?��?�� ???��.
			  }

		  }
	  }


	  HAL_GPIO_WritePin(LED_COM_GPIO_Port, LED_COM_Pin, GPIO_PIN_SET);
	  l_udChkTimeout++;
	  if (l_udChkTimeout > 5000)
	  {
		  // lec
		  HAL_UART_Transmit_IT(&huart3, UWB_Tx_lec, sizeof(UWB_Tx_lec));
		  l_udChkTimeout = 0;
	  }
    osDelay(1);
  }
  /* USER CODE END UWBTask */
}

/* USER CODE BEGIN Header_IMUTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUTask */
void IMUTask(void const * argument)
{
  /* USER CODE BEGIN IMUTask */
	MPU9250_Init(&mpu9250, MPU9250_Device_0, ACCEL_SCALE_16G, GYRO_SCALE_2000dps, MAG_SCALE_16bit);
  /* Infinite loop */
  for(;;)
  {
	  if (whoAmI_Check(&mpu9250) != HAL_ERROR)
				isDeviceConnected = 1;
			else
				isDeviceConnected = 0;
	  MPU9250_ReadAcc(&mpu9250);
	  MPU9250_ReadGyro(&mpu9250);
	  MPU9250_ReadMag(&mpu9250);
	  MPU9250_ReadTemperature(&mpu9250);
	  MadgwickAHRSupdate(mpu9250.gyro[0], mpu9250.gyro[1], mpu9250.gyro[2], mpu9250.acc[0], mpu9250.acc[1], mpu9250.acc[2], mpu9250.mag[0], mpu9250.mag[1], mpu9250.mag[2]);
	  //MPU_ComplementaryFilter(&mpu9250);
	  ConvertQuaterToEuler(location.g_fq[0], location.g_fq[1], location.g_fq[2], location.g_fq[3]);
    osDelay(1);
  }
  /* USER CODE END IMUTask */
}

/* USER CODE BEGIN Header_COMTask */
/**
* @brief Function implementing the comTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_COMTask */
void COMTask(void const * argument)
{
  /* USER CODE BEGIN COMTask */
	extern uint8_t busbuffer_rx[];
	extern uint8_t busbuffer_tx[];
	extern uint8_t busbuffer_size;
	ringomode.g_udStatusData = READY;

	memset(busbuffer_rx, 0x00, busbuffer_size);
	memset(busbuffer_tx, 0x00, busbuffer_size);

	//init_spiadd();

	HAL_SPI_DMAStop(&hspi1);
	HAL_SPI_TransmitReceive_DMA(&hspi1, busbuffer_tx, busbuffer_rx, busbuffer_size);

  /* Infinite loop */
  for(;;)
  {
	  CarNum_Init();
	  BusControlLoop();
	  osDelay(1);
  }
  /* USER CODE END COMTask */
}

/* USER CODE BEGIN Header_MOTIONTask */
/**
* @brief Function implementing the motionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MOTIONTask */
void MOTIONTask(void const * argument)
{
  /* USER CODE BEGIN MOTIONTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MOTIONTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
