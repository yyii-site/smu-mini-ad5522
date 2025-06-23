/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD5522.h"
#include "ad7190.h"
#include "dev_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUF_SIZE 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* USER CODE BEGIN PV */
static void AD7190_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask01(void *argument);
void Callback01(void *argument);

/* USER CODE BEGIN PFP */
handle_AD5522 h_PMU;
__IO uint16_t ADC_temp[5];
__IO uint16_t ADC_cnt = 5;
__IO uint16_t ADC_ptr = 0;

double input_range[2] = {0,3.3/3.3*65535};
double output_Irange[2] = {-150e-6,150e-6};
double output_Vrange[2] = {0,0.6};

// 将 clk 的电平切换到 High
void SPI_Send_Nop() {
  const uint8_t nop_data = 0;
  HAL_SPI_Transmit(&hspi1, &nop_data, 1, 1000);
}
void AD7190_CS_Low() {
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
}
void AD7190_CS_High() {
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
}
void AD7190_Spi_WriteByte(uint8_t cmd) {
  int resp = HAL_SPI_Transmit(&hspi1, &cmd,1,1000);
}
uint8_t AD7190_Spi_ReadByte() {
  uint8_t val=0;
  int resp = HAL_SPI_TransmitReceive(&hspi1, 0x00, &val,1,1000);
  return val;
}
void AD7190_Spi_Write(uint8_t* cmd, uint16_t len){
  int resp = HAL_SPI_Transmit(&hspi1, cmd, len, 1000);
}
void AD7190_Spi_Read(uint8_t* cmd, uint16_t len) {
  for (int i=0;i<len;i++) {
    uint8_t send = cmd[i];
    uint8_t read = 0;
    int resp = HAL_SPI_TransmitReceive(&hspi1, &send, &read, 1, 1000);
    cmd[i] = read;
  }
}
bool AD7190_CheckDataReadyPin(void) {
  return HAL_GPIO_ReadPin(ADC_READY_GPIO_Port, ADC_READY_Pin);
}
void AD7190_Delay_ms(uint32_t ms) {
  osDelay(ms);
}
AD7190_SpiDriver_Typedef h_ad7190 = {
  .AD7190_CS_Low = AD7190_CS_Low,
  .AD7190_CS_High = AD7190_CS_High,
  .AD7190_Spi_WriteByte = AD7190_Spi_WriteByte,
  .AD7190_Spi_ReadByte = AD7190_Spi_ReadByte,
  .AD7190_Spi_Write = AD7190_Spi_Write,
  .AD7190_Spi_Read = AD7190_Spi_Read,
  .AD7190_CheckDataReadyPin = AD7190_CheckDataReadyPin,
  .AD7190_Delay_ms = AD7190_Delay_ms
};
uint32_t ad7190_val[4];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char uart1_tx_buf[UART_BUF_SIZE];
char uart1_rx_buf[UART_BUF_SIZE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    if (HAL_UART_RXEVENT_TC == HAL_UARTEx_GetRxEventType(&huart1)) {
      sprintf(uart1_tx_buf, "rx_event_tc%d", Size);
      HAL_UART_Transmit(&huart1, (uint8_t *)uart1_tx_buf, strlen(uart1_tx_buf)+1, 100);
      uart_dmarx_done_isr(DEV_UART1);
    } else if (HAL_UART_RXEVENT_HT == HAL_UARTEx_GetRxEventType(&huart1)) {
      sprintf(uart1_tx_buf, "rx_event_ht%d", Size);
      HAL_UART_Transmit(&huart1, (uint8_t *)uart1_tx_buf, strlen(uart1_tx_buf)+1, 100);
      uart_dmarx_half_done_isr(DEV_UART1, Size);
    } else if (HAL_UART_RXEVENT_IDLE == HAL_UARTEx_GetRxEventType(&huart1)) {
      sprintf(uart1_tx_buf, "rx_event_idle%d", Size);
      HAL_UART_Transmit(&huart1, (uint8_t *)uart1_tx_buf, strlen(uart1_tx_buf)+1, 100);
      uart_dmarx_idle_isr(DEV_UART1, Size);
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uart_device_init(DEV_UART1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask01 */
  myTask01Handle = osThreadNew(StartTask01, NULL, &myTask01_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SMU_CS_Pin|ADC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED4_Pin|LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SMU_CS_Pin ADC_CS_Pin */
  GPIO_InitStruct.Pin = SMU_CS_Pin|ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SMU_BUSY_Pin ADC_READY_Pin */
  GPIO_InitStruct.Pin = SMU_BUSY_Pin|ADC_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LED5_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  uint16_t readsize;
  /* Infinite loop */
  for(;;)
  {
    readsize = uart_read(1, uart1_rx_buf, UART_BUF_SIZE);
    if (readsize) {
      readsize = 0;
    }
    
    HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);
    osDelay(166);
    HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
    osDelay(166);
    HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_SET);
    osDelay(166);
    HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_RESET);
    osDelay(166);
    HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
    osDelay(166);
    HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
    osDelay(166);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask01 */
/**
* @brief Function implementing the myTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN StartTask01 */
  //----------------------------AD5522 Init-----------------------------
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
  AD5522_init(&h_PMU,&hspi1,5.0);
  AD5522_Calibrate(&h_PMU);
	AD5522_StartHiZMV(&h_PMU,PMU_CH_2|PMU_CH_3) ;//configure CH2/3 to monitor voltage only
	// AD5522_SetClamp(&h_PMU,PMU_CH_0|PMU_CH_1,32767-30000,32767+30000,0,65535,PMU_DAC_SCALEID_EXT);
	AD5522_SetClamp_float(&h_PMU,PMU_CH_0|PMU_CH_1,-2e-3,2e-3,-0.1,0.5,PMU_DAC_SCALEID_200UA);
	AD5522_StartFVMI(&h_PMU,PMU_CH_0|PMU_CH_1,PMU_DAC_SCALEID_2MA); 
	// AD5522_StartFIMV(&h_PMU,PMU_CH_0|PMU_CH_1,PMU_DAC_SCALEID_200UA);

  __IO uint16_t value = 0;
	const uint16_t test_len = 3;
	// __IO uint16_t value_inc = 0;
	// uint16_t test_sig[5] = {0,32768,65535};
	float    test_float_V[3] = {-5,0,5};
	// float    test_float_I[3] = {1e-3,0,-1e-3};


  //----------------------------AD7190 Init-----------------------------
  HAL_SPI_DeInit(&hspi1);
  AD7190_SPI1_Init();
  SPI_Send_Nop();

  if (!AD7190_Init(&h_ad7190)) {
    ;  // AD7190 init failed
  }
  AD7190_RangeSetup(&h_ad7190, 1, AD7190_CONF_GAIN_1);  // ADC In unipolar 单极性0-5V
  AD7190_Calibrate(&h_ad7190, AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AINCOM);
  AD7190_Calibrate(&h_ad7190, AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AINCOM);
  AD7190_Calibrate(&h_ad7190, AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN2P_AINCOM);
  AD7190_Calibrate(&h_ad7190, AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN2P_AINCOM);
  AD7190_Calibrate(&h_ad7190, AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN3P_AINCOM);
  AD7190_Calibrate(&h_ad7190, AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN3P_AINCOM);
  AD7190_Calibrate(&h_ad7190, AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN4P_AINCOM);
  AD7190_Calibrate(&h_ad7190, AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN4P_AINCOM);
  AD7190_ChannelSelectAll(&h_ad7190);
  AD7190_ContinueMode_StatusAfterData(&h_ad7190);
  AD7190_AutoContinueMode(&h_ad7190, true);
  uint8_t adc_buf_channel = 0;
  uint32_t adc_buf_value = 0;


  uint8_t loop_count = 0;
  /* Infinite loop */
  for(;;)
  {
    loop_count++;
    if (0 == loop_count%2000)
    {
      HAL_SPI_DeInit(&hspi1);
      MX_SPI1_Init();	
      //AD5522_SetOutputCurrent(&h_PMU,PMU_CH_0|PMU_CH_1,value_inc);value_inc+=10;
      //AD5522_SetOutputVoltage(&h_PMU,PMU_CH_0|PMU_CH_1,test_sig[value++]);
      //AD5522_SetOutputVoltage(&h_PMU,PMU_CH_0|PMU_CH_1,32767+32767*wave[value++]);
      AD5522_SetOutputVoltage_float(&h_PMU,PMU_CH_0|PMU_CH_1,test_float_V[value]);
      //AD5522_SetOutputCurrent_float(&h_PMU,PMU_CH_0|PMU_CH_1,test_float_I[value]);
      //AD5522_SetClamp_float(&h_PMU,PMU_CH_0|PMU_CH_1,-10e-3,test_float_I[value],-0.6,0.6,PMU_DAC_SCALEID_2MA);
      value+=1;
      if(value >= test_len)
        value = 0;
    }
    if (0 == loop_count%12)
    {
      HAL_SPI_DeInit(&hspi1);
      AD7190_SPI1_Init();
      SPI_Send_Nop();
      if (AD7190_ReadDataRegister(&h_ad7190, &adc_buf_channel, &adc_buf_value)) {
        ad7190_val[adc_buf_channel] = adc_buf_value;
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartTask01 */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
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
