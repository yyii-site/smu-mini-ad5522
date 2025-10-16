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
#include <stdio.h>
#include "semphr.h"
#include "AD5522.h"
#include "ad7190.h"
#include "dev_uart.h"
#include "scpi/scpi.h"
#include "scpi-def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUF_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

SemaphoreHandle_t semaphore_scpi;
SemaphoreHandle_t semaphore_spi1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
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
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  // HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

handle_AD5522 h_PMU;
void AD5522_in(void) {
  xSemaphoreTake(semaphore_spi1, portMAX_DELAY);
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
}
void AD5522_out(void) {
  xSemaphoreGive(semaphore_spi1);
}

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
  HAL_SPI_Transmit(&hspi1, &cmd,1,1000);
}
uint8_t AD7190_Spi_ReadByte() {
  uint8_t val=0;
  int resp = HAL_SPI_TransmitReceive(&hspi1, 0x00, &val,1,1000);
  if (resp != HAL_OK) {
    return 0;
  }
  return val;
}
void AD7190_Spi_Write(uint8_t* cmd, uint16_t len){
  HAL_SPI_Transmit(&hspi1, cmd, len, 1000);
}
void AD7190_Spi_Read(uint8_t* cmd, uint16_t len) {
  for (int i=0;i<len;i++) {
    uint8_t send = cmd[i];
    uint8_t read = 0;
    int resp = HAL_SPI_TransmitReceive(&hspi1, &send, &read, 1, 1000);
    if (resp != HAL_OK) {
      return;
    }
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
uint8_t uart1_tx_buf[UART_BUF_SIZE];
uint8_t uart1_rx_buf[UART_BUF_SIZE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    if (HAL_UART_RXEVENT_TC == HAL_UARTEx_GetRxEventType(&huart1)) {
      // sprintf(uart1_tx_buf, "rx_event_tc%d", Size);
      // HAL_UART_Transmit(&huart1, (uint8_t *)uart1_tx_buf, strlen(uart1_tx_buf)+1, 100);
      uart_dmarx_done_isr(DEV_UART1);
    } else if (HAL_UART_RXEVENT_HT == HAL_UARTEx_GetRxEventType(&huart1)) {
      // sprintf(uart1_tx_buf, "rx_event_ht%d", Size);
      // HAL_UART_Transmit(&huart1, (uint8_t *)uart1_tx_buf, strlen(uart1_tx_buf)+1, 100);
      uart_dmarx_half_done_isr(DEV_UART1, Size);
    } else if (HAL_UART_RXEVENT_IDLE == HAL_UARTEx_GetRxEventType(&huart1)) {
      // sprintf(uart1_tx_buf, "rx_event_idle%d", Size);
      // HAL_UART_Transmit(&huart1, (uint8_t *)uart1_tx_buf, strlen(uart1_tx_buf)+1, 100);
      uart_dmarx_idle_isr(DEV_UART1, Size);
    }
  }
}

size_t SCPI_Write(scpi_t * context, const char * data, size_t len) {
    (void) context;
    if (HAL_OK == HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000)) {
      return len;
    } else {
      return 0;
    }
}

scpi_result_t SCPI_Flush(scpi_t * context) {
    (void) context;
    return SCPI_RES_OK;
}

int SCPI_Error(scpi_t * context, int_fast16_t err) {
    (void) context;

    char _err[50];
    sprintf(_err, "**ERROR: %d, \"%s\"\r\n", (int16_t) err, SCPI_ErrorTranslate(err));
    HAL_UART_Transmit(&huart1, (uint8_t *)_err, strlen(_err), 1000);
    return 0;
}

scpi_result_t SCPI_Control(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val) {
    (void) context;

    char _err[50];
    if (SCPI_CTRL_SRQ == ctrl) {
        sprintf(_err, "**SRQ: 0x%X (%d)\r\n", val, val);
    } else {
        sprintf(_err, "**CTRL %02x: 0x%X (%d)\r\n", ctrl, val, val);
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)_err, strlen(_err), 1000);

    return SCPI_RES_OK;
}

scpi_result_t SCPI_Reset(scpi_t * context) {
    (void) context;

    char _err[50];
    sprintf(_err, "**Reset\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)_err, strlen(_err), 1000);

    return SCPI_RES_OK;
}

scpi_result_t SCPI_SystemCommTcpipControlQ(scpi_t * context) {
    (void) context;

    return SCPI_RES_ERR;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  semaphore_scpi = xSemaphoreCreateMutex();
  semaphore_spi1 = xSemaphoreCreateMutex();

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
  printf("Characters: %c %c\n", 'a', 65);
  printf("Decimals: %d %ld\n", 1977, 650000L);
  printf("Preceding with blanks: %10d\n", 1977);
  printf("Preceding with zeros: %010d\n", 1977);
  printf("Some different radices: %d %x %o %#x %#o\n", 100, 100, 100, 100, 100);
  printf("floats: %4.2f %+.0e %E\n", 3.1416, 3.1416, 3.1416);
  printf("Width trick: %*d\n", 5, 10);
  printf("%s\n", "A string");

  
  //----------------------------AD5522 Init-----------------------------
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
  AD5522_init(&h_PMU,&hspi1,5.0);
  AD5522_Calibrate(&h_PMU);
  AD5522_StartHiZMV(&h_PMU,PMU_CH_1|PMU_CH_2|PMU_CH_3) ;//configure CH2/3 to monitor voltage only
  // AD5522_SetClamp(&h_PMU,PMU_CH_0|PMU_CH_1,32767-30000,32767+30000,0,65535,PMU_DAC_SCALEID_EXT);
  AD5522_SetClamp_float(&h_PMU,PMU_CH_0|PMU_CH_1,-80e-3,80e-3,-11.5,11.5,PMU_DAC_SCALEID_2MA);
  AD5522_StartFVMI(&h_PMU,PMU_CH_0,PMU_DAC_SCALEID_2MA); 
  // AD5522_StartFIMV(&h_PMU,PMU_CH_0|PMU_CH_1,PMU_DAC_SCALEID_200UA);
  // AD5522_SetOutputVoltage_float(&h_PMU,PMU_CH_0,-1.2);
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;  // 非常重要! AD5522使用LOW; AD7190使用HIGH
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
  SCPI_Init(&scpi_context,
      scpi_commands,
      &scpi_interface,
      scpi_units_def,
      SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
      scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
      scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);
  /* Infinite loop */
  for(;;)
  {
    readsize = uart_read(1, uart1_rx_buf, UART_BUF_SIZE);
    if (readsize) {
      HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
      xSemaphoreTake(semaphore_scpi, portMAX_DELAY);
      SCPI_Input(&scpi_context, (const char*)uart1_rx_buf, readsize);
      xSemaphoreGive(semaphore_scpi);
      HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
    }
    osDelay(10);
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
  uint8_t adc_buf_channel = 0;
  uint32_t adc_buf_value = 0;

  //----------------------------AD7190 Init-----------------------------
  if (xSemaphoreTake(semaphore_spi1, portMAX_DELAY)) {
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
    xSemaphoreGive(semaphore_spi1);
  }

  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(semaphore_spi1, portMAX_DELAY)) {
      HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_SET);
      if (hspi1.Init.CLKPolarity != SPI_POLARITY_HIGH) { //减少重复初始化
        HAL_SPI_DeInit(&hspi1);
        AD7190_SPI1_Init();
        SPI_Send_Nop();
      }
      if (AD7190_ReadDataRegister(&h_ad7190, &adc_buf_channel, &adc_buf_value)) {
        ad7190_val[adc_buf_channel] = adc_buf_value;
      }
      xSemaphoreGive(semaphore_spi1);
      HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_RESET);
    }
    osDelay(12);
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
