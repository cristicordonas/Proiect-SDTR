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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t Temperature;
  uint8_t Humidity;
} DHT11_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PORT GPIOA
#define DHT11_PIN  GPIO_PIN_1

#define LED_PORT GPIOB
#define LED_RED_PIN   GPIO_PIN_3
#define LED_GREEN_PIN GPIO_PIN_4
#define LED_BLUE_PIN  GPIO_PIN_5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId taskRTHandle;
osThreadId taskCommHandle;
osThreadId taskButtonHandle;
osMessageQId commsQueueHandle;
osSemaphoreId buttonSemHandle;
/* USER CODE BEGIN PV */
volatile uint8_t SystemMode = 0; // 0=Monitor, 1=Set Temp, 2=Set Hum
volatile uint8_t TempThreshold = 20;
volatile uint8_t HumThreshold = 60;
uint8_t rx_buffer[1];
uint8_t button_last_state = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
void vTaskRT(void const * argument);
void vTaskComm(void const * argument);
void StartTaskButton(void const * argument);

/* USER CODE BEGIN PFP */
void DHT11_Start(void);
uint8_t DHT11_Read(void);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void delay_us(uint32_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MicroDelay(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim14, 0);
  while (__HAL_TIM_GET_COUNTER(&htim14) < us);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DHT11_Read_Data(DHT11_Data_t *dht_data) {
  uint8_t i, j, byte;
  uint8_t data[5] = {0};
  uint32_t timeout;

  // 1. START
  Set_Pin_Output(DHT11_PORT, DHT11_PIN);
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
  osDelay(18);
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
  MicroDelay(20);
  Set_Pin_Input(DHT11_PORT, DHT11_PIN);

  // 2. CHECK RESPONSE
  timeout = 0;
  while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
    MicroDelay(1);
    if(++timeout > 100) return 1;
  }
  timeout = 0;
  while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) {
    MicroDelay(1);
    if(++timeout > 100) return 2;
  }
  timeout = 0;
  while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
    MicroDelay(1);
    if(++timeout > 100) return 3;
  }

  // 3. READ BITS
  for (j = 0; j < 5; j++) {
    byte = 0;
    for (i = 0; i < 8; i++) {
      timeout = 0;
      while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) {
        MicroDelay(1);
        if(++timeout > 100) return 4;
      }

      MicroDelay(40);

      if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        byte |= (1 << (7 - i));
        timeout = 0;
        while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
          MicroDelay(1);
          if(++timeout > 100) return 5;
        }
      }
    }
    data[j] = byte;
  }

  // 4. CHECKSUM
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    dht_data->Humidity = data[0];
    dht_data->Temperature = data[2];
    return 0;
  } else {
    return 6;
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
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //osKernelInitialize();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of buttonSem */
  osSemaphoreDef(buttonSem);
  buttonSemHandle = osSemaphoreCreate(osSemaphore(buttonSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of commsQueue */
  osMessageQDef(commsQueue, 4, uint32_t);
  commsQueueHandle = osMessageCreate(osMessageQ(commsQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of taskRT */
  osThreadDef(taskRT, vTaskRT, osPriorityNormal, 0, 128);
  taskRTHandle = osThreadCreate(osThread(taskRT), NULL);

  /* definition and creation of taskComm */
  osThreadDef(taskComm, vTaskComm, osPriorityLow, 0, 256);
  taskCommHandle = osThreadCreate(osThread(taskComm), NULL);

  /* definition and creation of taskButton */
  osThreadDef(taskButton, StartTaskButton, osPriorityHigh, 0, 128);
  taskButtonHandle = osThreadCreate(osThread(taskButton), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT11_DATA_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT11_DATA_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DHT11_DATA_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_Pin)
  {
    osSemaphoreRelease(buttonSemHandle);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (SystemMode == 1)
    {
      if (rx_buffer[0] == '+') {
        TempThreshold++;
      }
      else if (rx_buffer[0] == '-') {
        TempThreshold--;
      }
    }
    else if (SystemMode == 2)
    {
      if (rx_buffer[0] == '+') {
        HumThreshold++;
      }
      else if (rx_buffer[0] == '-') {
        HumThreshold--;
      }
    }
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_vTaskRT */
/**
  * @brief  Function implementing the taskRT thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vTaskRT */
void vTaskRT(void const * argument)
{
  /* USER CODE BEGIN 5 */
  HAL_TIM_Base_Start(&htim14);
  DHT11_Data_t SensorData;
  uint8_t status;
  osDelay(2000);

  for(;;)
  {
    if (SystemMode == 0)
    {
      status = DHT11_Read_Data(&SensorData);
      if (status == 0)
      {
        HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN, GPIO_PIN_RESET);

        if (SensorData.Temperature < TempThreshold)
        {
          // FRIG
          if (SensorData.Humidity > HumThreshold) {
            // Frig Umed -> CYAN (G+B)
            HAL_GPIO_WritePin(GPIOB, LED_GREEN_PIN|LED_BLUE_PIN, GPIO_PIN_SET);
          } else {
            // Frig Uscat -> BLUE
            HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, GPIO_PIN_SET);
          }
        }
        else if (SensorData.Temperature <= (TempThreshold + 5))
        {
          // CONFORTABIL / OK
          if (SensorData.Humidity > HumThreshold) {
            // Zapuseala -> YELLOW (R+G)
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_GREEN_PIN, GPIO_PIN_SET);
          } else {
            // Perfect -> GREEN
            HAL_GPIO_WritePin(GPIOB, LED_GREEN_PIN, GPIO_PIN_SET);
          }
        }
        else
        {
          // CALD
          if (SensorData.Humidity > HumThreshold) {
            // Tropical -> MAGENTA (R+B)
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_BLUE_PIN, GPIO_PIN_SET);
          } else {
            // Uscat -> RED
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, GPIO_PIN_SET);
          }
        }
        uint32_t packed = (SensorData.Temperature << 8) | SensorData.Humidity;
        osMessagePut(commsQueueHandle, packed, 0);
      }
      else
      {
        // Eroare senzor -> Flash Roșu
    	HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN, GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(GPIOB, LED_RED_PIN);
      }
    }
    else
    {

      HAL_GPIO_TogglePin(GPIOB, LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN);
      osMessagePut(commsQueueHandle, 0xFFFFFFFF, 0);
    }

    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vTaskComm */
/**
* @brief Function implementing the taskComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskComm */
void vTaskComm(void const * argument)
{
  /* USER CODE BEGIN vTaskComm */
  osEvent event;
  char bt_msg[80];
  uint32_t received_data;
  uint8_t temp, hum;

  static uint8_t last_temp = 255;
  static uint8_t last_hum = 255;
  static uint8_t last_t_thresh = 255;
  static uint8_t last_h_thresh = 255;
  static uint8_t last_mode = 255;

  uint8_t current_mode;
  uint8_t change_detected = 0;
  HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  for(;;)
  {
    event = osMessageGet(commsQueueHandle, osWaitForever);

    if (event.status == osEventMessage)
    {
      received_data = event.value.v;
      current_mode = SystemMode;
      change_detected = 0;

      if (current_mode != last_mode) {
        change_detected = 1;
      }

      if (current_mode == 0 && received_data != 0xFFFFFFFF)
      {
        temp = (received_data >> 8) & 0xFF;
        hum = received_data & 0xFF;

        if (temp != last_temp || hum != last_hum ||
      	  TempThreshold != last_t_thresh || HumThreshold != last_h_thresh) {
          change_detected = 1;
        }

        if (change_detected == 1) {
          sprintf(bt_msg, "Live: %dC | %d%% (Prag T:%d H:%d)\r\n", temp, hum, TempThreshold, HumThreshold);
          HAL_UART_Transmit(&huart1, (uint8_t*)bt_msg, strlen(bt_msg), 100);

          last_temp = temp;
          last_hum = hum;
          last_t_thresh = TempThreshold;
          last_h_thresh = HumThreshold;
          last_mode = current_mode;
        }
      }
      else if (current_mode == 1)
      {
        if (TempThreshold != last_t_thresh) {
    	  change_detected = 1;
        }
    	if (change_detected == 1) {
    	  sprintf(bt_msg, ">> SETARE TEMPERATURA <<\r\nActual: %d C\r\n(+/- pt modificare)\r\n", TempThreshold);
    	  HAL_UART_Transmit(&huart1, (uint8_t*)bt_msg, strlen(bt_msg), 100);
    	  last_t_thresh = TempThreshold;
    	  last_mode = current_mode;
        }
      }
      else
      {
        if (HumThreshold != last_h_thresh) {
    	  change_detected = 1;
    	}
    	if (change_detected == 1) {
          sprintf(bt_msg, ">> SETARE UMIDITATE <<\r\nActual: %d %% (+/-)\r\n", HumThreshold);
    	  HAL_UART_Transmit(&huart1, (uint8_t*)bt_msg, strlen(bt_msg), 100);
    	  last_h_thresh = HumThreshold;
    	  last_mode = current_mode;
        }
      }
    }
  }
  /* USER CODE END vTaskComm */
}

/* USER CODE BEGIN Header_StartTaskButton */
/**
* @brief Function implementing the taskButton thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskButton */
void StartTaskButton(void const * argument)
{
  /* USER CODE BEGIN StartTaskButton */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(buttonSemHandle, osWaitForever);
    osDelay(50);
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
      SystemMode = (SystemMode + 1) % 3; // Ciclam 0, 1, 2
      HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN, GPIO_PIN_RESET);
      //HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN, GPIO_PIN_SET);
      //osDelay(100);
      //HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN, GPIO_PIN_RESET);
    }
    while(osSemaphoreWait(buttonSemHandle, 0) == osOK);
  }
  /* USER CODE END StartTaskButton */
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
  if (htim->Instance == TIM6)
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
#ifdef USE_FULL_ASSERT
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
