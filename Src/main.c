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
#include <string.h>
#include <stdio.h>

#include "adxl345.h"
#include "dps310.h"
#include "gps_nmea.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KALMAN_UPDATE_RATE 100  // 100 Hz for filter prediction step
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* Definitions for AccelTask */
osThreadId_t AccelTaskHandle;
const osThreadAttr_t AccelTask_attributes = {
  .name = "AccelTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BaroTask */
osThreadId_t BaroTaskHandle;
const osThreadAttr_t BaroTask_attributes = {
  .name = "BaroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for GPSTask */
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
  .name = "GPSTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for KalmanTask */
osThreadId_t KalmanTaskHandle;
const osThreadAttr_t KalmanTask_attributes = {
  .name = "KalmanTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
/* Global data buffers for sensor readings */
volatile float g_accelZ = 0.0f;  // ADXL345 vertical accel minus gravity
volatile float g_baroAlt = 0.0f; // DPS310 altitude
volatile float g_gpsAlt  = 0.0f; // NEO-6M altitude from NMEA
volatile float g_gpsVel  = 0.0f; // NEO-6M vertical velocity (if available)

/* For DMA-based UART4 RX, store NMEA data here */
#define GPS_RX_BUFFER_SIZE 128
uint8_t gpsRxBuffer[GPS_RX_BUFFER_SIZE];
/* We'll parse the NMEA in the GPSTask */

/* Kalman filter state variables */
float kf_alt = 0.0f;   // altitude estimate
float kf_vel = 0.0f;   // velocity estimate
/* Covariance matrix P, or additional states if needed. For brevity, we keep it simple. */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
void StartAccelTask(void *argument);
void StartBaroTask(void *argument);
void StartGPSTask(void *argument);
void StartKalmanTask(void *argument);

/* USER CODE BEGIN PFP */
void Kalman_Predict(float dt, float accel);
void Kalman_Update_Alt(float z_meas, float R);
void Kalman_Update_Vel(float z_meas, float R);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of AccelTask */
  AccelTaskHandle = osThreadNew(StartAccelTask, NULL, &AccelTask_attributes);

  /* creation of BaroTask */
  BaroTaskHandle = osThreadNew(StartBaroTask, NULL, &BaroTask_attributes);

  /* creation of GPSTask */
  GPSTaskHandle = osThreadNew(StartGPSTask, NULL, &GPSTask_attributes);

  /* creation of KalmanTask */
  KalmanTaskHandle = osThreadNew(StartKalmanTask, NULL, &KalmanTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DSP310_CS_GPIO_Port, DSP310_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADXL345_CS_Pin */
  GPIO_InitStruct.Pin = ADXL345_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL345_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DSP310_CS_Pin */
  GPIO_InitStruct.Pin = DSP310_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DSP310_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*---------------- KALMAN FILTER IMPLEMENTATION (2D: alt, vel) -----------------*/
/* Simple version with minimal error checking. Expand with covariance P, etc. */
static float P[2][2] = {{10.0f, 0.0f},{0.0f,10.0f}}; // example initial covariance

void Kalman_Predict(float dt, float accel)
{
  // State update:
  // kf_alt += kf_vel * dt
  // kf_vel += accel * dt
  kf_alt += kf_vel * dt;
  kf_vel += accel * dt;

  // Covariance update: P = F P F^T + Q
  // For brevity, not fully shown. Example:
  // F = [[1, dt],[0,1]]
  // Q depends on dt, etc.
  float P00_temp = P[0][0] + dt*(P[1][0] + P[0][1]) + dt*dt*P[1][1];
  float P01_temp = P[0][1] + dt*P[1][1];
  float P10_temp = P[1][0] + dt*P[1][1];
  float P11_temp = P[1][1]; // ignoring accel noise for brevity

  // Add process noise
  float q_alt = 0.01f; // tune
  float q_vel = 0.1f;  // tune
  P00_temp += q_alt*dt;
  P11_temp += q_vel*dt;

  P[0][0] = P00_temp;
  P[0][1] = P01_temp;
  P[1][0] = P10_temp;
  P[1][1] = P11_temp;
}

void Kalman_Update_Alt(float z_meas, float R)
{
  // z_meas ~ h
  // H = [1, 0], residual = z_meas - kf_alt
  float y = z_meas - kf_alt;
  // S = H P H^T + R => S = P[0][0] + R
  float S = P[0][0] + R;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  // Update state
  kf_alt += K0 * y;
  kf_vel += K1 * y;

  // Update covariance
  float p00_temp = P[0][0] - K0 * P[0][0];
  float p01_temp = P[0][1] - K0 * P[0][1];
  float p10_temp = P[1][0] - K1 * P[0][0];
  float p11_temp = P[1][1] - K1 * P[0][1];

  P[0][0] = p00_temp;
  P[0][1] = p01_temp;
  P[1][0] = p10_temp;
  P[1][1] = p11_temp;
}

void Kalman_Update_Vel(float z_meas, float R)
{
  // z_meas ~ v
  // H = [0, 1], residual = z_meas - kf_vel
  float y = z_meas - kf_vel;
  // S = P[1][1] + R
  float S = P[1][1] + R;
  float K0 = P[0][1] / S;
  float K1 = P[1][1] / S;

  // Update state
  kf_alt += K0 * y;
  kf_vel += K1 * y;

  // Update covariance
  float p00_temp = P[0][0] - K0 * P[0][1];
  float p01_temp = P[0][1] - K0 * P[1][1];
  float p10_temp = P[1][0] - K1 * P[0][1];
  float p11_temp = P[1][1] - K1 * P[1][1];

  P[0][0] = p00_temp;
  P[0][1] = p01_temp;
  P[1][0] = p10_temp;
  P[1][1] = p11_temp;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAccelTask */
/**
  * @brief  Function implementing the AccelTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAccelTask */
void StartAccelTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	ADXL345_Init(&hspi2, ADXL345_CS_GPIO_Port, ADXL345_CS_Pin);
  /* Infinite loop */
  for(;;)
  {
	  /* Read raw acceleration (X,Y,Z) in g. Typically ADXL345_ReadAccel returns float array. */
	      float accelData[3];
	      ADXL345_ReadAccel(accelData);  // user-implemented driver function

	      /* We assume rocket is mostly vertical, so let's pick Z as the vertical axis.
	         Subtract 1g (9.81 m/s^2) to get net acceleration. If ADXL345 driver returns g, multiply by 9.81 if needed. */
	      float rawZ_g = accelData[2];
	      float rawZ_ms2 = rawZ_g * 9.81f;
	      g_accelZ = rawZ_ms2 - 9.81f;  // net rocket acceleration (Trapezoidal integration used in the Kalman)

	      osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBaroTask */
/**
* @brief Function implementing the BaroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBaroTask */
void StartBaroTask(void *argument)
{
  /* USER CODE BEGIN StartBaroTask */
	DPS310_Init(&hspi2, DSP310_CS_GPIO_Port, DSP310_CS_Pin);
  /* Infinite loop */
  for(;;)
  {
	  /* Read altitude from DPS310 in meters (the driver does pressure->alt conversion). */
	      g_baroAlt = DPS310_ReadAltitude();

	      osDelay(10); // ~100 Hz
  }
  /* USER CODE END StartBaroTask */
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
* @brief Function implementing the GPSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void *argument)
{
  /* USER CODE BEGIN StartGPSTask */
	HAL_UART_Receive_DMA(&huart4, gpsRxBuffer, GPS_RX_BUFFER_SIZE);
  /* Infinite loop */
  for(;;)
  {
	  /* Check if we have a new NMEA sentence in gpsRxBuffer.
	         A typical approach is to watch for '$' or '\n' and parse.
	         The user can implement a ring buffer or simpler approach. */

	      /* Pseudocode parse example: */
	      if(GPS_CheckForNewSentence(gpsRxBuffer))
	      {
	        GPS_Data_t gpsData;
	        if(GPS_ParseNMEA(gpsRxBuffer, &gpsData)) // fill gpsData with lat, lon, alt, velocity, etc.
	        {
	          g_gpsAlt = gpsData.altitude;    // in meters
	          g_gpsVel = gpsData.verticalVel; // if available, otherwise 0
	        }
	        // Clear or shift buffer as needed
	      }

	      osDelay(200); // parse at 5 Hz or so
  }
  /* USER CODE END StartGPSTask */
}

/* USER CODE BEGIN Header_StartKalmanTask */
/**
* @brief Function implementing the KalmanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKalmanTask */
void StartKalmanTask(void *argument)
{
  /* USER CODE BEGIN StartKalmanTask */
	/* Kalman filter initialization */
	  kf_alt = 0.0f;
	  kf_vel = 0.0f;
	  // P matrix, etc. can be set to some initial guess

	  float previousAccel = 0.0f; // for trapezoidal integration
	  uint32_t prevTick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  /* 1) Compute dt in seconds */
	      uint32_t nowTick = osKernelGetTickCount();
	      float dt = (nowTick - prevTick) / 1000.0f;
	      prevTick = nowTick;

	      /* 2) Prediction step: trapezoidal integration for acceleration */
	      float currentAccel = g_accelZ; // net rocket accel in m/s^2
	      float avgAccel = 0.5f * (previousAccel + currentAccel);
	      previousAccel = currentAccel;

	      Kalman_Predict(dt, avgAccel);

	      /* 3) Barometric update if new baro reading is available.
	            We'll do a simplistic approach: always update with g_baroAlt. */
	      float R_baro = 1.0f; // noise variance for baro altitude (approx)
	      Kalman_Update_Alt(g_baroAlt, R_baro);

	      /* 4) GPS update if new data is available. For simplicity, assume always. */
	      float R_gpsAlt = 4.0f; // GPS altitude variance ~2m RMS => 4 m^2
	      Kalman_Update_Alt(g_gpsAlt, R_gpsAlt);

	      /* If GPS velocity is reliable, we can also update velocity: */
	      float R_gpsVel = 0.25f; // e.g. 0.5 m/s RMS => 0.25 m^2
	      Kalman_Update_Vel(g_gpsVel, R_gpsVel);

	      /* 5) Check for apogee: if velocity crosses from positive to negative */
	      static float lastVel = 0.0f;
	      if(lastVel > 0.0f && kf_vel < 0.0f)
	      {
	        // Apogee event triggered
	        // Could deploy parachute or set a flag
	      }
	      lastVel = kf_vel;

	      osDelay(10);
  }
  /* USER CODE END StartKalmanTask */
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
