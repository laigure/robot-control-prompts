/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "OLED.h"
#include "serial.h"
#include "mpu6050.h"
#include <math.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int16_t AX = 0, AY = 0, AZ = 0, GX = 0, GY = 0, GZ = 0;
volatile uint8_t TimerErrorFlag = 0U;
volatile uint16_t TimerCount = 0U;

volatile float AngleAcc = 0.0f;
volatile float AngleGyro = 0.0f;
volatile float Angle = 0.0f;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  uint8_t timer_err = 0U;
  uint16_t timer_cnt = 0U;
  float angle_acc = 0.0f;
  float angle_gyro = 0.0f;
  float angle = 0.0f;
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  Led_Init();
  OLED_Init();
  MPU6050_Init();
  Serial_Init();

  OLED_Clear();
  OLED_ShowString(0, 0, "Hello", OLED_6X8);
  OLED_ShowString(0, 16, "STM32", OLED_8X16);
  HAL_Delay(1000);
  OLED_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    __disable_irq();
    ax = AX;
    ay = AY;
    az = AZ;
    gx = GX;
    gy = GY+6;
    gz = GZ;
    timer_err = TimerErrorFlag;
    timer_cnt = TimerCount;
    angle_acc = AngleAcc;
    angle_gyro = AngleGyro;
    angle = Angle;
    __enable_irq();

    OLED_Printf(0, 0, OLED_8X16, "%+06d", ax);
    OLED_Printf(0, 16, OLED_8X16, "%+06d", ay);
    OLED_Printf(0, 32, OLED_8X16, "%+06d", az);
    OLED_Printf(64, 0, OLED_8X16, "%+06d", gx);
    OLED_Printf(64, 16, OLED_8X16, "%+06d", gy);
    OLED_Printf(64, 32, OLED_8X16, "%+06d", gz);
    OLED_Printf(0, 48, OLED_8X16, "Flag:%1d", timer_err);
    OLED_Printf(64, 48, OLED_8X16, "C:%05d", timer_cnt);
    OLED_Update();

Serial_Printf("plot:%f,%f,%f\n", angle_acc, angle_gyro, angle);
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
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
	__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    const float alpha = 0.013f;
    int16_t gy;

    MPU6050_GetData((int16_t *)&AX, (int16_t *)&AY, (int16_t *)&AZ,
                    (int16_t *)&GX, (int16_t *)&GY, (int16_t *)&GZ);

    gy = (int16_t)(GY - 16);

    AngleAcc = (float)(-atan2((double)AX, (double)AZ) / 3.14159 * 180.0);
    AngleGyro = Angle + ((float)gy / 32768.0f * 2000.0f * 0.001f);
    Angle = alpha * AngleAcc + (1.0f - alpha) * AngleGyro;

    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
    {
      TimerErrorFlag = 1U;
      __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    }
    TimerCount = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
  }
}
/* USER CODE END 4 */

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
