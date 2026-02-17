#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "balance_car_app.hpp"

namespace {

/* Board startup helper:
 * - Initializes HAL and clock tree
 * - Initializes all peripherals used by the app
 * - Starts TIM1 update interrupt as 1 ms scheduler tick
 */
class BoardBootstrap {
public:
  /* Returns true when board-level initialization succeeds. */
  bool Init(void)
  {
    HAL_Init();
    ConfigureSystemClock();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    return HAL_TIM_Base_Start_IT(&htim1) == HAL_OK;
  }

private:
  /* Configure system clock to 72 MHz from HSE + PLL x9. */
  static void ConfigureSystemClock(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler();
    }
  }
};

/* Global startup and application singletons. */
BoardBootstrap g_board;
app::BalanceCarApp g_app;

}  // namespace

/* C++ entry point:
 * - Boot board
 * - Init application
 * - Run cooperative main loop forever
 */
int main(void)
{
  if (!g_board.Init())
  {
    Error_Handler();
  }

  g_app.Init();

  while (1)
  {
    g_app.Loop();
    
  }
}

/* HAL timer callback bridge:
 * Forward TIM period events to the app scheduler.
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  g_app.OnTimPeriodElapsed(htim);
}

/* Fatal error handler: stop system in an infinite loop with IRQ disabled. */
extern "C" void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/* Optional assert hook used by HAL/CMSIS debug checks. */
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
