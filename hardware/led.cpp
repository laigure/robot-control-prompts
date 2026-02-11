#include "led.h"
#include "main.h"

void Led_Init(void)
{
  /* Ensure LED is off on startup (GPIO should be initialized by MX_GPIO_Init). */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void Led_On(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void Led_Off(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void Led_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void Led_BlinkLoop(uint32_t delay_ms)
{
  for (;;)
  {
    Led_Toggle();
    HAL_Delay(delay_ms);
  }
}
