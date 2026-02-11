#include "key.h"

#include "main.h"
#include "stm32f1xx_hal.h"

static volatile uint8_t s_key_num = 0;

void Key_Init(void)
{
  /* GPIO is configured by CubeMX in MX_GPIO_Init(). */
  s_key_num = 0;
} 

uint8_t Key_GetNum(void)
{
  uint8_t temp = 0;
  __disable_irq();
  if (s_key_num != 0U)
  {
    temp = s_key_num;
    s_key_num = 0;
  }
  __enable_irq();
  return temp;
}

uint8_t Key_GetState(void)
{
  /* Active-low keys: pressed = GPIO_PIN_RESET. */
  if (HAL_GPIO_ReadPin(key_4_GPIO_Port, key_4_Pin) == GPIO_PIN_RESET)
  {
    return 1;
  }
  if (HAL_GPIO_ReadPin(key_3_GPIO_Port, key_3_Pin) == GPIO_PIN_RESET)
  {
    return 2;
  }
  if (HAL_GPIO_ReadPin(key_2_GPIO_Port, key_2_Pin) == GPIO_PIN_RESET)
  {
    return 3;
  }
  if (HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin) == GPIO_PIN_RESET)
  {
    return 4;
  }
  return 0;
}

void Key_Tick(void)
{
  static uint8_t count = 0;
  static uint8_t curr_state = 0;
  static uint8_t prev_state = 0;

  count++;
  if (count >= 20U)
  {
    count = 0;

    prev_state = curr_state;
    curr_state = Key_GetState();

    if ((curr_state == 0U) && (prev_state != 0U))
    {
      s_key_num = prev_state;
    }
  }
}
