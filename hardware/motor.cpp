#include "motor.h"

#include "main.h"
#include "pwm.h"

extern "C" {

void Motor_Init(void)
{
  /* Direction GPIO (PB12..PB15) is configured by CubeMX in MX_GPIO_Init(). */
  PWM_Init();

  /* Stop both motors by default. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  PWM_SetCompare1(0);
  PWM_SetCompare2(0);
}

void Motor_SetPWM(uint8_t n, int8_t pwm)
{
  uint16_t duty = (pwm >= 0) ? (uint16_t)pwm : (uint16_t)(-(int16_t)pwm);

  if (n == 1U)
  {
    if (pwm >= 0)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
      PWM_SetCompare1(duty);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
      PWM_SetCompare1(duty);
    }
  }
  else if (n == 2U)
  {
    if (pwm >= 0)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
      PWM_SetCompare2(duty);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
      PWM_SetCompare2(duty);
    }
  }
}

}  // extern "C"

