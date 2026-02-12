#include "motor.h"

#include "main.h"

/* Build a dual-motor driver around a PWM backend. */
DualMotorDriver::DualMotorDriver(PwmDriver &pwm)
    : pwm_(pwm)
{
}

/* Convert signed PWM command to bounded absolute duty (0..100). */
uint16_t DualMotorDriver::AbsDuty(int16_t pwm)
{
  int16_t value = pwm;
  if (value < 0)
  {
    value = (int16_t)(-value);
  }
  if (value > 100)
  {
    value = 100;
  }
  return (uint16_t)value;
}

/* Initialize direction pins and stop both channels. */
void DualMotorDriver::Init(void)
{
  pwm_.Init();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  pwm_.SetCompare1(0U);
  pwm_.SetCompare2(0U);
}

/* Apply left motor direction and duty. */
void DualMotorDriver::SetLeft(int16_t pwm)
{
  const uint16_t duty = AbsDuty(pwm);
  if (pwm >= 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  }
  pwm_.SetCompare1(duty);
}

/* Apply right motor direction and duty. */
void DualMotorDriver::SetRight(int16_t pwm)
{
  const uint16_t duty = AbsDuty(pwm);
  if (pwm >= 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  }
  pwm_.SetCompare2(duty);
}

/* Select motor by index and apply signed PWM command. */
void DualMotorDriver::SetPWM(uint8_t n, int16_t pwm)
{
  if (n == 1U)
  {
    SetLeft(pwm);
  }
  else if (n == 2U)
  {
    SetRight(pwm);
  }
}

namespace {
/* Board default motor driver. */
DualMotorDriver g_motor(BoardPwm());
}

/* Accessor for board motor singleton. */
DualMotorDriver &BoardMotor(void)
{
  return g_motor;
}

extern "C" {

/* C compatibility wrapper: initialize dual-motor driver. */
void Motor_Init(void)
{
  BoardMotor().Init();
}

/* C compatibility wrapper: set signed PWM for selected motor channel. */
void Motor_SetPWM(uint8_t n, int16_t pwm)
{
  BoardMotor().SetPWM(n, pwm);
}

}  // extern "C"
