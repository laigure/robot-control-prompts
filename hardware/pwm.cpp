#include "pwm.h"

#include "tim.h"

extern TIM_HandleTypeDef htim2;
void MX_TIM2_Init(void);

/* Bind one timer PWM instance to this driver. */
PwmDriver::PwmDriver(TIM_HandleTypeDef *htim)
    : htim_(htim), initialized_(false)
{
}

/* Initialize timer PWM and start CH1/CH2 outputs. */
void PwmDriver::Init(void)
{
  if (!initialized_)
  {
    if (htim_->State == HAL_TIM_STATE_RESET)
    {
      MX_TIM2_Init();
    }
    initialized_ = true;
  }

  (void)HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);
}

/* Set duty compare value for selected channel. */
void PwmDriver::SetCompare(uint32_t channel, uint16_t compare)
{
  __HAL_TIM_SET_COMPARE(htim_, channel, compare);
}

/* CH1 duty helper. */
void PwmDriver::SetCompare1(uint16_t compare)
{
  SetCompare(TIM_CHANNEL_1, compare);
}

/* CH2 duty helper. */
void PwmDriver::SetCompare2(uint16_t compare)
{
  SetCompare(TIM_CHANNEL_2, compare);
}

namespace {
/* Board default PWM driver on TIM2. */
PwmDriver g_pwm(&htim2);
}

/* Accessor for board PWM singleton. */
PwmDriver &BoardPwm(void)
{
  return g_pwm;
}

extern "C" {

/* C compatibility wrapper: initialize PWM backend. */
void PWM_Init(void)
{
  BoardPwm().Init();
}

/* C compatibility wrapper: set TIM2 CH1 compare value. */
void PWM_SetCompare1(uint16_t compare)
{
  BoardPwm().SetCompare1(compare);
}

/* C compatibility wrapper: set TIM2 CH2 compare value. */
void PWM_SetCompare2(uint16_t compare)
{
  BoardPwm().SetCompare2(compare);
}

}  // extern "C"
