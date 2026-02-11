#include "encoder.h"

#include "tim.h"

extern "C" {

void Encoder_Init(void)
{
  /* TIM3/TIM4 base configuration is generated and initialized in main. */
  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
}

int16_t Encoder_Get(uint8_t n)
{
  int16_t temp = 0;

  if (n == 1U)
  {
    temp = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return temp;
  }

  if (n == 2U)
  {
    temp = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return (int16_t)(-temp);
  }

  return 0;
}

}  // extern "C"
