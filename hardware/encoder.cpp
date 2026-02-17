#include "encoder.h"

#include "tim.h"

/* Bind left/right encoder timer handles to one reader. */
EncoderReader::EncoderReader(TIM_HandleTypeDef *left, TIM_HandleTypeDef *right, bool invert_right)
    : left_(left), right_(right), invert_right_(invert_right)
{
}

/* Start encoder interfaces and clear counters. */
void EncoderReader::Init(void)
{
  (void)HAL_TIM_Encoder_Start(left_, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(right_, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(left_, 0U);
  __HAL_TIM_SET_COUNTER(right_, 0U);
}

/* Read and reset left encoder delta count. */
int16_t EncoderReader::GetLeft(void)
{
  const int16_t temp = (int16_t)__HAL_TIM_GET_COUNTER(left_);
  __HAL_TIM_SET_COUNTER(left_, 0U);
  return temp;
}

/* Read and reset right encoder delta count (optional sign inversion). */
int16_t EncoderReader::GetRight(void)
{
  const int16_t temp = (int16_t)__HAL_TIM_GET_COUNTER(right_);
  __HAL_TIM_SET_COUNTER(right_, 0U);
  return invert_right_ ? (int16_t)(-temp) : temp;
}

/* Generic channel selector: 1->left, 2->right. */
int16_t EncoderReader::Get(uint8_t n)
{
  if (n == 1U) return GetLeft();
  if (n == 2U) return GetRight();
  return 0;
}

namespace {
/* Board default encoder reader on TIM3/TIM4. */
EncoderReader g_encoder(&htim3, &htim4, false);
}

/* Accessor for board encoder singleton. */
EncoderReader &BoardEncoder(void)
{
  return g_encoder;
}

extern "C" {

/* C compatibility wrapper: initialize encoder reader. */
void Encoder_Init(void)
{
  BoardEncoder().Init();
}

/* C compatibility wrapper: read encoder delta by channel index. */
int16_t Encoder_Get(uint8_t n)
{
  return BoardEncoder().Get(n);
}

}  // extern "C"
