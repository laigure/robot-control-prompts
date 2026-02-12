#pragma once

#include <stdint.h>
#include "tim.h"

#ifdef __cplusplus
/** @brief PWM driver wrapper for one HAL timer handle. */
class PwmDriver {
public:
  /** @brief Construct with target timer handle. */
  explicit PwmDriver(TIM_HandleTypeDef *htim);

  /** @brief Initialize/start PWM channels. */
  void Init(void);
  /** @brief Set compare value for selected channel. */
  void SetCompare(uint32_t channel, uint16_t compare);
  /** @brief Set compare value for CH1. */
  void SetCompare1(uint16_t compare);
  /** @brief Set compare value for CH2. */
  void SetCompare2(uint16_t compare);

private:
  TIM_HandleTypeDef *htim_;
  bool initialized_;
};

/** @brief Get board default PWM singleton. */
PwmDriver &BoardPwm(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief C compatibility: initialize PWM backend. */
void PWM_Init(void);
/** @brief C compatibility: set CH1 compare. */
void PWM_SetCompare1(uint16_t compare);
/** @brief C compatibility: set CH2 compare. */
void PWM_SetCompare2(uint16_t compare);

#ifdef __cplusplus
}
#endif
