#pragma once

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "pwm.h"

#ifdef __cplusplus
/** @brief Two-channel motor driver (direction GPIO + PWM duty). */
class DualMotorDriver {
public:
  /** @brief Construct with PWM backend. */
  explicit DualMotorDriver(PwmDriver &pwm);

  /** @brief Initialize outputs and stop motors. */
  void Init(void);
  /** @brief Set signed PWM command for motor index (1 or 2). */
  void SetPWM(uint8_t n, int16_t pwm);

private:
  void SetLeft(int16_t pwm);
  void SetRight(int16_t pwm);
  static uint16_t AbsDuty(int16_t pwm);

private:
  PwmDriver &pwm_;
};

/** @brief Get board default motor singleton. */
DualMotorDriver &BoardMotor(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief C compatibility: initialize motor driver. */
void Motor_Init(void);
/** @brief C compatibility: set motor PWM command. */
void Motor_SetPWM(uint8_t n, int16_t pwm);

#ifdef __cplusplus
}
#endif
