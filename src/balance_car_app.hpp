#pragma once

#include <stdint.h>

#include "main.h"
#include "tim.h"

#include "imu_mpu6050.hpp"
#include "oled_panel.hpp"
#include "motor.h"
#include "pid.h"

namespace app {

/** @brief Main application orchestrator for balance-car runtime tasks. */
class BalanceCarApp {
public:
  /** @brief Construct app object with default runtime state. */
  BalanceCarApp();

  /** @brief Initialize services, sensors, UI and control parameters. */
  void Init(void);
  /** @brief Non-blocking foreground loop body. */
  void Loop(void);
  /** @brief Periodic timer callback entry (called from HAL TIM ISR bridge). */
  void OnTimPeriodElapsed(TIM_HandleTypeDef *htim);

private:
  /** @brief Runtime telemetry snapshot used by UI/serial output. */
  struct RuntimeData {
    int16_t gy;
    uint8_t run_flag;
    uint8_t timer_error;
    uint16_t timer_count;
    uint16_t timer_count_max;
    float angle_pid_kp;
    float angle_pid_ki;
    float angle_pid_kd;
    float speed_pid_kp;
    float speed_pid_ki;
    float speed_pid_kd;
    float turn_pid_kp;
    float turn_pid_ki;
    float turn_pid_kd;
    float angle_pid_target;
    float speed_pid_target;
    float turn_pid_target;
    float angle_pid_out;
    float speed_pid_out;
    float turn_pid_out;
    float angle_acc;
    float angle_gyro;
    float angle;
    float ave_speed;
    float dif_speed;
  };

  /** @brief Copy ISR-updated runtime fields into a stable snapshot. */
  void CopyRuntimeData(RuntimeData *out) const;
  /** @brief Parse one bluetooth packet payload and update runtime parameters. */
  void HandleBluePacket(char *packet);
  /** @brief Clamp signed PWM command to motor range [-100, 100]. */
  static int16_t ClampPwm(int16_t pwm);

private:
  io::Mpu6050Imu imu_;
  io::OledPanel oled_;
  PidController angle_pid_;
  PidController speed_pid_;
  PidController turn_pid_;

  volatile int16_t ax_;
  volatile int16_t ay_;
  volatile int16_t az_;
  volatile int16_t gx_;
  volatile int16_t gy_;
  volatile int16_t gz_;

  volatile uint8_t timer_error_;
  volatile uint16_t timer_count_;
  volatile uint16_t timer_count_max_;

  volatile float angle_acc_;
  volatile float angle_gyro_;
  volatile float angle_;

  volatile float left_speed_;
  volatile float right_speed_;
  volatile float ave_speed_;
  volatile float dif_speed_;

  volatile uint8_t run_flag_;
  volatile int16_t dif_pwm_;
  volatile int16_t left_pwm_;
  volatile int16_t right_pwm_;
  volatile float desired_speed_target_;
  volatile float desired_turn_target_;
  volatile int16_t gy_offset_;
  volatile float angle_offset_;
  bool imu_ready_;
};

}  // namespace app
