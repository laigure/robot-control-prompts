#pragma once

#include <stdint.h>

#include "main.h"
#include "tim.h"

#include "imu_mpu6050.hpp"
#include "oled_panel.hpp"
#include "uart_plot.hpp"
#include "attitude_estimator.hpp"
#include "motor.h"

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
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int32_t left_speed_centi_rps;
    int32_t right_speed_centi_rps;
    float angle_acc;
    float angle_gyro;
    float angle;
  };

  /** @brief Copy ISR-updated runtime fields into a stable snapshot. */
  void CopyRuntimeData(RuntimeData *out) const;
  /** @brief Handle one debounced key event and update flow behavior. */
  void HandleKeyEvent(uint8_t key);

private:
  io::Mpu6050Imu imu_;
  io::OledPanel oled_;
  io::UartPlot uart_plot_;
  tasks::AttitudeEstimator estimator_;

  volatile int16_t ax_;
  volatile int16_t ay_;
  volatile int16_t az_;
  volatile int16_t gx_;
  volatile int16_t gy_;
  volatile int16_t gz_;
  volatile uint8_t timer_error_;
  volatile uint16_t timer_count_;
  volatile float angle_acc_;
  volatile float angle_gyro_;
  volatile float angle_;
  volatile int32_t left_speed_centi_rps_;
  volatile int32_t right_speed_centi_rps_;
  bool imu_ready_;

  uint16_t flow_period_ms_;
  bool flow_forward_;
  bool flow_enable_;
  int16_t left_pwm_;
  int16_t right_pwm_;
};

}  // namespace app
