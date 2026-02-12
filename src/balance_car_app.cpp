#include "balance_car_app.hpp"

#include "led.h"
#include "key.h"
#include "motor.h"
#include "encoder.h"
#include "blue_serial.h"
#include "irq_lock.hpp"

namespace app {
namespace {
/* Adjust this constant to your encoder+gearbox pulses per one wheel revolution. */
static const int32_t kEncoderCountsPerTurn = 780;
/* TIM1 period is 1 ms in current CubeMX config. */
static const int32_t kControlPeriodMs = 1;
/* Speed estimate/update window in ISR to reduce encoder quantization steps. */
static const uint8_t kSpeedWindowMs = 20U;
/* 1st-order IIR smoothing strength: y += (x - y) / kSpeedFilterDiv. */
static const int32_t kSpeedFilterDiv = 4;
}

/* Set deterministic startup values for runtime state and flow control. */
BalanceCarApp::BalanceCarApp()
    : ax_(0), ay_(0), az_(0), gx_(0), gy_(0), gz_(0),
      timer_error_(0), timer_count_(0),
      angle_acc_(0.0f), angle_gyro_(0.0f), angle_(0.0f),
      left_speed_centi_rps_(0), right_speed_centi_rps_(0),
      imu_ready_(false),
      flow_period_ms_(120U), flow_forward_(true), flow_enable_(true),
      left_pwm_(0), right_pwm_(0)
{
}

/* Initialize all board services and app-level modules once at boot. */
void BalanceCarApp::Init(void)
{
  /* Basic IO services: LED, key scan, and non-blocking flow runner. */
  BoardLed().Init();
  BoardKey().Init();
  BoardLedFlow().Init();
  BoardMotor().Init();
  BoardEncoder().Init();
  /* Board wiring maps logical left/right to opposite driver channels. */
  BoardMotor().SetPWM(1U, right_pwm_);
  BoardMotor().SetPWM(2U, left_pwm_);
  BoardLedFlow().SetPeriodMs(flow_period_ms_);
  BoardLedFlow().SetForward(flow_forward_);
  BoardLedFlow().SetEnable(flow_enable_);

  /* UI startup splash. */
  oled_.Init();
  oled_.ShowBootScreen();
  HAL_Delay(1000);
  BoardOled().Clear();

  /* Sensor and estimation pipeline setup. */
  imu_ready_ = imu_.Init();
  uart_plot_.Init();
  BoardBlueSerial().Init();
  BoardBlueSerial().Printf("BT Ready\r\n");
  BoardBlueSerial().Printf("Send packet: [hello]\r\n");
  estimator_.Configure(0.013f, 0.001f, 16.0f);
  estimator_.Reset();

  /* Keep an explicit on-screen error if IMU handshake fails. */
  if (!imu_ready_)
  {
    oled_.ShowError("MPU ID ERR");
  }
}

/* Main loop: handle events, render latest snapshot, and stream telemetry. */
void BalanceCarApp::Loop(void)
{
  RuntimeData data;
  const uint8_t key = BoardKey().GetNum();

  /* Key events are edge-triggered by the interrupt-side scanner. */
  if (key != 0U)
  {
    if (key == 1U)
    {
      left_pwm_ = (int16_t)(left_pwm_ + 10);
    }
    else if (key == 2U)
    {
      right_pwm_ = (int16_t)(right_pwm_ + 10);
    }
    else if (key == 3U)
    {
      left_pwm_ = (int16_t)(left_pwm_ - 10);
    }
    else if (key == 4U)
    {
      right_pwm_ = (int16_t)(right_pwm_ - 10);
    }

    if (left_pwm_ > 100) left_pwm_ = 100;
    if (left_pwm_ < -100) left_pwm_ = -100;
    if (right_pwm_ > 100) right_pwm_ = 100;
    if (right_pwm_ < -100) right_pwm_ = -100;

    /* Board wiring maps logical left/right to opposite driver channels. */
    BoardMotor().SetPWM(1U, right_pwm_);
    BoardMotor().SetPWM(2U, left_pwm_);
    HandleKeyEvent(key);
  }

  /* Copy volatile fields once to avoid tearing during rendering/output. */
  CopyRuntimeData(&data);

  {
    io::OledTelemetry telemetry;
    telemetry.ax = data.ax;
    telemetry.ay = data.ay;
    telemetry.az = data.az;
    telemetry.gx = data.gx;
    telemetry.gy = data.gy;
    telemetry.gz = data.gz;
    telemetry.left_speed_centi_rps = data.left_speed_centi_rps;
    telemetry.right_speed_centi_rps = data.right_speed_centi_rps;
    oled_.ShowTelemetry(telemetry);
  }

  {
    char packet[100];
    if (BoardBlueSerial().ReadPacket(packet, sizeof(packet)) != 0U)
    {
      BoardBlueSerial().Printf("[ACK:%s]\r\n", packet);
    }
  }

  BoardBlueSerial().Printf("plot,%f,%f,%f", data.angle_acc, data.angle_gyro, data.angle);
}

/* TIM1 periodic callback (1 ms): run time-critical non-blocking tasks. */
void BalanceCarApp::OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1)
  {
    return;
  }

  /* Non-blocking periodic services driven by timer tick. */
  BoardKey().Tick();
  BoardLedFlow().Tick1ms();

  /* Sample IMU and update estimator inside the fixed-rate context. */
  if (imu_ready_)
  {
    tasks::ImuSample sample;
    imu_.Read(&sample);
    estimator_.Update(sample);

    ax_ = sample.ax;
    ay_ = sample.ay;
    az_ = sample.az;
    gx_ = sample.gx;
    gy_ = sample.gy;
    gz_ = sample.gz;
    angle_acc_ = estimator_.AngleAccDeg();
    angle_gyro_ = estimator_.AngleGyroDeg();
    angle_ = estimator_.AngleDeg();
  }

  /* ISR-domain speed estimate: accumulate encoder counts in a short window,
   * then divide by window period and apply light IIR smoothing.
   */
  {
    static int32_t left_acc_count = 0;
    static int32_t right_acc_count = 0;
    static uint8_t window_ms = 0U;
    static int32_t left_filtered_centi_rps = 0;
    static int32_t right_filtered_centi_rps = 0;

    left_acc_count -= (int32_t)BoardEncoder().GetLeft();
    right_acc_count -= (int32_t)BoardEncoder().GetRight();
    window_ms = (uint8_t)(window_ms + kControlPeriodMs);

    if (window_ms >= kSpeedWindowMs)
    {
      const int64_t denominator = (int64_t)kEncoderCountsPerTurn * (int64_t)window_ms;
      if (denominator > 0)
      {
        /* centi-rps = delta_count * (100 * 1000) / (counts_per_turn * period_ms). */
        const int32_t left_raw_centi_rps =
            (int32_t)(((int64_t)left_acc_count * 100000LL) / denominator);
        const int32_t right_raw_centi_rps =
            (int32_t)(((int64_t)right_acc_count * 100000LL) / denominator);

        left_filtered_centi_rps += (left_raw_centi_rps - left_filtered_centi_rps) / kSpeedFilterDiv;
        right_filtered_centi_rps += (right_raw_centi_rps - right_filtered_centi_rps) / kSpeedFilterDiv;

        left_speed_centi_rps_ = left_filtered_centi_rps;
        right_speed_centi_rps_ = right_filtered_centi_rps;
      }

      left_acc_count = 0;
      right_acc_count = 0;
      window_ms = 0U;
    }
  }

  /* Detect overrun-style condition if update flag is still pending. */
  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
  {
    timer_error_ = 1U;
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
  }
  /* Capture current counter value for debug/monitor display. */
  timer_count_ = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
}

/* Map key IDs to flow-runner behavior changes. */
void BalanceCarApp::HandleKeyEvent(uint8_t key)
{
  /* KEY1: faster (shorter period). */
  if (key == 1U)
  {
    if (flow_period_ms_ > 20U)
    {
      flow_period_ms_ = (uint16_t)(flow_period_ms_ - 20U);
    }
    BoardLedFlow().SetPeriodMs(flow_period_ms_);
  }
  /* KEY2: slower (longer period). */
  else if (key == 2U)
  {
    if (flow_period_ms_ < 1000U)
    {
      flow_period_ms_ = (uint16_t)(flow_period_ms_ + 20U);
    }
    BoardLedFlow().SetPeriodMs(flow_period_ms_);
  }
  /* KEY3: reverse/forward direction toggle. */
  else if (key == 3U)
  {
    flow_forward_ = !flow_forward_;
    BoardLedFlow().SetForward(flow_forward_);
  }
  /* KEY4: enable/disable flow output. */
  else if (key == 4U)
  {
    flow_enable_ = !flow_enable_;
    BoardLedFlow().SetEnable(flow_enable_);
  }
}

/* Atomic snapshot copy from ISR-updated volatile members to plain struct. */
void BalanceCarApp::CopyRuntimeData(RuntimeData *out) const
{
  IrqLock lock;

  out->ax = ax_;
  out->ay = ay_;
  out->az = az_;
  out->gx = gx_;
  out->gy = gy_;
  out->gz = gz_;
  out->left_speed_centi_rps = left_speed_centi_rps_;
  out->right_speed_centi_rps = right_speed_centi_rps_;
  out->angle_acc = angle_acc_;
  out->angle_gyro = angle_gyro_;
  out->angle = angle_;
}

}  // namespace app
