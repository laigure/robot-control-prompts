#include "balance_car_app.hpp"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "blue_serial.h"
#include "encoder.h"
#include "irq_lock.hpp"
#include "key.h"
#include "led.h"
#include "motor.h"
#include "serial.h"

namespace app {
namespace {
static const float kAngleKpDefault = 3.0f;
static const float kAngleKiDefault = 0.1f;
static const float kAngleKdDefault = 3.0f;
static const float kPidOutMax = 100.0f;
static const float kPidOutMin = -100.0f;
static const int16_t kGyroBiasLsb = 16;
static const float kAngleAccOffsetDeg = 0.5f;
static const float kComplementaryAlpha = 0.01f;
static const float kGyroScaleDps = 2000.0f / 32768.0f;
static const float kControlStepSec = 0.01f;
static const float kStopAngleDeg = 50.0f;
static const uint16_t kControlDivider = 10U;
}

/* Set deterministic startup values for runtime state and PID parameters. */
BalanceCarApp::BalanceCarApp()
    : imu_(), oled_(), angle_pid_(kAngleKpDefault, kAngleKiDefault, kAngleKdDefault, kPidOutMin, kPidOutMax),
      ax_(0), ay_(0), az_(0), gx_(0), gy_(0), gz_(0), timer_error_(0), timer_count_(0), timer_count_max_(0),
      angle_acc_(0.0f), angle_gyro_(0.0f), angle_(0.0f),
      pid_out_(0.0f), run_flag_(0U),
      dif_pwm_(0), left_pwm_(0), right_pwm_(0),
      imu_ready_(false),
      angle_kp_(kAngleKpDefault), angle_ki_(kAngleKiDefault), angle_kd_(kAngleKdDefault)
{
}

/* Clamp signed PWM command into motor driver's accepted range. */
int16_t BalanceCarApp::ClampPwm(int16_t pwm)
{
  if (pwm > 100)
  {
    return 100;
  }
  if (pwm < -100)
  {
    return -100;
  }
  return pwm;
}

/* Initialize board services and PID control defaults once at boot. */
void BalanceCarApp::Init(void)
{
  BoardLed().Init();
  BoardKey().Init();
  BoardMotor().Init();
  BoardEncoder().Init();
  BoardSerial().Init();
  BoardMotor().SetPWM(1U, 0);
  BoardMotor().SetPWM(2U, 0);

  oled_.Init();
  BoardOled().Clear();

  imu_ready_ = imu_.Init();

  BoardBlueSerial().Init();
  BoardBlueSerial().Printf("BT Ready\r\n");
  BoardBlueSerial().Printf("Usage: [slider,1,3.00]\r\n");
  BoardBlueSerial().Printf("Usage: [slider,2,0.10]\r\n");
  BoardBlueSerial().Printf("Usage: [slider,3,3.00]\r\n");
  BoardBlueSerial().Printf("Usage: [joystick,0,0,0,0]\r\n");

  angle_pid_.Init();
  angle_pid_.Configure(angle_kp_, angle_ki_, angle_kd_);
  angle_pid_.SetOutputLimits(kPidOutMin, kPidOutMax);
  angle_pid_.SetTarget(0.0f);

  if (!imu_ready_)
  {
    oled_.ShowError("MPU ID ERR");
  }
}

/* Parse one bluetooth payload packet and update PID/target commands. */
void BalanceCarApp::HandleBluePacket(char *packet)
{
  char *tag = strtok(packet, ",");
  if (tag == 0)
  {
    return;
  }

  if (strcmp(tag, "slider") == 0)
  {
    char *channel = strtok(0, ",");
    char *value = strtok(0, ",");
    if ((channel == 0) || (value == 0))
    {
      return;
    }

    const float parsed = (float)atof(value);
    const int32_t channel_id = (int32_t)atoi(channel);
    bool gains_changed = false;
    float kp = angle_kp_;
    float ki = angle_ki_;
    float kd = angle_kd_;

    if ((channel_id == 1) || (strcmp(channel, "AngleKp") == 0))
    {
      kp = parsed;
      gains_changed = true;
    }
    else if ((channel_id == 2) || (strcmp(channel, "AngleKi") == 0))
    {
      ki = parsed;
      gains_changed = true;
    }
    else if ((channel_id == 3) || (strcmp(channel, "AngleKd") == 0))
    {
      kd = parsed;
      gains_changed = true;
    }

    if (gains_changed)
    {
      IrqLock lock;
      angle_kp_ = kp;
      angle_ki_ = ki;
      angle_kd_ = kd;
      angle_pid_.Configure(angle_kp_, angle_ki_, angle_kd_);
    }
  }
  else if (strcmp(tag, "joystick") == 0)
  {
    char *lh_s = strtok(0, ",");
    char *lv_s = strtok(0, ",");
    char *rh_s = strtok(0, ",");
    char *rv_s = strtok(0, ",");
    (void)lh_s;
    (void)rv_s;
    if ((lv_s == 0) || (rh_s == 0))
    {
      return;
    }

    const int16_t lv = (int16_t)atoi(lv_s);
    const int16_t rh = (int16_t)atoi(rh_s);
    const float target = (float)lv / 10.0f;
    const int16_t dif = (int16_t)(rh / 2);

    IrqLock lock;
    angle_pid_.SetTarget(target);
    dif_pwm_ = dif;
  }
}

/* Main loop: key toggle, OLED status, bluetooth command parse and plotting. */
void BalanceCarApp::Loop(void)
{
  RuntimeData data;
  const uint8_t key = BoardKey().GetNum();

  if (key == 1U)
  {
    bool stop_motors = false;
    {
      IrqLock lock;
      if (run_flag_ == 0U)
      {
        angle_pid_.Init();
        angle_pid_.Configure(angle_kp_, angle_ki_, angle_kd_);
        angle_pid_.SetOutputLimits(kPidOutMin, kPidOutMax);
        run_flag_ = 1U;
      }
      else
      {
        run_flag_ = 0U;
        pid_out_ = 0.0f;
        left_pwm_ = 0;
        right_pwm_ = 0;
        stop_motors = true;
      }
    }

    if (stop_motors)
    {
      BoardMotor().SetPWM(1U, 0);
      BoardMotor().SetPWM(2U, 0);
    }
  }

  CopyRuntimeData(&data);

  if (data.run_flag != 0U)
  {
    BoardLed().On();
  }
  else
  {
    BoardLed().Off();
  }

  BoardOled().Clear();
  BoardOled().Printf(0, 0, OLED_6X8, "  Angle");
  BoardOled().Printf(0, 8, OLED_6X8, "P:%05.2f", data.pid_kp);
  BoardOled().Printf(0, 16, OLED_6X8, "I:%05.2f", data.pid_ki);
  BoardOled().Printf(0, 24, OLED_6X8, "D:%05.2f", data.pid_kd);
  BoardOled().Printf(0, 32, OLED_6X8, "T:%+05.1f", data.pid_target);
  BoardOled().Printf(0, 40, OLED_6X8, "A:%+05.1f", data.angle+3);//
  BoardOled().Printf(0, 48, OLED_6X8, "O:%+05.0f", data.pid_out);
  BoardOled().Printf(0, 56, OLED_6X8, "R:%1u C:%2u/%2u E:%1u",
                     data.run_flag, data.timer_count, data.timer_count_max, data.timer_error);
  BoardOled().Update();

  {
    char packet[100];
    if (BoardBlueSerial().ReadPacket(packet, sizeof(packet)) != 0U)
    {
      HandleBluePacket(packet);
    }
  }

  {
    static uint32_t last_plot_ms = 0U;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_plot_ms) >= 50U)
    {
      last_plot_ms = now_ms;
      const int16_t target_x10 = (int16_t)(data.pid_target * 10.0f);
      const int16_t angle_x10 = (int16_t)((data.angle ) * 10.0f);
      BoardBlueSerial().Printf("[plot,%d,%d]", (int)target_x10, (int)angle_x10+3);
    }
  }
}

/* TIM1 periodic callback: key scan and 10-step control task scheduler. */
void BalanceCarApp::OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
  static uint16_t count0 = 0U;

  if (htim->Instance != TIM1)
  {
    return;
  }
  /* Per-cycle flag: clear first, then set only if an overrun is seen in this ISR. */
  timer_error_ = 0U;

  BoardKey().Tick();

  count0++;
  if (count0 >= kControlDivider)
  {
    count0 = 0U;

    if (imu_ready_)
    {
      tasks::ImuSample sample;
      imu_.Read(&sample);

      const int16_t gy_corr = (int16_t)(sample.gy - kGyroBiasLsb);
      const float angle_acc =
          (-atan2f((float)sample.ax, (float)sample.az) * 57.2957795f) + kAngleAccOffsetDeg;
      const float angle_gyro = angle_ + ((float)gy_corr * kGyroScaleDps * kControlStepSec);
      const float angle = kComplementaryAlpha * angle_acc + (1.0f - kComplementaryAlpha) * angle_gyro;

      ax_ = sample.ax;
      ay_ = sample.ay;
      az_ = sample.az;
      gx_ = sample.gx;
      gy_ = gy_corr;
      gz_ = sample.gz;
      angle_acc_ = angle_acc;
      angle_gyro_ = angle_gyro;
      angle_ = angle;

      if ((angle_ > kStopAngleDeg) || (angle_ < -kStopAngleDeg))
      {
        run_flag_ = 0U;
      }

      if (run_flag_ != 0U)
      {
        pid_out_ = angle_pid_.Update(angle_+3);//新变化
        const int16_t ave_pwm = (int16_t)(-pid_out_);
        left_pwm_ = ClampPwm((int16_t)(ave_pwm + dif_pwm_ / 2));
        right_pwm_ = ClampPwm((int16_t)(ave_pwm - dif_pwm_ / 2));
        BoardMotor().SetPWM(1U, left_pwm_);
        BoardMotor().SetPWM(2U, right_pwm_);
      }
      else
      {
        pid_out_ = 0.0f;
        left_pwm_ = 0;
        right_pwm_ = 0;
        BoardMotor().SetPWM(1U, 0);
        BoardMotor().SetPWM(2U, 0);
      }
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
  if (timer_count_ > timer_count_max_)
  {
    timer_count_max_ = timer_count_;
  }
}

/* Atomic snapshot copy from ISR-updated runtime members to plain struct. */
void BalanceCarApp::CopyRuntimeData(RuntimeData *out) const
{
  IrqLock lock;

  out->gy = gy_;
  out->run_flag = run_flag_;
  out->timer_error = timer_error_;
  out->timer_count = timer_count_;
  out->timer_count_max = timer_count_max_;
  out->pid_kp = angle_kp_;
  out->pid_ki = angle_ki_;
  out->pid_kd = angle_kd_;
  out->pid_target = angle_pid_.Target();
  out->pid_out = pid_out_;
  out->angle_acc = angle_acc_;
  out->angle_gyro = angle_gyro_;
  out->angle = angle_;
}

}  // namespace app
