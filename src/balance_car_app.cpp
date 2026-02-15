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
static const float kAngleKiDefault = 0.007f;
static const float kAngleKdDefault = 3.0f;
static const float kAnglePidOutMax = 100.0f;
static const float kAnglePidOutMin = -100.0f;

static const float kSpeedKpDefault = 2.0f;
static const float kSpeedKiDefault = 0.05f;
static const float kSpeedKdDefault = 0.0f;
static const float kSpeedPidOutMax = 20.0f;
static const float kSpeedPidOutMin = -20.0f;

static const float kTurnKpDefault = 4.0f;
static const float kTurnKiDefault = 3.0f;
static const float kTurnKdDefault = 0.0f;
static const float kTurnPidOutMax = 50.0f;
static const float kTurnPidOutMin = -50.0f;

static const int16_t kGyroBiasLsb = 16;
static const float kAngleAccOffsetDeg = 0.5f;
static const float kComplementaryAlpha = 0.01f;
static const float kGyroScaleDps = 2000.0f / 32768.0f;
static const float kControlStepSec = 0.01f;
static const float kSpeedStepSec = 0.05f;
static const float kStopAngleDeg = 50.0f;
static const uint16_t kControlDivider = 10U;   /* 10ms: angle loop */
static const uint16_t kSpeedDivider = 50U;     /* 50ms: speed loop */
}

/* Set deterministic startup values for runtime state and PID parameters. */
BalanceCarApp::BalanceCarApp()
    : imu_(), oled_(),
      angle_pid_(kAngleKpDefault, kAngleKiDefault, kAngleKdDefault, kAnglePidOutMin, kAnglePidOutMax),
      speed_pid_(kSpeedKpDefault, kSpeedKiDefault, kSpeedKdDefault, kSpeedPidOutMin, kSpeedPidOutMax),
      turn_pid_(kTurnKpDefault, kTurnKiDefault, kTurnKdDefault, kTurnPidOutMin, kTurnPidOutMax),
      ax_(0), ay_(0), az_(0), gx_(0), gy_(0), gz_(0),
      timer_error_(0), timer_count_(0), timer_count_max_(0),
      angle_acc_(0.0f), angle_gyro_(0.0f), angle_(0.0f),
      left_speed_(0.0f), right_speed_(0.0f), ave_speed_(0.0f), dif_speed_(0.0f),
      angle_pid_out_(0.0f), speed_pid_out_(0.0f), turn_pid_out_(0.0f), run_flag_(0U),
      dif_pwm_(0), left_pwm_(0), right_pwm_(0),
      imu_ready_(false),
      angle_kp_(kAngleKpDefault), angle_ki_(kAngleKiDefault), angle_kd_(kAngleKdDefault),
      speed_kp_(kSpeedKpDefault), speed_ki_(kSpeedKiDefault), speed_kd_(kSpeedKdDefault),
      turn_kp_(kTurnKpDefault), turn_ki_(kTurnKiDefault), turn_kd_(kTurnKdDefault)
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

  angle_pid_.Init();
  angle_pid_.Configure(angle_kp_, angle_ki_, angle_kd_);
  angle_pid_.SetOutputLimits(kAnglePidOutMin, kAnglePidOutMax);
  angle_pid_.SetTarget(0.0f);

  speed_pid_.Init();
  speed_pid_.Configure(speed_kp_, speed_ki_, speed_kd_);
  speed_pid_.SetOutputLimits(kSpeedPidOutMin, kSpeedPidOutMax);
  speed_pid_.SetTarget(0.0f);

  turn_pid_.Init();
  turn_pid_.Configure(turn_kp_, turn_ki_, turn_kd_);
  turn_pid_.SetOutputLimits(kTurnPidOutMin, kTurnPidOutMax);
  turn_pid_.SetTarget(0.0f);

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
    bool angle_changed = false;
    bool speed_changed = false;
    bool turn_changed = false;

    if ((channel_id == 1) || (strcmp(channel, "AngleKp") == 0))
    {
      angle_kp_ = parsed;
      angle_changed = true;
    }
    else if ((channel_id == 2) || (strcmp(channel, "AngleKi") == 0))
    {
      angle_ki_ = parsed;
      angle_changed = true;
    }
    else if ((channel_id == 3) || (strcmp(channel, "AngleKd") == 0))
    {
      angle_kd_ = parsed;
      angle_changed = true;
    }
    else if ((channel_id == 4) || (strcmp(channel, "SpeedKp") == 0))
    {
      speed_kp_ = parsed;
      speed_changed = true;
    }
    else if ((channel_id == 5) || (strcmp(channel, "SpeedKi") == 0))
    {
      speed_ki_ = parsed;
      speed_changed = true;
    }
    else if ((channel_id == 6) || (strcmp(channel, "SpeedKd") == 0))
    {
      speed_kd_ = parsed;
      speed_changed = true;
    }
    else if (strcmp(channel, "TurnKp") == 0)
    {
      turn_kp_ = parsed;
      turn_changed = true;
    }
    else if (strcmp(channel, "TurnKi") == 0)
    {
      turn_ki_ = parsed;
      turn_changed = true;
    }
    else if (strcmp(channel, "TurnKd") == 0)
    {
      turn_kd_ = parsed;
      turn_changed = true;
    }

    if (angle_changed || speed_changed || turn_changed)
    {
      IrqLock lock;
      if (angle_changed)
      {
        angle_pid_.Configure(angle_kp_, angle_ki_, angle_kd_);
      }
      if (speed_changed)
      {
        speed_pid_.Configure(speed_kp_, speed_ki_, speed_kd_);
      }
      if (turn_changed)
      {
        turn_pid_.Configure(turn_kp_, turn_ki_, turn_kd_);
      }
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
    const float speed_target = (float)lv / 25.0f;
    const float turn_target = (float)rh / 25.0f;

    IrqLock lock;
    speed_pid_.SetTarget(speed_target);
    turn_pid_.SetTarget(turn_target);
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
        speed_pid_.Init();
        turn_pid_.Init();
        angle_pid_.Configure(angle_kp_, angle_ki_, angle_kd_);
        speed_pid_.Configure(speed_kp_, speed_ki_, speed_kd_);
        turn_pid_.Configure(turn_kp_, turn_ki_, turn_kd_);
        angle_pid_.SetOutputLimits(kAnglePidOutMin, kAnglePidOutMax);
        speed_pid_.SetOutputLimits(kSpeedPidOutMin, kSpeedPidOutMax);
        turn_pid_.SetOutputLimits(kTurnPidOutMin, kTurnPidOutMax);
        run_flag_ = 1U;
      }
      else
      {
        run_flag_ = 0U;
        angle_pid_out_ = 0.0f;
        speed_pid_out_ = 0.0f;
        turn_pid_out_ = 0.0f;
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
  BoardOled().Printf(0, 8, OLED_6X8, "P:%05.2f", data.angle_pid_kp);
  BoardOled().Printf(0, 16, OLED_6X8, "I:%05.2f", data.angle_pid_ki);
  BoardOled().Printf(0, 24, OLED_6X8, "D:%05.2f", data.angle_pid_kd);
  BoardOled().Printf(0, 32, OLED_6X8, "T:%+05.1f", data.angle_pid_target);
  BoardOled().Printf(0, 40, OLED_6X8, "A:%+05.1f", data.angle+5);
  BoardOled().Printf(0, 48, OLED_6X8, "O:%+05.0f", data.angle_pid_out);
  BoardOled().Printf(0, 56, OLED_6X8, "GY:%+05d", data.gy);
  BoardOled().Printf(50, 0, OLED_6X8, "Speed");
  BoardOled().Printf(50, 8, OLED_6X8, "%05.2f", data.speed_pid_kp);
  BoardOled().Printf(50, 16, OLED_6X8, "%05.2f", data.speed_pid_ki);
  BoardOled().Printf(50, 24, OLED_6X8, "%05.2f", data.speed_pid_kd);
  BoardOled().Printf(50, 32, OLED_6X8, "%+05.1f", data.speed_pid_target);
  BoardOled().Printf(50, 40, OLED_6X8, "%+05.1f", data.ave_speed);
  BoardOled().Printf(50, 48, OLED_6X8, "%+05.0f", data.speed_pid_out);
  BoardOled().Printf(88, 0, OLED_6X8, "Turn");
  BoardOled().Printf(88, 8, OLED_6X8, "%05.2f", data.turn_pid_kp);
  BoardOled().Printf(88, 16, OLED_6X8, "%05.2f", data.turn_pid_ki);
  BoardOled().Printf(88, 24, OLED_6X8, "%05.2f", data.turn_pid_kd);
  BoardOled().Printf(88, 32, OLED_6X8, "%+05.1f", data.turn_pid_target);
  BoardOled().Printf(88, 40, OLED_6X8, "%+05.1f", data.dif_speed);
  BoardOled().Printf(88, 48, OLED_6X8, "%+05.0f", data.turn_pid_out);
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
      BoardBlueSerial().Printf("[plot,%f,%f]", data.turn_pid_target, data.dif_speed);
    }
  }
}

/* TIM1 periodic callback: key scan, angle loop (10ms), speed loop (50ms). */
void BalanceCarApp::OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
  static uint16_t count0 = 0U;
  static uint16_t count1 = 0U;

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
        angle_pid_out_ = angle_pid_.Update(angle_+5);
        const int16_t ave_pwm = (int16_t)(-angle_pid_out_);
        left_pwm_ = ClampPwm((int16_t)(ave_pwm + dif_pwm_ / 2));
        right_pwm_ = ClampPwm((int16_t)(ave_pwm - dif_pwm_ / 2));
        BoardMotor().SetPWM(1U, left_pwm_);
        BoardMotor().SetPWM(2U, right_pwm_);
      }
      else
      {
        angle_pid_out_ = 0.0f;
        speed_pid_out_ = 0.0f;
        turn_pid_out_ = 0.0f;
        left_pwm_ = 0;
        right_pwm_ = 0;
        BoardMotor().SetPWM(1U, 0);
        BoardMotor().SetPWM(2U, 0);
      }
    }
  }

  count1++;
  if (count1 >= kSpeedDivider)
  {
    count1 = 0U;

    left_speed_ = (float)BoardEncoder().Get(1) / 44.0f / kSpeedStepSec / 9.27666f;
    right_speed_ = (float)BoardEncoder().Get(2) / 44.0f / kSpeedStepSec / 9.27666f;

    ave_speed_ = (left_speed_ + right_speed_) / 2.0f;
    dif_speed_ = left_speed_ - right_speed_;

    if (run_flag_ != 0U)
    {
      speed_pid_out_ = speed_pid_.Update(ave_speed_);
      angle_pid_.SetTarget(speed_pid_out_);

      turn_pid_out_ = turn_pid_.Update(dif_speed_);
      dif_pwm_ = (int16_t)turn_pid_out_;
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
  out->angle_pid_kp = angle_kp_;
  out->angle_pid_ki = angle_ki_;
  out->angle_pid_kd = angle_kd_;
  out->speed_pid_kp = speed_kp_;
  out->speed_pid_ki = speed_ki_;
  out->speed_pid_kd = speed_kd_;
  out->turn_pid_kp = turn_kp_;
  out->turn_pid_ki = turn_ki_;
  out->turn_pid_kd = turn_kd_;
  out->angle_pid_target = angle_pid_.Target();
  out->speed_pid_target = speed_pid_.Target();
  out->turn_pid_target = turn_pid_.Target();
  out->angle_pid_out = angle_pid_out_;
  out->speed_pid_out = speed_pid_out_;
  out->turn_pid_out = turn_pid_out_;
  out->angle_acc = angle_acc_;
  out->angle_gyro = angle_gyro_;
  out->angle = angle_;
  out->ave_speed = ave_speed_;
  out->dif_speed = dif_speed_;
}

}  // namespace app
