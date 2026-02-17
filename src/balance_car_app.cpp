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
#include "tunewizard_bridge.h"

/* TuneWizard hot-updatable PID globals (written from UART ISR). */
volatile float g_tw_kp = 0.0f;
volatile float g_tw_ki = 0.0f;
volatile float g_tw_kd = 0.0f;
volatile uint8_t g_tw_params_updated = 0U;

volatile float g_tw_skp = 0.0f;
volatile float g_tw_ski = 0.0f;
volatile float g_tw_skd = 0.0f;
volatile uint8_t g_tw_speed_updated = 0U;

volatile float g_tw_tkp = 0.0f;
volatile float g_tw_tki = 0.0f;
volatile float g_tw_tkd = 0.0f;
volatile uint8_t g_tw_turn_updated = 0U;

namespace app {
namespace {
static const float kAngleKpDefault = 2.7f;
static const float kAngleKiDefault = 0.1f;
static const float kAngleKdDefault = 2.5f;
static const float kAnglePidOutMax = 100.0f;
static const float kAnglePidOutMin = -100.0f;

static const float kSpeedKpDefault = 1.5f;
static const float kSpeedKiDefault = 0.03f;
static const float kSpeedKdDefault = 0.0f;
static const float kSpeedPidOutMax = 12.0f;
static const float kSpeedPidOutMin = -12.0f;

static const float kTurnKpDefault = 4.0f;
static const float kTurnKiDefault = 3.0f;
static const float kTurnKdDefault = 0.0f;
static const float kTurnPidOutMax = 50.0f;
static const float kTurnPidOutMin = -50.0f;

static const int16_t kGyroBiasLsb = 16;
static const float kAngleAccOffsetDeg = 0.5f;   /* fallback if auto-cal fails */
static const float kComplementaryAlpha = 0.01f;
static const float kGyroScaleDps = 2000.0f / 32768.0f;
static const float kControlStepSec = 0.01f;
static const float kSpeedStepSec = 0.05f;
static const float kStopAngleDeg = 35.0f;   /* was 50, catch tilt earlier */
static const uint16_t kControlDivider = 10U;   /* 10ms: angle loop */
static const uint16_t kSpeedDivider = 50U;     /* 50ms: speed loop */
}

/* Set deterministic startup values for runtime state and PID parameters. */
BalanceCarApp::BalanceCarApp()
    : imu_(), oled_(),
      angle_pid_(kAngleKpDefault, kAngleKiDefault, kAngleKdDefault, kAnglePidOutMin, kAnglePidOutMax,0),
      speed_pid_(kSpeedKpDefault, kSpeedKiDefault, kSpeedKdDefault, kSpeedPidOutMin, kSpeedPidOutMax,0),
      turn_pid_(kTurnKpDefault, kTurnKiDefault, kTurnKdDefault, kTurnPidOutMin, kTurnPidOutMax,0),
      ax_(0), ay_(0), az_(0), gx_(0), gy_(0), gz_(0),
      timer_error_(0), timer_count_(0), timer_count_max_(0),
      angle_acc_(0.0f), angle_gyro_(0.0f), angle_(0.0f),
      left_speed_(0.0f), right_speed_(0.0f), ave_speed_(0.0f), dif_speed_(0.0f),
      run_flag_(0U), dif_pwm_(0), left_pwm_(0), right_pwm_(0),
      desired_speed_target_(0.0f), desired_turn_target_(0.0f),
      gy_offset_(kGyroBiasLsb), angle_offset_(kAngleAccOffsetDeg),
      imu_ready_(false)
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

  /* Auto-calibrate gyro and accelerometer zero offset at startup.
   * Car must be stationary and upright during this process.
   * Samples 200 readings (~200ms) and averages them. */
  if (imu_ready_)
  {
    BoardOled().Clear();
    BoardOled().Printf(0, 0, OLED_6X8, "Calibrating...");
    BoardOled().Printf(0, 8, OLED_6X8, "Keep still!");

    int32_t gy_sum = 0;
    float angle_acc_sum = 0.0f;
    const int kCalSamples = 200;
    for (int i = 0; i < kCalSamples; i++)
    {
      tasks::ImuSample s;
      imu_.Read(&s);
      gy_sum += s.gy;
      angle_acc_sum += (-atan2f((float)s.ax, (float)s.az) * 57.2957795f);
      HAL_Delay(1);
    }
    gy_offset_ = (int16_t)(gy_sum / kCalSamples);
    angle_offset_ = -(angle_acc_sum / (float)kCalSamples);  /* negate: so angle=0 when upright */

    BoardOled().Clear();
    BoardOled().Printf(0, 0, OLED_6X8, "Gy off: %d", gy_offset_);
    BoardOled().Printf(0, 8, OLED_6X8, "A  off: %.1f", angle_offset_);
    HAL_Delay(500);
  }

  /* Constructor already configured gains and output limits;
   * Init() only resets runtime state (target/error/integral/output). */
  angle_pid_.Init();
  speed_pid_.Init();
  turn_pid_.Init();

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

    const float v = (float)atof(value);
    IrqLock lock;

    if (strcmp(channel, "AngleKp") == 0 || strcmp(channel, "1") == 0)
      angle_pid_.Configure(v, angle_pid_.Ki(), angle_pid_.Kd());
    else if (strcmp(channel, "AngleKi") == 0 || strcmp(channel, "2") == 0)
      angle_pid_.Configure(angle_pid_.Kp(), v, angle_pid_.Kd());
    else if (strcmp(channel, "AngleKd") == 0 || strcmp(channel, "3") == 0)
      angle_pid_.Configure(angle_pid_.Kp(), angle_pid_.Ki(), v);
    else if (strcmp(channel, "SpeedKp") == 0 || strcmp(channel, "4") == 0)
      speed_pid_.Configure(v, speed_pid_.Ki(), speed_pid_.Kd());
    else if (strcmp(channel, "SpeedKi") == 0 || strcmp(channel, "5") == 0)
      speed_pid_.Configure(speed_pid_.Kp(), v, speed_pid_.Kd());
    else if (strcmp(channel, "SpeedKd") == 0 || strcmp(channel, "6") == 0)
      speed_pid_.Configure(speed_pid_.Kp(), speed_pid_.Ki(), v);
    else if (strcmp(channel, "TurnKp") == 0)
      turn_pid_.Configure(v, turn_pid_.Ki(), turn_pid_.Kd());
    else if (strcmp(channel, "TurnKi") == 0)
      turn_pid_.Configure(turn_pid_.Kp(), v, turn_pid_.Kd());
    else if (strcmp(channel, "TurnKd") == 0)
      turn_pid_.Configure(turn_pid_.Kp(), turn_pid_.Ki(), v);
	else if (strcmp(channel, "Offset") == 0)
      angle_pid_.SetOffset(v);
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
    desired_speed_target_ = (float)lv / 25.0f;
    desired_turn_target_ = (float)rh / 25.0f;
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
        /* Init() resets runtime state; gains and limits are preserved. */
        angle_pid_.Init();
        speed_pid_.Init();
        turn_pid_.Init();
        run_flag_ = 1U;
      }
      else
      {
        run_flag_ = 0U;
        angle_pid_.Init();
        speed_pid_.Init();
        turn_pid_.Init();
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
  /* Row 0-24: PID params (compact) */
  BoardOled().Printf(0,  0, OLED_6X8, "A P%.1f I%.2f D%.1f", data.angle_pid_kp, data.angle_pid_ki, data.angle_pid_kd);
  BoardOled().Printf(0,  8, OLED_6X8, "S P%.1f I%.2f D%.1f", data.speed_pid_kp, data.speed_pid_ki, data.speed_pid_kd);
  BoardOled().Printf(0, 16, OLED_6X8, "T P%.1f I%.1f D%.1f", data.turn_pid_kp, data.turn_pid_ki, data.turn_pid_kd);
  /* Row 24-32: targets and actuals */
  BoardOled().Printf(0, 24, OLED_6X8, "AT%+05.1f A%+05.1f", data.angle_pid_target, data.angle);
  BoardOled().Printf(0, 32, OLED_6X8, "ST%+05.1f V%+05.1f", data.speed_pid_target, data.ave_speed);
  /* Row 40: PID outputs */
  BoardOled().Printf(0, 40, OLED_6X8, "uA%+04.0f uS%+04.0f uT%+04.0f", data.angle_pid_out, data.speed_pid_out, data.turn_pid_out);
  /* Row 48: live sensor (Gy raw, angle) */
  BoardOled().Printf(0, 48, OLED_6X8, "Gy%+05d Ang%+05.1f", data.gy, data.angle);
  /* Row 56 (bottom): calibration offsets — always visible */
  BoardOled().Printf(0, 56, OLED_6X8, "GO%+04d AO%+05.1f %s",
      gy_offset_, angle_offset_, data.run_flag ? "RUN" : "STP");
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
      /* TuneWizard telemetry: report all three loops */
      TW_SendData3(data.angle_pid_target, data.angle, data.angle_pid_out,
                   data.ave_speed, data.speed_pid_out,
                   data.dif_speed, data.turn_pid_out);
    }
  }

  /* Apply TuneWizard PID params if received from PC. */
  if (g_tw_params_updated != 0U)
  {
    IrqLock lock;
    angle_pid_.Configure(g_tw_kp, g_tw_ki, g_tw_kd);
    g_tw_params_updated = 0U;
  }
  if (g_tw_speed_updated != 0U)
  {
    IrqLock lock;
    speed_pid_.Configure(g_tw_skp, g_tw_ski, g_tw_skd);
    g_tw_speed_updated = 0U;
  }
  if (g_tw_turn_updated != 0U)
  {
    IrqLock lock;
    turn_pid_.Configure(g_tw_tkp, g_tw_tki, g_tw_tkd);
    g_tw_turn_updated = 0U;
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

      const int16_t gy_corr = (int16_t)(sample.gy - gy_offset_);
      const float angle_acc =
          (-atan2f((float)sample.ax, (float)sample.az) * 57.2957795f) + angle_offset_;
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
        const int16_t ave_pwm = (int16_t)(-angle_pid_.Update(angle_ ));
        int16_t new_left  = ClampPwm((int16_t)(ave_pwm + dif_pwm_ / 2));
        int16_t new_right = ClampPwm((int16_t)(ave_pwm - dif_pwm_ / 2));

        /* Adaptive PWM slew-rate limiter.
         * When gyro detects fast rotation (large |gy|), the D-term
         * produces large spikes. Limit how fast PWM can change per
         * cycle to prevent violent motor jerks, especially on
         * joystick reversal. During calm balancing, allow full speed.
         *
         * |gy| < 100 dps  => max_step = 30 (fast response)
         * |gy| > 300 dps  => max_step = 8  (tight limit)
         * in between      => linear interpolation
         */
        const float abs_gy = (gy_ > 0) ? (float)gy_ : (float)(-gy_);
        const float gy_dps = abs_gy * 0.0610352f;  /* gy/16.4 for ±2000dps */
        int16_t max_step;
        if (gy_dps < 100.0f) {
          max_step = 30;
        } else if (gy_dps > 300.0f) {
          max_step = 8;
        } else {
          /* Linear from 30 down to 8 over 100..300 dps range */
          max_step = (int16_t)(30.0f - (gy_dps - 100.0f) * 22.0f / 200.0f);
        }

        int16_t dl = new_left - left_pwm_;
        int16_t dr = new_right - right_pwm_;
        if (dl >  max_step) dl =  max_step;
        if (dl < -max_step) dl = -max_step;
        if (dr >  max_step) dr =  max_step;
        if (dr < -max_step) dr = -max_step;

        left_pwm_  = ClampPwm(left_pwm_ + dl);
        right_pwm_ = ClampPwm(right_pwm_ + dr);
        BoardMotor().SetPWM(1U, left_pwm_);
        BoardMotor().SetPWM(2U, right_pwm_);
      }
      else
      {
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
      /* Ramp speed/turn targets toward desired values (smooth joystick). */
      static const float kSpeedRamp = 0.2f;  /* max change per 50ms step */
      static const float kTurnRamp  = 0.5f;

      float cur_st = speed_pid_.Target();
      float delta_s = desired_speed_target_ - cur_st;
      if (delta_s > kSpeedRamp) delta_s = kSpeedRamp;
      if (delta_s < -kSpeedRamp) delta_s = -kSpeedRamp;
      speed_pid_.SetTarget(cur_st + delta_s);

      float cur_tt = turn_pid_.Target();
      float delta_t = desired_turn_target_ - cur_tt;
      if (delta_t > kTurnRamp) delta_t = kTurnRamp;
      if (delta_t < -kTurnRamp) delta_t = -kTurnRamp;
      turn_pid_.SetTarget(cur_tt + delta_t);

      angle_pid_.SetTarget(speed_pid_.Update(ave_speed_));
      dif_pwm_ = (int16_t)turn_pid_.Update(dif_speed_);
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
  out->angle_pid_kp = angle_pid_.Kp();
  out->angle_pid_ki = angle_pid_.Ki();
  out->angle_pid_kd = angle_pid_.Kd();
  out->speed_pid_kp = speed_pid_.Kp();
  out->speed_pid_ki = speed_pid_.Ki();
  out->speed_pid_kd = speed_pid_.Kd();
  out->turn_pid_kp = turn_pid_.Kp();
  out->turn_pid_ki = turn_pid_.Ki();
  out->turn_pid_kd = turn_pid_.Kd();
  out->angle_pid_target = angle_pid_.Target();
  out->speed_pid_target = speed_pid_.Target();
  out->turn_pid_target = turn_pid_.Target();
  out->angle_pid_out = angle_pid_.Output();
  out->speed_pid_out = speed_pid_.Output();
  out->turn_pid_out = turn_pid_.Output();
  out->angle_acc = angle_acc_;
  out->angle_gyro = angle_gyro_;
  out->angle = angle_;
  out->ave_speed = ave_speed_;
  out->dif_speed = dif_speed_;
}

}  // namespace app
