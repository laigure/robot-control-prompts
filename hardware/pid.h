#pragma once

#include <stdint.h>

#ifdef __cplusplus
/** @brief Discrete PID controller with output clamp. */
class PidController {
public:
  /** @brief Construct with default gains/state (all zero, output range [-100, 100]). */
  PidController();
  /** @brief Construct and configure gains/output range. */
  PidController(float kp, float ki, float kd, float out_min, float out_max,float offset);

  /** @brief Reset runtime state (target/actual/error/integral/output). */
  void Init(void);
  /** @brief Update PID gains. */
  void Configure(float kp, float ki, float kd);
  /** @brief Set controller output clamp range. */
  void SetOutputLimits(float out_min, float out_max);
  /** @brief Set target setpoint. */
  void SetTarget(float target);
  /** @brief Get current setpoint. */
  float Target(void) const;
  /** @brief Get latest controller output. */
  float Output(void) const;
  /** @brief Get current proportional gain. */
  float Kp(void) const;
  /** @brief Get current integral gain. */
  float Ki(void) const;
  /** @brief Get current derivative gain. */
  float Kd(void) const;
  float Offset(void) const;
  void SetOffset(float offset);
  /** @brief Run one PID update step with current actual value. Returns clamped output. */
  float Update(float actual);

private:
  static float Clamp(float value, float min_value, float max_value);

private:
  float target_;
  float actual_;
  float out_;
  float kp_;
  float ki_;
  float kd_;
  float error0_;
  float error1_;
  float error_int_;
  float out_max_;
  float out_min_;
  float offset_;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Legacy C PID struct (kept for compatibility). */
typedef struct {
  float Target;
  float Actual;
  float Out;

  float Kp;
  float Ki;
  float Kd;

  float Error0;
  float Error1;
  float ErrorInt;

  float OutMax;
  float OutMin;
} PID_t;

/** @brief C compatibility: reset PID state. */
void PID_Init(PID_t *p);
/** @brief C compatibility: run one PID update. */
void PID_Update(PID_t *p);

#ifdef __cplusplus
}
#endif
