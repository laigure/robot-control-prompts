#include "pid.h"

/* Build PID controller with default gains/state. */
PidController::PidController()
    : target_(0.0f), actual_(0.0f), out_(0.0f),
      kp_(0.0f), ki_(0.0f), kd_(0.0f),
      error0_(0.0f), error1_(0.0f), error_int_(0.0f),
      out_max_(100.0f), out_min_(-100.0f)
{
}

/* Build PID controller with provided gains and output limits. */
PidController::PidController(float kp, float ki, float kd, float out_min, float out_max,float offset)
    : target_(0.0f), actual_(0.0f), out_(0.0f),
      kp_(kp), ki_(ki), kd_(kd),
      error0_(0.0f), error1_(0.0f), error_int_(0.0f),
      out_max_(out_max), out_min_(out_min),offset_(offset)
{
  SetOutputLimits(out_min, out_max);
}

/* Clamp helper to keep values within [min, max]. */
float PidController::Clamp(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

/* Reset runtime state. */
void PidController::Init(void)
{
  target_ = 0.0f;
  actual_ = 0.0f;
  out_ = 0.0f;
  error0_ = 0.0f;
  error1_ = 0.0f;
  error_int_ = 0.0f;
}

/* Update gains at runtime. */
void PidController::Configure(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

/* Update output clamp range and keep current output inside new limits. */
void PidController::SetOutputLimits(float out_min, float out_max)
{
  if (out_min > out_max)
  {
    const float temp = out_min;
    out_min = out_max;
    out_max = temp;
  }
  out_min_ = out_min;
  out_max_ = out_max;
  out_ = Clamp(out_, out_min_, out_max_);
}

/* Set controller setpoint. */
void PidController::SetTarget(float target)
{
  target_ = target;
}

/* Get current setpoint. */
float PidController::Target(void) const
{
  return target_;
}

/* Get latest output. */
float PidController::Output(void) const
{
  return out_;
}

/* Get proportional gain. */
float PidController::Kp(void) const
{
  return kp_;
}

/* Get integral gain. */
float PidController::Ki(void) const
{
  return ki_;
}

/* Get derivative gain. */
float PidController::Kd(void) const
{
  return kd_;
}

float PidController::Offset(void) const
{
  return offset_;
}
void PidController::SetOffset(float offset){
  offset_=offset;
}
/* Run one update step with anti-windup:
 * e(k)=target-actual
 * u=Kp*e + Ki*sum(e) + Kd*(e-e_prev)
 * u is clamped to configured output limits.
 *
 * Anti-windup: the integral term stops accumulating when the output
 * is already saturated in the same direction as the error. This
 * prevents "integral windup" that causes large oscillations after
 * fast movements (e.g. joystick release on a balance car).
 */
float PidController::Update(float actual)
{
  actual_ = actual;
  error1_ = error0_;
  error0_ = target_ - actual_;

  if (ki_ != 0.0f)
  {
    /* Conditional integration: only accumulate if output is not
     * saturated, or if the error would help reduce saturation. */
    const float new_int = error_int_ + error0_;
    const float unclamped_out = kp_ * error0_ + ki_ * new_int + kd_ * (error0_ - error1_);

    /* Allow integration if:
     * 1) output is within limits, OR
     * 2) the new error pushes the integral toward zero (unwinding) */
    if ((unclamped_out > out_min_ && unclamped_out < out_max_) ||
        (error0_ * error_int_ < 0.0f))
    {
      error_int_ = new_int;
    }

    /* Hard clamp: integral can never exceed the output range / Ki.
     * This bounds the maximum integral contribution. */
    const float int_max = (out_max_ - out_min_) / ki_;
    if (error_int_ > int_max)  error_int_ = int_max;
    if (error_int_ < -int_max) error_int_ = -int_max;
  }
  else
  {
    error_int_ = 0.0f;
  }

  out_ = kp_ * error0_ + ki_ * error_int_ + kd_ * (error0_ - error1_);
  if(out_>0)
	  out_+=offset_;
  if(out_<0)
	  out_-=offset_;
  out_ = Clamp(out_, out_min_, out_max_);
  return out_;
}

extern "C" {

/* C compatibility wrapper: reset PID runtime state. */
void PID_Init(PID_t *p)
{
  if (p == 0)
  {
    return;
  }
  p->Target = 0.0f;
  p->Actual = 0.0f;
  p->Out = 0.0f;
  p->Error0 = 0.0f;
  p->Error1 = 0.0f;
  p->ErrorInt = 0.0f;
}

/* C compatibility wrapper: one PID update step with output clamp. */
void PID_Update(PID_t *p)
{
  if (p == 0)
  {
    return;
  }

  p->Error1 = p->Error0;
  p->Error0 = p->Target - p->Actual;

  if (p->Ki != 0.0f)
  {
    p->ErrorInt += p->Error0;
  }
  else
  {
    p->ErrorInt = 0.0f;
  }

  p->Out = p->Kp * p->Error0
         + p->Ki * p->ErrorInt
         + p->Kd * (p->Error0 - p->Error1);
  
  if (p->Out > p->OutMax)
  {
    p->Out = p->OutMax;
  }
  if (p->Out < p->OutMin)
  {
    p->Out = p->OutMin;
  }
}

}  // extern "C"
