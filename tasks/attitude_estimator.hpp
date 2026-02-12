#pragma once

#include <stdint.h>
#include <math.h>

namespace tasks {

struct ImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

class AttitudeEstimator {
public:
  AttitudeEstimator()
      : alpha_(0.013f), dt_sec_(0.001f), gyro_bias_lsb_(16.0f),
        angle_acc_deg_(0.0f), angle_gyro_deg_(0.0f), angle_deg_(0.0f) {}

  void Configure(float alpha, float dt_sec, float gyro_bias_lsb)
  {
    alpha_ = alpha;
    dt_sec_ = dt_sec;
    gyro_bias_lsb_ = gyro_bias_lsb;
  }

  void Reset(void)
  {
    angle_acc_deg_ = 0.0f;
    angle_gyro_deg_ = 0.0f;
    angle_deg_ = 0.0f;
  }

  void Update(const ImuSample &sample)
  {
    const float gyro_y_lsb = (float)sample.gy - gyro_bias_lsb_;
    angle_acc_deg_ = (float)(-atan2((double)sample.ax, (double)sample.az) * 57.2957795);
    angle_gyro_deg_ = angle_deg_ + (gyro_y_lsb / 32768.0f) * 2000.0f * dt_sec_;
    angle_deg_ = alpha_ * angle_acc_deg_ + (1.0f - alpha_) * angle_gyro_deg_;
  }

  float AngleAccDeg(void) const { return angle_acc_deg_; }
  float AngleGyroDeg(void) const { return angle_gyro_deg_; }
  float AngleDeg(void) const { return angle_deg_; }

private:
  float alpha_;
  float dt_sec_;
  float gyro_bias_lsb_;
  float angle_acc_deg_;
  float angle_gyro_deg_;
  float angle_deg_;
};

}  // namespace tasks
