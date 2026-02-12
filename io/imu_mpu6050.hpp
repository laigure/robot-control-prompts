#pragma once

#include <stdint.h>

#include "mpu6050.h"
#include "attitude_estimator.hpp"

namespace io {

class Mpu6050Imu {
public:
  explicit Mpu6050Imu(Mpu6050Device &device = BoardMpu6050())
      : device_(device)
  {
  }

  bool Init(void)
  {
    (void)device_.Init();
    const uint8_t id = device_.GetID();
    return (id == 0x68U) || (id == 0x69U) || (id == 0x70U) || (id == 0x71U);
  }

  void Read(tasks::ImuSample *sample)
  {
    device_.GetData(&sample->ax, &sample->ay, &sample->az,
                    &sample->gx, &sample->gy, &sample->gz);
  }

private:
  Mpu6050Device &device_;
};

}  // namespace io
