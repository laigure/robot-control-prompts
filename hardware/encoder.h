#pragma once

#include <stdint.h>
#include "tim.h"

#ifdef __cplusplus
/** @brief Incremental encoder reader based on two HAL timer encoder interfaces. */
class EncoderReader {
public:
  /** @brief Construct with left/right timer handles and right-channel polarity option. */
  EncoderReader(TIM_HandleTypeDef *left, TIM_HandleTypeDef *right, bool invert_right);

  /** @brief Start encoder interfaces and clear counters. */
  void Init(void);
  /** @brief Read delta by channel index: 1=left, 2=right. */
  int16_t Get(uint8_t n);
  /** @brief Read and reset left encoder delta count. */
  int16_t GetLeft(void);
  /** @brief Read and reset right encoder delta count. */
  int16_t GetRight(void);

private:
  TIM_HandleTypeDef *left_;
  TIM_HandleTypeDef *right_;
  bool invert_right_;
};

/** @brief Get board default encoder singleton. */
EncoderReader &BoardEncoder(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief C compatibility: initialize encoder reader. */
void Encoder_Init(void);
/** @brief C compatibility: read encoder delta by index (1/2). */
int16_t Encoder_Get(uint8_t n);

#ifdef __cplusplus
}
#endif
