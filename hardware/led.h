#pragma once

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus

/** @brief One LED output channel descriptor. */
struct LedChannel {
  /** @brief GPIO port pointer. */
  GPIO_TypeDef *port;
  /** @brief GPIO pin mask (HAL GPIO_PIN_x). */
  uint16_t pin;
};

/** @brief Single LED helper class. */
class LedDevice {
public:
  /** @brief Construct with target GPIO port/pin. */
  LedDevice(GPIO_TypeDef *port, uint16_t pin);

  /** @brief Initialize LED to off state. */
  void Init(void);
  /** @brief Set LED on. */
  void On(void);
  /** @brief Set LED off. */
  void Off(void);
  /** @brief Toggle LED state. */
  void Toggle(void);
  /** @brief Legacy blocking blink loop. */
  void BlinkLoop(uint32_t delay_ms);

private:
  GPIO_TypeDef *port_;
  uint16_t pin_;
};

/** @brief Get board default LED singleton. */
LedDevice &BoardLed(void);

/** @brief Non-blocking LED flow/blink scheduler (driven by 1 ms ticks). */
class LedFlowRunner {
public:
  /** @brief Construct with channel table and channel count. */
  LedFlowRunner(const LedChannel *channels, uint8_t count);

  /** @brief Reset internal state and apply initial output. */
  void Init(void);
  /** @brief 1 ms periodic tick entry. */
  void Tick1ms(void);
  /** @brief Set step period in milliseconds. */
  void SetPeriodMs(uint16_t period_ms);
  /** @brief Enable or disable output. */
  void SetEnable(bool enable);
  /** @brief Set forward/backward running direction. */
  void SetForward(bool forward);
  /** @brief Get current direction flag. */
  bool IsForward(void) const;
  /** @brief Get current enable flag. */
  bool IsEnabled(void) const;
  /** @brief Get current step period in milliseconds. */
  uint16_t PeriodMs(void) const;

private:
  void ApplyState(void);
  void Advance(void);

private:
  const LedChannel *channels_;
  uint8_t count_;
  uint8_t index_;
  uint16_t elapsed_ms_;
  uint16_t period_ms_;
  bool enable_;
  bool forward_;
  bool blink_state_;
};

/** @brief Get board default LED flow singleton. */
LedFlowRunner &BoardLedFlow(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief C compatibility: initialize single LED. */
void Led_Init(void);
/** @brief C compatibility: LED on. */
void Led_On(void);
/** @brief C compatibility: LED off. */
void Led_Off(void);
/** @brief C compatibility: LED toggle. */
void Led_Toggle(void);
/** @brief C compatibility: blocking blink loop. */
void Led_BlinkLoop(uint32_t delay_ms);

/** @brief C compatibility: initialize flow runner. */
void LedFlow_Init(void);
/** @brief C compatibility: 1 ms flow tick. */
void LedFlow_Tick(void);
/** @brief C compatibility: set flow period in milliseconds. */
void LedFlow_SetPeriodMs(uint16_t period_ms);
/** @brief C compatibility: enable/disable flow output. */
void LedFlow_SetEnable(uint8_t enable);
/** @brief C compatibility: set flow direction flag. */
void LedFlow_SetForward(uint8_t forward);

#ifdef __cplusplus
}
#endif
