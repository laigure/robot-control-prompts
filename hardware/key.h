#pragma once

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
/** @brief Debounced 4-key input scanner (active-low). */
class KeyInput {
public:
  /** @brief Construct with four key GPIO definitions. */
  KeyInput(GPIO_TypeDef *key1_port, uint16_t key1_pin,
           GPIO_TypeDef *key2_port, uint16_t key2_pin,
           GPIO_TypeDef *key3_port, uint16_t key3_pin,
           GPIO_TypeDef *key4_port, uint16_t key4_pin);

  /** @brief Reset scanner state. */
  void Init(void);
  /** @brief Get one debounced key event (1..4), 0 if none. */
  uint8_t GetNum(void);
  /** @brief Read instant current key state (1..4), 0 if no press. */
  uint8_t GetState(void) const;
  /** @brief 1 ms periodic scan/debounce tick. */
  void Tick(void);

private:
  GPIO_TypeDef *key1_port_;
  uint16_t key1_pin_;
  GPIO_TypeDef *key2_port_;
  uint16_t key2_pin_;
  GPIO_TypeDef *key3_port_;
  uint16_t key3_pin_;
  GPIO_TypeDef *key4_port_;
  uint16_t key4_pin_;

  volatile uint8_t key_num_;
  uint8_t tick_count_;
  uint8_t curr_state_;
  uint8_t prev_state_;
};

/** @brief Get board default key scanner singleton. */
KeyInput &BoardKey(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief C compatibility: initialize key scanner. */
void Key_Init(void);
/** @brief C compatibility: get one key event (1..4), 0 if none. */
uint8_t Key_GetNum(void);
/** @brief C compatibility: read instant key state (1..4), 0 if none. */
uint8_t Key_GetState(void);
/** @brief C compatibility: 1 ms key scan tick. */
void Key_Tick(void);

#ifdef __cplusplus
}
#endif
