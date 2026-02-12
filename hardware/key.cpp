#include "key.h"

#include "main.h"
#include "stm32f1xx_hal.h"

/* Map four GPIO input pins to one key scanner object. */
KeyInput::KeyInput(GPIO_TypeDef *key1_port, uint16_t key1_pin,
                   GPIO_TypeDef *key2_port, uint16_t key2_pin,
                   GPIO_TypeDef *key3_port, uint16_t key3_pin,
                   GPIO_TypeDef *key4_port, uint16_t key4_pin)
    : key1_port_(key1_port), key1_pin_(key1_pin),
      key2_port_(key2_port), key2_pin_(key2_pin),
      key3_port_(key3_port), key3_pin_(key3_pin),
      key4_port_(key4_port), key4_pin_(key4_pin),
      key_num_(0U), tick_count_(0U), curr_state_(0U), prev_state_(0U)
{
}

/* Reset key scanner runtime state. */
void KeyInput::Init(void)
{
  key_num_ = 0U;
  tick_count_ = 0U;
  curr_state_ = 0U;
  prev_state_ = 0U;
}

/* Return one edge event key code, then clear event latch. */
uint8_t KeyInput::GetNum(void)
{
  uint8_t temp = 0U;
  __disable_irq();
  if (key_num_ != 0U)
  {
    temp = key_num_;
    key_num_ = 0U;
  }
  __enable_irq();
  return temp;
}

/* Read current key level state (active-low). */
uint8_t KeyInput::GetState(void) const
{
  /* Active-low keys: pressed = GPIO_PIN_RESET. */
  if (HAL_GPIO_ReadPin(key1_port_, key1_pin_) == GPIO_PIN_RESET)
  {
    return 1U;
  }
  if (HAL_GPIO_ReadPin(key2_port_, key2_pin_) == GPIO_PIN_RESET)
  {
    return 2U;
  }
  if (HAL_GPIO_ReadPin(key3_port_, key3_pin_) == GPIO_PIN_RESET)
  {
    return 3U;
  }
  if (HAL_GPIO_ReadPin(key4_port_, key4_pin_) == GPIO_PIN_RESET)
  {
    return 4U;
  }
  return 0U;
}

/* 1 ms scan tick with 20 ms debounce and release-edge event output. */
void KeyInput::Tick(void)
{
  tick_count_++;
  if (tick_count_ >= 20U)
  {
    tick_count_ = 0U;

    prev_state_ = curr_state_;
    curr_state_ = GetState();

    if ((curr_state_ == 0U) && (prev_state_ != 0U))
    {
      key_num_ = prev_state_;
    }
  }
}

namespace {
/* Board default key mapping from CubeMX pin defines. */
KeyInput g_key(key_1_GPIO_Port, key_1_Pin,
               key_2_GPIO_Port, key_2_Pin,
               key_3_GPIO_Port, key_3_Pin,
               key_4_GPIO_Port, key_4_Pin);
}

/* Accessor for the board key singleton. */
KeyInput &BoardKey(void)
{
  return g_key;
}

extern "C" {

/* C compatibility wrapper: initialize key scanner. */
void Key_Init(void)
{
  BoardKey().Init();
}

/* C compatibility wrapper: fetch one debounced key event. */
uint8_t Key_GetNum(void)
{
  return BoardKey().GetNum();
}

/* C compatibility wrapper: read instant key level state. */
uint8_t Key_GetState(void)
{
  return BoardKey().GetState();
}

/* C compatibility wrapper: 1 ms key scanner tick. */
void Key_Tick(void)
{
  BoardKey().Tick();
}

}  // extern "C"
