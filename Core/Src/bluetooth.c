/**
 * @file bluetooth.c
 * @author Evgeniy Shabin (germka@gmail.com)
 * @brief Bluetooth control program module
 * @version 0.1
 * @date 2024-03-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bluetooth.h"

extern bool R_gear_on;

void NextTrack(void)
{
  if (CONFIRMATION_SOUND)
  {
    Beep(0x08);
  }
  HAL_GPIO_WritePin(NEXT_GPIO_Port, NEXT_Pin, 1);
  osDelay(CSR_SHORT_PRESS);
  HAL_GPIO_WritePin(NEXT_GPIO_Port, NEXT_Pin, 0);
}

void PrevTrack(void)
{
  if (CONFIRMATION_SOUND)
  {
    Beep(0x10);
  }
  HAL_GPIO_WritePin(PREV_GPIO_Port, PREV_Pin, 1);
  osDelay(CSR_SHORT_PRESS);
  HAL_GPIO_WritePin(PREV_GPIO_Port, PREV_Pin, 0);
}

void PlayPause(void)
{
  if (CONFIRMATION_SOUND)
  {
    Beep(0x10);
  }
  HAL_GPIO_WritePin(PLAY_GPIO_Port, PLAY_Pin, 1);
  osDelay(CSR_SHORT_PRESS);
  HAL_GPIO_WritePin(PLAY_GPIO_Port, PLAY_Pin, 0);
}

// TODO: add funcs for bt discoverable and another states by whell and SID btns combinations

/**
 * @brief Enables audio modules by GPIO pins
 * 
 * @param state 
 */
void audioPower(bool state)
{
  if (state)
  {
    HAL_GPIO_WritePin(BTEN_GPIO_Port, BTEN_Pin, GPIO_PIN_SET);
    // Do not enable if reverse gear ON and engine ON
    HAL_GPIO_WritePin(AMPEN_GPIO_Port, AMPEN_Pin, R_gear_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (CONFIRMATION_SOUND)
    {
      Beep(0x04);
    }
  }
  else
  {
    HAL_GPIO_WritePin(BTEN_GPIO_Port, BTEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AMPEN_GPIO_Port, AMPEN_Pin, GPIO_PIN_RESET);
  }
}
