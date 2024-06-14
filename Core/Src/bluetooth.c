/**
 * @file bluetooth.c
 * @author Evgeniy Shabin (you@domain.com)
 * @brief Bluetooth control program module
 * @version 0.1
 * @date 2024-03-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bluetooth.h"

void NextTrack(void)
{
  if (CONFIRMATION_SOUND)
  {
    Beep(SOUND_TAC);
  }
  HAL_GPIO_WritePin(NEXT_GPIO_Port, NEXT_Pin, 1);
  osDelay(CSR_SHORT_PRESS);
  HAL_GPIO_WritePin(NEXT_GPIO_Port, NEXT_Pin, 0);
}

void PrevTrack(void)
{
  if (CONFIRMATION_SOUND)
  {
    Beep(SOUND_TIC);
  }
  HAL_GPIO_WritePin(PREV_GPIO_Port, PREV_Pin, 1);
  osDelay(CSR_SHORT_PRESS);
  HAL_GPIO_WritePin(PREV_GPIO_Port, PREV_Pin, 0);
}

void PlayPause(void)
{
  if (CONFIRMATION_SOUND)
  {
    Beep(SOUND_TIC);
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
    HAL_GPIO_WritePin(AMPENA_GPIO_Port, AMPENA_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(BTEN_GPIO_Port, BTEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AMPENA_GPIO_Port, AMPENA_Pin, GPIO_PIN_RESET);
  }
}
