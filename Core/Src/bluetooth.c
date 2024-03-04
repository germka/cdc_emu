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

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "bluetooth.h"

void NextTrack(void){
  if (CONFIRMATION_SOUND){
    Beep(0x08);
  }
  HAL_GPIO_WritePin(Next_GPIO_Port, Next_Pin, 1);
  osDelay(CSR_SHORT_PRESS);
  HAL_GPIO_WritePin(Next_GPIO_Port, Next_Pin, 0);
}

void PrevTrack(void){
  if (CONFIRMATION_SOUND){
    Beep(0x10);
  }
  HAL_GPIO_WritePin(Prev_GPIO_Port, Prev_Pin, 1);
  osDelay(CSR_SHORT_PRESS);
  HAL_GPIO_WritePin(Prev_GPIO_Port, Prev_Pin, 0);
}

void PlayPause(void){
  if (CONFIRMATION_SOUND){
	Beep(0x10);
  }
  HAL_GPIO_WritePin(Play_Pause_GPIO_Port, Play_Pause_Pin, 1);
  osDelay(CSR_SHORT_PRESS);
  HAL_GPIO_WritePin(Play_Pause_GPIO_Port, Play_Pause_Pin, 0);
}

//TODO: add funcs for bt discoverable and another states by whell and SID btns combinations

void audioPower(bool state){
	if (state && !HAL_GPIO_ReadPin(BTEN_GPIO_Port, BTEN_Pin)) {
		if (CONFIRMATION_SOUND && HAL_GPIO_ReadPin(AMPEN_GPIO_Port, AMPEN_Pin)){
			Beep(0x04);
		}
		HAL_GPIO_WritePin(BTEN_GPIO_Port, BTEN_Pin, 1);
	} else if (!state) {
		HAL_GPIO_WritePin(BTEN_GPIO_Port, BTEN_Pin, 0);
	}
}