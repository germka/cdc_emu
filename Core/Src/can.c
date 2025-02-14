/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "gpio.h"
#include "cmsis_os.h"
#include "CDC.h"
#include "stdbool.h"

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

extern bool cdcActive;

extern osSemaphoreId powerStateHandle;

extern osMessageQId nodeStatusQueueHandle;
extern osMessageQId cdcCtlQueueHandle;
extern osMessageQId wheelBtnQueueHandle;
extern osMessageQId sidBtnQueueHandle;
extern osMessageQId canEventQueueHandle;
extern osMessageQId indicatorQueueHandle;

extern uint8_t msg_beep[8];
extern uint8_t msg_tack[8];
extern uint8_t msg_tick[8];
extern uint8_t msg_seatbelt[8];
extern uint8_t msg_ding_dong[8];

BaseType_t xHigherPriorityTaskWoken;

uint16_t indRX;
uint16_t indTX;
uint16_t indERR;

uint8_t can_tx_err_cnt = 0;
uint8_t gear = 0;
uint8_t shift = 0;

bool can_offline = false;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 36;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_User_Config(void)
{
  /* Configure the CAN Filter */
  CAN_FilterTypeDef  sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x0368<<5;     // SID text priority    // DISPLAY_RESOURCE_GRANT
  sFilterConfig.FilterIdLow = 0x03C0<<5;      // CD Changer control   // CDC_CONTROL
  sFilterConfig.FilterMaskIdHigh = 0x06A1<<5; // Audio head unit      // NODE_STATUS_RX_IHU
  sFilterConfig.FilterMaskIdLow = 0x0290<<5;  // Buttons              // STEERING_WHEEL_BUTTONS
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sFilterConfig.FilterBank = 1;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x0280<<5;     // Rear gear state      // PEDALS_GEAR
  sFilterConfig.FilterIdLow = 0x03E0<<5;      // CD Changer control   // AUTOMATIC_GEARBOX
  sFilterConfig.FilterMaskIdHigh = 0x04A0<<5; // Steering wheel, VIN  // VIN_STATUS_TRIONIC
  sFilterConfig.FilterMaskIdLow = 0x0;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Transmission process */
  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x00;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
}

void CAN_GO_Offline(void)
{
  can_offline = true;
  xQueueReset(canEventQueueHandle);
  
  for (uint8_t i = HAL_CAN_GetTxMailboxesFreeLevel(&hcan); i < 3; i++)
  {
    if (HAL_CAN_AbortTxRequest(&hcan, TxMailbox) != HAL_OK)
    {
      break;
    }
  }
}

void CAN_Send_Data(uint32_t ID, uint8_t data[CAN_DATA_LEN])
{
  TxHeader.StdId = ID;
#if 0  // debug offline mode
  uint8_t m_cnt = uxQueueMessagesWaiting(canEventQueueHandle);
  uint8_t e_cnt = uxQueueMessagesWaiting(indicatorQueueHandle);
#endif
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
  {
    indERR = ( 30 << 8 ) + err_pin;
    xQueueSend(indicatorQueueHandle, &indERR, 0);
    
    can_tx_err_cnt++;
    if (can_tx_err_cnt > 9)
    {
      can_tx_err_cnt = 0;
      CAN_GO_Offline();

      if (cdcActive)
      {
        cdcActive = !cdcActive;
        xSemaphoreGive(powerStateHandle);
      }
      return;
    }
  }
  else if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
  {
    indERR = ( 255 << 8 ) + err_pin;
    xQueueSend(indicatorQueueHandle, &indERR, 0);
  }
  else
  {
    indTX = ( 10 << 8 ) + tx_pin;
    xQueueSend(indicatorQueueHandle, &indTX, 0);
  }
}

/**
 * Get RX message
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }
  else
  {
    can_offline = false;
    indRX = (10 << 8) + rx_pin;
    xQueueSendFromISR(indicatorQueueHandle, &indRX, &xHigherPriorityTaskWoken);
  }

  switch (RxHeader.StdId)
  {
  case NODE_STATUS_RX_IHU:
    xQueueSendToBackFromISR(nodeStatusQueueHandle, &RxData[3], 0);
    break;
  case CDC_CONTROL:
    if (RxData[0] == STATE_CHANGED)
    {
      xQueueSendToBackFromISR(cdcCtlQueueHandle, &RxData[1], 0);
    }
    break;
  case STEERING_WHEEL_BUTTONS:
    if (RxData[0] == STATE_CHANGED)
    {
      if (RxData[2] != 0x00)
      {
        xQueueSendToBackFromISR(wheelBtnQueueHandle, &RxData[2], 0); // In case Wheel buttons
      }
      else if (RxData[3] != 0x00)
      {
        xQueueSendToBackFromISR(sidBtnQueueHandle, &RxData[3], 0); // In case SID buttons
      }
    }
    break;
  case DISPLAY_RESOURCE_GRANT:
    if ((RxData[0] == (DESIRED_ROW & 0x0F)) && (RxData[1] == NODE_SID_FUNCTION_ID))
    {
    //  if (xSemaphoreGiveFromISR(allowTextSemaphoreHandle, &xHigherPriorityTaskWoken) != pdPASS)
    //  {
    //    Error_Handler();
    //  }
    }
    break;
  case AUTOMATIC_GEARBOX:
    if (RxData[0] == STATE_CHANGED)
    {
      if (RxData[1] == GEAR_REVERSE)
      {
        // Disable AMP power
        shift = GEAR_REVERSE;
      }
      else
      {
        // Enable AMP
        shift = RxData[0];
      }
      xSemaphoreGiveFromISR(powerStateHandle, &xHigherPriorityTaskWoken);
    }
    break;
  case PEDALS_GEAR:
    if (RxData[0] == STATE_CHANGED)  // State changed
    {
      if (RxData[1] == REVERS_ON && RxData[5] == ENGINE_ON)
      {
        // Disable AMP power
        gear = GEAR_REVERSE;
      }
      else
      {
        // Enable AMP
        gear = RxData[1];
      }
      xSemaphoreGiveFromISR(powerStateHandle, &xHigherPriorityTaskWoken);
    }
    break;
  default:
    break;
  }
}


void Beep(uint8_t type)
{
  can_event_t soundEvent = {0};
  switch (type)
  {
  case SOUND_ACK:
    soundEvent.data_ptr = msg_beep;
    break;
  case SOUND_TAC:
    soundEvent.data_ptr = msg_tack;
    break;
  case SOUND_TIC:
    soundEvent.data_ptr = msg_tick;
    break;
  case SOUND_SEATBELT:
    soundEvent.data_ptr = msg_seatbelt;
    break;
  case SOUND_ALERT:
    soundEvent.data_ptr = msg_ding_dong;
    break;
  default:
    soundEvent.data_ptr = msg_beep;
    break;
  }
  soundEvent.data_id = SOUND_REQUEST;
  soundEvent.priority = 0;
  soundEvent.data_len = CAN_DATA_LEN;
  xQueueSendToBack(canEventQueueHandle, &soundEvent, 0);
}

/* USER CODE END 1 */
