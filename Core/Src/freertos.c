/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "CDC.h"
#include "bluetooth.h"
#include "gpio.h"
#include <string.h>
#include "stdbool.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* CDC messages*/

uint8_t cdcActiveCmd[4][8] = {
  {0x32,0x00,0x00,0x16,0x01,0x02,0x00,0x00},
  {0x42,0x00,0x00,0x36,0x00,0x00,0x00,0x00},
  {0x52,0x00,0x00,0x36,0x00,0x00,0x00,0x00},
  {0x62,0x00,0x00,0x36,0x00,0x00,0x00,0x00},
};
uint8_t cdcPoweronCmd[4][8] = {
  {0x32,0x00,0x00,0x03,0x01,0x02,0x00,0x00},
  {0x42,0x00,0x00,0x22,0x00,0x00,0x00,0x00},
  {0x52,0x00,0x00,0x22,0x00,0x00,0x00,0x00},
  {0x62,0x00,0x00,0x22,0x00,0x00,0x00,0x00}
};
uint8_t cdcPowerdownCmd[4][8] = {
  {0x32,0x00,0x00,0x19,0x01,0x00,0x00,0x00},
  {0x42,0x00,0x00,0x38,0x01,0x00,0x00,0x00},
  {0x52,0x00,0x00,0x38,0x01,0x00,0x00,0x00},
  {0x62,0x00,0x00,0x38,0x01,0x00,0x00,0x00}
};
uint8_t cdcActiveStatus[] = {
  0xE0,0xFF,0x3F,0x41,0xFF,0xFF,0xFF,0xD0
};
uint8_t cdcInactiveStatus[] = {
  0xE0,0xFF,0x3F,0x00,0x00,0x00,0x00,0xD0
};
uint8_t cdcActiveStatusEvent[] = {
  0x20,0xFF,0x3F,0x41,0xFF,0xFF,0xFF,0xD0
};
uint8_t cdcInactiveStatusEvent[] = {
  0x20,0xFF,0x3F,0x00,0x00,0x00,0x00,0xD0
};

/* SID sounds messages */

uint8_t msg_beep[] = {
  0x80,0x04,0x00,0x00,0x00,0x00,0x00,0x00
};
uint8_t msg_tack[] = {
  0x80,0x08,0x00,0x00,0x00,0x00,0x00,0x00
};
uint8_t msg_tick[] = {
  0x80,0x10,0x00,0x00,0x00,0x00,0x00,0x00
};
uint8_t msg_seatbelt[] = {
  0x80,0x20,0x00,0x00,0x00,0x00,0x00,0x00
};
uint8_t msg_ding_dong[] = {
  0x80,0x40,0x00,0x00,0x00,0x00,0x00,0x00
};

bool cdcActive = false;
uint32_t canRXTimestamp = 0;

extern bool can_offline;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId indicatorHandleTaskHandle;
osThreadId nodeStatusTaskHandle;
osThreadId cdcCtlTaskHandle;
osThreadId wheelBtnHandleTaskHandle;
osThreadId sidBtnHandleTaskHandle;
osThreadId canSenderTaskHandle;
osThreadId audioPwrMngTaskHandle;
osThreadId idleTaskHandle;
osMessageQId indicatorQueueHandle;
osMessageQId nodeStatusQueueHandle;
osMessageQId cdcCtlQueueHandle;
osMessageQId wheelBtnQueueHandle;
osMessageQId sidBtnQueueHandle;
osMessageQId canEventQueueHandle;
osSemaphoreId powerStateHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void enterSleep(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartIndicatorHandleTask(void const * argument);
void StartNodeStatusTask(void const * argument);
void StartCdcCtlTask(void const * argument);
void StartWheelBtnHandleTask(void const * argument);
void StartSidBtnHandleTask(void const * argument);
void StartCanSenderTask(void const * argument);
void StartAudioPwrMngTask(void const * argument);
void StartIdleTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of powerState */
  osSemaphoreDef(powerState);
  powerStateHandle = osSemaphoreCreate(osSemaphore(powerState), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of indicatorQueue */
  osMessageQDef(indicatorQueue, 16, uint16_t);
  indicatorQueueHandle = osMessageCreate(osMessageQ(indicatorQueue), NULL);

  /* definition and creation of nodeStatusQueue */
  osMessageQDef(nodeStatusQueue, 16, uint8_t);
  nodeStatusQueueHandle = osMessageCreate(osMessageQ(nodeStatusQueue), NULL);

  /* definition and creation of cdcCtlQueue */
  osMessageQDef(cdcCtlQueue, 16, uint8_t);
  cdcCtlQueueHandle = osMessageCreate(osMessageQ(cdcCtlQueue), NULL);

  /* definition and creation of wheelBtnQueue */
  osMessageQDef(wheelBtnQueue, 16, uint8_t);
  wheelBtnQueueHandle = osMessageCreate(osMessageQ(wheelBtnQueue), NULL);

  /* definition and creation of sidBtnQueue */
  osMessageQDef(sidBtnQueue, 16, uint8_t);
  sidBtnQueueHandle = osMessageCreate(osMessageQ(sidBtnQueue), NULL);

  /* definition and creation of canEventQueue */
  osMessageQDef(canEventQueue, 3, can_event_t);
  canEventQueueHandle = osMessageCreate(osMessageQ(canEventQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of indicatorHandleTask */
  osThreadDef(indicatorHandleTask, StartIndicatorHandleTask, osPriorityIdle, 0, 128);
  indicatorHandleTaskHandle = osThreadCreate(osThread(indicatorHandleTask), NULL);

  /* definition and creation of nodeStatusTask */
  osThreadDef(nodeStatusTask, StartNodeStatusTask, osPriorityIdle, 0, 128);
  nodeStatusTaskHandle = osThreadCreate(osThread(nodeStatusTask), NULL);

  /* definition and creation of cdcCtlTask */
  osThreadDef(cdcCtlTask, StartCdcCtlTask, osPriorityIdle, 0, 128);
  cdcCtlTaskHandle = osThreadCreate(osThread(cdcCtlTask), NULL);

  /* definition and creation of wheelBtnHandleTask */
  osThreadDef(wheelBtnHandleTask, StartWheelBtnHandleTask, osPriorityIdle, 0, 128);
  wheelBtnHandleTaskHandle = osThreadCreate(osThread(wheelBtnHandleTask), NULL);

  /* definition and creation of sidBtnHandleTask */
  osThreadDef(sidBtnHandleTask, StartSidBtnHandleTask, osPriorityIdle, 0, 128);
  sidBtnHandleTaskHandle = osThreadCreate(osThread(sidBtnHandleTask), NULL);

  /* definition and creation of canSenderTask */
  osThreadDef(canSenderTask, StartCanSenderTask, osPriorityIdle, 0, 128);
  canSenderTaskHandle = osThreadCreate(osThread(canSenderTask), NULL);

  /* definition and creation of audioPwrMngTask */
  osThreadDef(audioPwrMngTask, StartAudioPwrMngTask, osPriorityIdle, 0, 128);
  audioPwrMngTaskHandle = osThreadCreate(osThread(audioPwrMngTask), NULL);

  /* definition and creation of idleTask */
  osThreadDef(idleTask, StartIdleTask, osPriorityIdle, 0, 128);
  idleTaskHandle = osThreadCreate(osThread(idleTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint32_t sysTick = 0;
  uint32_t eventTime = 0;
  can_event_t cdcStatusEvent = {0};
  cdcStatusEvent.data_id = GENERAL_STATUS_CDC;
  cdcStatusEvent.priority = 0;
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
    sysTick = osKernelSysTick();

    if (!can_offline && sysTick > eventTime)
    {
      eventTime = sysTick + CDC_STATUS_TX_BASETIME;
      cdcStatusEvent.data_ptr = cdcActive ? cdcActiveStatus : cdcInactiveStatus;
      cdcStatusEvent.data_len = sizeof(cdcActiveStatus);
      xQueueSendToBack(canEventQueueHandle, &cdcStatusEvent, 0);
#ifdef UART_LOGGING
      uart_log("[can] Sending cdc [%s] status", cdcActive ? "active" : "inactive");
#endif
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartIndicatorHandleTask */
/**
* @brief Function implementing the IndicatorHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIndicatorHandleTask */
void StartIndicatorHandleTask(void const * argument)
{
  /* USER CODE BEGIN StartIndicatorHandleTask */
  /* Infinite loop */
  uint16_t indEvent = 0;
  struct
  {
    uint8_t rx_delay;
    uint8_t tx_delay;
    uint8_t err_delay;
  } delays = {0};
  uint8_t *delays_val = (uint8_t *)&delays;

#define minEventLen 10
  const TickType_t xTicksToWait = pdMS_TO_TICKS(minEventLen);

  for (;;)
  {
    if (xQueueReceive(indicatorQueueHandle, &indEvent, xTicksToWait) != pdTRUE)
      indEvent = 0x00;

    // disable led's event
    for (uint8_t pin = 0; pin < sizeof(delays_val); pin++)
    {
      if (delays_val[pin] >= minEventLen)
      {
        delays_val[pin] -= minEventLen;

        if (delays_val[pin] < minEventLen)
        {
          delays_val[pin] = 0;

          if (pin == rx_pin && HAL_GPIO_ReadPin(LEDRX_GPIO_Port, LEDRX_Pin))
          {
            HAL_GPIO_WritePin(LEDRX_GPIO_Port, LEDRX_Pin, GPIO_PIN_RESET);
          }
          else if (pin == tx_pin && HAL_GPIO_ReadPin(LEDTX_GPIO_Port, LEDTX_Pin))
          {
            HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_PIN_RESET);
          }
          else if (pin == err_pin && HAL_GPIO_ReadPin(LEDERR_GPIO_Port, LEDERR_Pin))
          {
            HAL_GPIO_WritePin(LEDERR_GPIO_Port, LEDERR_Pin, GPIO_PIN_RESET);
          }
        }
      }
    }

    if (!indEvent)
      continue;

    uint8_t pin = indEvent & 0xFF;
    uint8_t delay = indEvent >> 8;

    // enable led's event
    switch (pin)
    {
    case rx_pin:
      // Save message timestamp
      canRXTimestamp = osKernelSysTick();
      HAL_GPIO_WritePin(LEDRX_GPIO_Port, LEDRX_Pin, GPIO_PIN_SET);
      delays.rx_delay += delay;
      break;
    case tx_pin:
      HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_PIN_SET);
      delays.tx_delay += delay;
      break;
    case err_pin:
      HAL_GPIO_WritePin(LEDERR_GPIO_Port, LEDERR_Pin, GPIO_PIN_SET);
      delays.err_delay += delay;
      break;
    default:
      break;
    }
  }
  /* USER CODE END StartIndicatorHandleTask */
}

/* USER CODE BEGIN Header_StartNodeStatusTask */
/**
* @brief Function implementing the nodeStatusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNodeStatusTask */
void StartNodeStatusTask(void const * argument)
{
  /* USER CODE BEGIN StartNodeStatusTask */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  uint8_t ReceivedValue;
  can_event_t statusCanEvent = {0};
  statusCanEvent.data_id = NODE_STATUS_TX_CDC;
  statusCanEvent.priority = 0;
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
    if (xQueueReceive(nodeStatusQueueHandle, &ReceivedValue, xTicksToWait) == pdPASS)
    {
#ifdef UART_LOGGING
      uart_log("[req] Received node status 0x%02X", ReceivedValue & 0x0F);
#endif
      switch (ReceivedValue & 0x0F)
      {
      case 0x3: // PowerOn
        statusCanEvent.data_ptr = (uint8_t *) cdcPoweronCmd;
        statusCanEvent.data_len = sizeof(cdcPoweronCmd);
        xQueueSendToBack(canEventQueueHandle, &statusCanEvent, 0);
#ifdef UART_LOGGING
        uart_log("[can] Sending node [%s] status", "power on");
#endif
        break;
      case 0x2: // Active
        statusCanEvent.data_ptr = (uint8_t *) cdcActiveCmd;
        statusCanEvent.data_len = sizeof(cdcActiveCmd);
        xQueueSendToBack(canEventQueueHandle, &statusCanEvent, 0);
#ifdef UART_LOGGING
        uart_log("[can] Sending node [%s] status", "active");
#endif
        break;
      case 0x8: // PowerOff
        statusCanEvent.data_ptr = (uint8_t *) cdcPowerdownCmd;
        statusCanEvent.data_len = sizeof(cdcPowerdownCmd);
        xQueueSendToBack(canEventQueueHandle, &statusCanEvent, 0);
#ifdef UART_LOGGING
        uart_log("[can] Sending node [%s] status", "power off");
#endif
        break;
      default:
        break;
      }
    }
  }
  /* USER CODE END StartNodeStatusTask */
}

/* USER CODE BEGIN Header_StartCdcCtlTask */
/**
* @brief Function implementing the cdcCtlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCdcCtlTask */
void StartCdcCtlTask(void const * argument)
{
  /* USER CODE BEGIN StartCdcCtlTask */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  uint8_t ReceivedValue;

  can_event_t statusCanEvent = {0};
  statusCanEvent.data_id = GENERAL_STATUS_CDC;
  statusCanEvent.priority = 0;
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
    if (xQueueReceive(cdcCtlQueueHandle, &ReceivedValue, xTicksToWait) == pdPASS)
    {
      switch (ReceivedValue)
      {
      case 0x24: // CDC = ON (CD/RDM button has been pressed twice)
        cdcActive = true;
        xSemaphoreGive(powerStateHandle);
        statusCanEvent.data_ptr = (uint8_t *) cdcActiveStatusEvent;
        statusCanEvent.data_len = sizeof(cdcActiveStatusEvent);
        xQueueSendToBack(canEventQueueHandle, &statusCanEvent, 0);
#ifdef UART_LOGGING
        uart_log("[ctl] Received cdc [%s] event", "ON");
#endif
        break;
      case 0x14: // CDC = OFF (Back to Radio or Tape mode)
        cdcActive = false;
        xSemaphoreGive(powerStateHandle);
        statusCanEvent.data_ptr = (uint8_t *) cdcInactiveStatusEvent;
        statusCanEvent.data_len = sizeof(cdcInactiveStatusEvent);
        xQueueSendToBack(canEventQueueHandle, &statusCanEvent, 0);
#ifdef UART_LOGGING
        uart_log("[ctl] Received cdc [%s] event", "OFF");
#endif
        break;
      case 0x35: // Seek >>
        NextTrack();
        break;
      case 0x36: // Seek <<
        PrevTrack();
        break;
      case 0x59: // Next btn
        break;
      case 0x45: // SEEK+ button long press on IHU
        break;
      case 0x46: // SEEK- button long press on IHU
        break;
      case 0x84: // SEEK button (middle) long press on IHU
        break;
      case 0x88: // > 2 second long press of SEEK button (middle) on IHU
        break;
      case 0x76: // Random ON/OFF (Long press of CD/RDM button)
        break;
      case 0x68: // Change CD param
        break;
      case 0xB0: // Audio mute off
        break;
      case 0xB1: // Audio mute on
        break;
      default:
        break;
      }
    }
  }
  /* USER CODE END StartCdcCtlTask */
}

/* USER CODE BEGIN Header_StartWheelBtnHandleTask */
/**
* @brief Function implementing the wheelBtnHandleT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWheelBtnHandleTask */
void StartWheelBtnHandleTask(void const * argument)
{
  /* USER CODE BEGIN StartWheelBtnHandleTask */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  uint8_t ReceivedValue;
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
    if (xQueueReceive(wheelBtnQueueHandle, &ReceivedValue, xTicksToWait) == pdPASS)
    {
      switch (ReceivedValue)
      {
      case NEXT: // NXT button on wheel
        PlayPause();
        break;
      case SEEK_NEXT: // Seek >> button on wheel
        // NextTrack();  // Reserved for long press and seek
        break;
      case SEEK_PREV: // Seek << button on wheel
        // PrevTrack();  // Reserved for long press and seek
        break;
      default:
        break;
      }
    }
  }
  /* USER CODE END StartWheelBtnHandleTask */
}

/* USER CODE BEGIN Header_StartSidBtnHandleTask */
/**
* @brief Function implementing the sidBtnHandleTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSidBtnHandleTask */
void StartSidBtnHandleTask(void const * argument)
{
  /* USER CODE BEGIN StartSidBtnHandleTask */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  uint8_t ReceivedValue;
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    if (xQueueReceive(sidBtnQueueHandle, &ReceivedValue, xTicksToWait) == pdPASS)
    {
      osDelay(10);
    }
  }
  /* USER CODE END StartSidBtnHandleTask */
}

/* USER CODE BEGIN Header_StartCanSenderTask */
/**
* @brief Function implementing the canSenderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanSenderTask */
void StartCanSenderTask(void const * argument)
{
  /* USER CODE BEGIN StartCanSenderTask */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(1000);
  can_event_t canEvent = {0};
  uint8_t send_buf[CAN_DATA_LEN];
  uint8_t frameCount;
  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
    if (xQueueReceive(canEventQueueHandle, &canEvent, xTicksToWait) == pdPASS)
    {
      if (canEvent.data_ptr != NULL && canEvent.data_len >= CAN_DATA_LEN)
      {
        frameCount = canEvent.data_len / CAN_DATA_LEN;

        for (uint8_t i = 1; i <= frameCount; i++)
        {
          memcpy(send_buf, canEvent.data_ptr + (( i - 1 ) * CAN_DATA_LEN), CAN_DATA_LEN);
          CAN_Send_Data(canEvent.data_id, send_buf);
          if (i < frameCount) // Skip delay after last frame
          {
            osDelay(NODE_STATUS_TX_INTERVAL);
          }
          else
          {
            osDelay(NODE_STATUS_TX_DELAY);
          }
        }
        memset(&canEvent, 0, sizeof(canEvent));
      }
    }
  }
  /* USER CODE END StartCanSenderTask */
}

/* USER CODE BEGIN Header_StartAudioPwrMngTask */
/**
* @brief Function implementing the audioPwrMngTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioPwrMngTask */
void StartAudioPwrMngTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioPwrMngTask */
  /* Infinite loop */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(500);
  bool lastStatus = false;

  for (;;)
  {
    osDelay(10);
    // Instant power switch
    if (xSemaphoreTake(powerStateHandle, xTicksToWait) == pdTRUE)
    {
      if (lastStatus != cdcActive && CONFIRMATION_SOUND)
      {
        Beep(SOUND_ACK);
      }
      audioPower(cdcActive);
      lastStatus = cdcActive;
#ifdef UART_LOGGING
      uart_log("[pwr] Switch audio [%s]", cdcActive ? "ON" : "OFF");
#endif
    }
    else // Auto switch every 0.5 sec
    {
      if (lastStatus != cdcActive && CONFIRMATION_SOUND)
      {
        Beep(SOUND_ACK);
      }
      audioPower(cdcActive);
      lastStatus = cdcActive;
#ifdef UART_LOGGING
      uart_log("[pwr] Auto switch audio [%s]", cdcActive ? "ON" : "OFF");
#endif
    }
  }
  /* USER CODE END StartAudioPwrMngTask */
}

/* USER CODE BEGIN Header_StartIdleTask */
/**
* @brief Function implementing the idleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIdleTask */
void StartIdleTask(void const * argument)
{
  /* USER CODE BEGIN StartIdleTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(0xFFFF);
  }
  /* USER CODE END StartIdleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void requestTextOnDisplay(uint8_t type)
{
  /* Format of NODE_DISPLAY_RESOURCE_REQ frame:
   ID: Node ID requesting to write on SID
   [0]: Request source
   [1]: SID object to write on; 0 = entire SID; 1 = 1st row; 2 = 2nd row
   [2]: Request type: 1 = Engineering test; 2 = Emergency; 3 = Driver action; 4 = ECU action; 5 = Static text; 0xFF = We don't want to write on SID
   [3]: Request source function ID
   [4-7]: Zeroed out; not in use
   */

  uint8_t displayRequestCmd[8];

  displayRequestCmd[0] = NODE_APL_ADR;
  displayRequestCmd[1] = DESIRED_ROW & (0x0F);
  displayRequestCmd[2] = type;
  displayRequestCmd[3] = NODE_SID_FUNCTION_ID;
  displayRequestCmd[4] = 0x00;
  displayRequestCmd[5] = 0x00;
  displayRequestCmd[6] = 0x00;
  displayRequestCmd[7] = 0x00;
  CAN_Send_Data(NODE_DISPLAY_RESOURCE_REQ, displayRequestCmd);
}

void writeTextOnDisplay(char message[15])
{
  if (cdcActive)
  {
    uint8_t defaultSIDtext[3][8] = {
        {0x42, 0x96, DESIRED_ROW, 'B', 'l', 'u', 'e', 't'},
        {0x01, 0x96, DESIRED_ROW, 'o', 'o', 't', 'h', 0x20},
        {0x00, 0x96, DESIRED_ROW, 0x20, 0x20, 0x00, 0x00, 0x00}
    };
    if (message != NULL)
    {
      for (uint8_t s = 0; s < 3; s++)
      {
        for (uint8_t r = 3; r < 8; r++)
        {
          if (message[5 * s + (r - 3)] != '\0')
          {
            defaultSIDtext[s][r] = message[5 * s + (r - 3)];
          }
          else
          {
            defaultSIDtext[s][r] = 0x20;
            break;
          }
        }
      }
    }
    for (uint8_t k = 0; k < 3; k++)
    {
      CAN_Send_Data(NODE_WRITE_TEXT_ON_DISPLAY, defaultSIDtext[k]);
      osDelay(10);
    }
  }
}

/* USER CODE END Application */

