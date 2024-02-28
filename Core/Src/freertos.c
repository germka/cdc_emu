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
#include "gpio.h"
#include <string.h>
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId IndicatorHandleHandle;
osMessageQId indicatorQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartIndicatorHandleTask(void const * argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of IndicatorHandle */
  osThreadDef(IndicatorHandle, StartIndicatorHandleTask, osPriorityIdle, 0, 128);
  IndicatorHandleHandle = osThreadCreate(osThread(IndicatorHandle), NULL);

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
	  uint8_t cdcActiveStatus[]   = {0xE0,0xFF,0x3F,0x41,0xFF,0xFF,0xFF,0xD0};
	  uint8_t cdcInactiveStatus[] = {0xE0,0xFF,0x3F,0x00,0x00,0x00,0x00,0xD0};
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    uint32_t sysTick = osKernelSysTick();

    if (sysTick % 950 == 0)
    {
    	CAN_Send_Data(CDC_CONTROL, cdcActiveStatus);
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
	struct {
		uint8_t rx_delay;
		uint8_t tx_delay;
		uint8_t err_delay;
	} delays = {0};
  uint8_t* delays_val = (uint8_t*) &delays;

	#define minEventLen 10

  for(;;)
  {
    if (xQueueReceive(indicatorQueueHandle, &indEvent, minEventLen) != pdTRUE)
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

