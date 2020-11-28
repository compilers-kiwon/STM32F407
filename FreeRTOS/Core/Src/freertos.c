/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	MAX_STATE		2
#define	MAX_STR_SIZE	10
#define	NUM_OF_LD		4
#define	GPIO_LD			GPIOD

#define	get_prev_ld(current)	(((current)+(NUM_OF_LD)-1)%(NUM_OF_LD))
#define	get_next_ld(current)	(((current)+1)%(NUM_OF_LD))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const uint8_t	state[MAX_STATE][MAX_STR_SIZE] = {"OFF","ON"};
const uint16_t	LD_GPIO_Pin[NUM_OF_LD] = {GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15};
const uint32_t	LD_ON_time[NUM_OF_LD] = {300,500,700,1100};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t LD3TaskHandle;
const osThreadAttr_t LD3Task_attributes = {
  .name = "LD3Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t LD4TaskHandle;
const osThreadAttr_t LD4Task_attributes = {
  .name = "LD4Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t LDBlinkTaskHandle;
const osThreadAttr_t LDBlinkTask_attributes = {
  .name = "LDBlinkTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LD3Task(void* argument);
void LD4Task(void* argument);
void LDBlinkTask(void* argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  //LD3TaskHandle = osThreadNew(LD3Task, NULL, &LD3Task_attributes);
  //printf("LD3Task has been created\n");
  //LD4TaskHandle = osThreadNew(LD4Task, NULL, &LD4Task_attributes);
  //printf("LD4Task has been created\n");
  LDBlinkTaskHandle = osThreadNew(LDBlinkTask, NULL, &LDBlinkTask_attributes);
  printf("LDBlinkTask has been created\n");
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
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void LD3Task(void* argument)
{
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
	printf("LD3 %s\n",&state[HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)][0]);
    osDelay(500);
  }
}

void LD4Task(void* argument)
{
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
	printf("LD4 %s\n",&state[HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)][0]);
    osDelay(700);
  }
}

void LDBlinkTask(void* argument)
{
  int	current = 0;

  HAL_GPIO_TogglePin(GPIO_LD,LD_GPIO_Pin[current]);
  printf("STM32CubeIDE 1.5.0 %d\n",current);
  osDelay(LD_ON_time[current]);

  for(;;)
  {
	HAL_GPIO_TogglePin(GPIO_LD,LD_GPIO_Pin[current]);
	current=get_next_ld(current);
	HAL_GPIO_TogglePin(GPIO_LD,LD_GPIO_Pin[current]);
	printf("STM32CubeIDE 1.5.0 %d\n",current);
	osDelay(LD_ON_time[current]);
  }
  /* USER CODE END StartDefaultTask */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
