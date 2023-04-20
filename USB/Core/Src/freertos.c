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
#include "usbd_cdc_if.h"
#include "i2c.h"
#include "rng.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	EEPROM_TEST_DATA_LEN	10
#define I2C_TIMEOUT_VALUE		10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define	get_os_random_delay_time(MAX_DELAY)	(HAL_RNG_GetRandomNumber(&hrng)%(MAX_DELAY))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Green_LED_TaskHandle;
osThreadId Blue_LED_TaskHandle;
osThreadId Red_LED_TaskHandle;
osThreadId Orange_LED_TaskHandle;
osThreadId printTaskHandle;
osThreadId eepromTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LEDTask_LD4(void const * argument);
void LEDTask_LD6(void const * argument);
void LEDTask_LD5(void const * argument);
void LEDTask_LD3(void const * argument);
void printTask_func(void const * argument);
void eepromTask_func(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Green_LED_Task */
  osThreadDef(Green_LED_Task, LEDTask_LD4, osPriorityNormal, 0, 128);
  Green_LED_TaskHandle = osThreadCreate(osThread(Green_LED_Task), NULL);

  /* definition and creation of Blue_LED_Task */
  osThreadDef(Blue_LED_Task, LEDTask_LD6, osPriorityNormal, 0, 128);
  Blue_LED_TaskHandle = osThreadCreate(osThread(Blue_LED_Task), NULL);

  /* definition and creation of Red_LED_Task */
  osThreadDef(Red_LED_Task, LEDTask_LD5, osPriorityNormal, 0, 128);
  Red_LED_TaskHandle = osThreadCreate(osThread(Red_LED_Task), NULL);

  /* definition and creation of Orange_LED_Task */
  osThreadDef(Orange_LED_Task, LEDTask_LD3, osPriorityNormal, 0, 128);
  Orange_LED_TaskHandle = osThreadCreate(osThread(Orange_LED_Task), NULL);

  /* definition and creation of printTask */
  osThreadDef(printTask, printTask_func, osPriorityNormal, 0, 128);
  printTaskHandle = osThreadCreate(osThread(printTask), NULL);

  /* definition and creation of eepromTask */
  osThreadDef(eepromTask, eepromTask_func, osPriorityNormal, 0, 128);
  eepromTaskHandle = osThreadCreate(osThread(eepromTask), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  if( b1_pushed == 1 )
	  {
		  if( osThreadIsSuspended(printTaskHandle) == osOK )
	  	  {
			  osThreadResume(printTaskHandle);
	  	  }

		  b1_pushed = 0;
	  }

	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LEDTask_LD4 */
/**
* @brief Function implementing the Green_LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask_LD4 */
void LEDTask_LD4(void const * argument)
{
  /* USER CODE BEGIN LEDTask_LD4 */
  GPIO_PinState	state;
  /* Infinite loop */
  for(state=GPIO_PIN_SET;;state=(state+1)%2)
  {
	  HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,state);
	  osDelay(get_os_random_delay_time(1400));
  }
  /* USER CODE END LEDTask_LD4 */
}

/* USER CODE BEGIN Header_LEDTask_LD6 */
/**
* @brief Function implementing the Blue_LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask_LD6 */
void LEDTask_LD6(void const * argument)
{
  /* USER CODE BEGIN LEDTask_LD6 */
  GPIO_PinState	state;
  /* Infinite loop */
  for(state=GPIO_PIN_SET;;state=(state+1)%2)
  {
	  HAL_GPIO_WritePin(LD6_GPIO_Port,LD6_Pin,state);
	  osDelay(get_os_random_delay_time(1600));
  }
  /* USER CODE END LEDTask_LD6 */
}

/* USER CODE BEGIN Header_LEDTask_LD5 */
/**
* @brief Function implementing the Red_LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask_LD5 */
void LEDTask_LD5(void const * argument)
{
  /* USER CODE BEGIN LEDTask_LD5 */
  GPIO_PinState	state;
  /* Infinite loop */
  for(state=GPIO_PIN_SET;;state=(state+1)%2)
  {
	  HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin,state);
	  osDelay(get_os_random_delay_time(1500));
  }
  /* USER CODE END LEDTask_LD5 */
}

/* USER CODE BEGIN Header_LEDTask_LD3 */
/**
* @brief Function implementing the Orange_LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTask_LD3 */
void LEDTask_LD3(void const * argument)
{
  /* USER CODE BEGIN LEDTask_LD3 */
  GPIO_PinState	state;
  /* Infinite loop */
  for(state=GPIO_PIN_SET;;state=(state+1)%2)
  {
	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,state);
	  osDelay(get_os_random_delay_time(1300));
  }
  /* USER CODE END LEDTask_LD3 */
}

/* USER CODE BEGIN Header_printTask_func */
/**
* @brief Function implementing the printTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_printTask_func */
void printTask_func(void const * argument)
{
  /* USER CODE BEGIN printTask_func */
  /* Infinite loop */
  osDelay(2000);

  for(;;)
  {
    osThreadSuspend(NULL);
    USB_Printf("[%02d:%02d:%02d:%02d.%03d]\n",
    		m_SysTimer.DAY,m_SysTimer.HOUR,m_SysTimer.MIN,m_SysTimer.SEC,m_SysTimer.MSEC);
  }
  /* USER CODE END printTask_func */
}

/* USER CODE BEGIN Header_eepromTask_func */
/**
* @brief Function implementing the eepromTask thread.
* @param argument: Not used
* @retval None
*/
static uint8_t	write_data[EEPROM_TEST_DATA_LEN];
static uint8_t	read_data[EEPROM_TEST_DATA_LEN];

/* USER CODE END Header_eepromTask_func */
void eepromTask_func(void const * argument)
{
  /* USER CODE BEGIN eepromTask_func */

  /* Infinite loop */
  for(int i=0;;i++)
  {
	memset(write_data,0xFF,EEPROM_TEST_DATA_LEN);
	memset(read_data,0x00,EEPROM_TEST_DATA_LEN);

	for(uint32_t ptr=0;ptr<EEPROM_TEST_DATA_LEN;ptr++)
	{
		write_data[ptr] = HAL_RNG_GetRandomNumber(&hrng)&0xFF;
	}

	if( HAL_I2C_Mem_Write(&hi2c1,0xA0,0x00,I2C_MEMADD_SIZE_8BIT,
			write_data,sizeof(write_data),I2C_TIMEOUT_VALUE) != HAL_OK )
	{
		USB_Printf("%s\n","There is an error while writing eeprom.");
	}

	osDelay(10);

	if( HAL_I2C_Mem_Read(&hi2c1,0xA0,0x00,I2C_MEMADD_SIZE_8BIT,
			read_data,sizeof(read_data),I2C_TIMEOUT_VALUE) != HAL_OK )
	{
		USB_Printf("%s\n","There is an error while reading eeprom.");
	}

	for(uint32_t ptr=0;ptr<EEPROM_TEST_DATA_LEN;ptr++)
	{
		USB_Printf("[%d]0x%02X 0x%02X\n",(int)ptr,
				(unsigned int)write_data[ptr],(unsigned int)read_data[ptr]);
	}

	if( memcmp(write_data,read_data,EEPROM_TEST_DATA_LEN) == 0 )
	{
		USB_Printf("[%05d]EEPROM is OK!!\n",i);
	}
	else
	{
		USB_Printf("[%05d]EEPROM is not OK!!\n",i);
	}

	osDelay(5000);
  }
  /* USER CODE END eepromTask_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
