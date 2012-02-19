/**
  ******************************************************************************
  * @file    STM32F4-Discovery FreeRTOS demo\main.c
  * @author  T.O.M.A.S. Team
  * @version V1.1.0
  * @date    14-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "hw_config.h"

/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DELAY 125     /* msec */
#define queueSIZE	6

/* Private macro -------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Task functions declarations */
static void vLEDTask( void *pvParameters );
static void vSWITCHTask( void *pvParameters );
static void vMEMSTask(void *pvParameters);
static void vBALANCETask(void *pvParameters);

/* handlers to tasks to better control them */
xTaskHandle xLED_Tasks[4];
xTaskHandle xMEMS_Task, xBALANCE_Task;

/* variables used by tasks */
volatile int32_t ITM_RxBuffer;
/* initial arguments for vLEDTask task (which LED and what is the delay) */
static const int LEDS[4][2] = {{LED3,DELAY*1},
							   {LED4,DELAY*2},
							   {LED5,DELAY*3},
							   {LED6,DELAY*4}};

/* semaphores, queues declarations */
xSemaphoreHandle xSemaphoreSW  = NULL;
xQueueHandle xQueue;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{ 
	/* create a pipe for MEMS->TIM4 data exchange */
	xQueue=xQueueCreate(1,queueSIZE*sizeof(uint8_t));

	/* create semaphores... */
	vSemaphoreCreateBinary( xSemaphoreSW );

	/* ...and clean them up */
	if(xSemaphoreTake(xSemaphoreSW, ( portTickType ) 0) == pdTRUE);

	/* initialize hardware... */
	prvSetupHardware();

	/* Start the tasks defined within this file/specific to this demo. */
	xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED3", configMINIMAL_STACK_SIZE, (void *)LEDS[0],tskIDLE_PRIORITY, &xLED_Tasks[0] );
	xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED4", configMINIMAL_STACK_SIZE, (void *)LEDS[1],tskIDLE_PRIORITY, &xLED_Tasks[1] );
	xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED5", configMINIMAL_STACK_SIZE, (void *)LEDS[2],tskIDLE_PRIORITY, &xLED_Tasks[2] );
	xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED6", configMINIMAL_STACK_SIZE, (void *)LEDS[3],tskIDLE_PRIORITY, &xLED_Tasks[3] );
	xTaskCreate( vSWITCHTask, ( signed portCHAR * ) "SWITCH", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL );
	xTaskCreate( vMEMSTask, ( signed portCHAR * ) "MEMS", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, &xMEMS_Task );
	xTaskCreate( vBALANCETask, ( signed portCHAR * ) "BALANCE", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, &xBALANCE_Task );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;  
}

/*-----------------------------------------------------------*/

void vMEMSTask(void *pvParameters)
{
	/* queue for MEMS data length */
	uint8_t xBuffer_send[queueSIZE];

	for(;;)
	{
		LIS302DL_Read(xBuffer_send, LIS302DL_OUT_X_ADDR, queueSIZE);
		xQueueSendToBack(xQueue,xBuffer_send ,0);
	    vTaskDelay(DELAY/portTICK_RATE_MS);
	}
}

/*-----------------------------------------------------------*/

void vBALANCETask(void *pvParameters)
{
	uint8_t temp1, temp2 = 0;
	__IO uint8_t TempAcceleration = 0;
	uint8_t xBuffer_receive[queueSIZE];
	for( ;; )
	{
	 if(xQueueReceive(xQueue,xBuffer_receive,0)==pdPASS)
		{
		/* Disable All TIM4 Capture Compare Channels */
		TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);

		/* Update autoreload and capture compare registers value*/
		temp1=((int8_t)(xBuffer_receive[0])<0)?(int8_t)(xBuffer_receive[0])*(-1):(int8_t)(xBuffer_receive[0]); //ABS
		temp2=((int8_t)(xBuffer_receive[2])<0)?(int8_t)(xBuffer_receive[2])*(-1):(int8_t)(xBuffer_receive[2]); //ABS
		TempAcceleration = (temp1<temp2)?temp2:temp1; //MAX(temp1,temp2)

		if(TempAcceleration != 0)
		{
			if ((int8_t)xBuffer_receive[0] < -2)
			{
				/* Enable TIM4 Capture Compare Channel 4 */
				TIM_CCxCmd(TIM4, TIM_Channel_4, ENABLE);
				/* Sets the TIM4 Capture Compare4 Register value */
				TIM_SetCompare4(TIM4, TIM_CCR/TempAcceleration);
			}
			if ((int8_t)xBuffer_receive[0] > 2)
			{
				/* Enable TIM4 Capture Compare Channel 2 */
				TIM_CCxCmd(TIM4, TIM_Channel_2, ENABLE);
				/* Sets the TIM4 Capture Compare2 Register value */
				TIM_SetCompare2(TIM4, TIM_CCR/TempAcceleration);
			}
			if ((int8_t)xBuffer_receive[2] > 2)
			{
				/* Enable TIM4 Capture Compare Channel 1 */
				TIM_CCxCmd(TIM4, TIM_Channel_1, ENABLE);
				/* Sets the TIM4 Capture Compare1 Register value */
				TIM_SetCompare1(TIM4, TIM_CCR/TempAcceleration);
			}
			if ((int8_t)xBuffer_receive[2] < -2)
			{
				/* Enable TIM4 Capture Compare Channel 3 */
				TIM_CCxCmd(TIM4, TIM_Channel_3, ENABLE);
				/* Sets the TIM4 Capture Compare3 Register value */
				TIM_SetCompare3(TIM4, TIM_CCR/TempAcceleration);
			}

			/* Time base configuration */
			TIM_SetAutoreload(TIM4,  TIM_ARR/TempAcceleration);
		}
	 }
	taskYIELD(); 	//task is going to ready state to allow next one to run
	}
}

/*-----------------------------------------------------------*/

void vLEDTask( void *pvParameters )
{
    volatile int *LED;
    LED = (int *) pvParameters;

	for( ;; )
	{
		STM_EVAL_LEDToggle((Led_TypeDef)LED[0]);
	    vTaskDelay(LED[1]/portTICK_RATE_MS);
	}
}

/*-----------------------------------------------------------*/

void vSWITCHTask( void *pvParameters )
{
	static int i=0;
	for( ;; )
	{
		if(xSemaphoreTake(xSemaphoreSW,( portTickType ) 0) == pdTRUE)
		{
			i^=1;		//just switch the state if semaphore was given

			if(i==0)	//LED3..LD6 tasks ready, BALANCE, MEMS suspended
			{
				vTaskSuspend(xBALANCE_Task);
				TIM_Cmd(TIM4, DISABLE);
				vTaskSuspend(xMEMS_Task);
				prvLED_Config(GPIO);
				vTaskResume(xLED_Tasks[0]);
				vTaskResume(xLED_Tasks[1]);
				vTaskResume(xLED_Tasks[2]);
				vTaskResume(xLED_Tasks[3]);
			}
			else		//MEMS and BALANCE ready, LED tasks suspended
			{
				vTaskSuspend(xLED_Tasks[0]);
				vTaskSuspend(xLED_Tasks[1]);
				vTaskSuspend(xLED_Tasks[2]);
				vTaskSuspend(xLED_Tasks[3]);
				prvLED_Config(TIMER);
				TIM_Cmd(TIM4, ENABLE);
				vTaskResume(xBALANCE_Task);
				vTaskResume(xMEMS_Task);
			}
		}
		taskYIELD(); 	//task is going to ready state to allow next one to run
	}
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amout of FreeRTOS heap that 
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}
/*-----------------------------------------------------------*/

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
