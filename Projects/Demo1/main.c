/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
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
#include "stm32f429i_discovery.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include <stdio.h>
#include <stdint.h>




GPIO_InitTypeDef  GPIO_InitStructure;

xQueueHandle MsgQueue;


void Hardware_Init(void);
void Red_LED_On(void);
void Red_LED_Off(void);
void Green_LED_On(void);
void Green_LED_Off(void);
void Sender_Task(void*);
void Receiver_Task(void*);
void Monitor_Task(void*);


uint32_t  SendNumPlus=0;
uint32_t  RecNumPlus=0;



int main(void)
{
       Hardware_Init();
 
        vTraceEnable(TRC_INIT);
        vTraceEnable(TRC_START);


       MsgQueue=xQueueCreate(500,sizeof(uint32_t));

       xTaskCreate(Sender_Task,
                  "Sender_Task",
                   configMINIMAL_STACK_SIZE,
                   (void*) NULL,
                   tskIDLE_PRIORITY + 4UL,
                   NULL
       );
       xTaskCreate(Receiver_Task,
                  "Receiver_Task",
                   configMINIMAL_STACK_SIZE,
                   (void*) NULL,
                   tskIDLE_PRIORITY + 3UL,
                   NULL
       );
       xTaskCreate(Monitor_Task,
                  "Monitor_Task",
                   configMINIMAL_STACK_SIZE,
                   (void*) NULL,
                   tskIDLE_PRIORITY + 2UL,
                   NULL
       );

	
	vTaskStartScheduler();

	
	for( ;; );

}


/**
 * Hardware_Init: 
 */
void Hardware_Init(void)
{
        /* GPIOG Periph clock enable */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

        /* Configure PG13, PG14 in output pushpull mode */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOG, &GPIO_InitStructure);

}
/**
 * Red_LED_On: 
 */
void Red_LED_On(void)
{
//    GPIO_SetBits(GPIOG, GPIO_Pin_14);
    GPIOG->ODR |= 0x4000;
}

/**
 * Red_LED_Off: 
 */
void Red_LED_Off(void)
{
//    GPIO_ResetBits(GPIOG, GPIO_Pin_14);
  GPIOG->ODR &= 0xBFFF;
}

/**
 * Green_LED_On: 
 */
void Green_LED_On(void)
{
//    GPIO_SetBits(GPIOG, GPIO_Pin_13);
    GPIOG->ODR |= 0x2000;
}

/**
 * Green_LED_Off: 
 */
void Green_LED_Off(void)
{
//    GPIO_ResetBits(GPIOG, GPIO_Pin_13);
    GPIOG->ODR &= 0xDFFF;
}



void Sender_Task(void *pvParameters)
{uint32_t  SendNum=1;
 while(1)
 {
  xQueueSend(MsgQueue,(void*)&SendNum,0);
  SendNumPlus=SendNumPlus+SendNum;
  SendNum++;
     if(SendNum>10000) 
            SendNum=1;
  vTaskDelay(2);
 }
}


void Receiver_Task(void *pvParameters)
{uint32_t  ReceiveNum=0;
 for(;;)
  {while(xQueueReceive(MsgQueue,&ReceiveNum,100/portTICK_RATE_MS)==pdPASS)
      {RecNumPlus=RecNumPlus+ReceiveNum;
      }
    vTaskDelay(1000);
   }  
}

void Monitor_Task(void *pvParameters)
{
 
 while(1)
 {
  if((RecNumPlus>0)&&(RecNumPlus==SendNumPlus)) 
    { Red_LED_On();
      Green_LED_Off();
     vTaskDelay(10000);
     Red_LED_Off();
     Green_LED_On();
    }
  else 
     {Green_LED_On(); 
      vTaskDelay(10000);
      Green_LED_Off();
     }
  }
}






void vApplicationTickHook( void )
{
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
