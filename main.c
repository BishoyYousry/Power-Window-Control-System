/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* MCAL includes. */
#include "uart0.h"

/* The HW setup function */
static void prvSetupHardware( void );

/* FreeRTOS tasks */
void vTask1(void *pvParameters);
void vTask2(void *pvParameters);

/* Used to hold the handle of Tasks */
TaskHandle_t xTask2Handle;

int main()
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /* Create Tasks here */
	xTaskCreate(vTask1, "Task 1", 256, NULL, 1,  NULL);
	xTaskCreate(vTask2, "Task 2", 256, NULL, 2,  &xTask2Handle);

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */ 
	vTaskStartScheduler();
	
	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for (;;);
	
}


static void prvSetupHardware( void )
{
	/* Place here any needed HW initialization such as GPIO, UART, etc.  */
    UART0_Init();
}

void vTask1(void *pvParameters)
{
    for (;;)
    {
        UART0_SendString("Task1 is running\r\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
        UART0_SendString("Task1 Creating Task2\r\n");
        xTaskCreate(vTask2, "Task 2", 256, NULL, 2,  &xTask2Handle);

    }
}

void vTask2(void *pvParameters)
{
    for (;;)
    {
        UART0_SendString("Task2 is running\r\n");
        vTaskDelete(NULL);
    }
}


/*-----------------------------------------------------------*/
