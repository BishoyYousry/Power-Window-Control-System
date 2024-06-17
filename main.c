/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"
#include "tm4c123gh6pm_registers.h"

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 369

void Delay_MS(unsigned long long n)
{
    volatile uint64 count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}

/* The HW setup function */
static void prvSetupHardware( void );

/* FreeRTOS tasks */
void vRedLedTask(void *pvParameters);

int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /* Create Tasks here */
    xTaskCreate(vRedLedTask, "Red Task", 256, NULL, 1, NULL);

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
    GPIO_BuiltinButtonsLedsInit();
    GPIO_SW1EdgeTriggeredInterruptInit();
}

void vRedLedTask(void *pvParameters)
{
    for (;;) {
        taskENTER_CRITICAL(); /* Disable all Interrupts */

        GPIO_RedLedOn();

        /* Trigger SW1/PF4 Edges triggered interrupt within this delay time.
         * PF4 handler will not executed until interrupts is enabled again */
        Delay_MS(10000);

        GPIO_RedLedOff();

        Delay_MS(2000);

        taskEXIT_CRITICAL(); /* Enable all Interrupts */
    }
}

void GPIOPortF_Handler(void)
{
    GPIO_GreenLedOn();

    Delay_MS(2000);

    GPIO_GreenLedOff();

    Delay_MS(2000);

    /* Clear Trigger flag for PF4 (Interrupt Flag) */
    GPIO_PORTF_ICR_REG   |= (1<<4);
}
