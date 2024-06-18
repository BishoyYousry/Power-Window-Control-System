/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "Semphr.h"
#include "Queue.h"
#include "TM4C123GH6PM.h"

/*Macros*/
#define GPIO_PORTA_BASE 0
#define GPIO_PORTB_BASE 1
#define GPIO_PORTC_BASE 2
#define GPIO_PORTD_BASE 3
#define GPIO_PORTE_BASE 4
#define GPIO_PORTF_BASE 5

/*User Defined DataTypes*/
typedef struct
{
    uint8_t portNumber;
    uint8_t pinNumber;
    uint8_t edgeType;

}InterruptInfo_t;

/* The HW setup function */
static void prvSetupHardware( void );

void GPIO_DU_EdgeTriggeredInterruptInit(void);
void GPIO_DD_EdgeTriggeredInterruptInit(void);
void GPIO_PU_EdgeTriggeredInterruptInit(void);
void GPIO_PD_EdgeTriggeredInterruptInit(void);
void GPIO_Jamming_EdgeTriggeredInterruptInit(void);
void GPIO_Lock_EdgeTriggeredInterruptInit(void);
void GPIO_LimitU_EdgeTriggeredInterruptInit(void);
void GPIO_LimitD_EdgeTriggeredInterruptInit(void);

/*Shared Resource*/
InterruptInfo_t xInterruptInfo;

/* Tasks Handles */
TaskHandle_t xStateMachineHandle;
TaskHandle_t xDriverUpHandle;
TaskHandle_t xDriverDownHandle;
TaskHandle_t xDriverOffHandle;
TaskHandle_t xPassengerUpHandle;
TaskHandle_t xPassengerDownHandle;
TaskHandle_t xPassengerOffHandle;


/* Semaphore Handles */
SemaphoreHandle_t xPortASemaphore;
SemaphoreHandle_t xPortBSemaphore;

/* FreeRTOS tasks */
void vStateMachine(void *pvParameters);
void vDriverUp(void *pvParameters);
void vDriverDown(void *pvParameters);
void vDriverOff(void *pvParameters);
void vPassengerUp(void *pvParameters);
void vPassengerDown(void *pvParameters);
void vPassengerOff(void *pvParameters);


int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /* Create Semaphores here */
    xPortASemaphore = xSemaphoreCreateBinary();
    xPortBSemaphore = xSemaphoreCreateBinary();


    xTaskCreate(vStateMachine,
                "DriverUp",
                configMINIMAL_STACK_SIZE,
                (void*)NULL,
                1,
                &xDriverUpHandle
              );

    /* Create Tasks here */
    xTaskCreate(vDriverUp,
                "DriverUp",
                configMINIMAL_STACK_SIZE,
                (void*)NULL,
                1,
                &xDriverUpHandle
              );

    xTaskCreate(vDriverDown,
                 "DriverDown",
                 configMINIMAL_STACK_SIZE,
                 (void*)NULL,
                 1,
                 &xDriverDownHandle
               );

    xTaskCreate(vDriverOff,
                 "DriverOff",
                 configMINIMAL_STACK_SIZE,
                 (void*)NULL,
                 2,
                 &xDriverOffHandle
               );


    xTaskCreate(vPassengerUp,
                 "PassengerUp",
                 configMINIMAL_STACK_SIZE,
                 (void*)NULL,
                 1,
                 &xPassengerUpHandle
               );

    xTaskCreate(vPassengerDown,
                 "PassengerDown",
                 configMINIMAL_STACK_SIZE,
                 (void*)NULL,
                 1,
                 &xPassengerDownHandle
               );

    xTaskCreate( vPassengerOff,
                 "PassengerOff",
                 configMINIMAL_STACK_SIZE,
                 (void*)NULL,
                 2,
                 &xPassengerOffHandle
               );

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
    GPIO_DU_EdgeTriggeredInterruptInit();
    GPIO_DD_EdgeTriggeredInterruptInit();
    GPIO_PU_EdgeTriggeredInterruptInit();
    GPIO_PD_EdgeTriggeredInterruptInit();
    GPIO_Jamming_EdgeTriggeredInterruptInit();
    GPIO_Lock_EdgeTriggeredInterruptInit();
    GPIO_LimitU_EdgeTriggeredInterruptInit();
    GPIO_LimitD_EdgeTriggeredInterruptInit();
}



void vStateMachine(void *pvParameters)
{
    for(;;){

    }
}

void vDriverUp(void *pvParameters)
{
    for (;;) {

    }
}


void vDriverDown(void *pvParameters)
{
    for (;;) {

    }
}

void vDriverOff(void *pvParameters)
{
    for (;;) {

    }
}


void vPassengerUp(void *pvParameters)
{
    for (;;) {

    }
}

void vPassengerDown(void *pvParameters)
{
    for (;;) {

    }
}

void vPassengerOff(void *pvParameters)
{
    for (;;) {

    }
}



void GPIO_DU_EdgeTriggeredInterruptInit(void)
{
    GPIOA->IS   &= ~(1<<2);      /* PA2  detect edges */
    GPIOA->IBE  &= ~(1<<2);      /* PA2 will detect a certain edge */
    GPIOA->IEV  &= ~(1<<2);      /* PA2 will detect a falling edge */
    GPIOA->ICR  |= (1<<2);       /* Clear Trigger flag for PA2 (Interrupt Flag) */
    GPIOA->IM    |= (1<<2);       /* Enable Interrupt on PA2 pin */
    /* Set GPIO PORTA priority as 5 by setting bits 0, 1, and 2 with value 5 */
    NVIC->IP[0] = (NVIC->IP[0] & GPIO_PORTA_PRIORITY_MASK) | (GPIO_PORTA_INTERRUPT_PRIORITY << GPIO_PORTA_PRIORITY_BITS_POS);
    NVIC->ISER[0] |= 0x00000001;   /* Enable NVIC Interrupt for GPIO PORTA by setting bit 0 in EN0 Register */
}

void GPIO_DD_EdgeTriggeredInterruptInit(void)
{
    GPIOA->IS   &= ~(1<<3);      /* PA3  detect edges */
    GPIOA->IBE  &= ~(1<<3);      /* PA3 will detect a certain edge */
    GPIOA->IEV  &= ~(1<<3);      /* PA3 will detect a falling edge */
    GPIOA->ICR  |= (1<<3);       /* Clear Trigger flag for PA3 (Interrupt Flag) */
    GPIOA->IM    |= (1<<3);       /* Enable Interrupt on PA3 pin */
    /* Set GPIO PORTA priority as 5 by setting bits 0, 1, and 2 with value 5 */
    NVIC->IP[0] = (NVIC->IP[0] & GPIO_PORTA_PRIORITY_MASK) | (GPIO_PORTA_INTERRUPT_PRIORITY << GPIO_PORTA_PRIORITY_BITS_POS);
    NVIC->ISER[0] |= 0x00000001;   /* Enable NVIC Interrupt for GPIO PORTA by setting bit 0 in EN0 Register */
}

void GPIO_PU_EdgeTriggeredInterruptInit(void)
{
    GPIOA->IS   &= ~(1<<4);      /* PA4  detect edges */
    GPIOA->IBE  &= ~(1<<4);      /* PA4 will detect a certain edge */
    GPIOA->IEV  &= ~(1<<4);      /* PA4 will detect a falling edge */
    GPIOA->ICR  |= (1<<4);       /* Clear Trigger flag for PA4 (Interrupt Flag) */
    GPIOA->IM    |= (1<<4);       /* Enable Interrupt on PA4 pin */
    /* Set GPIO PORTA priority as 5 by setting bits 0, 1, and 2 with value 5 */
    NVIC->IP[0] = (NVIC->IP[0] & GPIO_PORTA_PRIORITY_MASK) | (GPIO_PORTA_INTERRUPT_PRIORITY << GPIO_PORTA_PRIORITY_BITS_POS);
    NVIC->ISER[0] |= 0x00000001;   /* Enable NVIC Interrupt for GPIO PORTA by setting bit 0 in EN0 Register */
}

void GPIO_PD_EdgeTriggeredInterruptInit(void)
{
    GPIOA->IS   &= ~(1<<5);      /* PA5  detect edges */
    GPIOA->IBE  &= ~(1<<5);      /* PA5 will detect a certain edge */
    GPIOA->IEV  &= ~(1<<5);      /* PA5 will detect a falling edge */
    GPIOA->ICR  |= (1<<5);       /* Clear Trigger flag for PA5 (Interrupt Flag) */
    GPIOA->IM    |= (1<<5);       /* Enable Interrupt on PA5 pin */
    /* Set GPIO PORTA priority as 5 by setting bits 0, 1, and 2 with value 5 */
    NVIC->IP[0] = (NVIC->IP[0] & GPIO_PORTA_PRIORITY_MASK) | (GPIO_PORTA_INTERRUPT_PRIORITY << GPIO_PORTA_PRIORITY_BITS_POS);
    NVIC->ISER[0] |= 0x00000001;   /* Enable NVIC Interrupt for GPIO PORTA by setting bit 0 in EN0 Register */
}

void GPIO_Jamming_EdgeTriggeredInterruptInit(void)
{
    GPIOA->IS   &= ~(1<<6);      /* PA6  detect edges */
    GPIOA->IBE  &= ~(1<<6);      /* PA6 will detect a certain edge */
    GPIOA->IEV  &= ~(1<<6);      /* PA6 will detect a falling edge */
    GPIOA->ICR  |= (1<<6);       /* Clear Trigger flag for PA6 (Interrupt Flag) */
    GPIOA->IM    |= (1<<6);       /* Enable Interrupt on PA6 pin */
    /* Set GPIO PORTA priority as 5 by setting bits 0, 1, and 2 with value 5 */
    NVIC->IP[0] = (NVIC->IP[0] & GPIO_PORTA_PRIORITY_MASK) | (GPIO_PORTA_INTERRUPT_PRIORITY << GPIO_PORTA_PRIORITY_BITS_POS);
    NVIC->ISER[0] |= 0x00000001;   /* Enable NVIC Interrupt for GPIO PORTA by setting bit 0 in EN0 Register */
}

void GPIO_Lock_EdgeTriggeredInterruptInit(void)
{
    GPIOA->IS   &= ~(1<<7);      /* PA7  detect edges */
    GPIOA->IBE  &= ~(1<<7);      /* PA7 will detect a certain edge */
    GPIOA->IEV  &= ~(1<<7);      /* PA7 will detect a falling edge */
    GPIOA->ICR  |= (1<<7);       /* Clear Trigger flag for PA7 (Interrupt Flag) */
    GPIOA->IM    |= (1<<7);       /* Enable Interrupt on PA7 pin */
    /* Set GPIO PORTA priority as 5 by setting bits 0, 1, and 2 with value 5 */
    NVIC->IP[0] = (NVIC->IP[0] & GPIO_PORTA_PRIORITY_MASK) | (GPIO_PORTA_INTERRUPT_PRIORITY << GPIO_PORTA_PRIORITY_BITS_POS);
    NVIC->ISER[0] |= 0x00000001;   /* Enable NVIC Interrupt for GPIO PORTA by setting bit 0 in EN0 Register */
}


void GPIO_LimitU_EdgeTriggeredInterruptInit(void)
{
    GPIOB->IS  &= ~(1<<2);      /* PB2 detect edges */
    GPIOB->IBE &= ~(1<<2);      /* PB2 will detect a certain edge */
    GPIOB->IEV &= ~(1<<2);      /* PB2 will detect a falling edge */
    GPIOB->ICR |= (1<<2);       /* Clear Trigger flag for PB2 (Interrupt Flag) */
    GPIOB->IM  |= (1<<2);       /* Enable Interrupt on PB2 pin */
    /* Set GPIO PORTB priority as 5 by setting bits 3, 4, and 5 with value 5 */
    NVIC->IP[0] = (NVIC->IP[0] & GPIO_PORTB_PRIORITY_MASK) | (GPIO_PORTB_INTERRUPT_PRIORITY << GPIO_PORTB_PRIORITY_BITS_POS);
    NVIC->ISER[0] |= 0x00000002;   /* Enable NVIC Interrupt for GPIO PORTB by setting bit 1 in EN0 Register */
}

void GPIO_LimitD_EdgeTriggeredInterruptInit(void)
{
    GPIOB->IS  &= ~(1<<3);      /* PB3 detect edges */
    GPIOB->IBE &= ~(1<<3);      /* PB3 will detect a certain edge */
    GPIOB->IEV &= ~(1<<3);      /* PB3 will detect a falling edge */
    GPIOB->ICR |= (1<<3);       /* Clear Trigger flag for PB3 (Interrupt Flag) */
    GPIOB->IM  |= (1<<3);       /* Enable Interrupt on PB3 pin */
    NVIC->IP[0] = (NVIC->IP[0] & GPIO_PORTB_PRIORITY_MASK) | (GPIO_PORTB_INTERRUPT_PRIORITY << GPIO_PORTB_PRIORITY_BITS_POS);
    NVIC->ISER[0] |= 0x00000002;   /* Enable NVIC Interrupt for GPIO PORTB by setting bit 1 in EN0 Register */
}


void GPIOPortA_Handler(void)
{
    xSemaphoreGive( xPortASemaphore );

    /* Clear Trigger flag for PA0 (Interrupt Flag) */
    GPIOA->ICR  |= (1<<0);
}

void GPIOPortB_Handler(void)
{
    /* Clear Trigger flag for PB0 (Interrupt Flag) */
    GPIOB->ICR  |= (1<<0);
}

void GPIOPortC_Handler(void)
{

}


void GPIOPortD_Handler(void)
{


}

void GPIOPortE_Handler(void)
{


}

void GPIOPortF_Handler(void)
{
     BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

     xInterruptInfo.portNumber = GPIO_PORTF_BASE;
     if(GPIOF->RIS & (1<<0))           /* PF0 handler code */
     {
         xInterruptInfo.pinNumber = 0;
         GPIOF->ICR |= (1<<0);       /* Clear Trigger flag for PF4 (Interrupt Flag) */
     }
     else if(GPIOF->RIS & (1<<4))      /* PF4 handler code */
     {
         xInterruptInfo.pinNumber = 4;
         GPIOF->ICR |= (1<<4);       /* Clear Trigger flag for PF4 (Interrupt Flag) */
     }
}
