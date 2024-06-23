/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "Semphr.h"
#include "Queue.h"
#include "TM4C123GH6PM.h"
#include "Buttons.h"
#include "motor.h"

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
TaskHandle_t xDriverUpHandle;
TaskHandle_t xDriverDownHandle;
TaskHandle_t xPassengerUpHandle;
TaskHandle_t xPassengerDownHandle;


/* Semaphore Handles */
SemaphoreHandle_t xPassengerUp;
SemaphoreHandle_t xPassengerDown;
SemaphoreHandle_t xDriverUp;
SemaphoreHandle_t xDriverDown;

/* Mutex Handle */
SemaphoreHandle_t xMotor;

/*Queue Handle*/
QueueHandle_t xLock_Queue;

/* FreeRTOS tasks */
void vDriverUp(void *pvParameters);
void vDriverDown(void *pvParameters);
void vPassengerUp(void *pvParameters);
void vPassengerDown(void *pvParameters);


int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /* Create Semaphores here */
    xPassengerUp = xSemaphoreCreateBinary();
    xPassengerDown = xSemaphoreCreateBinary();
    xDriverUp = xSemaphoreCreateBinary();
    xDriverDown = xSemaphoreCreateBinary();

    /*Create Mutex */
    xMotor = xSemaphoreCreateBinary();

    /*Create Queue*/
    xLock_Queue = xQueueCreate(1, sizeof(int));


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
    /*Enable Pins*/
    BottonsConfig();
    MotorConfig();
    /* Place here any needed HW initialization such as GPIO, UART, etc.
    GPIO_DU_EdgeTriggeredInterruptInit();
    GPIO_DD_EdgeTriggeredInterruptInit();
    GPIO_PU_EdgeTriggeredInterruptInit();
    GPIO_PD_EdgeTriggeredInterruptInit();
    GPIO_Jamming_EdgeTriggeredInterruptInit();
    GPIO_Lock_EdgeTriggeredInterruptInit();
    GPIO_LimitU_EdgeTriggeredInterruptInit();
    GPIO_LimitD_EdgeTriggeredInterruptInit();*/
}


void vDriverUp(void *pvParameters)
{
    xSemaphoreTake(xDriverUp,0);
    for (;;) {

        /*Take semaphore to start running*/
       if(xSemaphoreTake(xDriverUp,portMAX_DELAY)==pdTRUE)
       {
        vTaskDelay(500/ portTICK_PERIOD_MS);
        /*Manual mode*/
        if(DriverUpPressed())
        {
            if(xSemaphoreTake(xMotor,portMAX_DELAY)==pdTRUE)
            {
            MotorOn(CLOCKWISE);
            while(DriverUpPressed());
            MotorOff();
            xSemaphoreGive(xMotor);
            }
        }
        else
        {
            /*Automatic*/
            if(xSemaphoreTake(xMotor,portMAX_DELAY)==pdTRUE)
              {
                MotorOn(CLOCKWISE);
                while(!UpLimitSwitch());
                MotorOff();
                xSemaphoreGive(xMotor);
              }
        }
       }


    }
}


void vDriverDown(void *pvParameters)
{
    xSemaphoreTake(xDriverDown,0);
    for (;;) {

        /*Take semaphore to start running*/
       if(xSemaphoreTake(xDriverDown,portMAX_DELAY)==pdTRUE)
       {
           vTaskDelay(500/ portTICK_PERIOD_MS);
        /*Manual mode*/
        if(DriverDownPressed())
        {
            if(xSemaphoreTake(xMotor,portMAX_DELAY)==pdTRUE)
            {
            MotorOn(ANTICLOCKWISE);
            while(DriverDownPressed());
            MotorOff();
            xSemaphoreGive(xMotor);
            }
        }
        else
        {
            /*Automatic*/
            if(xSemaphoreTake(xMotor,portMAX_DELAY)==pdTRUE)
              {
                MotorOn(CLOCKWISE);
                while(!UpLimitSwitch());
                MotorOff();
                xSemaphoreGive(xMotor);
              }
        }
       }


    }
}



void vPassengerUp(void *pvParameters)
{
    xSemaphoreTake(xPassengerUp,0);
    for (;;) {
        /*if lock is on Do nothing */
        while(LockRead()==1);

        /*Take semaphore to start running*/
       if(xSemaphoreTake(xPassengerUp,portMAX_DELAY)==pdTRUE)
       {
           vTaskDelay(500/ portTICK_PERIOD_MS);
        /*Manual mode*/
        if(PassengerUpPressed())
        {
            if(xSemaphoreTake(xMotor,portMAX_DELAY)==pdTRUE)
            {
            MotorOn(ANTICLOCKWISE);
            while(PassengerUpPressed());
            MotorOff();
            xSemaphoreGive(xMotor);
            }
        }
        else
        {
            /*Automatic*/
            if(xSemaphoreTake(xMotor,portMAX_DELAY)==pdTRUE)
              {
                MotorOn(CLOCKWISE);
                while(!UpLimitSwitch());
                MotorOff();
                xSemaphoreGive(xMotor);
              }
        }
       }


    }
}


void vPassengerDown(void *pvParameters)
{
    xSemaphoreTake(xPassengerDown,0);
    for (;;) {
        /*if lock is on Do nothing */
        while(LockRead()==1);

        /*Take semaphore to start running*/
       if(xSemaphoreTake(xPassengerDown,portMAX_DELAY)==pdTRUE)
       {
           vTaskDelay(500/ portTICK_PERIOD_MS);
        /*Manual mode*/
        if(PassengerDownPressed())
        {
            if(xSemaphoreTake(xMotor,portMAX_DELAY)==pdTRUE)
            {
            MotorOn(ANTICLOCKWISE);
            while(PassengerDownPressed());
            MotorOff();
            xSemaphoreGive(xMotor);
            }
        }
        else
        {
            /*Automatic*/
            if(xSemaphoreTake(xMotor,portMAX_DELAY)==pdTRUE)
              {
                MotorOn(CLOCKWISE);
                while(!UpLimitSwitch());
                MotorOff();
                xSemaphoreGive(xMotor);
              }
        }
       }


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

void GPIOPortA_Handler(void){}
void GPIOPortB_Handler(void){}
void GPIOPortC_Handler(void){}
void GPIOPortE_Handler(void){}
void GPIOPortF_Handler(void){}

void GPIOPortD_Handler(void)
{
    BaseType_t flag = pdFALSE;
   if(DriverUpPressed())
   {
       GPIOD->ICR |= (1<<DriverBottonupPin);       /* Clear Trigger flag for the pin (Interrupt Flag) */
       xSemaphoreGiveFromISR(xDriverUp,&flag);
   }

   if(DriverDownPressed())
      {
          GPIOD->ICR |= (1<<DriverBottonDownPin);       /* Clear Trigger flag for the pin (Interrupt Flag) */
          xSemaphoreGiveFromISR(xDriverDown,&flag);
      }
   if(PassengerUpPressed())
      {
          GPIOD->ICR |= (1<<PassengerBottonupPin);       /* Clear Trigger flag for the pin (Interrupt Flag) */
          xSemaphoreGiveFromISR(xPassengerUp,&flag);
      }

  if(PassengerDownPressed())
     {
         GPIOD->ICR |= (1<<PassengerBottonDownPin);       /* Clear Trigger flag for the pin (Interrupt Flag) */
         xSemaphoreGiveFromISR(xPassengerDown,&flag);
     }

}




