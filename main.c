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

#define LOCKED 0
#define UNLOCKED 1

unsigned char LockStatus = UNLOCKED;

/* The HW setup function */
static void prvSetupHardware( void );


/*Shared Resource*/
InterruptInfo_t xInterruptInfo;

/* Tasks Handles */
TaskHandle_t xDriverUpHandle;
TaskHandle_t xDriverDownHandle;
TaskHandle_t xPassengerUpHandle;
TaskHandle_t xPassengerDownHandle;
TaskHandle_t xUpdateLockStatusHandle;



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
void UpdateLockFunc(void *pvParameters);

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
    xMotor = xSemaphoreCreateMutex();

    /*Create Queue*/
    xLock_Queue = xQueueCreate(1, sizeof(int));

    /* Create Tasks here */
    xTaskCreate(vDriverUp,
                "DriverUp",
                configMINIMAL_STACK_SIZE,
                (void*)NULL,
                2,
                &xDriverUpHandle
              );

    xTaskCreate(vDriverDown,
                 "DriverDown",
                 configMINIMAL_STACK_SIZE,
                 (void*)NULL,
                 2,
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

    xTaskCreate(UpdateLockFunc,
                 "UpdateLock",
                 configMINIMAL_STACK_SIZE,
                 (void*)NULL,
                 3,
                 &xUpdateLockStatusHandle
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
            while(DriverUpPressed())
            {
                if(JammingPressed())
                {
                    MotorOn(ANTICLOCKWISE);
                    vTaskDelay(500/ portTICK_PERIOD_MS);
                    break;
                }
            }
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
                while(!UpLimitSwitch())
                {
                    if(JammingPressed())
                    {
                        MotorOn(ANTICLOCKWISE);
                        vTaskDelay(500/ portTICK_PERIOD_MS);
                        break;
                    }
                }
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
        while(LockStatus==LOCKED);

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
            while(PassengerUpPressed())
            {
                if(JammingPressed())
                {
                    MotorOn(ANTICLOCKWISE);
                    vTaskDelay(500/ portTICK_PERIOD_MS);
                    break;
                }
            }
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
                while(!UpLimitSwitch())
                {
                    if(JammingPressed())
                    {
                        MotorOn(ANTICLOCKWISE);
                        vTaskDelay(500/ portTICK_PERIOD_MS);
                        break;
                    }
                }
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
        while(LockStatus==LOCKED);

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


void UpdateLockFunc(void *pvParameters)
{
    unsigned char changeLock ;
    xQueueReceive(xLock_Queue,&changeLock,0);

    for(;;)
    {
        if( xQueueReceive(xLock_Queue,&changeLock,portMAX_DELAY))
        {
           LockStatus = changeLock;
        }
    }
}




void GPIOPortA_Handler(void){}
void GPIOPortB_Handler(void){}
void GPIOPortC_Handler(void){}
void GPIOPortE_Handler(void){}
void GPIOPortF_Handler(void)
{
   int LockValue = LockRead();
   BaseType_t flag = pdFALSE;
   xQueueSendToFrontFromISR(xLock_Queue,&LockValue,&flag);
   GPIOF->ICR |= (1<<LockPin);       /* Clear Trigger flag for the pin (Interrupt Flag) */
   portEND_SWITCHING_ISR(flag);

}

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




