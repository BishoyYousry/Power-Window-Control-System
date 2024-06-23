/*
 * Buttons.c
 *
 *  Created on: Jun 22, 2024
 *      Author: Nirvana
 */

#include "Buttons.h"
#include "MCAL\GPIO\gpio.h"

void OneButtonConfig(GPIOA_Type *GPIOx, uint16_t pin_no)
{
    gpio_configure_pin_unlock(GPIOx,pin_no);

    gpio_configure_pin_mode(GPIOx,pin_no,GPIO_PIN_DIGITAL);
    gpio_configure_pin_iotype(GPIOx,pin_no,GPIO_PIN_INPUT);
    gpio_configure_pin_pupd(GPIOx,pin_no,GPIO_PIN_PULL_UP);
    //gpio_set_alt_function(GPIOx,pin_no,GPIO_PIN_PULL_UP);
    gpio_enable_interrupt(GPIOx,pin_no);
    gpio_configure_interrupt(GPIOx,pin_no,GPIO_INT_FALLING_EDGE);
    gpio_clear_interrupt(GPIOx,pin_no);

}
void  BottonsConfig()
{
    RCC_GPIOD_CLK_ENABLE();
    RCC_GPIOF_CLK_ENABLE();
    OneButtonConfig(GPIOD,DriverBottonupPin);
    OneButtonConfig(GPIOD,DriverBottonDownPin);
    OneButtonConfig(GPIOD,PassengerBottonupPin);
    OneButtonConfig(GPIOD,PassengerBottonDownPin);
    NVIC_enable_interrupt(3); /*Enable NVIC for port D*/

    OneButtonConfig(GPIOF,JammingPin);
    OneButtonConfig(GPIOF,LimitUpPin);
    OneButtonConfig(GPIOF,LimitDownPin);

    /*config lock button */
    gpio_configure_pin_unlock(GPIOF,LockPin);

    gpio_configure_pin_mode(GPIOF,LockPin,GPIO_PIN_DIGITAL);
    gpio_configure_pin_iotype(GPIOF,LockPin,GPIO_PIN_INPUT);
    gpio_configure_pin_pupd(GPIOF,LockPin,GPIO_PIN_PULL_UP);
    //gpio_set_alt_function(GPIOx,pin_no,GPIO_PIN_PULL_UP);
    gpio_enable_interrupt(GPIOF,LockPin);
    gpio_configure_interrupt(GPIOF,LockPin,GPIO_INT_BOTH_EDGES);
    gpio_clear_interrupt(GPIOF,LockPin);

    /*enable NVIC to port F to enable lock interrupt*/
    NVIC_enable_interrupt(30); /*Enable NVIC for port D*/


}
unsigned char DriverUpPressed()
{
    return gpio_read_from_pin(GPIOD,DriverBottonupPin);
}
unsigned char JammingPressed()
{
    return gpio_read_from_pin(GPIOF,JammingPin);
}

unsigned char DriverDownPressed()
{
    return gpio_read_from_pin(GPIOD,DriverBottonDownPin);

}
unsigned char PassengerUpPressed()
{
    return gpio_read_from_pin(GPIOD,PassengerBottonupPin);

}
unsigned char PassengerDownPressed()
{
    return gpio_read_from_pin(GPIOD,PassengerBottonDownPin);
}
unsigned char UpLimitSwitch()
{
    return gpio_read_from_pin(GPIOF,LimitUpPin);

}
unsigned char DownLimitSwitch()
{
    return gpio_read_from_pin(GPIOF,LimitUpPin);

}

unsigned char LockRead()
{
    return gpio_read_from_pin(GPIOF,LockPin);

}
