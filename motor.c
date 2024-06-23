/*
 * motor.c
 *
 *  Created on: Jun 22, 2024
 *      Author: Nirvana
 */
#include "MCAL\GPIO\gpio.h"
#include "motor.h"

void MotorConfig()
{
    RCC_GPIOE_CLK_ENABLE()
    gpio_configure_pin_unlock(GPIOE,4);
    gpio_configure_pin_unlock(GPIOE,5);


    gpio_configure_pin_mode(GPIOE,4,GPIO_PIN_DIGITAL);
    gpio_configure_pin_iotype(GPIOE,4,GPIO_PIN_OUTPUT);
    gpio_configure_pin_pupd(GPIOE,4,GPIO_PIN_PULL_UP);
    //gpio_set_alt_function(GPIOE,4,GPIO_PIN_PULL_UP);


    gpio_configure_pin_mode(GPIOE,5,GPIO_PIN_DIGITAL);
    gpio_configure_pin_iotype(GPIOE,5,GPIO_PIN_OUTPUT);
    gpio_configure_pin_pupd(GPIOE,5,GPIO_PIN_PULL_UP);
   // gpio_set_alt_function(GPIOE,5,GPIO_PIN_PULL_UP);
}
void MotorOff()
{
    gpio_write_to_pin(GPIOE,4,0);
    gpio_write_to_pin(GPIOE,5,0);
}
void MotorOn(unsigned char direction)
{
    if(direction==CLOCKWISE)
    {
        gpio_write_to_pin(GPIOE,4,0);
        gpio_write_to_pin(GPIOE,5,1);
    }
    else
    {
        gpio_write_to_pin(GPIOE,4,1);
        gpio_write_to_pin(GPIOE,5,0);
    }

}



