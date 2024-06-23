/*
 * Buttons.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Nirvana
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

/*Macros*/
#define GPIO_PORTA_BASE 0
#define GPIO_PORTB_BASE 1
#define GPIO_PORTC_BASE 2
#define GPIO_PORTD_BASE 3
#define GPIO_PORTE_BASE 4
#define GPIO_PORTF_BASE 5



#define DriverBottonUpPort 'D'
#define DriverBottonupPin   0

#define DriverBottonDownPort 'D'
#define DriverBottonDownPin    1

#define PassengerBottonUpPort 'D'
#define PassengerBottonupPin    2

#define PassengerBottonDownPort 'D'
#define PassengerBottonDownPin   3

#define JammingPort 'F'
#define JammingPin   1

#define LimitUpPort 'F'
#define LimitUpPin   2

#define LimitDownPort 'F'
#define LimitDownPin   3

#define  LockPort  'F'
#define LockPin   4


void  BottonsConfig();

unsigned char DriverUpPressed();
unsigned char DriverDownPressed();
unsigned char PassengerUpPressed();
unsigned char PassengerDownPressed();
unsigned char UpLimitSwitch();
unsigned char DownLimitSwitch();

unsigned char LockRead();

#endif /* BUTTONS_H_ */
