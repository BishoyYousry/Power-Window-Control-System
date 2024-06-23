/*
 * motor.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Nirvana
 */

#ifndef MOTOR_H_
#define MOTOR_H_



#define CLOCKWISE 0
#define ANTICLOCKWISE 1

void MotorConfig();
void MotorOff();
void MotorOn(unsigned char direction);



#endif /* MOTOR_H_ */
