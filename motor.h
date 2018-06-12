/*
 * motor.h
 *
 *  Created on: 2012/9/15
 *      Author: robert
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#define AXIS_NUM_MAX				3	//X,Y,Z
#define AXIS_NUM					2	//X,Y
//#define COUNT_PER_MM		1000	//pulse count per mm
//
typedef struct MOTOR
{
	float fPos; 	//in unit mm
	int nPos; 		//pulse count
	BYTE bCWLS;		//clockwise limit switch
	BYTE bCCWLS;	//counter clockwise limit switch
	BYTE bMoving;
	BYTE bAddr;		//RS485 address
	BYTE bConnected; //0:disconnected, > 0 connected
} MOTOR;

void Motor_Command(char *szCommand);
int Motor_GetStatus(int nAxis);
int Motor_GetAllStatus(void);
float Motor_GetPosition(int nAxis);
void Motor_DecipherStatus(char *szStatus);
void Motor_GetAllInfo(int nAxis, char *szBuffer);
int Motor_MoveAbsolute(int nAxis, float fPos);
int Motor_MoveRelative(int nAxis, float fPos);
int Motor_Jog(int nAxis, int nDirection, int nSpeed);
int Motor_Stop(int nAxis);
int Motor_ResetPos(int nAxis); //0:X, 1:Y

#endif /* MOTOR_H_ */
