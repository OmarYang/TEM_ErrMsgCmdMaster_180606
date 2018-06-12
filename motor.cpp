/*
 * motor.cpp
 *
 *  Created on: 2012/9/15
 *      Author: robert
 */

#include <predef.h>
#include <stdio.h>
#include <stdlib.h>			//atoi()
#include <string.h>
#include <serial.h>
#include <basictypes.h>
#include <startnet.h>		//TICKS_PER_SECOND, writeall
#include <pins.h>
#include "cpu_cmd.h"
#include "io.h"
#include "main.h"
#include "data_struct.h"
#include "motor.h"
#include "function.h"
//
BYTE bWaitMotorResponse[AXIS_NUM_MAX] = {0, 0, 0};
MOTOR Motor[AXIS_NUM_MAX]; //XYZ
extern int fdSerial2;
extern NV_SETTINGS NV_Settings;
BYTE bPrintResponse = 0;
//
void Motor_Init(void)
{
	RS485_WriteString(2, "");
	return;
}
//MC: command
void Motor_Command(char *szCommand)
{
	int nLen;
	bPrintResponse = 1;
	nLen = strlen(szCommand);
	if (szCommand[nLen - 1] != '\n')
		strcat(szCommand, "\n");
	RS485_WriteString(2, szCommand);
	return;
}
//-----------------------------------------------------------
//1AR, all information report
//1_1111111_123456
//1PR, position report
//1_123456
//1SM, status report
//1_10  busy=1,procedure executing=0
//1SR, status report
//1_1111111		CW,CCW,HOME,SUBH,TIMING,OVH,MOVING
//nAxis = 0(X),1(Y),2(Z)
//-----------------------------------------------------------
int Motor_GetStatus(int nAxis)
{
	char szBuffer[32];
	int nAddr;
	nAddr = nAxis + MOTOR_RS485_ADDR;
	sprintf(szBuffer, "%dAR\n", nAddr);
	RS485_WriteString(2, szBuffer);
	if (bWaitMotorResponse[nAxis] < 3)
		bWaitMotorResponse[nAxis]++;
	return 0;
}
//
int Motor_GetAllStatus(void)
{
	static int nAxis = 0;
	//
	Motor_GetStatus(nAxis);
	nAxis = (nAxis + 1) % AXIS_NUM;
	return 0;
}
// return in unit mm
float Motor_GetPosition(int nAxis)
{
	char szBuffer[32];
	int nAddr;
	nAddr = nAxis + MOTOR_RS485_ADDR;
	sprintf(szBuffer, "%dPR\n", nAddr);
	RS485_WriteString(2, szBuffer);
	if (bWaitMotorResponse[nAxis] < 3)
		bWaitMotorResponse[nAxis]++;
	return Motor[nAxis].fPos;
}
//
void Motor_DecipherStatus(char *szStatus)
{
	BYTE bAddr;
	BYTE bV;
	char *p;
	//
	//if (bPrintResponse == 1) {
	//	bPrintResponse = 0;
	//	printf("%s\n", szStatus);
	//}
	bAddr = szStatus[0] - '0' - MOTOR_RS485_ADDR;
	//change bAddr=4,5,6 to 0,1,2
	if (bAddr >= AXIS_NUM) return; //invalid axis number
	bWaitMotorResponse[bAddr] = 0;
	//aAR_11111111_123456
	if (strncmp(&szStatus[1], "AR", 2) == 0) {
		p = strchr(szStatus, '_');
		if (p == NULL) return;
		p++;
		Motor[bAddr].bAddr = bAddr;
		Motor[bAddr].bCWLS = (p[0] == '1') ? 1 : 0;
		Motor[bAddr].bCCWLS = (p[1] == '1') ? 1 : 0;
		Motor[bAddr].bMoving = (p[6] == '1') ? 1 : 0;
		p = strchr(&szStatus[10], '_');
		if (p == NULL) return;
		p++;
		Motor[bAddr].nPos = atoi(p);
		Motor[bAddr].fPos = (float)Motor[bAddr].nPos / NV_Settings.fPulsePerMM;
	}
	//aSR_11111111
	else if (strncmp(&szStatus[1], "SR", 2) == 0) {
		Motor[bAddr].bAddr = bAddr;
		bV = (szStatus[4] == '1') ? 1 : 0;
		Motor[bAddr].bCWLS = bV;
		bV = (szStatus[5] == '1') ? 1 : 0;
		Motor[bAddr].bCCWLS = bV;
		bV = (szStatus[10] == '1') ? 1 : 0;
		Motor[bAddr].bMoving = bV;
	}
	//aPR_23432
	else if (strncmp(&szStatus[1], "PR", 2) == 0) {
		Motor[bAddr].nPos = atoi(&szStatus[4]);
		Motor[bAddr].fPos = (float)Motor[bAddr].nPos / NV_Settings.fPulsePerMM;
	}
}
//
void Motor_GetAllInfo(int nAxis, char *szBuffer)
{
	//Motor_GetStatus(nAxis);
	//Motor_GetPosition(nAxis);
	//OSTimeDly(1);
	sprintf(szBuffer, "AXIS=%d,POS=%.4f,CW=%d,CCW=%d,MOVING=%d", nAxis,
		Motor[nAxis].fPos, Motor[nAxis].bCWLS, Motor[nAxis].bCCWLS, Motor[nAxis].bMoving);
}
//-------------------------------------------------
// nAxis = 0,1,2
int Motor_MoveAbsolute(int nAxis, float fPos)
{
	int nPos;
	int nAddr;
	char szBuffer[64];
	//
	//Motor_GetStatus(nAxis);
	//OSTimeDly(1);
	//
	nAddr = nAxis + MOTOR_RS485_ADDR;
	if (Motor[nAxis].bMoving == 1) { //already moving
		sprintf(szBuffer, "%dK\n", nAddr);
		RS485_WriteString(2, szBuffer);
	}
	nPos = (fPos - Motor[nAxis].fPos) * NV_Settings.fPulsePerMM;
	sprintf(szBuffer, "%dD%d\n", nAddr, nPos);
	RS485_WriteString(2, szBuffer);
	//
	sprintf(szBuffer, "%dGO\n", nAddr);
	RS485_WriteString(2, szBuffer);
	return 0;
}
//
int Motor_MoveRelative(int nAxis, float fPos)
{
	return Motor_MoveAbsolute(nAxis, fPos + Motor[nAxis].fPos);
}

int Motor_Jog(int nAxis, int nDirection, int nSpeed)
{
	char szBuffer[64];
	char cDir;
	int nAddr;
	//
	nAddr = nAxis + MOTOR_RS485_ADDR;
	cDir = (nDirection == 1) ? '+' : '-';
	sprintf(szBuffer, "%dJOG%c%d\n", nAddr, cDir, nSpeed);
	RS485_WriteString(2, szBuffer);
	return 0;
}

int Motor_Stop(int nAxis)
{
	char szBuffer[64];
	int nAddr;
	nAddr = nAxis + MOTOR_RS485_ADDR;
	sprintf(szBuffer, "%dK\n", nAddr); //kill motion
	RS485_WriteString(2, szBuffer);
	return 0;
}
//-------------------------------------------------
//nAxis 0:X, 1:Y, 2:Z
//set current position as home position
//-------------------------------------------------
int Motor_ResetPos(int nAxis)
{
	char szBuffer[64];
	int nAddr;
	nAddr = nAxis + MOTOR_RS485_ADDR;
	sprintf(szBuffer, "%dPSET0\n", nAddr); //kill motion
	RS485_WriteString(2, szBuffer);
	//Motor[nAxis].fPos = 0;	//unit mm
	//Motor[nAxis].nPos = 0;	//unit count
	return 0;
}
