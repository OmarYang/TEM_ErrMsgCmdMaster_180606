/*
 * hv_ctrl.cpp
 *
 *  Created on: 2014/4/4
 *      Author: Robert Lee
 */
#include <predef.h>
#include <stdio.h>
#include <string.h>
#include <serial.h>
//#include <NetworkDebug.h>
//#include <math.h>	//fabs()
#include <ucos.h>
#include "mod_sel.h"
#include <basictypes.h>		//BYTE, PWORD
#include <startnet.h>		//TICKS_PER_SECOND
//#include <pins.h>
#include "main.h"
//#include "cpu_cmd.h"
#include "io.h"
#include "data_struct.h"
#include "function.h"
#include "hv_ctrl.h"
//
extern NV_SETTINGS NV_Settings;
extern SYS_PARAM_3 SysParam;
BYTE bWaitHVWDT = 0;
//
int HV_DecipherResponse(char *p)
{
	int i, nPN;
	int nID[6];
	BYTE bAddr;
	bAddr = *p - '0';
	if (bAddr != HV_RS485_ADDR)
		return -1;
	p++;
	if (strncmp(p, "SN=", 3) != 0)
		return -1;
	p = p + 3;
	SysParam.bErrorHWVersion |= ERR_HW_HVID;
	nPN = sscanf(p, "%02X%02X%02X%02X%02X%02X",
		&nID[0],&nID[1],&nID[2],&nID[3],&nID[4],&nID[5]);
	if (nPN != HV_ID_NUM) return -1;
	for (i = 0; i < HV_ID_NUM; i++) {
		if (nID[i] != NV_Settings.bHVID[i])
			return -1;
	}
	SysParam.bErrorHWVersion &= (~ERR_HW_HVID);
	return 0;
}

void HV_GetSN(void)
{
	char szBuffer[32];
	sprintf(szBuffer, "%dSN:0:0\n", HV_RS485_ADDR);
	bWaitHVWDT = 0;
	RS485_WriteString(2, szBuffer);
	return;
}

void HV_SetSN(char *p)
{
	char szBuffer[32];
	p[12] = 0;
	sprintf(szBuffer, "%dSN:1:%s\n", HV_RS485_ADDR, p);
	RS485_WriteString(2, szBuffer);
	return;
}

void HV_SetAcc(float fVolt)
{
	char szBuffer[32];
	sprintf(szBuffer, "%dSETV:0:%f\n", HV_RS485_ADDR, fVolt);
	if (bWaitHVWDT < 3) bWaitHVWDT++;
	RS485_WriteString(2, szBuffer);
	return;
}

void HV_SetBias(float fVolt)
{
	char szBuffer[32];
	sprintf(szBuffer, "%dSETV:1:%f\n", HV_RS485_ADDR, fVolt);
	if (bWaitHVWDT < 3) bWaitHVWDT++;
	RS485_WriteString(2, szBuffer);
	return;
}

void HV_SetFila(float fVolt)
{
	char szBuffer[32];
	sprintf(szBuffer, "%dSETV:7:%f\n", HV_RS485_ADDR, fVolt);
	if (bWaitHVWDT < 3) bWaitHVWDT++;
	RS485_WriteString(2, szBuffer);
	return;
}

void HV_SetOn(int nEnable)
{
	char szBuffer[32];
	sprintf(szBuffer, "%dHVON:%d\n", HV_RS485_ADDR, nEnable);
	if (bWaitHVWDT < 3) bWaitHVWDT++;
	RS485_WriteString(2, szBuffer);
	return;
}
