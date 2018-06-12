//==============================================================================
//
// Title:       Turbo_Control_Protocol
// Purpose:     A short description of the application.
//
// Created on:  2010/5/26 at 09:20:51 by FangJianMin.
// Copyright:   NSRRC. All Rights Reserved.
//
//==============================================================================

//==============================================================================
// Include files
#include <predef.h>
#include <stdio.h>		//sprintf()
#include <stdlib.h>
#include <string.h> 	//strlen(), memmove()
#include <startnet.h>
#include "mod_sel.h"
#include <basictypes.h>		//BYTE, PWORD
//#include <pins.h>
#include <ucos.h>		//OSTimeDly()
//
#include "main.h"
#include "cpu_cmd.h"
#include "data_struct.h"
#include "Turbo_Control_Protocol.h"
#include "function.h"
//
extern SYS_PARAM_3 SysParam;
int nSimuTPSpeed = 0;
//
int nTurboRotationSpeed = 0;
int nTurboTemperature = 0;
int nTurboReadySpeed = 1500;
int nTurboErrorSpeed = 1490;
int nTurboLowSpeed = 100;
//
extern BYTE bChkTP;
extern NV_SETTINGS NV_Settings;
extern BYTE bDebug;
//==============================================================================
// Constants

//==============================================================================
// Types

//==============================================================================
// Static global variables
extern int fdSerial1;
extern char szSerial1Buffer[MAX_STRING_SIZE];
extern BYTE bSerial1Got;
//==============================================================================
// Static functions

int GetTurboResponse (char* szResponse, char* szResponseParameter, char* szResponseData);
//varian pump
//      02803030383130034241
//STOP  02803030303130034232
//START 02803030303131034233
//TEMP  028032313130033831
//SPEED 028032303330033832
//==============================================================================
// Global variables

//==============================================================================
// Global functions

/// HIFN The main entry-point function.
//------------------------------------------------------------------------------
int InitTurbo (void)
{
	if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		nTurboReadySpeed = NV_Settings.nTurboReadySpeed[1]; //READY_SPEED
		nTurboLowSpeed = NV_Settings.nTurboLowSpeed[1];
	}
	else if (NV_Settings.nTurboType == TURBO_AGILENT) {
		nTurboReadySpeed = NV_Settings.nTurboReadySpeed[2]; //READY_SPEED
		nTurboLowSpeed = NV_Settings.nTurboLowSpeed[2];
	}
	else { //0:TURBO_CUSTOM
		nTurboReadySpeed = NV_Settings.nTurboReadySpeed[0]; //READY_SPEED
		nTurboLowSpeed = NV_Settings.nTurboLowSpeed[0];
	}
	return 0;
}

int CloseTurbo(void)
{
	return 0;
}

int WaitTurboResponse(int nTimeout)
{
	int i;
	for (i = 0; i < nTimeout; i++) {
		OSTimeDly(1);
		if (bSerial1Got == 1)
			return SUCCESS;
	}
	return ERROR_FAIL;
}

void StartTurbo (void)
{	//custum(0), Peiffer(1), Agilent(2)
	char szBuffer[256];
	SetTurboPump(1);
	if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		//a2,a1,a0-10-n2,n1,n0-I1,I0-dn,...d0-c2,c1,c0-\r
		//a2,a1,a0: RS485 address -001
		//n2,n1,n0: vacuum parameter number -010-
		//I1,I0: data length -06-
		//dn,d0: data -111111-
		//c2,c1,c0: checksum -015-
		ClearSerial1Buffer();
		strcpy(szBuffer, "0011001006111111015\r"); //strange behavior!!!!
		if (SysParam.bSimuVAC == 0)
			RS485_WriteString_P1(szBuffer);
		//GetTurboErrorMsg(szError);
		//printf("%s,%d\n", szBuffer, (int)strlen(szBuffer));
		nTurboReadySpeed = NV_Settings.nTurboReadySpeed[1]; //READY_SPEED
		nTurboLowSpeed = NV_Settings.nTurboLowSpeed[1];
	}
	else if (NV_Settings.nTurboType == TURBO_AGILENT) {
		if (SysParam.bSimuVAC == 0) {
			RS232_WriteHex(1,"02803030383130034241");
			OSTimeDly(5);
			RS232_WriteHex(1,"02803030303131034233");
		}
		nTurboReadySpeed = NV_Settings.nTurboReadySpeed[2]; //READY_SPEED
		nTurboLowSpeed = NV_Settings.nTurboLowSpeed[2];
	}
	else { //0:TURBO_CUSTOM
		nTurboReadySpeed = NV_Settings.nTurboReadySpeed[0]; //READY_SPEED
		nTurboLowSpeed = NV_Settings.nTurboLowSpeed[0];
	}
	nTurboErrorSpeed = nTurboReadySpeed - 10;
#if SHOW_VACUUM_STATE == 1
	printf("Start Turbo\n");
#endif
}

void StopTurbo (int nEnableVenting)
{
	static int nReentry = 0;
	char szBuffer[128];
	//int i;
	//if (nReentry == 1)
	//	return;
	//nReentry = 1;
	SetTurboPump(1); //shutdown turbo power
	if (SysParam.bSimuVAC == 1) {
		//do nothing
		goto StopTurboEnd;
	}
	else if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		//----------------------------------------------------
		//a2,a1,a0-10-n2,n1,n0-I1,I0-dn,...d0-c2,c1,c0-\r
		//a2,a1,a0: RS485 address -001
		//n2,n1,n0: vacuum parameter number -010-
		//I1,I0: data length -06-
		//dn,d0: data -000000-
		//c2,c1,c0: checksum -009-
		//----------------------------------------------------
		if (nEnableVenting) {
			//001-10-012-06-111111-017
			strcpy(szBuffer, "0011001206111111017\r");
			RS485_WriteString_P1(szBuffer);
		}
		else {
			//001-10-012-06-000000-011
			strcpy(szBuffer, "0011001206000000011\r");
			RS485_WriteString_P1(szBuffer);
		}
		OSTimeDly(4); //MUST >=4, 2016/12/9, important !!!!
		ClearSerial1Buffer();
		strcpy(szBuffer, "0011001006000000009\r");
		RS485_WriteString_P1(szBuffer);
	}
	else if (NV_Settings.nTurboType == TURBO_AGILENT) { //Varian or Agilent
		RS232_WriteHex(1,"02803030383130034241");
		OSTimeDly(3);
		RS232_WriteHex(1,"02803030303130034232");
	}
	else {
		//
	}
	//OSTimeDly(4); //wait a moment, then turn off power
	//SetTurboPump(0); //shutdown turbo power
StopTurboEnd:
#if SHOW_VACUUM_STATE == 1
	printf("Stop Turbo, VENT=%d\n", nEnableVenting);
#endif
	nReentry = 0;
}
//
void EnableTurboVent(int nEnableVenting)
{
	char szBuffer[128];
	if (nEnableVenting) {
		//001-10-012-06-111111-017
		strcpy(szBuffer, "0011001206111111017\r");
		RS485_WriteString_P1(szBuffer);
	}
	else {
		//001-10-012-06-000000-011
		strcpy(szBuffer, "0011001206000000011\r");
		RS485_WriteString_P1(szBuffer);
	}
}
//
// return rotation speed in unit Hz
#define TP_TRY_NUM		20
//
int GetTurboRotationSpeed (int nUpdate)
{
	char szTemp[256];
	char szResponse[256], szResponseParameter[4], szResponseData[100];
	int nResponseParameter;
	int nTempSpeed = -100;
	int nRet;
	static int nErrorCount = 0;
	static int nReentry = 0;
	//
	if (SysParam.bSimuVAC == 1) {
		nTurboRotationSpeed = nSimuTPSpeed;
		return nTurboRotationSpeed;
	}
	if (nUpdate == 0)
		return nTurboRotationSpeed;
	else if (bChkTP == 0)
		return 0;
	if (nReentry == 1)
		return nTurboRotationSpeed;
	nReentry = 1;
	if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		//a2,a1,a0-10-n2,n1,n0-02-=?-c2,c1,c0-\r
		//a2,a1,a0: RS485 address -001
		//n2,n1,n0: vacuum parameter number -309-
		//c2,c1,c0: checksum -107-
		sprintf (szTemp, "0010030902=?107\r"); //001-00-309
		ClearSerial1Buffer();
		nRet = RS485_WriteString(1, szTemp);
		if ((nRet == 0) && (WaitTurboResponse(6) == SUCCESS)) {
			//response example: "0011030906000047032" //001-10-309-06-000047-032
			if (GetTurboResponse(szResponse, szResponseParameter, szResponseData) == 6) {
				sscanf (szResponseData, "%d", &nTempSpeed);
				sscanf (szResponseParameter, "%d", &nResponseParameter);
			}
			if ((nTempSpeed >= 0) && (nTempSpeed <= 2000)) {
				nErrorCount = 0;
				nTurboRotationSpeed = nTempSpeed;
			}
			else {
				if (nErrorCount == 0) {
					sprintf(szTemp, "TPSP_ERR=%s", szResponse);
					SaveErrorMessage(szTemp, 1);
				}
				nErrorCount++;
			}
		}
		else if (nErrorCount < TP_TRY_NUM) {
			if (nErrorCount == 3) {
				sprintf(szTemp, "GET PF TPSP NO RESP,%d", nTempSpeed);
				SaveErrorMessage(szTemp, 1);
			}
			nErrorCount++;
		}
		if (nErrorCount >= TP_TRY_NUM) {
			nTurboRotationSpeed = -1;
		}
	}
	else {
		ClearSerial1Buffer();
		RS232_WriteHex(1,"028032303330033832");
		if (WaitTurboResponse(6) == SUCCESS) { //increase from 5 to 6, 2017/11/14
			if (GetTurboResponse(szResponse, szResponseParameter, szResponseData) == 10) {
				sscanf (szResponseData, "%d", &nTempSpeed);
				sscanf (szResponseParameter, "%d", &nResponseParameter);
			}
			if ((nTempSpeed >= 0) && (nTempSpeed <= 2000)) {
				nErrorCount = 0;
				nTurboRotationSpeed = nTempSpeed;
			}
			else {
				if (nErrorCount == 0) {
					sprintf(szTemp, "TPSP_ERR=%s", szResponse);
					SaveErrorMessage(szTemp, 1);
				}
				nErrorCount++;
			}
		}
		else if (nErrorCount < TP_TRY_NUM) {
			if (nErrorCount == 3) {
				SaveErrorMessage("GET AG TPSP NO RESP", 1);
			}
			nErrorCount++;
		}
		if (nErrorCount >= TP_TRY_NUM)
			nTurboRotationSpeed = -1;
	}
	nReentry = 0;
	return nTurboRotationSpeed;
}

//get turbo motor temperature in degree centigrade
int GetTurboMotorTemp (int nUpdate)
{
	char szTemp[256];
	char szResponse[256], szResponseParameter[4], szResponseData[100];
	int nResponseParameter;
	int nTemp = -100;
	int nRet;
	static int nErrorCount = 0;
	static int nReentry = 0;
	//
	if (nUpdate == 0) //return previous value
		return nTurboTemperature;
	else if (bChkTP == 0)
		return 25;
	if (nReentry == 1)
		return nTurboTemperature;
	nReentry = 1;
	if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		//a2,a1,a0-00-n2,n1,n0-02-=?-c2,c1,c0-\r
		//a2,a1,a0: RS485 address -001-
		//n2,n1,n0: vacuum parameter number -346-
		//c2,c1,c0: checksum -108-
		sprintf (szTemp, "0010034602=?108\r");
		ClearSerial1Buffer();
		nRet = RS485_WriteString(1, szTemp);
		if ((nRet == 0) && (WaitTurboResponse(4) == SUCCESS)) {
			//response example: "0011034606000047032" //001-10-346-06-000047-032, 47 degree
			if (GetTurboResponse(szResponse, szResponseParameter, szResponseData) == 6) {
				sscanf(szResponseData, "%d", &nTemp);
				sscanf(szResponseParameter, "%d", &nResponseParameter);
			}
			if ((nTemp > -30) && (nTemp <= 150)) {
				nErrorCount = 0;
				nTurboTemperature = nTemp;
			}
			else nErrorCount++;
		}
		else if (nErrorCount < TP_TRY_NUM) {
			if (nErrorCount == 3) {
				SaveErrorMessage("GET PF TEMP NO RESP", 1);
			}
			nErrorCount++;
		}
		if (nErrorCount >= TP_TRY_NUM)
			nTurboTemperature = -1;
	}
	else {
		ClearSerial1Buffer();
		RS232_WriteHex(1,"028032303430033835");
		if (WaitTurboResponse(4) == SUCCESS) {
			if (GetTurboResponse(szResponse, szResponseParameter, szResponseData) == 10) {
				sscanf(szResponseData, "%d", &nTemp);
				sscanf(szResponseParameter, "%d", &nResponseParameter);
			}
			if ((nTemp > -30) && (nTemp <= 120)) {
				nErrorCount = 0;
				nTurboTemperature = nTemp;
			}
			else nErrorCount++;
		}
		else if (nErrorCount < TP_TRY_NUM) {
			if (nErrorCount == 3) {
				SaveErrorMessage("GET AG TEMP NO RESP", 1);
			}
			nErrorCount++;
		}
		if (nErrorCount >= TP_TRY_NUM)
			nTurboTemperature = -1;
	}
	nReentry = 0;
	return nTurboTemperature;
}
//-------------------------------------------------------------------------------
// nGV=0 or 3
// nOn=VALVE_OPEN, VALVE_CLOSE
// GV can not be controlled directly
// EM100, turbo gate valve = 0, GV0:TP-AIR tube, GV1:SCROLL
// EM200, turbo gate valve = 3
//-------------------------------------------------------------------------------
void SetTurboGV(int nOn)
{
	char szBuffer[128];
	//if (nGV != TP_GATE_VALVE)
	//	return;
	if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		//a2,a1,a0-10-n2,n1,n0-l1,l0-dn ... d0,c2-c1-c0,\r
		if (nOn == VALVE_OPEN) { //OPEN:"00110-035-03-007-133"
			//"00110-030-03-002122"
			strcpy(szBuffer, "0011003003002122\r");
			RS485_WriteString_P1(szBuffer); //=VentToAir(VENT_DIRECT)
		}
		else {//OPEN:"00110-035-03-006-132"
			//"00110-030-03-000120"
			strcpy(szBuffer, "0011003003000120\r");
			RS485_WriteString_P1(szBuffer); //=VentToAir(VENT_DELAY)
		}
	}
	return;
}
//------------------------------------------------------------------------------
// get response string in szResponse
//------------------------------------------------------------------------------
int GetTurboResponse(char* szResponse, char* szResponseParameter, char* szResponseData)
{
	char szBuffer[5];
	int n, nDataLength = 0;
	char *p;
	//
	strcpy(szResponse, szSerial1Buffer);
	n = strlen(szResponse);
	if (bDebug == 12) printf("%s\r\n", szResponse);
	if (n == 0) { //disconnect
		sprintf (szResponse, "-1");
		sprintf (szResponseData, "-1");
		return -1;
	}
	else if (n > 120) { //too long
		sprintf (szResponse, "-4");
		sprintf (szResponseData, "-4");
		return -4;
	}
	else if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		//a2,a1,a0-10-n2,n1,n0-l1,l0-dn ... d0,c2-c1-c0,\r
		//example szResponse = "0011030906000508033"
		//"00110-309-06-000508-033"
		//szBuffer = "06"
		//szResponseParameter = "309"
		//szResponseData = "000508"
		//"00110-346-06-000047-032", 47 degree
		p = strstr(szResponse, "001");
		if (p == NULL) {
			sprintf (szResponse, "-3");
			sprintf (szResponseData, "-3");
			return -2;
		}
		strcpy(szResponse, p);
		memmove(szBuffer, &szResponse[8], 2);
		szBuffer[2] = 0;
		nDataLength = atoi(szBuffer);
		if (nDataLength > 21) { //illegal length
			sprintf (szResponse, "-2");
			sprintf (szResponseData, "-2");
			return -2;
		}
		memmove(szResponseParameter, &szResponse[5], 3); //309
		szResponseParameter[3] = 0; //byte num
		//
		memmove(szResponseData, &szResponse[10], nDataLength); //000508
		szResponseData[nDataLength] = 0; //temperature
	}
	else { //TURBO_AGILENT
		memmove(szResponseParameter, &szResponse[0], 3);
		szResponseParameter[3] = 0;
		memmove(szResponseData, &szResponse[3], 7); //speed
		szResponseData[7] = 0;
		nDataLength = 10;
	}
	return nDataLength;
}

//-------------------------------------------------------
//1: delayed venting, gate valve will close later
//2: no venting
//3: direct venting, gate valve will keep open
//-------------------------------------------------------
void VentToAir(int nType)
{
	char szBuffer[128];
	if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		//if (nEnable == 1)
		//	RS485_WriteString(1, "0011003503007132\r"); //¬ª®ð
		//else if (nEnable == 0)
		//	RS485_WriteString(1, "0011003503006131\r"); //¤£¬ª®ð
		if (nType == VENT_DELAY) {
			strcpy(szBuffer, "0011003003000120\r");
			RS485_WriteString_P1(szBuffer);
		}
		else if (nType == VENT_NONE) {
			strcpy(szBuffer, "0011003003001121\r");
			RS485_WriteString_P1(szBuffer);
		}
		else if (nType == VENT_DIRECT)
			strcpy(szBuffer, "0011003003002122\r");
			RS485_WriteString_P1(szBuffer);
	}
	else {  //TURBO_AGILENT
		//RS232_WriteHex(1,"02803030383130034241");
		//OSTimeDly(3);
		//RS232_WriteHex(1,"02803030303130034232");
	}
}

void TurboVent(int nVent)
{
	if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
	}
	else if (NV_Settings.nTurboType == TURBO_AGILENT) {
		if (nVent == 1)
			SetGateValve(TP_GATE_VALVE, VALVE_OPEN); //open to air
		else
			SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
	}
}

int nTurboCommand[TURBO_CMD_NUM];
int nTurboCmdIn = 0;
int nTurboCmdOut = 0;
void ClearTurboCommand(void)
{
	nTurboCmdIn = 0;
	nTurboCmdOut = 0;
}

int GetTurboCommandNum(void)
{
	return ((nTurboCmdIn - nTurboCmdOut + TURBO_CMD_NUM) % TURBO_CMD_NUM);
}

void PushTurboCommand(int nCommand)
{
	nTurboCommand[nTurboCmdIn] = nCommand;
	nTurboCmdIn = (nTurboCmdIn + 1) % TURBO_CMD_NUM;
}

int PopTurboCommand(void)
{
	int nCommand;
	nCommand = nTurboCommand[nTurboCmdOut];
	nTurboCmdOut = (nTurboCmdOut + 1) % TURBO_CMD_NUM;
	return nCommand;
}

void ExecuteTurboCommand(int nCommand)
{
	switch (nCommand) {
		case TP_CMD_STOP_VENT:
			StopTurbo(1);
			break;
		case TP_CMD_STOP_NO_VENT:
			StopTurbo(0);
			break;
		case TP_CMD_START:
			StartTurbo();
			break;
		case TP_CMD_VENT_DELAY:
			VentToAir(VENT_DELAY);
			break;
		case TP_CMD_VENT_DIRECT:
			VentToAir(VENT_DIRECT);
			break;
		case TP_CMD_VENT_NONE:
			VentToAir(VENT_NONE);
			break;
	}
}

void GetTurboErrorMsg(char *szError)
{
	char szBuffer[128], szResponse[128];
	if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
		//ClearSerial1Buffer();
		strcpy(szBuffer, "0010030302=?101\r");
		RS485_WriteString_P1(szBuffer);
		if (WaitTurboResponse(6) == SUCCESS) {
			strcpy(szResponse, szSerial1Buffer);
			strcpy(szError, szResponse);
		}
	}
}
