/*
 * vacuum.cpp
 *
 *  Created on: 2014/8/21
 *      Author: USER
 */
#include <predef.h>
#include <stdio.h>
#include <stdlib.h> //atoi, srand
#include <string.h>
#include <startnet.h>
#include <autoupdate.h>
//#include <dhcpclient.h>
//#include <tcp.h>
//#include <udp.h>
#include <serial.h>
#include <NetworkDebug.h>
#include "mod_sel.h"
#include <cfinter.h>
#include <pins.h>
#include <ucos.h>
#include "main.h"
#include "cpu_cmd.h"
#include "data_struct.h"
#include "rtc.h"
#include "function.h"
#include "Turbo_Control_Protocol.h"

/*--------------------------------------------------------------
			GREEN			RED
AIR			Off				On
AIR_WAIT	Off				Blink 1 Hz
LOW_WAIT	Blink 1 Hz		Off (DC=1/4)
LOW			Blink 1 Hz		On
HI_WAIT		Blink 1 Hz		Off	(DC=2/4)
HI			On				Off
UH_WAIT		Blink 1 Hz		Off (DC=3/4)
UH			On				Off
ERROR1		Blink_1Hz_1P4	Blink_1Hz_1P4
ERROR2		Blink_1Hz_2P4	Blink_1Hz_2P4
--------------------------------------------------------------*/
#define FAST_DROP_TIMEOUT	4
#define TPGV_INIT			91
#define ERROR_COUNT_MAX		6
//
extern NV_SETTINGS NV_Settings;
extern SYS_PARAM_3 SysParam;
extern BYTE bDebug;
extern BYTE bSerDebug;
extern BYTE bScanAbort;
extern BYTE bScanPause;
extern BYTE bEnableIO;
extern BYTE bChkTP;
extern BYTE bVACBypassOK[VAC_GAUGE_NUM_MAX];
extern float fVACChangeRate[VAC_GAUGE_NUM_MAX];
extern float fVACLevel[VAC_GAUGE_NUM_MAX]; //used to calculate change rate
extern int nIdleTimeout;
extern int nVacOn;
extern int nVacOff;
extern int nVacOffSent;
extern int nGotoAir;
extern long lVacTotalTime; // time since VACON command
extern long lTick1S;
BYTE bVacStateReactivate = 0;
//
extern WORD wDI;
extern int nReadyToErrorNdx[3];
extern int nReadyToErrorNum;
extern int nReadyCount;
//
extern int nTurboRotationSpeed;
extern int nTurboTemperature;
extern int nTurboReadySpeed;
extern int nTurboErrorSpeed;
extern int nTurboLowSpeed;
extern BYTE bInitOS;
extern BYTE bInitOK;
//
//extern BYTE bLEDState[LED_NUM];
extern float fTemperature[TEMP_SENSOR_NUM_MAX];
//
int nTPGVTimeout = -1;	//TP GV open/close timeout
int nVacTimeout = -1;	//vacuum state change timeout
int nIGTimeout = 0; 	//wait until IG is stable
int nVentTimeout = 0;	//GV2 motor motion
int nVACCounter = 0;
//
//IP pump gas leak at INT_HI_WAIT stage
BYTE bIPLeakDone[CHECK_VOLT_NUM] = {0, 0, 0, 0};
int nIPLeakNdx = 0;
int nIPLeakTimeout = 0;
BYTE bIPLeakAllDone = 0;
//
int nVentTime = 0;
int nAutoVAC = 0;
int nChamberCheck = 0;
BYTE bErrorIP = 0;
VAC_ST_LOG VacStLog[VAC_ST_LOG_NUM];
//
//auto vacuum process
void CheckAutoVAC(void)
{
	if (NV_Settings.bAutoVAC != 1) { //auto VAC function disabled
		nAutoVAC = 0;
		return;
	}
	if (IsChamberClose(NULL) == DOOR_CLOSE) {
#if GAUGE_OP_MODE == 1 //EM100
		if ((SysParam.bVacuumState == VAC_ST_STANDBY) || (SysParam.bVacuumState == VAC_ST_ERROR))
#else //EM200
		if ((SysParam.bVacuumState == VAC_ST_STANDBY0) || (SysParam.bVacuumState == VAC_ST_ERROR2))
#endif
			nAutoVAC++;
	}
	else {
		nAutoVAC = 0;
	}
	if (nAutoVAC >= 3) {
		nAutoVAC = 0;
		nVacOn = 1;
		nVacOff = 0;
		nChamberCheck = 0;
		lVacTotalTime = 0;
	}
}

void PrintVacStLog(void)
{
	int i;
	char szValue[32];
	char szBuffer[64];
	printf("current time=%ld", lTick1S);
	for (i = 0; i < VAC_ST_LOG_NUM; i++) {
		if (VacStLog[i].bState == VAC_ST_NONE) continue;
		if (GetVacuumStateName(VacStLog[i].bState, szValue) != 0)
			continue;
		printf("----------------------\n");
		sprintf(szBuffer, "%d,%ld s,%s,PG1=%.3fV,IG=%.3fV(%.3e torr)\n", i, VacStLog[i].lTick, szValue, VacStLog[i].fPG[1],
			VacStLog[i].fIG, VoltToTorr(VacStLog[i].fIG));
		printf(szBuffer);
		sprintf(szBuffer, "TEMP=%.1f,%.1f,%.1f,%.1f deg C\n", VacStLog[i].fTempe[0],
			VacStLog[i].fTempe[1], VacStLog[i].fTempe[2], VacStLog[i].fTempe[3]);
		printf(szBuffer);
	}
}

void PrintVacTimeout(void)
{
#if MODEL_TYPE == EM_200
	printf("VAC_TIMEOUT=%d min\n", SysParam.sVacTimeout);
	printf("IG_TIMEOUT=%d min\n", SysParam.sIGTimeout);
#else
	printf("VAC_TIMEOUT=%d s\n", SysParam.sVacTimeout);
	//printf("IG_TIMEOUT=%d s\n", SysParam.sIGTimeout);
#endif
	printf("TPGV_TIMEOUT=%d, TP gate valve\n", SysParam.sTPGVTimeout);
	printf("VENT_TIMEOUT=%d, GV2 Motor\n", nVentTimeout);
}

int IsTurboStartBrake(void)
{
	if (nTurboRotationSpeed <= NV_Settings.sTurboBrakeSpeed)
		return 1;
	else return 0;
}
//return 1: speed high eneough
//return 0: speed not high enough
int IsTurboSpeedHighEnough(void)
{
	if (bChkTP == 0)
		return 1;
	if (nTurboRotationSpeed >= nTurboReadySpeed - 15)
		return 1;
	return 0;
}

//return 1: is low speed
//return 0: speed not low enough
int IsTurboSpeedLowEnough(void)
{
	static int nLow = 0;
	if (bChkTP == 0) //if 0, always low speed
		return 1;
	if (nTurboRotationSpeed <= nTurboLowSpeed) {
		if (nLow < 100) nLow++;
	}
	else nLow = 0;
	if (nLow > 3) //speed is low
		return 1;
	else
		return 0; //not low
}

//return 1: turbo speed is lower down
//return 0: turbo speed is stable or increasing
int IsTurboSpeedLower(void)
{
	static int nNOK = 0;
	//if (bChkTP == 0) //don't check TP speed here
	//	return 1; //pass error check
	if (nTurboRotationSpeed <= nTurboErrorSpeed) {
		if (nNOK < 100) nNOK++; //not OK
	}
	else nNOK = 0;
	if (nNOK > 18) //6:3 seconds, 24:12 seconds
		return 1;
	else
		return 0;
}
//
void ClearVacuumParameters(int nSet)
{
	nVacOff = 0;
	nVacOn = 0;
	if (nSet == 0)
		return;
	bScanAbort = 1;	//stop scanning
	bScanPause = 0; //stop pause
	nVacTimeout = -1;
	nReadyCount = 0;
}
//------------------------------------------------------------
//#define GAUGE_OP_MODE 1	//use gauge(PG,G1)
//#define GAUGE_OP_MODE	2	//use gauge(PG,G1)+ion gauge(IG,G3)
//------------------------------------------------------------
#if GAUGE_OP_MODE == 1
//use gauge 1(PG) only
//0:chamber, 1:gun, 2:reserve, 3: ion gauge
// nNdx = 0,1,2 //pirani gauge
// nNdx = 3, ion gauge
// return VAC_AIR, VAC_LO, VAC_HI
int GetGaugeStatus(int nNdx, char *szValue)
{
	//WORD wV;
	char szStatus[64] = "NONE";
	static int nPreCheck[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0};
	static int nCurrentState[VAC_GAUGE_NUM_MAX] = {VAC_NONE, VAC_NONE, VAC_NONE, VAC_NONE};
	int nCheck[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0};
	static int nOK[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0};
	static int nBNOK[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0};
	float fV = 0;
	//---------------------------------------------------------
	// vacuum low   --> Vc hi --> AOx low      --> VDOx low
	// gauge broken --> Vc hi --> AOx negative --> VDOx low
	//---------------------------------------------------------
	if (szValue != NULL) sprintf(szValue, "VAC%d:OK,", nNdx);
	if (nNdx >= VAC_GAUGE_NUM_MAX) //invalid index
		goto StatusNOK;
//#if G0_ALWAYS_OK == 1
//	if (nNdx == 0) goto StatusOK;
//#endif
	//----------------------------------
	//wV = GetDigitalInput();
	if ((nNdx >= 0) && (nNdx <= 2)) {
		if (SysParam.bSimuVAC == 1)
			fV = SysParam.fADC[ADC_CH_VAC0 + nNdx];
		else
			fV = GetVoltage(ADC_CH_VAC0 + nNdx); //AIR,LO_HI
	}
	if (nNdx == 0) { //parani gauge
		nCheck[nNdx] = (bVACBypassOK[nNdx] != VAC_NONE) ? bVACBypassOK[nNdx] :
			(fV > NV_Settings.fPGLOTh) ? VAC_AIR :
			(fV > NV_Settings.fPGHITh) ? VAC_LO : VAC_HI;
	}
	else if (nNdx == 1) { //pirani gauge
		nCheck[nNdx] = (bVACBypassOK[nNdx] != VAC_NONE) ? bVACBypassOK[nNdx] :
			(fV > NV_Settings.fPGLOTh) ? VAC_AIR :
			(fV > NV_Settings.fPGHITh) ? VAC_LO : VAC_HI;
	}
	else if (nNdx == 2) { //pirani gauge
		nCheck[nNdx] = (bVACBypassOK[nNdx] != VAC_NONE) ? bVACBypassOK[nNdx] :
			(fV > NV_Settings.fPGLOTh) ? VAC_AIR :
			(fV > NV_Settings.fPGHITh) ? VAC_LO : VAC_HI;
	}
	else if (nNdx == 3) { //ion gauge
		if (SysParam.bSimuVAC == 1)
			fV = SysParam.fADC[ADC_CH_IG];
		else
			fV = GetIonGauge(); //1.268 V/decade
		nCheck[nNdx] = (bVACBypassOK[nNdx] != VAC_NONE) ? bVACBypassOK[nNdx] :
			(fV > NV_Settings.fIGLOTh) ? VAC_AIR :
			(fV > NV_Settings.fIGHITh) ? VAC_LO :
			(fV > NV_Settings.fIGMHTh) ? VAC_HI :
			(fV > NV_Settings.fIGUHTh) ? VAC_MH : VAC_UH;
	}
	fVACLevel[nNdx] = fV; //used to calculate change rate
	if ((fV < 0) && (bVACBypassOK[nNdx] == VAC_NONE)) { //negative voltage, check if check broken, for PG only
		nBNOK[nNdx]++;
		if (nBNOK[nNdx] < NV_Settings.sVACNOKTh[1]) //threshold for gauge broken
			goto StatusOK;
		if (szValue != NULL) {
			sprintf(szValue, "VAC%d:NOK,BROKEN,", nNdx);
			if (SysParam.bScanning) printf("%s\n", szValue);
		}
		return VAC_BROKEN;
	}
	else nBNOK[nNdx] = 0;
	//
	if (nCheck[nNdx] == nPreCheck[nNdx]) { //state not changed
		nOK[nNdx]++;
		if (nOK[nNdx] > NV_Settings.sVACNOKTh[0]) { //threshold for stable state
			//change state
			nCurrentState[nNdx] = nCheck[nNdx];
			nOK[nNdx] = 0;
		}
		goto StatusOK;
	}
	else {
		nOK[nNdx] = 0;
		//nBNOK[nNdx] = 0;
	}
	nPreCheck[nNdx] = nCheck[nNdx];
	goto StatusOK;
StatusNOK:
	if (szValue != NULL) sprintf(szValue, "VAC%d:NOK,", nNdx);
	return nCurrentState[nNdx];
StatusOK:
	if (szValue != NULL) {
		if (nCurrentState[nNdx] == VAC_AIR)
			sprintf(szStatus, "AIR");
		else if (nCurrentState[nNdx] == VAC_LO)
			sprintf(szStatus, "LO");
		else if (nCurrentState[nNdx] == VAC_HI)
			sprintf(szStatus, "HI");
		sprintf(szValue, "VAC%d:%s,", nNdx, szStatus);
	}
	return nCurrentState[nNdx];
}
//
void VacuumStateTask(void * pd)
{
	int nTimeTick2Hz = 0;
	BYTE bTick_4Hz = 0;
	char szTemp[128];
	int i;
	int nTempOK[TEMP_SENSOR_NUM_MAX] = {SUCCESS, SUCCESS, SUCCESS, SUCCESS, SUCCESS};
	int nTempFailCount[TEMP_SENSOR_NUM_MAX] = {0, 0, 0, 0, 0};
	int nTempIsOK = SUCCESS;
	int nFastDropTimeout = FAST_DROP_TIMEOUT;
	int nBroken = 0;
	char szValue[64], szBuffer[128];
	//
	//initial state, wait for I/O initialization
	OSTimeDly(TICKS_PER_SECOND * 3);
	SetHVPower(0);
	SetHVSwitch(0);
	SetScrollPump(0);	//turn off scroll pump power
	SetTurboPump(1);	//turn on(1)/off(0) turbo pump power
	StopTurbo(0); //ENVENT=0, GV0 close
	SetGateValve(0, VALVE_CLOSE); //GV0 is controlled by turbo pump
	SetGateValve(1, VALVE_CLOSE); //GV1 close
	//
	//SysParam.bPrevVacuumState = VAC_ST_NONE;
	SysParam.bVacuumState = VAC_ST_STANDBY;
	SysParam.bNextVacuumState = SysParam.bVacuumState;
	while (1)
	{	//
		//GV0:TP-to-AIR, GV1:SCROLL-AIR(direct control)
		//gauge0:chamber, gauge1:gun
		//abnormal: gauge0 OK, gauge1 NOK
		//
		OSTimeDly(TICKS_PER_SECOND/4); //wait 1/4 sec
		//if (bSerDebug == 0)
		//	Motor_GetAllStatus(); //get motor position, limit switch status
		if (bInitOK == 0)
			continue;
		bTick_4Hz ^= 1; 	//toggle every 1/4 seconds
		if (bTick_4Hz == 0)
			continue;
		nTimeTick2Hz++; 		//increase every 1/2 seconds
		//the following code execute 2 times per seconds
		if (nTimeTick2Hz % 2 == 0) {
			lVacTotalTime++;
			CheckAutoVAC();
		}
		//used to debug IO<->SCAN interface
		if ((bEnableIO == 1) || (bEnableIO == 4))
			GetDigitalInput(1); //execute twice per second
		//
		if (SysParam.bSimuVAC == 1) { //for simulation
			SysParam.sGaugeStatus[0] = GetGaugeStatus(0, NULL); //pirani gauge
			SysParam.sGaugeStatus[2] = GetGaugeStatus(2, NULL);
			SysParam.sGaugeStatus[3] = GetGaugeStatus(3, NULL); //ion gauge
		}
		SysParam.sGaugeStatus[1] = GetGaugeStatus(1, NULL);
		//--------------------------------------
		// check temperature
		//--------------------------------------
		for (i = 0; i < TEMP_SENSOR_NUM; i++) {
			nTempOK[i] = CheckTemperature(i);
			if (nTempOK[i] == ERROR_FAIL) {
				if (nTempFailCount[i] <= 3)
					nTempFailCount[i]++;
			}
			else {
				nTempFailCount[i] = 0;
			}
			if (nTempFailCount[i] >= 3) {
				nTempIsOK = ERROR_FAIL;
				break; //break for loop
			}
			else
				nTempIsOK = SUCCESS;
		}
		if ((nTempIsOK == ERROR_FAIL) && (SysParam.bVacuumState != VAC_ST_ERROR)) {
			for (i = 0; i < TEMP_SENSOR_NUM; i++) {
				if (nTempFailCount[i] != 0) {
					sprintf(szTemp, "TEMP_ERR=%d,%d,%.1f,\n", i, nTempFailCount[i], fTemperature[i]);
					SaveErrorMessage(szTemp, 1); //nPrint=1
				}
			}
			SysParam.bNextVacuumState = VAC_ST_ERROR;
		}
		if (SysParam.sGaugeStatus[1] == VAC_BROKEN) {
			if (SysParam.bVacuumState != VAC_ST_ERROR) {
				nBroken++;
				if (nBroken >= 3) {
					SaveErrorMessage("VAC1_BROKEN", 1);
					SysParam.bNextVacuumState = VAC_ST_ERROR;
					nBroken = 0;
					goto NextState;
				}
			}
		}
		else nBroken = 0; //vacuum gauge broken
		//
		if (SysParam.bVacuumState == VAC_ST_AIR) {
			if (nVacOn == 1) { //continuous sequence to READY
				//close turbo pump GV0
				//bLEDState[LEDB] = ST_LED_BLINK_1HZ;
				SetLEDState(LEDB, ST_LED_BLINK_1HZ);
				SetGateValve(0, VALVE_CLOSE); //GV0:TP-AIR tube, GV1:SCROLL
				SetGateValve(1, VALVE_CLOSE);  //GV1 close first
				StopTurbo(0); //ENVENT=0
				OSTimeDly(TICKS_PER_SECOND / 2);
				SetScrollPump(1);
				OSTimeDly(TICKS_PER_SECOND * 5);
				SetGateValve(1, VALVE_OPEN);
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT;
			}
			if (nVacOff == 1) nVacOff = 0;
		}
		else if (SysParam.bVacuumState == VAC_ST_STANDBY) {
			if (nVacOn == 1) { //continuous sequence to READY
				//bLEDState[LEDB] = ST_LED_BLINK_1HZ;
				SetLEDState(LEDB, ST_LED_BLINK_1HZ);
				SetGateValve(0, VALVE_CLOSE); //high vacuum
				SetGateValve(1, VALVE_CLOSE);  //GV1 close first
				StopTurbo(0); //ENVENT=0
				OSTimeDly(TICKS_PER_SECOND / 2);
				SetScrollPump(1);
				OSTimeDly(TICKS_PER_SECOND * 5);
				SetGateValve(1, VALVE_OPEN);  //GV1 open, rough pumping
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT;
			}
			if (nVacOff == 1) {
				if (NV_Settings.bEnableVent == 1) {
					SysParam.bNextVacuumState = VAC_ST_AIR_WAIT; //go to AIR
				}
				else nVacOff = 0;
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_LO) {
			if (SysParam.sGaugeStatus[1] == VAC_AIR) {
				SaveErrorMessage("LOW:G1=AIR", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR;
			}
			if (IsChamberClose(NULL) == DOOR_OPEN) {
				SaveErrorMessage("LOW:CHAMBER_OPEN", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR;
			}
			if (nVacOn == 1)
				SysParam.bNextVacuumState = VAC_ST_HI_WAIT; 	//go to HVAC
			if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;	//go to AIR
		}
		else if (SysParam.bVacuumState == VAC_ST_HI) {
			if ((SysParam.sGaugeStatus[1] & VAC_A0L) != 0) { //AIR or LO
				nReadyToErrorNdx[1]++;
				SaveErrorMessage("HI:G1=AIR_OR_LOW", 1);
				//RedundantProcess();
			}
			else if (IsChamberClose(NULL) == DOOR_OPEN) {
				nReadyToErrorNdx[2]++;
				SaveErrorMessage("HI:CHAM_OPEN,DI=0X%04X\n", 1);
			}
			else if (bChkTP && IsTurboSpeedLower()) {
				nReadyToErrorNdx[2]++;
				SaveErrorMessage("HI:TP_SPEED_LOW", 1);
			}
			else {
				nReadyToErrorNdx[0] = 0;
				nReadyToErrorNdx[1] = 0;
				nReadyToErrorNdx[2] = 0;
			}
			for (i = 0; i < 3; i++) {
				//0:vac0, 1:vac1, 2:chamber close
				if (nReadyToErrorNdx[i] >= nReadyToErrorNum) {
					if (SysParam.bScanning != OP_IDLE) {
						bScanAbort = 1;
						nReadyToErrorNdx[i] = 0;
						sprintf(szTemp, "%d,%d_ERR%d_SCAN_ABORT", nReadyToErrorNdx[i], nReadyToErrorNum, i);
						SaveErrorMessage(szTemp, 1);
					}
					else {
						sprintf(szTemp, "VAC%d_ERR_IN_HI", i);
						SaveErrorMessage(szTemp, 1);
						SysParam.bNextVacuumState = VAC_ST_ERROR;
					}
				}
			}
			if (nVacOn == 1) nVacOn = 0; //process complete
			if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;		//go to LOW
		}
		else if (SysParam.bVacuumState == VAC_ST_ERROR) {
			if (nVacOn == 1)
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT; //go to low
			else if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT; //go to AIR or STANDBY
		}
		else if (SysParam.bVacuumState == VAC_ST_AIR_WAIT) {
			//0:chamber, 1:gun
			if (NV_Settings.bEnableVent == 0) { //go to standby
				if (bChkTP == 0) //bypass check
					SysParam.bNextVacuumState = VAC_ST_STANDBY;
				else if (IsTurboSpeedLower())
					SysParam.bNextVacuumState = VAC_ST_STANDBY;
			}
			else if (NV_Settings.bEnableVent == 1) { //go to air
				if (IsTurboSpeedLowEnough()) {
					if (nVentTime > 0) {
						TurboVent(1);
						OSTimeDly(TICKS_PER_SECOND * 8); //wait ten second
						SysParam.sGaugeStatus[1] = GetGaugeStatus(1, NULL);
						OSTimeDly(TICKS_PER_SECOND * 2);
						TurboVent(0);
						nVentTime--;
					}
					if (nVentTime == 0)
						SysParam.bNextVacuumState = VAC_ST_AIR;
					else if (SysParam.sGaugeStatus[1] == VAC_AIR) //higher than low threshold
						SysParam.bNextVacuumState = VAC_ST_AIR;
				}
			}
			if (nVacTimeout == 0) {
				SaveErrorMessage("AIR_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR;
			}
			else if (nVacOn == 1)
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT; //go to low
			else if (nVacOffSent == 1) { //send VACON:0 again
				nVacOffSent = 0;
				bVacStateReactivate = 1;
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
			}
		}
		//-------------------------------------------------------------------------
		// wait state
		//-------------------------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_LO_WAIT) { //go to LOW
			//turbo pump speed is zero if moving from AIR to LOW
			//turbo pump speed is not zero if moving from LOW to AIR
			//0:chamber, 1:gun
			if (nVacOn == 1) {
				if (IsChamberClose(NULL) == DOOR_OPEN)
					nChamberCheck++;
				else nChamberCheck = 0;
				if (nChamberCheck > 3) {
					SaveErrorMessage("LO_WAIT,CHAM_OPEN", 1);
					SysParam.bNextVacuumState = VAC_ST_ERROR;
				}
			}
			if ((nVacOn == 1) && ((SysParam.sGaugeStatus[1] & VAC_L0H) != 0) && (nFastDropTimeout == 0)) {//don't care VAC1 for VacOn process
				//nFastDropTimeout=0, VAC isn't decreasing rapidly
				if (nVacTimeout < NV_Settings.sLowWaitTimeout - 6) //wait at least 20 seconds
					SysParam.bNextVacuumState = VAC_ST_LO;
			}
			else if (nVacTimeout == 0) {
				SysParam.bNextVacuumState = VAC_ST_ERROR;
				SaveErrorMessage("LO_WAIT_TIMEOUT", 1);
			}
			else if (nVacOff == 1) //shutdown from LOW_WAIT state
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
		}
		else if (SysParam.bVacuumState == VAC_ST_HI_WAIT) {
			if (IsTurboSpeedHighEnough() && (SysParam.sGaugeStatus[1] == VAC_HI)) {
				nReadyToErrorNdx[1] = 0; //reset error counter
				if (nReadyCount >= READY_COUNT_OK)
					SysParam.bNextVacuumState = VAC_ST_HI;
				else
					nReadyCount++;
			}
			else if (nVacTimeout == 0) {
				SaveErrorMessage("HI_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR;
			}
			else if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
		}
		//-------------------------------------------------------------------------
		// next vacuum state process
		//-------------------------------------------------------------------------
NextState:
		if ((SysParam.bNextVacuumState != SysParam.bVacuumState) || (bVacStateReactivate)) {
			//bLEDState[LEDG] = ST_LED_SOLID;
			SetLEDState(LEDG, ST_LED_SOLID);
			switch (SysParam.bNextVacuumState)
			{
				case VAC_ST_AIR_WAIT:	//go to AIR or STANDBY state
					bScanAbort = 1;
					SetHVSwitch(0);
					VentingProcess();
					nVentTime = 3;
					nVacTimeout = NV_Settings.sAirWaitTimeout;
					//bLEDState[LEDG] = ST_LED_SOLID;
					//bLEDState[LEDR] = ST_LED_BLINK_1HZ;
					//bLEDState[LEDB] = ST_LED_BLINK_1HZ;
					SetLEDState(LEDB, ST_LED_BLINK_1HZ);
					break;
				case VAC_ST_AIR:
					nVacOff = 0;
					nVacTimeout = -1;
					SetHVSwitch(0);
					SetGateValve(1, VALVE_CLOSE); //GV1 close
					//bLEDState[LEDG] = ST_LED_SOLID;
					//bLEDState[LEDR] = ST_LED_SOLID;
					//bLEDState[LEDB] = ST_LED_DARK;
					SetLEDState(LEDB, ST_LED_DARK);
					SetLEDState(LEDY, ST_LED_DARK); //ready
					nReadyCount = 0;
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime); //process end point
					break;
				case VAC_ST_STANDBY:
					nVacOff = 0;
					nVacTimeout = -1;
					SetHVSwitch(0);
					SetGateValve(1, VALVE_CLOSE); //GV1 close
					//bLEDState[LEDG] = ST_LED_SOLID;
					//bLEDState[LEDR] = ST_LED_SOLID;
					//bLEDState[LEDB] = ST_LED_DARK;
					SetLEDState(LEDB, ST_LED_DARK);
					SetLEDState(LEDY, ST_LED_DARK); //ready
					nReadyCount = 0;
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime); //process end point
					break;
				case VAC_ST_LO_WAIT:			//go to LOW
					SetHVSwitch(0);
					//gate valve is uncontrollable by NB
					//SetGateValve(0, VALVE_OPEN);	//turbo, Gate valve 0 open(1=ON), close(0=OFF)
					//gate valve is uncontrollable by turbo
					if (nVacOn == 1) {
						SetScrollPump(1);
						OSTimeDly(TICKS_PER_SECOND * 2);
						SetGateValve(1, VALVE_OPEN);	//external, gate valve 1 open(1=ON), close(0=OFF)
					}
					else if (nVacOff == 1) {
						StopTurbo(0); //stop turbo, no venting(0)
					}
					nFastDropTimeout = FAST_DROP_TIMEOUT;
					nVacTimeout = NV_Settings.sLowWaitTimeout;
					//bLEDState[LEDG] = ST_LED_SOLID;
					//bLEDState[LEDR] = ST_LED_DARK;
					//bLEDState[LEDB] = ST_LED_BLINK_1HZ;
					SetLEDState(LEDB, ST_LED_BLINK_1HZ);
					break;
				case VAC_ST_LO:
					//bLEDState[LEDG] = ST_LED_SOLID;
					//bLEDState[LEDR] = ST_LED_SOLID;
					//bLEDState[LEDB] = ST_LED_BLINK_2HZ;
					SetLEDState(LEDB, ST_LED_BLINK_2HZ);
					nVacTimeout = -1;
					nReadyCount = 0;
					break;
				case VAC_ST_HI_WAIT:			//low vacuum state
					SetHVSwitch(0);
					//SetGateValve(0, VALVE_OPEN);	//ON
					StopTurbo(0); //MUST, stop and then start again
					SetGateValve(1, VALVE_OPEN); 	//ON
					OSTimeDly(TICKS_PER_SECOND * 2); //MUST
					StartTurbo();
					nFastDropTimeout = FAST_DROP_TIMEOUT;
					nVacTimeout = NV_Settings.sHighWaitTimeout;
					//bLEDState[LEDG] = ST_LED_SOLID;
					//bLEDState[LEDR] = ST_LED_BLINK_1HZ;
					//bLEDState[LEDB] = ST_LED_BLINK_2HZ;
					SetLEDState(LEDB, ST_LED_BLINK_2HZ);
					nReadyCount = 0;
					break;
				case VAC_ST_HI:
					nVacOn = 0;
					nVacTimeout = -1;
					SetHVPower(1);
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime); //process end point
					//bLEDState[LEDG] = ST_LED_SOLID;
					//bLEDState[LEDR] = ST_LED_DARK;
					//bLEDState[LEDB] = ST_LED_SOLID;
					SetLEDState(LEDB, ST_LED_SOLID);
					SetLEDState(LEDY, ST_LED_SOLID); //ready
					break;
				case VAC_ST_ERROR:		//low vacuum operation
					ClearVacuumParameters(1);
					VentingProcess();
					//bLEDState[LEDG] = ST_LED_BLINK_1HZ;
					//bLEDState[LEDB] = ST_LED_BLINK_1HZ;
					SetLEDState(LEDG, ST_LED_BLINK_1HZ);
					SetLEDState(LEDB, ST_LED_BLINK_1HZ);
					SetLEDState(LEDY, ST_LED_DARK); //ready
					break;
			}
			//
			szBuffer[0] = 0;
			GetVacuumStateName(SysParam.bPrevVacuumState, szTemp);
			sprintf(szValue, "\nPREV=%s,", szTemp);
			strcat(szBuffer, szValue);
			GetVacuumStateName(SysParam.bVacuumState, szTemp);
			sprintf(szValue, "CURR=%s,", szTemp);
			strcat(szBuffer, szValue);
			GetVacuumStateName(SysParam.bNextVacuumState, szTemp);
			sprintf(szValue, "NEXT=%s\n", szTemp);
			strcat(szBuffer, szValue);
			printf(szBuffer);
			//
			SysParam.bPrevVacuumState = SysParam.bVacuumState;
			SysParam.bVacuumState = SysParam.bNextVacuumState; //set new SysParam.bVacuumState
			memmove(&VacStLog[0], &VacStLog[1], (VAC_ST_LOG_NUM_M1) * sizeof(VAC_ST_LOG));
			VacStLog[VAC_ST_LOG_NUM_M1].lTick = lTick1S;
			VacStLog[VAC_ST_LOG_NUM_M1].bState = SysParam.bVacuumState;
			VacStLog[VAC_ST_LOG_NUM_M1].fPG[0] = GetVoltage(ADC_CH_VAC0);
			VacStLog[VAC_ST_LOG_NUM_M1].fPG[1] = GetVoltage(ADC_CH_VAC1);
			VacStLog[VAC_ST_LOG_NUM_M1].fIG = GetVoltage(ADC_CH_IG);
			VacStLog[VAC_ST_LOG_NUM_M1].fTempe[0] = fTemperature[0];
			VacStLog[VAC_ST_LOG_NUM_M1].fTempe[1] = fTemperature[1]; //coil HS
			VacStLog[VAC_ST_LOG_NUM_M1].fTempe[2] = (float)nTurboTemperature;
			VacStLog[VAC_ST_LOG_NUM_M1].fTempe[3] = fTemperature[3]; //power HS
			bVacStateReactivate = 0;
		} //if (SysParam.bNextVacuumState != SysParam.bVacuumState)
		//------------------------------------------
		//processed per 2 seconds
		//------------------------------------------
		if (nTimeTick2Hz % CHECK_PER_2SEC == 0) {
			GetVacuumChangeRate(); //shift data per 2 seconds
			// reset timeout if vacuum is lowering down quickly
			// if not change quickly, nFastDropTimeout = 0
			if (nFastDropTimeout > 0) nFastDropTimeout--;
			//NV_Settings.fLowChangeRate is negative
			if ((SysParam.bVacuumState == VAC_ST_LO_WAIT) && (fVACChangeRate[1] < NV_Settings.fLowChangeRate)) {
				nVacTimeout = NV_Settings.sLowWaitTimeout;
				nFastDropTimeout = FAST_DROP_TIMEOUT;
				if (bDebug == 10) printf("LO_WAIT:FAST DROP!\n"); //reset timeout
			}
			else if ((SysParam.bVacuumState == VAC_ST_HI_WAIT) && (fVACChangeRate[1] < NV_Settings.fHighChangeRate)) {
				nVacTimeout = NV_Settings.sHighWaitTimeout; //reset timeout
				nFastDropTimeout = FAST_DROP_TIMEOUT;
				if (bDebug == 10) printf("HI_WAIT:FAST DROP!\n"); //reset timeout
			}
		}
		//------------------------------------------
		// processed per second
		//------------------------------------------
		if (nTimeTick2Hz % CHECK_PER_SEC == 0) {
			IncVACCounter();
			if (nVacTimeout > 0) {
				if (bDebug == 10) { //debug vac change rate
					//do nothing
				}
				else if ((nVacTimeout % 60) == 0)
					printf("\nTIMEOUT=%03d", nVacTimeout);
				else if ((nVacTimeout % 6) == 0)
					printf("*");
				nVacTimeout--;
				if (nVacTimeout == 0)
					printf("\n");
			}
			SysParam.sVacTimeout = nVacTimeout; //unit sec
			SysParam.sTPGVTimeout = (short)nTPGVTimeout;
			if ((nIdleTimeout > 0) && (NV_Settings.nIdleTimeout > 0)) {// 2 minutes
				nIdleTimeout--;
#if EN_IDLE_TIMEOUT == 1
				if (nIdleTimeout == 0) { // 2 minutes
					SetHVSwitch(0);
					SetObjOn(0); //turn off obj
					//SetVOPPower(0);
					bScanAbort = 1;
					printf("idle timeout, shutdown system\n");
					//SaveErrorMessage("IDLE_TIMEOUT", 1);
				}
#endif
			}
			if (bDebug == 10) {
				//check vacuum status change rate
				printf("VAC_RATE=%.4f,%.4f,TMO=%d,TP=%d\n", fVACChangeRate[0], fVACChangeRate[1],
						nVacTimeout, nTurboRotationSpeed);
				//printf("VAC_LEVEL=%.4f,%.4f\n", fVACLevel[0], fVACLevel[1]);
				//printf("ADC=%.4f,%.4f\n", fADCVal[0], fADCVal[1]);
			}
		}
		//bSerDebug=1, debug serial interface
		if (bSerDebug == 0) { //1:test SI:X:string
			if (nTimeTick2Hz % 4 == 1) { //execute once per two seconds
				GetTurboRotationSpeed(1);
			}
			else if (nTimeTick2Hz % 4 == 3) //execute once per two seconds
				GetTurboMotorTemp(1);
		}
	} //while(1)
}
#else //GAUGE_OP_MODE == 2, use gauge 1(PG) and gauge 3(IG)
//
void InitIPLeak(void)
{
	int i = 0;
	for (i = 0; i < CHECK_VOLT_NUM; i++) {
		bIPLeakDone[i] = 0;
	}
	nIPLeakNdx = 0;
	nIPLeakTimeout = 0;
	bIPLeakAllDone = 0;
}

//execute CheckIPLeak() once per second
void CheckIPLeak(void)
{
	float fV;
	fV = GetIonGauge();
	if (nIPLeakNdx >= NV_Settings.sIPLeakNum) { //CHECK_VOLT_NUM) //all processes complete
		bIPLeakAllDone = 1;
		return;
	}
	if (bIPLeakDone[nIPLeakNdx] != 0)
		goto IPLeakCheck;
	if (fV < NV_Settings.fIPLeakCheckV[nIPLeakNdx]) //vacuum better, fV smaller
		goto IPLeakCheck;
	return;
IPLeakCheck:
	if (nIPLeakTimeout > 0) {
		if (bDebug == 2) {
			if (nIPLeakTimeout % 10 == 0) printf("*");
		}
		nIPLeakTimeout--;
		return;
	}
	if (bIPLeakDone[nIPLeakNdx] == 2) {
		bIPLeakDone[nIPLeakNdx] = 3; //complete
		nIPLeakNdx++;
		if (nIPLeakNdx >= NV_Settings.sIPLeakNum) //all processes complete
			bIPLeakAllDone = 1;
		nIPLeakTimeout = 0;
		printf("IP_LEAK:3,IG=%.3f,IP=0,%d/%d done", fV, nIPLeakNdx, NV_Settings.sIPLeakNum);
	}
	else if (bIPLeakDone[nIPLeakNdx] == 1) {
		nIPLeakTimeout = NV_Settings.sIPLeakOffTime[nIPLeakNdx];
		SetIonPump(0);
		bIPLeakDone[nIPLeakNdx] = 2;
		printf("IP_LEAK:2,IG=%.3f,IP=0,TMO=%d", fV, nIPLeakTimeout);
	}
	else if (bIPLeakDone[nIPLeakNdx] == 0) {
		nIPLeakTimeout = NV_Settings.sIPLeakOnTime[nIPLeakNdx];
		SetIonPump(1);
		bIPLeakDone[nIPLeakNdx] = 1;
		printf("IP_LEAK:1,IG=%.3f,IP=1,TMO=%d", fV, nIPLeakTimeout);
	}
}
//0:chamber, 1:gun, 2:reserve, 3: ion gauge
// nNdx = 0,1,2 //pirani gauge
// nNdx = 3, ion gauge
// return 1: vacuum below threshold, OK, 0: vacuum NOT OK
int GetGaugeStatus(int nNdx, char *szValue)
{
	//WORD wV;
	char szStatus[64] = "NONE";
	static int nPreCheck[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0};
	static int nCurrentState[VAC_GAUGE_NUM_MAX] = {VAC_NONE, VAC_NONE, VAC_NONE, VAC_NONE};
	int nCheck[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0};
	static int nOK[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0};
	static int nBNOK[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0};
	float fV = 0;
	//---------------------------------------------------------
	// vacuum low   --> Vc hi --> AOx low      --> VDOx low
	// gauge broken --> Vc hi --> AOx negative --> VDOx low
	//---------------------------------------------------------
	if (szValue != NULL) sprintf(szValue, "VAC%d:OK,", nNdx);
	if (nNdx >= VAC_GAUGE_NUM_MAX) //invalid index
		goto StatusNOK;
//#if G0_ALWAYS_OK == 1
//	if (nNdx == 0) goto StatusOK;
//#endif
	//----------------------------------
	//wV = GetDigitalInput();
	if (nNdx == 0) { //parani gauge
		fV = GetVoltage(ADC_CH_VAC0 + nNdx);
		nCheck[nNdx] = (bVACBypassOK[nNdx] != VAC_NONE) ? bVACBypassOK[nNdx] :
			(fV > NV_Settings.fPGLOTh) ? VAC_AIR :
			(fV > NV_Settings.fPGHITh) ? VAC_LO : VAC_HI;
	}
	else if (nNdx == 1) { //pirani gauge
		fV = GetVoltage(ADC_CH_VAC0 + nNdx); //AIR,LO_HI
		nCheck[nNdx] = (bVACBypassOK[nNdx] != VAC_NONE) ? bVACBypassOK[nNdx] :
			(fV > NV_Settings.fPGLOTh) ? VAC_AIR :
			(fV > NV_Settings.fPGHITh) ? VAC_LO : VAC_HI;
	}
	else if (nNdx == 2) {
		//nCheck[nNdx] = ((wV & DI_VAC_GAUGE2) == 0) ? VAC_OK : VAC_NOT_OK;
		fV = GetVoltage(ADC_CH_VAC0 + nNdx);
		nCheck[nNdx] = (bVACBypassOK[nNdx] != VAC_NONE) ? bVACBypassOK[nNdx] :
			(fV > NV_Settings.fPGLOTh) ? VAC_AIR :
			(fV > NV_Settings.fPGHITh) ? VAC_LO : VAC_HI;
	}
	else if (nNdx == 3) {
		fV = GetIonGauge(); //1.268 V/decade
		nCheck[nNdx] = (bVACBypassOK[nNdx] != VAC_NONE) ? bVACBypassOK[nNdx] :
			(fV > NV_Settings.fIGLOTh) ? VAC_AIR :
			(fV > NV_Settings.fIGHITh) ? VAC_LO :
			(fV > NV_Settings.fIGMHTh) ? VAC_HI :
			(fV > NV_Settings.fIGUHTh) ? VAC_MH : VAC_UH;
	}
	fVACLevel[nNdx] = fV;
	if ((fV < 0) && (bVACBypassOK[nNdx] == VAC_NONE)) { //negative voltage, check if check broken, for PG only
		nBNOK[nNdx]++;
		if (nBNOK[nNdx] < NV_Settings.sVACNOKTh[1])
			goto StatusOK;
		if (szValue != NULL) {
			sprintf(szValue, "VAC%d:NOK,BROKEN,", nNdx);
			if (SysParam.bScanning) printf("%s\n", szValue);
		}
		return VAC_BROKEN;
	}
	else nBNOK[nNdx] = 0;
	//
	if (nCheck[nNdx] == nPreCheck[nNdx]) { //state not changed
		nOK[nNdx]++;
		if (nOK[nNdx] > NV_Settings.sVACNOKTh[0]) {
			//change state
			nCurrentState[nNdx] = nCheck[nNdx];
			nOK[nNdx] = 0;
		}
		goto StatusOK;
	}
	else {
		nOK[nNdx] = 0;
		//nBNOK[nNdx] = 0;
	}
	nPreCheck[nNdx] = nCheck[nNdx];
	goto StatusOK;
StatusNOK:
	if (szValue != NULL) sprintf(szValue, "VAC%d:NOK,", nNdx);
	return nCurrentState[nNdx];
StatusOK:
	if (szValue != NULL) {
		if (nCurrentState[nNdx] == VAC_AIR)
			sprintf(szStatus, "AIR");
		else if (nCurrentState[nNdx] == VAC_LO)
			sprintf(szStatus, "LO");
		else if (nCurrentState[nNdx] == VAC_HI)
			sprintf(szStatus, "HI");
		else if (nCurrentState[nNdx] == VAC_MH)
			sprintf(szStatus, "MH");
		else if (nCurrentState[nNdx] == VAC_UH)
			sprintf(szStatus, "UH");
		sprintf(szValue, "VAC%d:%s,", nNdx, szStatus);
	}
	return nCurrentState[nNdx];
}

void VacuumStateTask(void * pd)
{
	int nTimeTick2Hz = 0;
	int nTick = 0;
	BYTE bTick_1Hz = 0;
	BYTE bTick_2Hz = 0;
	char szTemp[128];
	int i;
	int nTempOK[TEMP_SENSOR_NUM_MAX] = {SUCCESS, SUCCESS, SUCCESS, SUCCESS, SUCCESS};
	int nTempFailCount[TEMP_SENSOR_NUM_MAX] = {0, 0, 0, 0, 0};
	int nTempIsOK = SUCCESS;
	int nVAC1IsOK = SUCCESS;
	int nFastDropTimeout = FAST_DROP_TIMEOUT;
	int nBroken = 0;
	short int sGaugeStatus[VAC_GAUGE_NUM_MAX] = {VAC_NONE, VAC_NONE, VAC_NONE, VAC_NONE};
	int nV;
	int nVentTPGV = 0;
	int nErrorCount = 0;
	float fV = 0, fV1;
	char szValue[64], szBuffer[128];
	int nCommand;
	//
	//initial state, wait until I/O initialization is complete
	OSTimeDly(TICKS_PER_SECOND);
	//
	InitIPLeak();
	SetHVPower(0);
	SetHVSwitch(0);
	//SetIonPump(0);
	SetScrollPump(0);	//turn off scroll pump power
	SetTurboPump(1);	//turn on turbo pump power
	StopTurbo(0); //ENVENT=0, GV0 close
	SetIonGauge(1);
	SetGateValve(0, VALVE_CLOSE); 		//GV0, chamber
	//SetGateValve(1, VALVE_CLOSE);  	//GV1, IP to turbo (AIR)
	//SetGateValve(2, VALVE_CLOSE);		//GV2, motor gate valve (between pipe and chamber)
	SetGateValve(3, VALVE_CLOSE);		//GV3, turbo pump
	//----------------------------------------------------
	SysParam.bVacuumState = VAC_ST_NONE;
	SysParam.bNextVacuumState = VAC_ST_INIT;
	//
	while (1)
	{	//
		//gauge0:chamber, gauge1:gun (IG)
		//
		OSTimeDly(TICKS_PER_SECOND/4); //wait 1/4 sec
		if (bInitOK == 0)
			continue;
		//
		nTick++;
		bTick_1Hz = ((nTick % 4) < 2) ? 1 : 0; 	//1,1,0,0,1,1,0,0
		bTick_2Hz = ((nTick % 2) == 0) ? 1 : 0;	//0,1,0,1,0,1,0,1
		if (bTick_2Hz == 0)
			continue;
		nTimeTick2Hz++; 		//increase every 1/2 seconds
		if (nTimeTick2Hz % 2 == 0) {
			lVacTotalTime++;
			CheckAutoVAC();
		}
		//the following code execute once per second
		//if ((bEnableIO == 1) && (bDebug < 22))
		//	GetDigitalInput(1); //execute once per second
		//
		if (SysParam.bSimuVAC == 1) { //for simulation
			SysParam.sGaugeStatus[0] = GetGaugeStatus(0, NULL);
			SysParam.sGaugeStatus[2] = GetGaugeStatus(2, NULL);
		}
		SysParam.sGaugeStatus[1] = GetGaugeStatus(1, NULL);
		//
		SysParam.sGaugeStatus[3] = GetGaugeStatus(3, NULL);
		//-----------------------------------------------------------
		// check temperature
		//-----------------------------------------------------------
		for (i = 0; i < TEMP_SENSOR_NUM; i++) {
			nTempOK[i] = CheckTemperature(i);
			if (nTempOK[i] == ERROR_FAIL) {
				if (nTempFailCount[i] <= 3)
					nTempFailCount[i]++;
			}
			else { //nTempOK[i] == SUCCESS
				nTempFailCount[i] = 0;
			}
			if (nTempFailCount[i] >= 3) {
				nTempIsOK = ERROR_FAIL;
				break; //break for loop
			}
			else
				nTempIsOK = SUCCESS;
		}
		//--------------------------------------------------------
		// abnormal temperature
		//--------------------------------------------------------
		if (nTempIsOK == ERROR_FAIL) {
			if ((SysParam.bVacuumState != VAC_ST_ERROR1) && (SysParam.bVacuumState != VAC_ST_ERROR2)) {
				for (i = 0; i < TEMP_SENSOR_NUM; i++) {
					if (nTempFailCount[i] != 0) {
						sprintf(szTemp, "TEMP_ERR=%d,%d,%.1f,\n", i, nTempFailCount[i], fTemperature[i]);
						SaveErrorMessage(szTemp, 1); //nPrint=1
					}
				}
			}
			if (SysParam.bVacuumState == VAC_ST_ERROR2) {
				//do nothing, stay at ERROR2
			}
			else if ((SysParam.sGaugeStatus[3] & VAC_M0U) != 0) {

				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (SysParam.bVacuumState != VAC_ST_ERROR1) {
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
		}
		//--------------------------------------------------------
		// abnormal VAC1 reading
		//--------------------------------------------------------
		if (SysParam.sGaugeStatus[1] == VAC_BROKEN) {
			if (nBroken < 3)
				nBroken++;
			if (nBroken >= 3) {
				nVAC1IsOK = ERROR_FAIL;
			}
			else nVAC1IsOK = SUCCESS;
		}
		else {
			nBroken = 0;
			nVAC1IsOK = SUCCESS;
		}
		if (nVAC1IsOK == ERROR_FAIL) {
			if ((SysParam.bVacuumState != VAC_ST_ERROR1) && (SysParam.bVacuumState != VAC_ST_ERROR2)) {
				sprintf(szTemp, "VAC1_BROKEN");
				SaveErrorMessage(szTemp, 1);
			}
			if (SysParam.bVacuumState == VAC_ST_ERROR2) {
				//do nothing, stay at ERROR2
			}
			else if ((SysParam.sGaugeStatus[3] & VAC_M0U) != 0) { //IG status
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
				goto NextState;
			}
			else if (SysParam.bVacuumState != VAC_ST_ERROR1) {
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
				goto NextState;
			}
		}
		//--------------------------------------------------------
		if (SysParam.bVacuumState == VAC_ST_INIT) {
			if ((nIGTimeout == 0) || (bChkTP == 0)) { //wait until IG is stable
				printf("check IG state...\r\n");
				fV = GetIonGauge();
				fV1 = GetPiraniGauge(1);
				//SHORT->1
				if ((SysParam.bPZTMoveX == 0) && (SysParam.bPZTMoveY == 1)) {
					printf("Go to INT2_LO_WAIT, IG=%.3f, PG=%.3f volts\n\r", fV, fV1);
					SysParam.bNextVacuumState = VAC_ST_INT2_LO_WAIT;
				}
				else if ((SysParam.sGaugeStatus[3] & VAC_M0U) != 0) { //MH or UH
					printf("Go to STANDBY0, IG=%.3f, PG=%.3f volts\n\r", fV, fV1);
					SysParam.bNextVacuumState = VAC_ST_STANDBY0;
				}
				//if ((SysParam.sGaugeStatus[3] & VAC_M0U) != 0) { //MH or UH
				//	printf("Go to STANDBY0, IG=%.3f, PG=%.3f volts\n\r", fV, fV1);
				//	SysParam.bNextVacuumState = VAC_ST_STANDBY0;
				//}
				else {
					printf("Go to AIR, IG=%.3f, PG=%.3f volts\n\r", fV, fV1);
					SysParam.bNextVacuumState = VAC_ST_AIR; //VAC_ST_AIR_WAIT;
				}
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_INT2_LO_WAIT) {
			if (nVacTimeout == 0) {
				SaveErrorMessage("INT2_LO_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			if ((SysParam.sGaugeStatus[1] & VAC_L0H) != 0) {
				fV = GetIonGauge();
				//if ((SysParam.sGaugeStatus[3] & VAC_H0M0U) != 0)
				if (fV < NV_Settings.fIGINT2Th)
					SysParam.bNextVacuumState = VAC_ST_INT2_HI_WAIT;
				else
					SysParam.bNextVacuumState = VAC_ST_INT_LO_WAIT;
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_INT2_HI_WAIT) {
			if (nVacTimeout == 0) {
				SaveErrorMessage("INT2_HI_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			if (IsTurboSpeedHighEnough() && ((SysParam.sGaugeStatus[1] & VAC_HI) != 0)) {
				SysParam.bNextVacuumState = VAC_ST_INT_HI_WAIT;
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_AIR) {
			if (nTPGVTimeout == 0) {
				SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
				printf("Close turbo pump gate valve\n\r");
			}
			if (nVacOn == 1) {
				nTPGVTimeout = 0;
				SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
				SysParam.bNextVacuumState = VAC_ST_INT_LO_WAIT;
			}
			if (nVacOff == 1) { //vent again
				nTPGVTimeout = 0;
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
			}
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_INT_LO_WAIT) {
			if (nVacTimeout == 0) {
				SaveErrorMessage("INT_LO_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			if (nVacOn == 1) {
				//PG=VAC_LO or VAC_HI
				//wait until IG is LO,HI,MH,UH
				if ((SysParam.sGaugeStatus[3] & VAC_L0H0M0U) != 0)
					SysParam.bNextVacuumState = VAC_ST_INT_HI_WAIT;
			}
			if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_INT_HI_WAIT) {
			if (nTimeTick2Hz % 2 == 0) //???
				CheckIPLeak();
			if (nVacTimeout == 0) {
				sprintf(szTemp, "INT_HI_WAIT_TIMEOUT,TPSP=%d,G1=%d,G3=%d",
					nTurboRotationSpeed, SysParam.sGaugeStatus[1], SysParam.sGaugeStatus[3]);
				SaveErrorMessage(szTemp, 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			//
			if ((nVacOn == 1) && (bIPLeakAllDone == 1)) { //bIPLeakAllDone
				if (IsTurboSpeedHighEnough() == 0) { //turbo speed too slow
					nErrorCount++;
					if (nErrorCount >= 4) {
						sprintf(szTemp, "INT_HI_WAIT_ERRTP,TPSP=%d,G1=%d,G3=%d",
							nTurboRotationSpeed, SysParam.sGaugeStatus[1], SysParam.sGaugeStatus[3]);
						SaveErrorMessage(szTemp, 1);
						SysParam.bNextVacuumState = VAC_ST_ERROR1;
					}
				}
				else if (SysParam.sGaugeStatus[3] & VAC_AIR) {
					nErrorCount++;
					if (nErrorCount >= 4) {
						sprintf(szTemp, "INT_HI_WAIT_ERRG3,TPSP=%d,G1=%d,G3=%d",
							nTurboRotationSpeed, SysParam.sGaugeStatus[1], SysParam.sGaugeStatus[3]);
						SaveErrorMessage(szTemp, 1);
						SysParam.bNextVacuumState = VAC_ST_ERROR1;
					}
				}
				else nErrorCount = 0;
				//PG=VAC_HI
				if (IsTurboSpeedHighEnough() && ((SysParam.sGaugeStatus[3] & VAC_H0M0U) != 0))
					if (SysParam.sGaugeStatus[1] == VAC_HI) {
						SysParam.bNextVacuumState = VAC_ST_INT_HI;
					}
			}
			if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
		}
		else if (SysParam.bVacuumState == VAC_ST_INT_HI) {
			if ((SysParam.sGaugeStatus[3] & VAC_H0M0U) == 0) {
				SaveErrorMessage("INT_HI,G3!=HI_OR_MH_OR_UH", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			if (nVacOn == 1) //shutdown from LOW_WAIT state
				SysParam.bNextVacuumState = VAC_ST_INT_UH_WAIT;
			if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_INT_MH_WAIT) { //skip this state
			//if (nTimeTick2Hz % 2 == 0) //???
			//	CheckIPLeak();
			if (nVacTimeout == 0) {
				sprintf(szTemp, "INT_MH_WAIT_TIMEOUT,TPSP=%d,G1=%d,G3=%d",
					nTurboRotationSpeed, SysParam.sGaugeStatus[1], SysParam.sGaugeStatus[3]);
				SaveErrorMessage(szTemp, 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			else if ((SysParam.sGaugeStatus[1] & VAC_L0H) == 0) {//not VAC_LO AND not VAC_HIGH
				SaveErrorMessage("INT_MH_WAIT,G1!=LO_OR_HI", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			else if (nVacOn == 1) {
				if ((SysParam.sGaugeStatus[3] & VAC_M0U) != 0) { //MH or UH
					SysParam.bNextVacuumState = VAC_ST_INT_UH_WAIT;
				}
			}
			else if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_INT_UH_WAIT) {
			//if (nTimeTick2Hz % 2 == 0) //???
			//	CheckIPLeak();
			if (nVacTimeout == 0) {
				sprintf(szTemp, "INT_UH_WAIT_TIMEOUT,TPSP=%d,G1=%d,G3=%d",
					nTurboRotationSpeed, SysParam.sGaugeStatus[1], SysParam.sGaugeStatus[3]);
				SaveErrorMessage(szTemp, 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			else if ((SysParam.sGaugeStatus[1] & VAC_L0H) == 0) {//not VAC_LO AND not VAC_HIGH
				SaveErrorMessage("INT_UH_WAIT,G1!=LO_OR_HI", 1);
				if (nErrorCount < ERROR_COUNT_MAX)
					nErrorCount++;
				else {
					SysParam.bNextVacuumState = VAC_ST_ERROR1;
				}
			}
			else if (bChkTP && IsTurboSpeedLower()) {
				SaveErrorMessage("INT_UH_WAIT,TP_SPEED_LOW", 1);
				if (nErrorCount < ERROR_COUNT_MAX)
					nErrorCount++;
				else {
					SysParam.bNextVacuumState = VAC_ST_ERROR1;
				}
			}
			else if (nVacOn == 1) {
				if (SysParam.sGaugeStatus[3] == VAC_UH) { //MH or UH
					SysParam.bNextVacuumState = VAC_ST_UH;
				}
			}
			else if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
			else nErrorCount = 0;
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_UH) {
			if (SysParam.sGaugeStatus[1] != VAC_HI) { //not VAC_LO AND not VAC_HI
				SaveErrorMessage("UH,G1!=HI", 1);
				if (nErrorCount < ERROR_COUNT_MAX)
					nErrorCount++;
				else {
					SysParam.bNextVacuumState = VAC_ST_ERROR2;
					bErrorIP = 0;
				}
			}
			else if ((SysParam.sGaugeStatus[3] & VAC_M0U) == 0) { //operable under MH or UH
				SaveErrorMessage("UH,G3!=UH,MH", 1);
				if (nErrorCount < ERROR_COUNT_MAX)
					nErrorCount++;
				else {
					SysParam.bNextVacuumState = VAC_ST_ERROR2;
					bErrorIP = 1;
				}
			}
			else if (bChkTP && IsTurboSpeedLower()) {
				SaveErrorMessage("UH,TP_SPEED_LOW", 1);
				if (nErrorCount < ERROR_COUNT_MAX)
					nErrorCount++;
				else {
					SysParam.bNextVacuumState = VAC_ST_ERROR2;
					bErrorIP = 0;
				}
			}
			else if (nVacOff == 1) {
				if (NV_Settings.bEnableVent == 1) {
					VentToAir(VENT_DELAY);
					SysParam.bNextVacuumState = VAC_ST_VENT_WAIT;
				}
				else {
					VentToAir(VENT_NONE);
					SysParam.bNextVacuumState = VAC_ST_NOVENT_WAIT;
				}
			}
			else if (nVacOn == 1)
				nVacOn = 0;
			else nErrorCount = 0;
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_STANDBY0) {
			if (nVacOff == 1) { //go to AIR
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
			}
			else if ((SysParam.sGaugeStatus[3] & VAC_M0U) == 0) {
				SaveErrorMessage("STANDBY,G3!=MH_OR_UH", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (nVacOn == 1) { //go to READY
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT;
			}
			else if (nVacOff == 2) { //VACON:2, standby0 to standby1
				//if (SysParam.sGaugeStatus[1] == VAC_AIR) //PG
				//	VentToAir(VENT_DIRECT);
				//else
				//	VentToAir(VENT_DELAY);
				//VentToAir(VENT_DIRECT);
				SysParam.bNextVacuumState = VAC_ST_STANDBY_WAIT;
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_STANDBY_WAIT) { //go to STANDBY1
			/*if (IsTurboSpeedLowEnough()) {
				if (nVentTPGV > 0) { //STANDBY_WAIT, 0,29,28,27,26,...2,1,0
					if ((nTPGVTimeout % 20) == 18)
						SetGateValve(TP_GATE_VALVE, VALVE_OPEN); //open to air
					else if ((nTPGVTimeout % 20) == 16) {
						SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
						nVentTPGV--;
					}
				}
				else if ((SysParam.sGaugeStatus[1] & VAC_A0L) != 0) {
					SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
					VentToAir(VENT_NONE);
					SysParam.bNextVacuumState = VAC_ST_STANDBY1;
				}
			}*/
			/*if (nVentTPGV > 0) { //STANDBY_WAIT, 0,29,28,27,26,...2,1,0
				if ((nTPGVTimeout % 20) == 18)
					SetGateValve(0, VALVE_OPEN); //open to air
				else if ((nTPGVTimeout % 20) == 16) {
					SetGateValve(0, VALVE_CLOSE);
					nVentTPGV--;
				}
			}*/
			if (nTPGVTimeout <= 10) {
				if (nTPGVTimeout == 0) {
					SaveErrorMessage("STANDBY_WAIT_TIMEOUT:TPGV", 1);
					SysParam.bNextVacuumState = VAC_ST_ERROR2;
				}
				else if ((SysParam.sGaugeStatus[1] & VAC_A0L) != 0) {
					SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
					VentToAir(VENT_NONE);
					SysParam.bNextVacuumState = VAC_ST_STANDBY1;
				}
			}
			if (nVacTimeout == 0) {
				SaveErrorMessage("STANDBY_WAIT_TIMEOUT:VAC", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			if (nVacOn == 1) {
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT;
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_STANDBY1) {
			if (nTPGVTimeout == 0) { //wait until TPGV timeout
				SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
				printf("Close turbo pump gate valve\n\r");
			}
			if (nVacOff == 1) { //gun to AIR, VACON:0,ENVENT:1
				if (NV_Settings.bEnableVent == 1) {
					SetIonPump(0);
					nGotoAir = 2; //->HI --> GUN TO AIR
					nVacOn = 1;
					nVacOff = 0;
					SysParam.bNextVacuumState = VAC_ST_LO_WAIT;
					//SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
				}
				else { //go to STANDBY1 again
					if (SysParam.sGaugeStatus[1] == VAC_AIR)
						VentToAir(VENT_DIRECT);
					else
						VentToAir(VENT_DELAY);
					SysParam.bNextVacuumState = VAC_ST_STANDBY_WAIT;
				}
			}
			else if ((SysParam.sGaugeStatus[3] & VAC_M0U) == 0) {
				SaveErrorMessage("STANDBY1,G3!=MH_OR_UH", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (nVacOn == 1) { //go to READY
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT;
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_GUN_TO_AIR) {
			if (nVacTimeout == 0)
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT;
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_ERROR1) { //INT_ state ERROR
			if (nVacOn == 1) {
				if (nIGTimeout == 0) { //wait until IG is stable
					SetIonGauge(1); //turn on IG
					nIGTimeout = NV_Settings.sIGWaitTimeout;
					printf("IG on, wait %d s until IG is stable\r\n", nIGTimeout);
				}
				else if (nIGTimeout == 1) { //wait until IG is stable
					SysParam.bNextVacuumState = VAC_ST_INT_LO_WAIT;
				}
			}
			if (nVacOff == 1) { //go to AIR
				SysParam.bNextVacuumState = VAC_ST_AIR_WAIT; //go to AIR
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_ERROR2) {
			//??? VACON:1 go to UH_WAIT
			//??? VACON:0 and ENVENT=1, go to INT_LO_WAIT
			if (nVacOn == 1) { //go to
				if (nIGTimeout == 0) { //wait until IG is stable
					SetIonGauge(1);
					nIGTimeout = NV_Settings.sIGWaitTimeout;
					printf("IG on, wait %d s until IG is stable. Go to UH_WAIT.\r\n", nIGTimeout);
				}
				else if (nIGTimeout == 1) { //wait until IG is stable
					if ((SysParam.sGaugeStatus[3] & VAC_H0M0U) != 0)
						SysParam.bNextVacuumState = VAC_ST_CHECK_UH_WAIT;
					else { //VAC_L0H, stay at ERROR2
						nVacOn = 0; //stop the process if vacuum is not OK //???
						//SysParam.bNextVacuumState = VAC_ST_INT_LO_WAIT;
					}
				}
			}
			if (nVacOff == 1) {
				if (nGotoAir == 1) //VACON:3, go to AIR
					SysParam.bNextVacuumState = VAC_ST_INT_LO_WAIT;
				else if (NV_Settings.bEnableVent == 1) //VACON:0+ENVENT:1, go to STANDBY1
					SysParam.bNextVacuumState = VAC_ST_STANDBY_WAIT;
				/*if (nIGTimeout == 0) { //wait until IG is stable
					SetIonGauge(1);
					nIGTimeout = NV_Settings.sIGWaitTimeout;
					printf("IG on, wait %d s until IG is stable. Go to STANDBY.\r\n", nIGTimeout);
				}
				else if (nIGTimeout == 1) { //wait until IG is stable
					SysParam.bNextVacuumState = VAC_ST_STANDBY0; //go to AIR
				}*/
			}
		}
		else if (SysParam.bVacuumState == VAC_ST_CHECK_UH_WAIT) {
			if (nVacTimeout == 0) {
				SaveErrorMessage("CHECK_UH_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if ((SysParam.sGaugeStatus[3] & VAC_H0M0U) != 0) { //go to ready
				SysParam.bNextVacuumState = VAC_ST_STANDBY0;
			}
			else if (nVacOff == 1) //go to ERROR2
				SysParam.bNextVacuumState = VAC_ST_ERROR2; //go to AIR
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_AIR_WAIT) { //go to AIR
			//0:chamber, 1:gun
			/*
			if (IsTurboSpeedLowEnough()) {
				if (NV_Settings.bEnableVent == 1) {
					if ((SysParam.sGaugeStatus[3] != VAC_AIR) && (nVentTPGV > 0)) {
						SetGateValve(TP_GATE_VALVE, VALVE_OPEN); //open to air
						OSTimeDly(TICKS_PER_SECOND * 5);
						SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
						OSTimeDly(TICKS_PER_SECOND * 2);
						if (nVentTPGV > 0) nVentTPGV--;
					}
					if (SysParam.sGaugeStatus[3] == VAC_AIR)  { //ion gauge status
						SysParam.bNextVacuumState = VAC_ST_AIR;
					}
				}
				else { //ENVENT=0
					if (SysParam.sGaugeStatus[1] == VAC_AIR) //PG
						SysParam.bNextVacuumState = VAC_ST_AIR;
				}
			}*/
			if (NV_Settings.bEnableVent == 1) {
				if (SysParam.sGaugeStatus[3] == VAC_AIR)  { //ion gauge status
					SysParam.bNextVacuumState = VAC_ST_AIR;
				}
			}
			else { //ENVENT=0
				if (SysParam.sGaugeStatus[1] == VAC_AIR) //PG
					SysParam.bNextVacuumState = VAC_ST_AIR;
			}
			if (nTPGVTimeout == 0) {
				SaveErrorMessage("AIR_WAIT_TIMEOUT:TPGV", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			if (nVacTimeout == 0) {
				SaveErrorMessage("AIR_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR1;
			}
			else if (nVacOn == 1) {
				SysParam.bNextVacuumState = VAC_ST_INT_LO_WAIT; //go to INT_LOW
			}
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_LO_WAIT) { //go to LOW
			//turbo pump speed is zero if moving from AIR to LOW
			//turbo pump speed is not zero if moving from LOW to AIR
			//0:chamber, 1:gun
			if ((SysParam.sGaugeStatus[3] & VAC_M0U) == 0) { //???
				SaveErrorMessage("LO_WAIT,G3!=MH_OR_UH", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			if (nVacOn == 1) {
				if (IsChamberClose(NULL) == DOOR_OPEN)
					nChamberCheck++;
				else nChamberCheck = 0;
				if (nChamberCheck > 3) {
					SaveErrorMessage("LO_WAIT,CHAM_OPEN", 1);
					SysParam.bNextVacuumState = VAC_ST_ERROR2;
				}
			}
			if ((nVacOn == 1) && (nFastDropTimeout == 0)) {
				//if nFastDropTimeout=0, VAC isn't decreasing rapidly
				if (SysParam.sGaugeStatus[1] == VAC_HI) //go to HI_WAIT directly
					SysParam.bNextVacuumState = VAC_ST_HI_WAIT;
				else if (SysParam.sGaugeStatus[1] == VAC_LO) {
					//wait at least 20 seconds
					if (nVacTimeout < NV_Settings.sLowWaitTimeout - 6)
						SysParam.bNextVacuumState = VAC_ST_HI_WAIT;
				}
			}
			else if (nVacTimeout == 0) {
				SaveErrorMessage("LO_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (nVacOff == 1) //shutdown from LOW_WAIT state
				SysParam.bNextVacuumState = VAC_ST_STANDBY0; //???
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_HI_WAIT) {
			if ((SysParam.sGaugeStatus[3] & VAC_M0U) == 0) { //???
				SaveErrorMessage("HI_WAIT,G3!=MH_OR_UH", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			if (IsTurboSpeedHighEnough() && ((SysParam.sGaugeStatus[3] & VAC_H0M0U) != 0)) {
				//SysParam.bNextVacuumState = VAC_ST_MH_WAIT;
				if (SysParam.sGaugeStatus[1] == VAC_HI) { //go to UH_WAIT directly
					if (nGotoAir == 2) { //STANDBY1 -> AIR
						SysParam.bNextVacuumState = VAC_ST_GUN_TO_AIR;
					}
					else
						SysParam.bNextVacuumState = VAC_ST_UH_WAIT;
					nGotoAir = 0;
				}
			}
			else if (nVacTimeout == 0) {
				SaveErrorMessage("HI_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_STANDBY0;
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_MH_WAIT) {
			if (SysParam.sGaugeStatus[1] != VAC_HI) { //must be HI
				SaveErrorMessage("MH_WAIT,G1!=HI", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if ((SysParam.sGaugeStatus[3] & VAC_M0U) != 0) {
				SysParam.bNextVacuumState = VAC_ST_UH_WAIT;
			}
			if (nVacTimeout == 0) {
				SaveErrorMessage("MH_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_STANDBY0; //???
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_UH_WAIT) {
			if (SysParam.sGaugeStatus[1] != VAC_HI) { //must be HI
				SaveErrorMessage("UH_WAIT,G1!=HI", 1);
				if (nErrorCount < ERROR_COUNT_MAX)
					nErrorCount++;
				else {
					SysParam.bNextVacuumState = VAC_ST_ERROR2;
					bErrorIP = 0;
				}
			}
			else if (bChkTP && IsTurboSpeedLower()) {
				SaveErrorMessage("UH_WAIT,TP_SPEED_LOW", 1);
				if (nErrorCount < ERROR_COUNT_MAX)
					nErrorCount++;
				else {
					SysParam.bNextVacuumState = VAC_ST_ERROR2;
					bErrorIP = 0;
				}
			}
			else if ((SysParam.sGaugeStatus[3] & VAC_A0L) != 0) {
				if (nErrorCount < ERROR_COUNT_MAX)
					nErrorCount++;
				else {
					SysParam.bNextVacuumState = VAC_ST_ERROR2; //???
					bErrorIP = 1;
				}
			}
			else if (SysParam.sGaugeStatus[3] == VAC_UH) {
				SysParam.bNextVacuumState = VAC_ST_UH;
			}
			else nErrorCount = 0;
			//if ((sGaugeStatus[3] = VAC_MH) && SysParam.sGaugeStatus[3];
			if (nVacTimeout == 0) {
				SaveErrorMessage("UH_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (nVacOff == 1)
				SysParam.bNextVacuumState = VAC_ST_STANDBY0; //???
		}
		//--------------------------------------------------------
		else if (SysParam.bVacuumState == VAC_ST_NOVENT_WAIT) {
			if (nVacTimeout == 0) {
				SaveErrorMessage("NOVENT_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (nVentTimeout == 0) { //wait GV2 motion
				SysParam.bNextVacuumState = VAC_ST_STANDBY0;
			}
			if (nVacOn == 1)
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT;
		}
		else if (SysParam.bVacuumState == VAC_ST_VENT_WAIT) {
			if (nVacTimeout == 0) {
				SaveErrorMessage("VENT_WAIT_TIMEOUT", 1);
				SysParam.bNextVacuumState = VAC_ST_ERROR2;
			}
			else if (nVentTimeout == 0) {
				SetScrollPump(0); //close again
				SetGateValve(TP_GATE_VALVE, VALVE_CLOSE);
				SysParam.bNextVacuumState = VAC_ST_STANDBY_WAIT;
			}
			if (nVacOn == 1)
				SysParam.bNextVacuumState = VAC_ST_LO_WAIT;
		}
		//-------------------------------------------------------------------------
		// next vacuum state process
		//-------------------------------------------------------------------------
NextState:
		sGaugeStatus[1] = SysParam.sGaugeStatus[1];
		sGaugeStatus[3] = SysParam.sGaugeStatus[3];
		//-------------------------------------------------------------------------
		if (SysParam.bNextVacuumState != SysParam.bVacuumState) {
			//bLEDState[LEDG] = ST_LED_SOLID;
			SetLEDState(LEDG, ST_LED_SOLID);
			switch (SysParam.bNextVacuumState)
			{
				case VAC_ST_GUN_TO_AIR:
					nVacOff = 1;
					nVacOn = 0;
					SetHVSwitch(0);
					SetIonPump(0);
					SetGateValve(2, VALVE_CLOSE);
					OSTimeDly(TICKS_PER_SECOND);
					SetGateValve(1, VALVE_OPEN);
					NV_Settings.bEnableVent = 1;
					StopTurbo(1);
					nVacTimeout = 20;
					break;
				case VAC_ST_AIR_WAIT:
					//StopTurbo(1); //1: enable venting
					bScanAbort = 1;
					SetHVSwitch(0);
					SetIonGauge(1); //G3=CL
					SetIonPump(0);
					OSTimeDly(TICKS_PER_SECOND);
					VentingProcess();
					SetGateValve(0, VALVE_OPEN);
					nV = (NV_Settings.bEnableVent == 1) ? VALVE_OPEN : VALVE_CLOSE;
					SetGateValve(1, nV);
					//SetGateValve(2, nV);	//motor gate valve
					SetGateValve(2, VALVE_CLOSE);	//motor gate valve
					SetGateValve(3, nV); 	//TP gate vale
					nVacTimeout = NV_Settings.sAirWaitTimeout;
					nTPGVTimeout = NV_Settings.sTPGVTimeout;
					//bLEDState[LEDB] = ST_LED_BLINK_1HZ;
					SetLEDState(LEDB, ST_LED_BLINK_1HZ);
					nVentTPGV = TPGV_INIT; //3;
					break;
				case VAC_ST_AIR:
					nVacOff = 0;
					nReadyCount = 0;
					SetHVSwitch(0);
#if USE_GLOBAL_IG == 0
					SetIonGauge(0);	//close ion gauge, ???
#endif
					SetGateValve(0, VALVE_CLOSE);
					nV = (NV_Settings.bEnableVent == 1) ? VALVE_OPEN : VALVE_CLOSE;
					SetGateValve(1, nV);
					SetGateValve(2, VALVE_CLOSE);
					if (nTPGVTimeout == -1)
						SetGateValve(3, VALVE_CLOSE);
					//bLEDState[LEDB] = ST_LED_DARK;
					SetLEDState(LEDB, ST_LED_DARK);
					nVacTimeout = -1;
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime);
					break;
				case VAC_ST_STANDBY0:
				case VAC_ST_STANDBY1:
					//stop VAC_OFF and VAC_ON process
					ClearVacuumParameters(0); //nVacOn=nVacOff=0
					nReadyCount = 0;
					SetHVSwitch(0);
					SetScrollPump(0);
					if (NV_Settings.bEnableVent == 1) {
						//StopTurbo(1); //1:vent, go to STANDBY1
						PushTurboCommand(TP_CMD_STOP_VENT);
						SysParam.bStandby = 1;
					}
					else {
						//StopTurbo(0); //0:no vent, go to STANDBY0
						PushTurboCommand(TP_CMD_STOP_NO_VENT);
						SysParam.bStandby = 0;
					}
					SetIonGauge(1);
					SetIonPump(1);
					SetGateValve(0, VALVE_CLOSE);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					if (SysParam.bNextVacuumState == VAC_ST_STANDBY0) //STANDBY0
						SetGateValve(3, VALVE_CLOSE);
					else if (nTPGVTimeout == -1)
						SetGateValve(3, VALVE_CLOSE);
					//bLEDState[LEDB] = ST_LED_DARK;
					SetLEDState(LEDB, ST_LED_DARK);
					SetLEDState(LEDY, ST_LED_DARK);
					nVacTimeout = -1;
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime);
					break;
				case VAC_ST_INT_LO_WAIT:			//go to LOW
					SetHVSwitch(0);
					SetIonGauge(1);
					SetIonPump(0);
					SetGateValve(0, VALVE_CLOSE);  //scroll (0)
					OSTimeDly(TICKS_PER_SECOND / 2);
					SetScrollPump(1);
					OSTimeDly(TICKS_PER_SECOND * 3);
					SetGateValve(0, VALVE_OPEN);
					SetGateValve(1, VALVE_OPEN);
					SetGateValve(2, VALVE_OPEN);
					SetGateValve(3, VALVE_CLOSE);
					StopTurbo(0); //ENVENT=0
					OSTimeDly(TICKS_PER_SECOND);
					nFastDropTimeout = FAST_DROP_TIMEOUT;
					//bLEDState[LEDB] = ST_LED_BLINK_1P4;
					SetLEDState(LEDB, ST_LED_BLINK_1P4);
					nTPGVTimeout = -1;
					nVacTimeout = NV_Settings.sLowWaitTimeout;
					nIGTimeout = 0;
					break;
				case VAC_ST_INT_HI_WAIT:
					StopTurbo(0); //stop and then start again
					SetHVSwitch(0);
					SetIonGauge(1);
					SetGateValve(0, VALVE_CLOSE);
					OSTimeDly(TICKS_PER_SECOND / 2);
					SetScrollPump(1);
					OSTimeDly(TICKS_PER_SECOND * 2);
					SetGateValve(0, VALVE_OPEN);
					SetGateValve(1, VALVE_OPEN);
					SetGateValve(2, VALVE_OPEN);
					SetGateValve(3, VALVE_CLOSE);
					//wait at least 2 seconds after StopTurbo()
					StartTurbo();
					nFastDropTimeout = FAST_DROP_TIMEOUT;
					//bLEDState[LEDB] = ST_LED_BLINK_2P4;
					SetLEDState(LEDB, ST_LED_BLINK_2P4);
					nVacTimeout = NV_Settings.lHighWaitTimeout;
					InitIPLeak();
					nIGTimeout = 0;
					break;
				case VAC_ST_INT2_LO_WAIT:			//go to LOW
					SetHVSwitch(0);
					SetIonGauge(1);
					SetIonPump(0);
					SetGateValve(0, VALVE_CLOSE);  //scroll (0)
					OSTimeDly(TICKS_PER_SECOND / 2);
					SetScrollPump(1);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					StopTurbo(0); //ENVENT=0
					OSTimeDly(TICKS_PER_SECOND);
					nFastDropTimeout = FAST_DROP_TIMEOUT;
					//bLEDState[LEDB] = ST_LED_BLINK_1P4;
					SetLEDState(LEDB, ST_LED_BLINK_1P4);
					nTPGVTimeout = -1;
					nVacTimeout = NV_Settings.sLowWaitTimeout;
					nIGTimeout = 0;
					break;
				case VAC_ST_INT2_HI_WAIT:
					SetGateValve(0, VALVE_CLOSE);
					OSTimeDly(TICKS_PER_SECOND / 2);
					SetScrollPump(1);
					OSTimeDly(TICKS_PER_SECOND * 2);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					StartTurbo();
					nFastDropTimeout = FAST_DROP_TIMEOUT;
					SetLEDState(LEDB, ST_LED_BLINK_2P4);
					nVacTimeout = NV_Settings.sLowWaitTimeout;
					nIGTimeout = 0;
					break;
				case VAC_ST_INT_HI:
					ClearVacuumParameters(0); //nVacOn=nVacOff=0
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime);
					break;
				case VAC_ST_INT_MH_WAIT:
					SetHVSwitch(0);
					SetGateValve(0, VALVE_OPEN);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					SetIonGauge(1);
					SetIonPump(1);
					//bLEDState[LEDB] = ST_LED_BLINK_3AP4;
					SetLEDState(LEDB, ST_LED_BLINK_3AP4);
					nVacTimeout = NV_Settings.sMHighWaitTimeout;
					nIGTimeout = 0;
					break;
				case VAC_ST_INT_UH_WAIT:
					SetHVSwitch(0);
					SetGateValve(0, VALVE_OPEN);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					SetIonGauge(1);
					OSTimeDly(TICKS_PER_SECOND / 2);
					SetIonPump(1);
					//bLEDState[LEDB] = ST_LED_BLINK_3P4;
					SetLEDState(LEDB, ST_LED_BLINK_3P4);
					nVacTimeout = NV_Settings.lUHighWaitTimeout;
					nIGTimeout = 0;
					nErrorCount = 0;
					bErrorIP = 0;
					break;
				case VAC_ST_VENT_WAIT:
					bScanAbort = 1;
					//StopTurbo(1); //1: enable venting
					PushTurboCommand(TP_CMD_STOP_VENT);
					//SetScrollPump(0);
					SetGateValve(0, VALVE_CLOSE);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					SetLEDState(LEDY, ST_LED_DARK);  //ready
					nVacTimeout = NV_Settings.sAirWaitTimeout;
					nIGTimeout = 0;
					nVentTimeout = NV_Settings.sVentWaitTimeout;
					break;
				case VAC_ST_NOVENT_WAIT:
					bScanAbort = 1;
					//StopTurbo(0); //0: no venting
					PushTurboCommand(TP_CMD_STOP_NO_VENT);
					SetGateValve(0, VALVE_CLOSE);
					OSTimeDly(TICKS_PER_SECOND/2);
					SetScrollPump(0);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					SetLEDState(LEDY, ST_LED_DARK); //ready
					nVacTimeout = 300;
					nIGTimeout = 0;
					nVentTimeout = NV_Settings.sVentWaitTimeout;
					break;
				case VAC_ST_STANDBY_WAIT:
					SetScrollPump(0);
					//StopTurbo(1); //go to VENT, STANDBY1
					PushTurboCommand(TP_CMD_STOP_VENT);
					SetGateValve(0, VALVE_OPEN);
					//SetGateValve(3, VALVE_OPEN); //=VentToAir(VENT_DIRECT):
					//SetGateValve(TP_GATE_VALVE, VALVE_OPEN); //=VentToAir(VENT_DIRECT):
					VentToAir(VENT_DELAY);
					EnableTurboVent(1);
					SysParam.bStandby = 1;
					nVacTimeout = NV_Settings.sStandbyWaitTimeout;
					nTPGVTimeout = NV_Settings.sTPGVTimeout;
					nVentTPGV = TPGV_INIT; //3;
					break;
				case VAC_ST_UH:	//=VAC_ST_READY:
					ClearVacuumParameters(0);
					SetGateValve(0, VALVE_OPEN);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					nVacTimeout = -1;
					//bLEDState[LEDB] = ST_LED_SOLID;
					SetLEDState(LEDB, ST_LED_SOLID);
					SetLEDState(LEDY, ST_LED_SOLID); //ready
					nIGTimeout = 0;
					nErrorCount = 0;
					bErrorIP = 0;
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime);
					break;
				case VAC_ST_CHECK_UH_WAIT:
					SetIonGauge(1);
					SetIonPump(1);
					SetGateValve(0, VALVE_CLOSE);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					nVacTimeout = NV_Settings.lUHighWaitTimeout;
					//bLEDState[LEDB] = ST_LED_BLINK_3P4;
					SetLEDState(LEDB, ST_LED_BLINK_3P4);
					nIGTimeout = 0;
					break;
				case VAC_ST_LO_WAIT:
					SetIonGauge(1); //0
					StopTurbo(0); //ENVENT=0
					SetGateValve(0, VALVE_CLOSE);
					OSTimeDly(TICKS_PER_SECOND / 2);
					SetScrollPump(1);
					OSTimeDly(TICKS_PER_SECOND * 3);
					SetGateValve(0, VALVE_OPEN);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					nVacTimeout = NV_Settings.sLowWaitTimeout;
					//bLEDState[LEDB] = ST_LED_BLINK_1P4;
					SetLEDState(LEDB, ST_LED_BLINK_1P4);
					nTPGVTimeout = -1;
					nIGTimeout = 0;
					break;
				case VAC_ST_HI_WAIT:
					StopTurbo(0); //stop and then start again
					SetIonGauge(1); //GLOBAL_IG
					SetGateValve(0, VALVE_OPEN);
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					//
					OSTimeDly(TICKS_PER_SECOND); //MUST!!!, important
					//delay at least 2 seconds after StopTurbo()
					StartTurbo();
					nVacTimeout = NV_Settings.sHighWaitTimeout;
					//bLEDState[LEDB] = ST_LED_BLINK_2P4;
					SetLEDState(LEDB, ST_LED_BLINK_2P4);
					nIGTimeout = 0;
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime);
					break;
				case VAC_ST_MH_WAIT: //HI to MH
					SetIonGauge(1);
					nVacTimeout = NV_Settings.sMHighWaitTimeout;
					//bLEDState[LEDB] = ST_LED_BLINK_3AP4;
					SetLEDState(LEDB, ST_LED_BLINK_3AP4);
					nIGTimeout = 0;
					break;
				case VAC_ST_UH_WAIT: //MH to UH
					SetIonGauge(1);
					nVacTimeout = NV_Settings.lUHighWaitTimeout;
					//bLEDState[LEDB] = ST_LED_BLINK_3P4;
					SetLEDState(LEDB, ST_LED_BLINK_3P4);
					nIGTimeout = 0;
					nErrorCount = 0;
					bErrorIP = 0;
					printf("\nVAC_TOT_TIME=%ld s\n", lVacTotalTime);
					break;
				case VAC_ST_ERROR1:		//low vacuum operation
					bScanAbort = 1;
					ClearVacuumParameters(1); //nVacOn=0,nVacOff=0,etc
#if USE_GLOBAL_IG == 0
					SetIonGauge(0);	//close ion gauge, ???
#endif
					SetIonPump(0);
					StopTurbo(1); //envent=1
					SetGateValve(0, VALVE_CLOSE);
					OSTimeDly(TICKS_PER_SECOND);
					//SetScrollPump(0);
					VentingProcess();
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					//bLEDState[LEDG] = ST_LED_BLINK_1HZ;
					//bLEDState[LEDB] = ST_LED_BLINK_1HZ;
					SetLEDState(LEDG, ST_LED_BLINK_1HZ);
					SetLEDState(LEDB, ST_LED_BLINK_1HZ);
					SetLEDState(LEDY, ST_LED_DARK);
					nTPGVTimeout = -1;
					nVacTimeout = -1;
					nIGTimeout = 0;
					nErrorCount = 0;
					bErrorIP = 0;
					break;
				case VAC_ST_ERROR2: //high vaccuum
					bScanAbort = 1;
					nGotoAir = 0;
					ClearVacuumParameters(1);  //nVacOn=0,nVacOff=0,etc
#if USE_GLOBAL_IG == 0
					SetIonGauge(0);	//close ion gauge, ???
#endif
					SetGateValve(0, VALVE_CLOSE);
					VentToAir(VENT_NONE);
					OSTimeDly(TICKS_PER_SECOND);
					VentingProcess();
					SetGateValve(1, VALVE_CLOSE);
					SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					OSTimeDly(TICKS_PER_SECOND);
					if (bErrorIP == 1)
						SetIonPump(0);
					SetLEDState(LEDG, ST_LED_BLINK_2HZ);
					SetLEDState(LEDB, ST_LED_BLINK_2HZ);
					SetLEDState(LEDY, ST_LED_DARK);
					nTPGVTimeout = -1;
					nVacTimeout = -1;
					nIGTimeout = 0;
					nErrorCount = 0;
					bErrorIP = 0;
					break;
				case VAC_ST_INIT:
					ClearVacuumParameters(0);
					SetIonGauge(1);
					SetScrollPump(0);
					//SetIonPump(0);
					StopTurbo(0);
					SetGateValve(0, VALVE_CLOSE);
					//SetGateValve(1, VALVE_CLOSE);
					//SetGateValve(2, VALVE_CLOSE);
					SetGateValve(3, VALVE_CLOSE);
					//bLEDState[LEDG] = ST_LED_BLINK_1HZ;
					//bLEDState[LEDB] = ST_LED_BLINK_1HZ;
					SetLEDState(LEDB, ST_LED_BLINK_1HZ);
					nIGTimeout = NV_Settings.sIGWaitTimeout;
					nTPGVTimeout = -1;
					break;
			}
			//
			szBuffer[0] = 0;
			GetVacuumStateName(SysParam.bPrevVacuumState, szTemp);
			sprintf(szValue, "\nPREV=%s,", szTemp);
			strcat(szBuffer, szValue);
			GetVacuumStateName(SysParam.bVacuumState, szTemp);
			sprintf(szValue, "CURR=%s,", szTemp);
			strcat(szBuffer, szValue);
			GetVacuumStateName(SysParam.bNextVacuumState, szTemp);
			sprintf(szValue, "NEXT=%s\n", szTemp);
			strcat(szBuffer, szValue);
			printf(szBuffer);
			//
			SysParam.bPrevVacuumState = SysParam.bVacuumState;
			SysParam.bVacuumState = SysParam.bNextVacuumState; //set new SysParam.bVacuumState
			memmove(&VacStLog[0], &VacStLog[1], (VAC_ST_LOG_NUM_M1) * sizeof(VAC_ST_LOG));
			VacStLog[VAC_ST_LOG_NUM_M1].lTick = lTick1S;
			VacStLog[VAC_ST_LOG_NUM_M1].bState = SysParam.bVacuumState;
			VacStLog[VAC_ST_LOG_NUM_M1].fPG[0] = GetVoltage(ADC_CH_VAC0);
			VacStLog[VAC_ST_LOG_NUM_M1].fPG[1] = GetVoltage(ADC_CH_VAC1);
			VacStLog[VAC_ST_LOG_NUM_M1].fIG = GetVoltage(ADC_CH_IG);
			VacStLog[VAC_ST_LOG_NUM_M1].fTempe[0] = fTemperature[0];
			VacStLog[VAC_ST_LOG_NUM_M1].fTempe[1] = fTemperature[1]; //coil HS
			VacStLog[VAC_ST_LOG_NUM_M1].fTempe[2] = (float)nTurboTemperature;
			VacStLog[VAC_ST_LOG_NUM_M1].fTempe[3] = fTemperature[3]; //power HS
			if ((SysParam.bVacuumState == VAC_ST_STANDBY0) || (SysParam.bVacuumState == VAC_ST_STANDBY1))
				printf("STANDBY%d (1:VENT, 0:VACUUM)\n", SysParam.bStandby);
		} //if (SysParam.bNextVacuumState != SysParam.bVacuumState)
		//------------------------------------------
		// processed per 2 seconds
		//------------------------------------------
		if (nTimeTick2Hz % CHECK_PER_2SEC == 0) {
			GetVacuumChangeRate(); //shift data per 2 seconds
			// reset timeout if vacuum is lowering down quickly
			// if not change quickly enough, nFastDropTimeout = 0
			if (nFastDropTimeout > 0) nFastDropTimeout--;
			//NV_Settings.fLowChangeRate is negative
			if ((SysParam.bVacuumState == VAC_ST_LO_WAIT) || (SysParam.bVacuumState == VAC_ST_INT_LO_WAIT)) {
				if (fVACChangeRate[1] < NV_Settings.fLowChangeRate) { //more negative
					nVacTimeout = NV_Settings.sLowWaitTimeout;
					nFastDropTimeout = FAST_DROP_TIMEOUT;
					if (bDebug == 10) printf("LO_WAIT:FAST DROP!\n"); //reset timeout
				}
			}
			else if ((SysParam.bVacuumState == VAC_ST_HI_WAIT) || (SysParam.bVacuumState == VAC_ST_INT_HI_WAIT)) {
				if (fVACChangeRate[1] < NV_Settings.fHighChangeRate) { //more negative
					if (SysParam.bVacuumState == VAC_ST_HI_WAIT) {
						nVacTimeout = NV_Settings.sHighWaitTimeout; //reset timeout
						if (bDebug == 10) printf("HI_WAIT:FAST DROP!\n"); //reset timeout
					}
					if (SysParam.bVacuumState == VAC_ST_INT_HI_WAIT) {
						nVacTimeout = NV_Settings.lHighWaitTimeout; //reset timeout
						if (bDebug == 10) printf("INT_HI_WAIT:FAST DROP!\n"); //reset timeout
					}
					nFastDropTimeout = FAST_DROP_TIMEOUT;
				}
			}
		}
		//-----------------------------------------------------------
		// the following codes are processed per second
		//-----------------------------------------------------------
		if (nTimeTick2Hz % CHECK_PER_SEC == 0) {
			IncVACCounter();
			if (IsTurboStartBrake() == 1) { //lower than 1050
				if (nTPGVTimeout >= 0)
					nTPGVTimeout--;
			}
			if (nVacTimeout > 0) {
				if (bDebug == 10) { //debug vac change rate
					//do nothing
				}
				else if ((nVacTimeout % 60) == 0)
					printf("\nTIMEOUT=%d:", nVacTimeout);
				else if ((nVacTimeout % 10) == 0)
					printf("*");
				nVacTimeout--;
				if (nVacTimeout == 0)
					printf("\n");
			}
			if (nIGTimeout > 0) {
				if ((SysParam.bSimuVAC == 1) && (nIGTimeout > 5))
					nIGTimeout = 5;
				else
					nIGTimeout--;
				if (nIGTimeout % 10 == 0) {
					printf("wait %d s until IG is stable.\r\n", nIGTimeout);
				}
			}
			//nVentTimeout for GV2 movement
			if (nVentTimeout > 0) nVentTimeout--;
			//------------------------------------------------------
			//for EM-200, .sVacTimeout unit = min
			//for EM-100, .sVacTimeout unit = sec
			//------------------------------------------------------
			SysParam.sVacTimeout = (short)(nVacTimeout / 60);
			SysParam.sIGTimeout = (short)(nIGTimeout / 60);
			SysParam.sTPGVTimeout = (short)nTPGVTimeout;
			//
			if ((nIdleTimeout > 0) && (NV_Settings.nIdleTimeout > 0)) {	// 2 minutes
				nIdleTimeout--;
#if EN_IDLE_TIMEOUT == 1
				if (nIdleTimeout == 0) { // 2 minutes
					SetHVSwitch(0);
					SetObjOn(0); //turn off obj
					//SetVOPPower(0);
					bScanAbort = 1;
					printf("idle timeout, shutdown system\n");
					SaveErrorMessage("IDLE_TIMEOUT", 1);
				}
#endif
			}
			if (bDebug == 10) {
				//check vacuum status change rate
				printf("VAC_RATE=%.4f,%.4f,TMO=%d,TP=%d\n", fVACChangeRate[0], fVACChangeRate[1],
						nVacTimeout, nTurboRotationSpeed);
				//printf("VAC_LEVEL=%.4f,%.4f\n", fVACLevel[0], fVACLevel[1]);
				//printf("ADC=%.4f,%.4f\n", fADCVal[0], fADCVal[1]);
			}
		}
		while (GetTurboCommandNum() != 0) {
			nCommand = PopTurboCommand();
			ExecuteTurboCommand(nCommand);
		}
		//bSerDebug=1, debug serial interface
		if (bSerDebug == 0) { //1:test SI:X:string
			if (nTimeTick2Hz % 8 == 1) { //execute once per 6 seconds
				GetTurboRotationSpeed(1);
			}
			else if (nTimeTick2Hz % 8 == 4) { //execute once per 6 seconds
				GetTurboMotorTemp(1);
			}
		}
	} //while(1)
}
#endif

void ResetVACCounter(void)
{
	nVACCounter = 0;
}

void IncVACCounter(void)
{
	nVACCounter++;
}
