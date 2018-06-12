//
#include <predef.h>
#include <stdio.h>
#include <stdlib.h> //atoi
#include <string.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
//#include <tcp.h>
#include <udp.h>
#include <serial.h>
#include <NetworkDebug.h>
#include <pins.h>
#include <math.h> //sqrt
#include <ucos.h>
#include "main.h"
#include "cpu_cmd.h"
#include "data_struct.h"
#include "rtc.h"
#include "Turbo_Control_Protocol.h"
#include "function.h"
#include "motor.h"
#include "hv_ctrl.h"
#include "ErrorMsg.h"
//
#define RAM_TEST_NUM	40
extern int nPrevConn;
//
//extern BYTE bScanning;
extern BYTE bScanRestart;
extern BYTE bScanAbort;
extern BYTE bScanPause;
extern BYTE bAutoFocusStart;
//extern BYTE bAutoFocusMode;
//extern BYTE bAutoFocusRefresh;
//
extern BYTE bInitOK;
//
extern BYTE bStopType;
extern BYTE bIRQACK;
extern int nScanImageNum;		//10
extern int nScanImageNdx;	//0 ~ 9
//extern int nScanInterval;	//msec, 1000
//extern int nScanIntervalTick; // 200,400,600,800,1000
extern int nScanPixelNumX;
extern int nScanPixelNumY;
extern int nYNdxInc;
extern int nXNdxInc;
//
extern int nTurboReadySpeed;
extern int nTurboLowSpeed;
extern int nTurboRotationSpeed;
//
extern int nResendNum;
extern int nResendY[RESEND_NUM_MAX];
extern float fObjStandbyV;
//
//extern int bSimuMode;
extern int nShadowMode;
extern BYTE bImageMask;
extern BYTE bSecondImage;
extern int nSimuShift;
extern int nSimuTPSpeed;
//extern int nObjOn;
//extern int nTurboPumpOn;
//extern int nScrollPumpOn;
extern int nVacOn;
extern int nVacOff;
extern int nVacOffSent;
extern int nGotoAir;
extern long lVacTotalTime;
//
extern float fFocusVC;
extern float fFocusVF;
extern float fOverscan;
//
extern WORD wDI;
extern WORD wSCAN_DO;
extern WORD wIO_DO;
extern BYTE bLEDTest;
//
//extern BYTE bTcpConnected;
//
//extern int nSenrSel;		//nCoilSenSwitch
extern int nCoilSwitch;	//select large or small coil, fAPerMM[] change
//extern int nCoilShunt;		//coil shunt, fAPerMM[] is not changed
extern int nSenRSet;
extern int nShuntSet;
//
//extern BYTE bEnablePLModX;
//extern BYTE bEnablePLModY;
//extern BYTE bHVPower;
//extern BYTE bGVPower;
//extern BYTE bVOPPower;
//extern int nHVON;
extern int nHVOnTime;
extern int nShutdownVOP;
//extern int nGateValveState[VAC_GATE_NUM_MAX];
//extern BYTE bVAC_OK[VAC_GAUGE_NUM_MAX];
//extern int nGaugeStatus[VAC_GAUGE_NUM_MAX];
extern BYTE bVACBypassOK[VAC_GAUGE_NUM_MAX];
extern float fVACChangeRate[VAC_GAUGE_NUM_MAX];
//extern float fTimeXAsc;
//extern float fTimeX;
//
extern int nDelayIDec; 	//delay increase with time
extern int nDelayIDecStep; //the larger, the slower
//extern int nDelayDescent;
extern int nPixelDwellTime;
extern int nDelayPerLine;
//extern BYTE bAutoXLineDelay;
//extern WORD wXLineDelay;
extern BYTE bFastScan;
extern BYTE bRegionScan;
extern float fRegion;
extern float fRegionStart;
extern int nRegionYStart;
extern int nRegionYEnd;
//
extern BYTE bFastClock;	//1: 30 MHz
//BYTE bVADCClock = 0; //vadc_clk_num=0,VADC_SYSCLK=0
extern BYTE bMidDelay;	//middle delay
extern int nDelayEdge;
extern int nXNumMStep;
extern int nXNumPStep;
//
//extern BYTE bCoilZoomCtrl;
//extern float fDefSenR;
//extern float fMagNow;
extern float fMagMinimum;
extern float fMagMaximum;
extern float fImageWidthReal;
//---------------------------------------------
extern float fRatioDefSenR;
extern float fRatioShunt;
extern float fRatioDefSenRMax;
extern float fRatioShuntMax;
//
//extern BYTE bEnableGradientByFW;
extern BYTE bEnableHisto;
extern BYTE bEnableLED;
extern BYTE bLEDState[LED_NUM];
//
extern float fCoarse2FineRatio;
extern float fObjScale;
extern BYTE bACK;
extern BYTE bDB2Busy;
#if SCAN_PCB_VER >= 9
extern BYTE bDB3Busy;
#endif
extern BYTE bPktNum;
//
extern NV_SETTINGS NV_Settings;
SET_PARAM SetParam;
SYS_PARAM_3 SysParam;	//system parameters
SYS_PARAM_2 SysParam2;	//HV parameters
//
extern float fDACVMax[BOARD_SCAN][DAC_CH_NUM_MAX];
extern float fDACVMin[BOARD_SCAN][DAC_CH_NUM_MAX];
extern float fVDACVMax[VDAC_CH_NUM_MAX];
extern float fVDACVMin[VDAC_CH_NUM_MAX];
extern PWORD pwFpga;	//nCS2
extern PWORD pwSram;	//nCS1
extern int nEchoTimeout;
extern int nIdleTimeout;
//
extern int nTxTestNum;
float fObjPercent = 0.5; 	//0~99%, used for simulation only
float fBrightness[BR_NUM] = {50, 50, 50, 50, 50, 50};	//percentage
float fContrast[CO_NUM] = {50, 50, 50};	//percentage
float fFocus = 50;  	//percentage
float fFocusRange = 4.0;  	//percentage
BYTE bBlanking = 0;
extern BYTE bEnableIO;
extern BYTE bUUID_PID[UUID_NUM];
//
BYTE bReturn = 1;
BYTE bSelfTest = 0;
BYTE bTestMode = 0;
int nConnTest = CONN_UART0;
BYTE bChkTP = 1;
//BYTE bCalVideo = 0;		//video signal calibration
BYTE bDebug = 0;
BYTE bSerDebug = 0;
BYTE bEnableEDS = 0;
int nEDSDWT = 5; //EDS dwelling time in msec
int nEDSHandshake = 0;
BYTE bCommandDelay = 0;
BYTE bHWUSDelay = 1; //HW_US_DELAY;
int nPktDelay1 = 1000; //us
int nPktDelay2 = 500;  //us
int nPktDelay3 = 5000;
int nPktDelay4 = 5000;
int nConnType = CONN_UDP;

BYTE bHVCommand = 0;
BYTE bFilaConstMode = 0; //set custom HV operation mode
float fAcckV = ACC_KV_BASE;
int nObjTableNdx = 0; //0:15,1:12,2:10,3:8
//
int nVacErrToleranceNum[2] = {5*CHECK_PER_SEC, 5*CHECK_PER_SEC}; //3, 10 will be better
//bScaleMode=SCALE_BY_TRIG_W_ROT
//use DAC_SINE, DAC_COSINE to set ratio, DAC_VID_REF is fixed to +5V
//ratio = fCoilRatio*fRotateRatio
//bScaleMode=SCALE_BY_DACVID
//use DAC_VID_REF to set ratio, DAC_SINE, DAC_COSINE to angle 0, (fRotateAngle=0)
//ratio = fCoilRatio*fDACRefRatio
#if ENABLE_ROTATION == 2 //has rotation board
//BYTE bScaleMode = SCALE_BY_TRIG_WO_ROT;	//_WO_ = without
//BYTE bEnableRotation = 0;
#elif ENABLE_ROTATION == 1 //has rotation board
//no EDS function
//BYTE bScaleMode = SCALE_BY_TRIG_W_ROT;	//_W_ = with
//BYTE bEnableRotation = 1;
#else //no rotation board
//no EDS function
//BYTE bScaleMode = SCALE_BY_DACVID; //video DAC
//BYTE bEnableRotation = 0;
#endif
//
//extern int nPixelDwellingTime;
extern int nVADCAvgNum;
extern int nInterlockCode;
extern BYTE bFPGAMajorVersion;
extern BYTE bFPGAMinorVersion;
extern BYTE bIOFPGAMajorVersion;
extern BYTE bIOFPGAMinorVersion;
extern BYTE bFPGADateMonth;
extern BYTE bFPGADateDay;
extern BYTE bErrorHWVersion;
extern BYTE bErrorPID;
//
#define ERROR_MSG_NUM		40
#define ERROR_MSG_NUM_M1	39
#define ERROR_MSG_LEN		70
//
long lErrorTime[ERROR_MSG_NUM];
char szErrorMessage[ERROR_MSG_NUM][ERROR_MSG_LEN];
//
float fDACXFSet = 0.0;
int nEnableDACXFSet = 0;
int nDACChBR[BR_NUM] = {DAC_CH_BR0, DAC_CH_BR1, DAC_CH_BR2, DAC_CH_BR3,
		DAC_CH_BR4,	DAC_CH_BR5};
int nDACChCO[CO_NUM] = {DAC_CH_CO0, DAC_CH_CO1, DAC_CH_CO2};
//
WORD wParameter = 0;  //
WORD wSetFIFO = 0;
//
BYTE bPLSync = 0; //power line synchronization
BYTE bEnableCommand = 0;
//

void InitSetParam(void)
{
	//if (NV_Settings.bEnablePLModX == 1)
	//	wParameter |= PARA_EN_PL_MODX;
	//if (NV_Settings.bEnablePLModY == 1)
	//	wParameter |= PARA_EN_PL_MODY;
	pwFpga[SET_PARAMETER] = wParameter;
	if ((NV_Settings.sXNdxInc >= 1) && (NV_Settings.sXNdxInc <= 8))
		nXNdxInc = NV_Settings.sXNdxInc;
}

void InitSysParam(void)
{
	//SysParam.bAutoXLineDelay = 1;
	//SysParam.nDelayDescent = 4;
	//SysParam.nPixelDwellTime = 20;
	memset(&SysParam, 0, sizeof(SYS_PARAM_3));
	SysParam.bVersion = SYS_PARAM_VER;
	SysParam.bSimuMode = 0;
	SysParam.wXLineDelay = 10000; //250 us
	SysParam.bScanning = OP_IDLE;
	SysParam.bPrevScanning = OP_IDLE;
	SysParam.bVacuumState = VAC_ST_AIR;		//current vacuum state
	SysParam.bNextVacuumState = VAC_ST_AIR;	//next vacuum state
	SysParam.bPrevVacuumState = VAC_ST_AIR;
	SysParam.fTimeX = 0.004; //second
	SysParam.fTimeXAsc = 0.004;
	SysParam.sTPSpeed = 0;
	SysParam.fTempTP = 0;
	SysParam.fHVBiasI = 0;
	SysParam.fHVBiasV = 0;
	SysParam.bSimuVAC = 0;
	BlankingProcess(0); //generate table
}

void ClearErrorMessage(void)
{
	int i;
	for (i = 0; i < ERROR_MSG_NUM; i++) {
		lErrorTime[i] = 0;
		szErrorMessage[i][0] = 0;
	}
}
//return 1: scanning is processing
//return 0: scanning is paused or stopped
int IsScanning(void)
{
	if (SysParam.bScanning == OP_IDLE)
		return 0;
	if (bScanPause == 1)
		return 0;
	return 1;
}
// ENABLED: return 1
// DISABLED: return 0
int IsCommandEnabled(BYTE bType)
{	//HOBOCHEN:0x01
	if ((bType == CONN_UART0) && ((bEnableCommand & 0x01) == 0))
		return ERROR_FAIL;
	if ((bType == CONN_UART0_BIN) && ((bEnableCommand & 0x01) == 0))
		return ERROR_FAIL;
	//HOBOCHEN:0x02
	if ((bType == CONN_UDP) && ((bEnableCommand & 0x02) == 0))
		return ERROR_FAIL;
	if ((bType == CONN_TCP) && ((bEnableCommand & 0x02) == 0))
		return ERROR_FAIL;
	return SUCCESS;
}
//
extern long lTick1S;
void SaveErrorMessage(char *szStr, int nPrint)
{
	int i;
	//
	szStr[ERROR_MSG_LEN - 1] = 0; //avoid long error message
	RemoveLFCR(szStr);
	if (nPrint) printf("%ld,%s\n", lTick1S, szStr);
	//shift error message
	for (i = 0; i < ERROR_MSG_NUM - 1; i++) {
		strcpy(szErrorMessage[i], szErrorMessage[i + 1]);
	}
	memmove(&lErrorTime[0], &lErrorTime[1], (ERROR_MSG_NUM - 1) * sizeof(long));
	strcpy(szErrorMessage[ERROR_MSG_NUM_M1], szStr);
	lErrorTime[ERROR_MSG_NUM_M1] = lTick1S;
}
//
void DisplayErrorMessage(int nType)
{
	int i, k;
	char szBuffer[256];
	k = 1;
	sprintf(szBuffer, "CURRENT TIME = %ld s\nERROR LIST\n", lTick1S);
	TxString(nType, szBuffer);
	for (i = 0; i < ERROR_MSG_NUM; i++) {
		if (szErrorMessage[i][0] == 0)
			continue;
		sprintf(szBuffer, "%2d,%8ld,%s\n", k, lErrorTime[i], szErrorMessage[i]);
		k++;
		TxString(nType, szBuffer);
	}
}
//
void GetErrorMessage(int nCode, char *szValue)
{
	switch(nCode)
	{
		case ERR_INV_PARAM: //invalid parameter
			sprintf(szValue, "INV_PARAM");
			break;
		case ERR_INV_CMD:	//invalid command
			sprintf(szValue, "INV_CMD");
			break;
		case ERR_REENTRY:
			sprintf(szValue, "REENTRY");
			break;
		case ERR_OVER_RANGE:
			sprintf(szValue, "OVER_RANGE");
			break;
		case ERR_UNDER_RANGE:
			sprintf(szValue, "UNDER_RANGE");
			break;
		case ERR_INV_VAC_ST:
			sprintf(szValue, "INV_VAC_STATE");
			break;
		case ERR_INTLK_FAIL:
			sprintf(szValue, "INTLK");
			if (nInterlockCode == ERROR_TP_NOK)
				strcat(szValue, ":TP_NOT_READY");
			else if (nInterlockCode == ERROR_CHAM_OPEN)
				strcat(szValue, ":CHAM_OPEN");
			else if (nInterlockCode == ERROR_CASE_OPEN)
				strcat(szValue, ":CASE_OPEN");
			else if (nInterlockCode == ERROR_VAC0_NOK)
				strcat(szValue, ":VAC0_NOT_OK");
			else if (nInterlockCode == ERROR_VAC1_NOK)
				strcat(szValue, ":VAC1_NOT_OK");
			else if (nInterlockCode == ERROR_BTEMP_HI)
				sprintf(szValue, "BTEMP_ERROR");
			break;
		case ERR_INV_PID:
			sprintf(szValue, "PCB_ID_MISMATCH,0X%02X", SysParam.bErrorHWVersion);
			break;
		case ERR_INV_VERSION: //invalid version
			sprintf(szValue, "HW_VER_MISMATCH,0X%02X", SysParam.bErrorHWVersion);
			break;
		case ERR_EDS_ONLY:
			sprintf(szValue, "NOT_IN_EDS_MODE");
			break;
		case ERR_MAG:
			sprintf(szValue, "MAG_ERROR");
			break;
		case ERR_MAG_TOO_SMALL:
			sprintf(szValue, "MAG_TOO_SMALL");
			break;
		case ERR_MAG_TOO_LARGE:
			sprintf(szValue, "MAG_TOO_LARGE");
			break;
		case ERR_INV_MODE:
			sprintf(szValue, "INV_SCAN_MODE");
			break;
		/*
		case ERR_SCANPAUSE_INV_PARAM:
			sprintf(szValue, "SCANPAUSE_INV_PARAM");
			break;
		*/
		default:
			//sprintf(szValue, "UNKNOWN"); Omar
			break;
	}
	//SaveErrorMessage(szValue);
}
// receive settings from PC
int DecipherDataPacket(int nType, BYTE *pData, int nBytes)
{
	int nV, i;
	if (nBytes == sizeof(SET_PARAM)) {
		if (pData[0] != SET_HEADER_1)
			return 0;
		memmove((BYTE *)&SetParam, pData, nBytes);
		NV_Settings.nTurboType = SetParam.bTurboType;
		for (i = 0; i < OBJ_ENERGY_NUM; i++) {
			NV_Settings.fObjIMax[i] = ByteSwapFloat(SetParam.fObjIMax[i]);
			NV_Settings.fObjIMin[i] = ByteSwapFloat(SetParam.fObjIMin[i]);
		}
		NV_Settings.bScaleMode = SetParam.bScaleMode;
		//bHWUSDelay = SetParam.bHWUSDelay; //always 1
		NV_Settings.fAPerMM[0][0] = ByteSwapFloat(SetParam.fAPerMM[0]);
		NV_Settings.fAPerMM[0][1] = ByteSwapFloat(SetParam.fAPerMM[1]);
		//
		//NV_Settings.bEnablePLModX = SetParam.bEnablePLModX;		//26, enable PL moduation X
		//NV_Settings.bEnablePLModY = SetParam.bEnablePLModY;
		//NV_Settings.fPLAmp = ByteSwapFloat(SetParam.fPLAmp);
		//NV_Settings.fPLPhase = ByteSwapFloat(SetParam.fPLPhase);
		NV_Settings.bPLFreq = SetParam.bPLFreq;
		NV_Settings.bPLSyncAuto = SetParam.bPLSyncAuto;
		nV = NV_Settings.nTurboType;
		NV_Settings.nTurboReadySpeed[nV] = ByteSwapWord(SetParam.wTurboReadySpeed);
		NV_Settings.bBinPkt = SetParam.bBinPkt;
	}
	else if (nBytes == sizeof(NV_SETTINGS)) {
	}
	return 0;
}
//
int DecipherMultipleCommand(int nType, char *szCommand)
{
	char *p1, *p2;
	int nRet = SUCCESS;
	static int nReentry = 0;
	//
	if (nReentry == 1) {
		goto DecipherFail;
	}
	nReentry = 1;
	//
	nConnType = nType;
	p1 = szCommand;
	p2 = strchr(p1, '#');
	if (p2 == NULL) {//one command
		nRet = DecipherCommand(nType, p1);
	}
	else {
		while (p2 != NULL) {
			*p2 = 0;
			if (DecipherCommand(nType, p1) != SUCCESS)
				nRet = ERROR_FAIL;
			p1 = p2 + 1;
			p2 = strchr(p1, '#');
			if (bCommandDelay) OSTimeDly(1); //parameter set in DecipherCommand()
		}
	}
	if (nRet == SUCCESS)
		goto DecipherOK;
DecipherFail:
	nReentry = 0;
	return ERROR_FAIL;
DecipherOK:
	nReentry = 0;
	return SUCCESS;
}
//
// nConnType == CONN_UART0 or CONN_UDP
//
int DecipherCommand(int nType, char *szCommand)
{
	float fV, fXC, fYC;
	float fV1, fV2, fV3, fV4, fMag;
	float fBR;
	float fVR[ISEN_CH_NUM];
	float fI;	//current
	int nV = 0;
	int nB, nCh, nN, nV1, nV2, nV3, nT;
	int i, j, nNX, nNY;
	int nPN; //parameter number
	int nRet;
	//int nYear, nMon, nDay, nHour, nMin, nSec;
	WORD wV, wV1, wV2;
	//WORD wVAD[VIDEO_ADC_ALL_CH_NUM];
	//WORD wVAD_DIFF[VIDEO_ADC_ALL_CH_NUM];
	float fADC[ADC_CH_NUM_MAX];
	//float fMaxI;
	char szValue[128], szTemp[64];
	char szBuffer[2048];
	int nErrCode = ERR_UNKNOWN;
	DWORD dwAddr, dwCS;
	char *p, c;
	unsigned int nID[UUID_NUM];
	BYTE bUUID[UUID_NUM];
	char szCommandOrg[256];
	static int nReentry = 0;
	int nTestNdx = 0, nTestNum = 1;
	BYTE bTestOK[RAM_TEST_NUM];
	//
	if (nReentry == 1)
		return ERROR_FAIL;
	nReentry = 1;
	//
	bCommandDelay = 0;
	//---------- test commands begin ------------------------------
	nPrevConn = nType;
	strcpy(szCommandOrg, szCommand);
	strupr(szCommand);
#define CMD_HOBOCHEN		"HOBOCHEN:"
#define LEN_HOBOCHEN		(sizeof(CMD_HOBOCHEN)-1)
	if (strncmp(szCommand, CMD_HOBOCHEN, LEN_HOBOCHEN) == 0) {//scan stop
		p = strchr(szCommand, ':');
		if (p == NULL)
			goto DecipherRet;
		nPN = sscanf(&szCommand[LEN_HOBOCHEN], "%d", &nV1);
		bEnableCommand = (nV1 <= 3) ? nV1 : 0;
		goto DecipherRet;
	}
	//---------- test commands end ------------------------------
	if (bDebug == 1)	{ //print out command
		if (strncmp(szCommand, "GETSYS", 6) == 0) { //don't print GETSYS, too many message
		}
		else if (strncmp(szCommand, "GETDI", 5) == 0) { //don't print GETDI
		}
		else printf("%s\n", szCommand);
	}
	if (bDebug == 14)
		printf("%s\n", szCommand);
	//-----------Prioritized command-------------------------------
#define CMD_ACCKV		"ACCKV:"
#define LEN_ACCKV		(sizeof(CMD_ACCKV)-1)
	if (strncmp(szCommand, CMD_ACCKV, LEN_ACCKV) == 0)	{
		nPN = sscanf(&szCommand[LEN_ACCKV], "%d:%f", &nV1, &fV1);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ACCKV_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (nV1 == 0){
			if( (fV1 < 0) || (fV1 >1)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_ACCKV_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			NV_Settings.bAcckVAdjust = (BYTE)fV1;
		}
		else if (nV1 == 1) //used in formula
			fAcckV = fV1;
		else if (nV1 == 2)
			nObjTableNdx = (int)fV1;
		else{
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ACCKV_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		goto DecipherRet; //goto DecipherOK;
	}
#define CMD_ACK		"ACK:"
#define LEN_ACK		(sizeof(CMD_ACK)-1)
	if (strncmp(szCommand, CMD_ACK, LEN_ACK)  == 0)
	{	//if bACK == 2, stop UDP/TCP transmission
		//if bACK == 0, continue next image
		if (szCommand[LEN_ACK] == '?') { //ACK:?
			printf("ACK=%d\r\n", (int)bACK);
		}
		else{
			bACK = atoi(&szCommand[LEN_ACK]);
			if( (bACK != 0) && (bACK != 2)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_ACK_INV_PARAM_VAL));
				goto DecipherFail;
			}
		}
		goto DecipherRet;
	}
#define CMD_SCANPAUSE		"SCANPAUSE:"
#define LEN_SCANPAUSE		(sizeof(CMD_SCANPAUSE)-1)
	else if (strncmp(szCommand, CMD_SCANPAUSE, LEN_SCANPAUSE) == 0) {//scan stop
		if (bDebug == 11) printf("%s\n", szCommand);
		p = strchr(szCommand, ':');
		if (p == NULL)
			goto DecipherRet;
		nPN = sscanf(&szCommand[LEN_SCANPAUSE], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCANPAUSE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if(  (nV1 < 0) || (nV1 >1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCANPAUSE_INV_PARAM_VAL));
			goto DecipherFail;
		}

		bScanPause = (nV1 == 1) ? 1 : 0;
		nScanImageNdx = 0; //restart process
		goto DecipherRet;
	}
#define CMD_SCANSTOP		"SCANSTOP"
#define LEN_SCANSTOP		(sizeof(CMD_SCANSTOP)-1)
	else if (strncmp(szCommand, CMD_SCANSTOP, LEN_SCANSTOP) == 0) {//scan stop
		printf("SCANSTOP\n");
		bScanAbort = 1; //stop all power supply
		bScanPause = 0;
		bStopType = 1;
		bBlanking = 0;
		p = strchr(szCommand, ':');
		if (p == NULL) //SCANSTOP = SCANSTOP:1
			goto DecipherRet;
		nPN = sscanf(&szCommand[LEN_SCANSTOP + 1], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCANSTOP_INV_PARAM_NUM));
			goto DecipherFail;
		}
		//SCANSTOP:1 --> shutdown all power
		//SCANSTOP:0 --> keep OBJ and scan related power on
		if( (nV1 < 0) || (nV1 >1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCANSTOP_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bStopType = (nV1 == 1) ? nV1 : 0;
		bReturn = 1;
		bCommandDelay = 1;
		goto DecipherRet;
	}
	//-----------queued command during scan-------------------------
	//ACK and SCANSTOP don't need process in queue
	//if (bDB3Busy == 1) {
	//	PushCommandToQueue(szCommand, nType);
	//	return SUCCESS;
	//}
#define CMD_DACT		"DACT:"
#define LEN_DACT		(sizeof(CMD_DACT)-1)
	if (strncmp(szCommand, CMD_DACT, LEN_DACT) == 0)
	{
		nPN = sscanf(&szCommand[LEN_DACT], "%d:%d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_DACT_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if(  (nV1 < 0) || (nV1 >1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_DACT_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		NV_Settings.bDACType[nV1] = (BYTE)nV2;
		goto DecipherRet; //goto DecipherOK;
	}
#define CMD_FILACNST		"FILACNST:"
#define LEN_FILACNST		(sizeof(CMD_FILACNST)-1)
	if (strncmp(szCommand, CMD_FILACNST, LEN_FILACNST) == 0)
	{
		nPN = sscanf(&szCommand[LEN_FILACNST], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_FILACNST_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0) || (nV1 >2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_FILACNST_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bHVCommand = 1;
		bFilaConstMode = (BYTE)nV1;
		goto DecipherRet; //goto DecipherOK;
	}
#define CMD_GETST		"GETST"
#define LEN_GETST		(sizeof(CMD_GETST)-1)
	if (strncmp(szCommand, CMD_GETST, LEN_GETST) == 0) {
		//vadc_conv_start, running, sysclk, n_reset,
		//vadc_scan_stop, vadc_scan_start, set_dacx, vadc_to_ram,
		//scanning, histo_calc, row_data_ready, n_eneds_hw,
		//dout_io, delay_us_timeout, get_video_adc, x_descent} :
		//example 0x7096=0111_0000_1001_0110,vadc_conv_start=0,get_video_adc=1,dout_io=0
		//example 0x709E=0111_0000_1001_1110,vadc_conv_start=0,get_video_adc=1,dout_io=1
		wV = pwFpga[GET_STATUS];
		WordToBinaryString(wV, szValue); //bbbb_bbbb_bbbb_bbbb_
		printf("STATUS1=%s,%04X\n", szValue, wV);
		//5'b0_0000, sign_gradient1, sign_gradient2, sel_coil_small,
		//obj_on, vop_on, senr_sel, shunt,
		//eds_on_reg, relay_on, n_vadc_eoc[1:0]} :
		wV = pwFpga[GET_STATUS2];
		WordToBinaryString(wV, szValue); //bbbb_bbbb_bbbb_bbbb_
		printf("STATUS2=%s,%04X\n", szValue, wV);
		goto DecipherRet; //goto DecipherOK;
	}
#define CMD_GETNV		"GETNV"
#define LEN_GETNV		(sizeof(CMD_GETNV)-1)
	if (strncmp(szCommand, CMD_GETNV, LEN_GETNV) == 0) {
		if (IsCommandEnabled(nType) != SUCCESS) goto DecipherRet;
		TxSetParam(nType);
		goto DecipherRet;
	}
#define CMD_GETSYS		"GETSYS"
#define LEN_GETSYS		(sizeof(CMD_GETSYS)-1)
	if (strncmp(szCommand, CMD_GETSYS, LEN_GETSYS) == 0)
	{
		if (IsCommandEnabled(nType) != SUCCESS) goto DecipherRet;
		//wait until power on process is complete
		if (bInitOK == 0) goto DecipherRet;
		c = szCommand[LEN_GETSYS + 1];
		if (nType == CONN_UART0) {//RS232 command
			//c = szCommand[LEN_GETSYS + 1];
			if (c == '?') {
				printf("1: IP_PWR_CTRL information\n");
				printf("2: HV information\n");
				goto DecipherRet;
			}
			if (c == '1') //GETSYS:1, query IP_PWR_CTRL information
				goto SysText1; //SO:1:2:$1,IPINFO\n
			else if (c == '2') //GETSYS:2, query custom HV information
				goto SysText2; //SO:1:2:$2,GETSYS:1\n
			else if (c == '3') //GETSYS:3
				goto SysText3;
			else
				goto SysText;
		}
		if (NV_Settings.bBinPkt == 1) {
			if (c == '4') { //binary mode, GETSYS:4
				bHVCommand = 2; //send GETV command
				TxSysParam2(nType); //GETSYS:4
			}
			else //GETSYS:5
				TxSysParam3(nType);
			goto DecipherRet;
		}
SysText:  //GETSYS:0
		//.bBinPkt == 0
		wV = GetDigitalInput(0);
		WordToBinaryString(wV, szValue); //bbbb_bbbb_bbbb_bbbb_
		sprintf(szBuffer, "STATUS=%s,%04X\n", szValue, wDI);
		//gate valve status, GV0, GV1, GV2
		for (i = 0; i < VAC_GATE_NUM; i++) {
			nV = GetValveStatus(i, szValue);
			strcat(szBuffer, szValue);
		}
		strcat(szBuffer, "\n");
		//vacuum status VAC0,VAC1,VAC2
		for (i = 0; i < VAC_PG_NUM; i++) {
			nV = GetGaugeStatus(i, szValue);
			strcat(szBuffer, szValue);
		}
		strcat(szBuffer, "\n");
		//sprintf(szValue, "VAC_RATE=%.3f,%.3f\n", fVACChangeRate[0], fVACChangeRate[1]);
		//strcat(szBuffer, szValue);
		/*for (i = 0; i < VAC_GAUGE_NUM_MAX; i++) {
			sprintf(szValue, "VACOK=%d,%d,", i, SysParam.bVAC_OK[i]);
			strcat(szBuffer, szValue);
		}
		strcat(szBuffer, "\n"); */
		for (i = 0; i < VAC_GAUGE_NUM_MAX; i++) {
			sprintf(szValue, "GST=%d,%d,", i, SysParam.sGaugeStatus[i]);
			strcat(szBuffer, szValue);
		}
		strcat(szBuffer, "\n");
		//get HV status
		//GetHVStatus(szValue);
		//strcat(szBuffer, szValue);
		//
		GetVacuumStateName(SysParam.bVacuumState, szTemp);
		sprintf(szValue,"VAC_STATE=%d,%s\n", (int)SysParam.bVacuumState, szTemp); //vacuum state machine
		strcat(szBuffer, szValue);
#if ENABLE_IG == 1
		sprintf(szValue,"STANDBY=%d\n", (int)SysParam.bStandby); //1:VENT, 0:VACUUM
		strcat(szBuffer, szValue);
#endif
		//
		sprintf(szValue,"CSHUNT=%d\n", (int)SysParam.bCoilShunt); //shunt coil control
		strcat(szBuffer, szValue);
		sprintf(szValue,"SENRSEL=%d\n", (int)SysParam.bSenrSel); //deflector control, senr_sel
		strcat(szBuffer, szValue);
#if SCAN_PCB_VER < 9 //no mid line delay problem any more
		sprintf(szValue,"DELAYEDGE=%d,MIDDELAY=%d,PS=%d,MS=%d\n",
			nDelayEdge, bMidDelay, nXNumPStep, nXNumMStep); //FASTSCAN=1
		strcat(szBuffer, szValue);
#endif
		sprintf(szValue,"DELAYXLINE=%d\n", SysParam.wXLineDelay);
		strcat(szBuffer, szValue);
		//sprintf(szValue,"DNDELAY=%d\nDNDELAYSTEP=%d\n", nDelayIDec, nDelayIDecStep); //FASTSCAN=0
		//strcat(szBuffer, szValue);
		sprintf(szValue,"ERROR_HW=0X%02X\n", SysParam.bErrorHWVersion);
		strcat(szBuffer, szValue);
		IsChamberClose(szValue);
		strcat(szBuffer, szValue);
		IsCaseClose(szValue);
		strcat(szBuffer, szValue);
		sprintf(szValue,"HVON=%d\n", (int)SysParam.bHVON);
		strcat(szBuffer, szValue);
		fV = GetHVBiasCurrentV(); //unit voltage
		sprintf(szValue,"HV_ACC_I=%.3f V,%.3f uA\n", fV, fV * NV_Settings.fAccMonI);
		strcat(szBuffer, szValue);
		sprintf(szValue,"HV_ON_TIME=%d s\n", nHVOnTime);
		strcat(szBuffer, szValue);
#if ENABLE_IP == 1
		sprintf(szValue,"IPON=%d\n", (int)SysParam.bIonPumpOn);
		strcat(szBuffer, szValue);
#endif
		sprintf(szValue,"LEDSTATE=%d,%d,%d,%d,\n", (int)bLEDState[0], (int)bLEDState[1],
			(int)bLEDState[2], (int)bLEDState[3]);
		strcat(szBuffer, szValue);
		sprintf(szValue,"MAGNOW=%.1fX\n", SysParam.fMagNow);
		strcat(szBuffer, szValue);
		sprintf(szValue,"OBJON=%d\n", (int)SysParam.bObjOn);
		strcat(szBuffer, szValue);
		//sprintf(szValue, "PLENMODX=%d\n", bEnablePLModX);
		//strcat(szBuffer, szValue);
		//sprintf(szValue, "PLENMODY=%d\n", bEnablePLModY);
		//strcat(szBuffer, szValue);
		sprintf(szValue,"PLSYNC=%d\n", (int)bPLSync);
		strcat(szBuffer, szValue);
		sprintf(szValue,"PWR_HV=%d\nPWR_GV=%d\nPWR_VOP=%d\n",
			(int)SysParam.bHVPower, (int)SysParam.bGVPower, (int)SysParam.bVOPPower);
		strcat(szBuffer, szValue);
		sprintf(szValue,"PZTMOVEX=%d\n", (int)SysParam.bPZTMoveX);
		strcat(szBuffer, szValue);
		sprintf(szValue,"PZTMOVEY=%d\n", (int)SysParam.bPZTMoveY);
		strcat(szBuffer, szValue);
		sprintf(szValue, "ROTATE=%.1f deg\n", SysParam.fRotateAngle);
		strcat(szBuffer, szValue);
		sprintf(szValue,"SCANNING=%d(1:SEM,2:EDS,3:FOCUS,4:POINT)\n", (int)SysParam.bScanning);
		strcat(szBuffer, szValue);
		sprintf(szValue,"SENR=%.2f\n", SysParam.fDefSenR);
		strcat(szBuffer, szValue);
		sprintf(szValue,"SIMU=%d\n", (int)SysParam.bSimuMode); //simulation image data
		strcat(szBuffer, szValue);
		sprintf(szValue,"SIMUVAC=%d\n", (int)SysParam.bSimuVAC); //simulation data
		strcat(szBuffer, szValue);
		sprintf(szValue,"SPON=%d\n", (int)SysParam.bScrollPumpOn);
		strcat(szBuffer, szValue);
		sprintf(szValue,"TCP_CONN=%d\n", (int)SysParam.bTcpConnected);
		strcat(szBuffer, szValue);
		sprintf(szValue,"TICK1S=%ld s\n", lTick1S);
		strcat(szBuffer, szValue);
#if SCAN_PCB_VER >= 9
		sprintf(szValue,"TIMEXASC=%.7f s\n", SysParam.fTimeXAsc); // unit seconds
		strcat(szBuffer, szValue);
		sprintf(szValue,"TIMEX=%.7f s\n", SysParam.fTimeX);
		strcat(szBuffer, szValue);
#endif
		sprintf(szValue,"TPON=%d\n", (int)SysParam.bTurboPumpOn);
		strcat(szBuffer, szValue);
		nV = GetTurboRotationSpeed(0); //bUpdate=0
		SysParam.sTPSpeed = nV;
		sprintf(szValue,"TPSPEED=%d Hz\n", nV);
		strcat(szBuffer, szValue);
		nV = GetTurboMotorTemp(0); //nUpdate=0
		SysParam.fTempTP = (float)nV;
#if IO_PCB_VER >= 1
		sprintf(szValue,"TEMP_TP=%d deg C\n", nV);
		strcat(szBuffer, szValue);
		fV = GetTemperature(TEMP_IN_CASE); 	//TEMPE0
		SysParam.fTempIN = fV;
		sprintf(szValue,"TEMP_IN=%.1f deg C\n", fV);
		strcat(szBuffer, szValue);
		fV = GetTemperature(TEMP_COIL_HS);	//TEMPE1
		SysParam.fTempHS[0] = fV;
		sprintf(szValue,"TEMP_HS1=%.1f deg C\n", fV); //heat sink? second temperature sensor
		strcat(szBuffer, szValue);
		fV = GetTemperature(TEMP_POWER_HS);	//TEMPE2
		SysParam.fTempHS[1] = fV;
		sprintf(szValue,"TEMP_HS2=%.1f deg C\n", fV); //heat sink? second temperature sensor
		strcat(szBuffer, szValue);
		fV = GetTemperature(TEMP_TEMPE3);	//TEMPE3
		SysParam.fTempHS[2] = fV;
		sprintf(szValue,"TEMP_HS3=%.1f deg C\n", fV); //heat sink? second temperature sensor
		strcat(szBuffer, szValue);
#elif SCAN_PCB_VER >= 8
		sprintf(szValue,"TPTEMP=%d deg C\n", nV);
		strcat(szBuffer, szValue);
		fV = GetTemperature(TEMP_IN_CASE); //0
		sprintf(szValue,"INTEMP=%.1f deg C\n", fV);
		strcat(szBuffer, szValue);
		fV = GetTemperature(1);
		sprintf(szValue,"HSTEMP=%.1f deg C\n", fV); //heat sink? second temperature sensor
		strcat(szBuffer, szValue);
#endif
		sprintf(szValue,"ZOOMCTRL=%d\n", (int)SysParam.bZoomCtrl); //heat sink? second temperature sensor
		strcat(szBuffer, szValue);
		TxString(nType, szBuffer);
		goto DecipherRet; //goto DecipherOK;
SysText1:
		sprintf(szValue,"IPHV=%.3f V\n", SysParam.fIpHV);
		TxString(nType, szValue);
		sprintf(szValue,"IPAMP=%.3f mA\n", SysParam.fIpAmp);
		TxString(nType, szValue);
		sprintf(szValue,"MOTORMOVE=%d\n", (int)SysParam.bMotorMove);
		TxString(nType, szValue);
		sprintf(szValue,"MOTORTIME=%d\n", (int)SysParam.sMotorTime);
		TxString(nType, szValue);
		sprintf(szValue,"IPON=%d\n", (int)SysParam.bIPON);
		TxString(nType, szValue);
		goto DecipherRet; //goto DecipherOK;
SysText2:
		sprintf(szValue,"FILAOK=%d\n", (int)SysParam.bFilaOK);
		TxString(nType, szValue);
		sprintf(szValue,"FILAWDT=%d\n", (int)SysParam.bFilaWDT);
		TxString(nType, szValue);
		sprintf(szValue,"ACCV=%.3f V\n", SysParam2.fAccVoltage);
		TxString(nType, szValue);
		sprintf(szValue,"ACCMONI=%.3f uA\n", SysParam2.fAccCurrent);
		TxString(nType, szValue);
		sprintf(szValue,"FILAI=%.3f A\n", SysParam.fFilaI);
		TxString(nType, szValue);
		sprintf(szValue,"FILAV=%.3f V\n", SysParam.fFilaV);
		TxString(nType, szValue);
		sprintf(szValue,"HVON=%d\n", (int)SysParam.bCusHVON);
		TxString(nType, szValue);
		sprintf(szValue,"BIASV=%.2f V\n", SysParam.fBiasV);
		TxString(nType, szValue);
		goto DecipherRet; //goto DecipherOK;
SysText3: //GETSYS:3
		sprintf(szValue,"CHAM=%d\n", (int)SysParam.bChamClose);
		TxString(nType, szValue);
		sprintf(szValue,"CASE=%d\n", (int)SysParam.bCaseClose);
		TxString(nType, szValue);
		sprintf(szValue,"TXDATA=%d\n", (int)SysParam.bTxData);
		TxString(nType, szValue);
		goto DecipherRet;
	}
	//---------- EM commands begin ------------------------------
	//ADDR_RW:<NCS>:<ADDR>:<RnW>:<wV><CR><LF>
#define CMD_ADDR_RW		"ADDR_RW:"
#define LEN_ADDR_RW		(sizeof(CMD_ADDR_RW)-1)
	if (strncasecmp(szCommand, CMD_ADDR_RW, LEN_ADDR_RW) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[8], "%d %d %d %d", &nB, &nN, &nV1, &nV2);
		if (nPN != 4) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ADDR_RW_INV_PARAM_NUM));
			goto DecipherFail;
		}
		wV = Addr_RW(nB, (WORD)nN, nV1, (WORD)nV2); //nCS,ADDR,RnW,wV
		if (nV1 == 1) //RnW = 1, read
		{	//
			WordToBinaryString(wV, szValue); //bbbb_bbbb_bbbb_bbb
			fV1 = WordToVolt(wV, ADC_16BIT, ADC_IN_MAX, ADC_IN_MIN); //general purpose ADC
			fV2 = WordToVolt(wV, ADC_12BIT, 5, 0);		//video ADC
			sprintf(szBuffer, "V=%s,0X%04X,%d,%.3f(16),%.3f(12)\n", szValue, wV, wV, fV1, fV2);
			TxString(nType, szBuffer);
		}
		goto DecipherOK;
	}
#define CMD_APERMM	"APERMM:"
#define LEN_APERMM	(sizeof(CMD_APERMM)-1)
	if (strncmp(szCommand, CMD_APERMM, LEN_APERMM) == 0) { //ampere per mm, APERMM:0:0.33:0.33
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_APERMM], "%d %f %f", &nV1, &fV1, &fV2);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_APERMM_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( nV1 != 0){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_APERMM_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (fV1 < 0)||(fV1 > 3)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_APERMM_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if( (fV2 < 0)||(fV2 > 3)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_APERMM_INV_PARAM3_VAL));
			goto DecipherFail;
		}
		//APERMM:0:0.22:0.22 for the first coil
		//APERMM:1:1:1 for the second coil (2nd smaller coil)
		NV_Settings.fAPerMM[nV1][0] = fV1;
		NV_Settings.fAPerMM[nV1][1] = fV2;
		sprintf(szBuffer, "A per mm=%.3f,%.3f\n", fV1, fV2);
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
#define CMD_AUTOVAC		"AUTOVAC:"
#define LEN_AUTOVAC		(sizeof(CMD_AUTOVAC)-1)
	if (strncmp(szCommand, CMD_AUTOVAC, LEN_AUTOVAC) == 0)	{
		nPN = sscanf(&szCommand[LEN_AUTOVAC], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_AUTOVAC_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_AUTOVAC_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bAutoVAC = (BYTE)nV1;
		goto DecipherRet;
	}
#define CMD_BLANKING	"BLANKING:"
#define LEN_BLANKING	(sizeof(CMD_BLANKING)-1)
	if (strncmp(szCommand, CMD_BLANKING, LEN_BLANKING) == 0) {	//BLANKING:<NV>
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_BLANKING], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BLANKING_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 3)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BLANKING_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bBlanking = (BYTE)nV1;
		if (bBlanking == 2) {
			SetVoltage(BOARD_SCAN, DAC_CH_VID_REF1, DACF_PV); //DAC_REF=5.0, maximal value
			SetRotationAngle(0.0);
			EnableBlanking(2); //FPGA control blanking
		}
		else if ((bBlanking == 1) || (bBlanking == 3)) {
			if (SysParam.bScanning != OP_BLANKING) { //save parameter
				SysParam.bPrevScanning = SysParam.bScanning;
				SysParam.bScanning = OP_BLANKING;
			}
			SetVoltage(BOARD_SCAN, DAC_CH_VID_REF1, DACF_PV); //DAC_REF=5.0, maximal value
			SetRotationAngle(0.0);
			EnableBlanking(1);
		}
		else if (bBlanking == 0) { //return to previous status
			if (SysParam.bScanning == OP_BLANKING) {
				SysParam.bScanning = SysParam.bPrevScanning;
			}
			//SysParam.bScanning = OP_SEM_SCAN;
			//SetVoltage(BOARD_SCAN, DAC_CH_DEFX_F, 0.0);
			//SetVoltage(BOARD_SCAN, DAC_CH_DEFY_F, 0.0);
			SetDeflectorFineV(DAC_CH_DEFX_F, 0);
			SetDeflectorFineV(DAC_CH_DEFY_F, 0);
			SetRotationAngle(SysParam.fRotateAngle);
			EnableBlanking(0);
		}
		goto DecipherOK;
	}
#define CMD_BAUD	"BAUD:"
#define LEN_BAUD	(sizeof(CMD_BAUD)-1)
	if (strncmp(szCommand, CMD_BAUD, LEN_BAUD) == 0) {	//BAUD:0:115200
		//BAUD:0:115200
		//BAUD:1:9600
		//BAUD:2:9600
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_BAUD], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BAUD_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 5)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BAUD_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nV2 < 0)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BAUD_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if ((nV1 >= 0) && (nV1 <= 3)) {
			NV_Settings.nBaudrate[nV1] = nV2;
			OpenSerialPort(nV1);
		}
		else if ((nV1 >= 4) && (nV1 <= 5)) {
			nV1 -= 4;
			NV_Settings.nPIDBaudrate[nV1] = nV2;
			SPI_SetBaud(nV1, nV2);
		}
		goto DecipherOK;
	}
#define CMD_BINPKT	"BINPKT:"
#define LEN_BINPKT	(sizeof(CMD_BINPKT)-1)
	if (strncmp(szCommand, CMD_BINPKT, LEN_BINPKT) == 0) {	//BINPKT:0
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_BINPKT], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BINPKT_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BINPKT_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bBinPkt = nV1;
		goto DecipherOK;
	}
#define CMD_BRSEL	"BRSEL:"
#define LEN_BRSEL	(sizeof(CMD_BRSEL)-1)
	if (strncmp(szCommand, CMD_BRSEL, LEN_BRSEL) == 0) {	//BRSEL:0
		//ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_BRSEL], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BRSEL_INV_PARAM_NUM));
			goto DecipherFail;
		}
		sprintf(szBuffer, "$%d,BRSEL:%d\n", HV_ADDR, nV1);
		RS485_WriteString(2, szBuffer);
		goto DecipherOK;
	}
#define CMD_BSEMODE		"BSEMODE:"
#define LEN_BSEMODE		(sizeof(CMD_BSEMODE)-1)
	if (strncasecmp(szCommand, CMD_BSEMODE, LEN_BSEMODE) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_BSEMODE], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BSEMODE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 1)||(nV1 > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_BSEMODE_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bBSEMode = (BYTE)nV1;
		goto DecipherOK;
	}
#define CMD_CALVID	"CALVID:"	//video signal calibration
#define LEN_CALVID	(sizeof(CMD_CALVID)-1)
	if (strncmp(szCommand, CMD_CALVID, LEN_CALVID) == 0) {
		if (szCommand[LEN_CALVID] == '?') {
			for (i = 0; i < VADC_CH_NUM; i++)
				printf("CALVID%d=%.3f\n", i, NV_Settings.fCalVideo[i]);
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_CALVID], "%f", &fV);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_CALVID_INV_PARAM_NUM));
			goto DecipherFail;
		}
		PreampOutputCalibration(fV);
		goto DecipherOK;
	}
#define CMD_CHECK	"CHECK"
#define LEN_CHECK	(sizeof(CMD_CHECK)-1)
	if (strncmp(szCommand, CMD_CHECK, LEN_CHECK) == 0) {
		printf("GETSET:1 -> BASE_CO[0-2], RANGE_CO[0-2], SETR:?\n");
		printf("GETSET:1 -> BSEMODE (1:4,2:2)\n");
		printf("GETSET:1 -> BAUD:2:XX, IP_PWR_CTRL, HV RS485 baudrate=115200\n");
		printf("GETSET:1 -> DELAY:4:OVERSCAN:0 (%d=1 usec)\n", NV_Settings.nSysClk);
		printf("GETSET:1 -> HVTYPE(0:S,1:M,2:Custom)\n");
		printf("VACPARA:?\n");
		printf("FOCUS:?  -> FOCUS:14:OBJTURN, FOCUS:15:USE_IN\n");
		printf("GETSET:4 -> UDPPORT (0:CMD_PORT(default), 1:nSrcPort), for CL control\n");
		printf("GETSET:4 -> AUTOVAC\n");
		printf("GETSET:4 -> ACCKV\n");
		printf("GETSET:4 -> DACT (0:AD5328, 1:AD5468, 2:AD5668)\n");
		printf("GETSET:4 -> ENFIFO\n");
		printf("GETSET:4 -> IDLET\n");
		printf("GETSET:4 -> OBJR(1:17 ohms, 2:14 kohms, 3:12 kohms)\n");
		printf("GETSET:4 -> OBJSTBYR (standby obj ratio)\n");
		goto DecipherRet;
	}
#define CMD_CHKTP	"CHKTP:"
#define LEN_CHKTP	(sizeof(CMD_CHKTP)-1)
	if (strncmp(szCommand, CMD_CHKTP, LEN_CHKTP) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_CHKTP], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_CHKTP_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_CHKTP_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bChkTP = nV1 & 0x0001;
		goto DecipherOK;
	}
#define CMD_CLEARSET	"CLEARSET"
#define LEN_CLEARSET	(sizeof(CMD_CLEARSET)-1)
	if (strncmp(szCommand, CMD_CLEARSET, LEN_CLEARSET) == 0) {
		ClearNVSettings();
		if (szCommand[LEN_CLEARSET + 1] == '1') //CLEARSET:1
			SaveUserParameters(&NV_Settings, sizeof(NV_SETTINGS));
		goto DecipherOK;
	}
#define CMD_CSHUNT	"CSHUNT:"
#define LEN_CSHUNT	(sizeof(CMD_CSHUNT)-1)
#define R0_VAL		8.2
#define R1_VAL		1.5
#define RP01_VAL		(R0_VAL*R1_VAL)/(R0_VAL+R1_VAL)
	if (strncmp(szCommand, CMD_CSHUNT, LEN_CSHUNT) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		if (szCommand[LEN_CSHUNT] == '?') {
			szBuffer[0] = 0;
			nV = (wSCAN_DO & DO_SHUNT0) ? 1 : 0;
			sprintf(szValue, "CSHUNT:0:ON_OFF,%d\n", nV);
			strcat(szBuffer, szValue);
			//strcat(szBuffer, "CSHUNT:1:ON_OFF\n"); //obsolete
			sprintf(szValue, "CSHUNT:2:SHUNTRATIO[0],%.3f\n", NV_Settings.fCoilShuntRatio[0]);
			strcat(szBuffer, szValue);
			//sprintf(szValue, "CSHUNT:3:SHUNTRATIO[1],%.3f\n", NV_Settings.fCoilShuntRatio[1]); //obsolete
			//strcat(szBuffer, szValue);
			sprintf(szValue, "CSHUNT:4:<R0> CSHUNT:4:%.3f,%.3f(R0)\n", R0_VAL, NV_Settings.fSenRCalc[0]);	//larger, 8.2
			strcat(szBuffer, szValue);
			sprintf(szValue, "CSHUNT:5:<R1> CSHUNT:5:%.3f,%.3f(R0||R1)\n", RP01_VAL, NV_Settings.fSenRCalc[1]); 	//parallel linked value, smaller
			strcat(szBuffer, szValue);
			sprintf(szValue, "CSHUNT:6:<R6> CSHUNT:6:%.1f,%.1f(R0)\n", R0_VAL, NV_Settings.fSenR[0]); 	//single resistor, larger one
			strcat(szBuffer, szValue);
			sprintf(szValue, "CSHUNT:7:<R7> CSHUNT:7:%.1f,%.1f(R1)\n", R1_VAL, NV_Settings.fSenR[1]);		//single resistor, smaller one
			strcat(szBuffer, szValue);
			sprintf(szValue, "CSHUNT:8:<RS0> CSHUNT:8:4.7,%.1f\n", NV_Settings.fShuntR[0]);
			strcat(szBuffer, szValue);
			sprintf(szValue, "CSHUNT:9:<RS1> CSHUNT:9:135 (270//270),%.1f\n", NV_Settings.fShuntR[1]);
			strcat(szBuffer, szValue);
			sprintf(szValue, "CSHUNT:10:IFACTOR %.3f\n", NV_Settings.fIMaxFactor);
			strcat(szBuffer, szValue);
			sprintf(szValue, "CSHUNT:11:COILR %.3f (small/whole)\n", NV_Settings.fCoilRatio);
			strcat(szBuffer, szValue);
			//coil with different MAG, switch threshold
			sprintf(szValue, "CSHUNT:12:ITHR %.3f,IMAX_TH %.3f\n", NV_Settings.fIThRatio, IMAX_LOWER_TH);
			strcat(szBuffer, szValue);
			TxString(nType, szBuffer);
			goto DecipherRet;
		}
		nPN = sscanf(&szCommand[LEN_CSHUNT], "%d %f", &nV1, &fV);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_CSHUNT_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 12)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_CSHUNT_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		nV2 = (int)fV;
		if (nV1 == 0) {			//CSHUNT:0:nV2
			SetZoomIO(nV1, nV2);
		}
		//else if (nV1 == 1) {	//CSHUNT:1:nV2
		//	SetZoomIO(nV1, nV2);
		//}
		else if (nV1 == 2) {	//CSHUNT:2:26.0
			NV_Settings.fCoilShuntRatio[0] = fV;
			goto DecipherOK;
		}
		else if (nV1 == 3) {	//CSHUNT:3:51.0
			NV_Settings.fCoilShuntRatio[1] = fV;
			goto DecipherOK;
		}
		else if (nV1 == 4) {	//CSHUNT:4:8.2
			//NV_Settings.fSenRCalc[0] = fV;	//larger resistor
			goto DecipherOK;
		}
		else if (nV1 == 5) {	//CSHUNT:5:1.268
			//NV_Settings.fSenRCalc[1] = fV; 	//smaller, parallel linked resistor
			goto DecipherOK;
		}
		else if (nV1 == 6) {	//CSHUNT:6:8.2
			NV_Settings.fSenR[0] = fV; //8.2, real resistor value
			fV1 = fV;
			fV2 = NV_Settings.fSenR[1]; //1.5, real resistor value
			NV_Settings.fSenRCalc[0] = fV;
			NV_Settings.fSenRCalc[1] = (fV1 * fV2) / (fV1 + fV2);
			goto DecipherOK;
		}
		else if (nV1 == 7) {	//CSHUNT:7:1.5
			NV_Settings.fSenR[1] = fV;
			fV1 = NV_Settings.fSenR[0];
			fV2 = fV;
			NV_Settings.fSenRCalc[1] = (fV1 * fV2) / (fV1 + fV2);
			goto DecipherOK;
		}
		else if (nV1 == 8) {	//CSHUNT:8:4.7, for example
			NV_Settings.fShuntR[0] = fV;
			NV_Settings.fCoilShuntRatio[0] = (NV_Settings.fShuntR[1]/NV_Settings.fShuntR[0]) + 1.0;
			goto DecipherOK;
		}
		else if (nV1 == 9) {	//CSHUNT:9:135, for example
			NV_Settings.fShuntR[1] = fV;
			NV_Settings.fCoilShuntRatio[0] = (NV_Settings.fShuntR[1]/NV_Settings.fShuntR[0]) + 1.0;
			goto DecipherOK;
		}
		else if (nV1 == 10) {	//CSHUNT:10:0.99
			NV_Settings.fIMaxFactor = fV;
			goto DecipherOK;
		}
		else if (nV1 == 11) {	//CSHUNT:11:0.666
			NV_Settings.fCoilRatio = fV; //smaller coil/whole coil ratio
			goto DecipherOK;
		}
		else if (nV1 == 12) {	//CSHUNT:12:0.8
			NV_Settings.fIThRatio = fV;
			goto DecipherOK;
		}

		goto DecipherOK;
	}
#define CMD_DEBUG	"DEBUG:"
#define LEN_DEBUG	(sizeof(CMD_DEBUG)-1)
	if (strncmp(szCommand, CMD_DEBUG, LEN_DEBUG) == 0) {
		if (IsCommandEnabled(nType) != SUCCESS) goto DecipherRet;
		ReplaceChar(szCommand, ':', ' ');
		if (szCommand[LEN_DEBUG] == '?') {
			sprintf(szBuffer, "DEBUG=%d\n", (int)bDebug);
			strcat(szBuffer, "DEBUG:1   debug command queue\n");
			strcat(szBuffer, "DEBUG:2   vacuum status\n");
			strcat(szBuffer, "DEBUG:3   deflector current\n");
			strcat(szBuffer, "DEBUG:4   TCP/IP transmission\n");
			strcat(szBuffer, "DEBUG:5   gradient calculation\n");
			strcat(szBuffer, "DEBUG:6   ADC, DAC\n");
			strcat(szBuffer, "DEBUG:7   auto focus\n");
			strcat(szBuffer, "DEBUG:8   debug GetIO_In\n");
			strcat(szBuffer, "DEBUG:9   SI:X:output test\n");
			strcat(szBuffer, "DEBUG:10  vacuum change rate\n");
			strcat(szBuffer, "DEBUG:11  scanning process\n");
			strcat(szBuffer, "DEBUG:12  turbo pump message\n");
			strcat(szBuffer, "DEBUG:13  power line info\n");
			strcat(szBuffer, "DEBUG:14  debug command\n");
			strcat(szBuffer, "DEBUG:15  debug db3 video signal\n");
			strcat(szBuffer, "DEBUG:16  debug db3 video DAC\n");
			strcat(szBuffer, "DEBUG:17  rotation, obj\n");
			strcat(szBuffer, "DEBUG:18  SetIO_Out()\n");
			strcat(szBuffer, "DEBUG:19  serial2 message\n");
			strcat(szBuffer, "DEBUG:20  IP_PWR_CTRL\n");
			strcat(szBuffer, "DEBUG:21  custom HV\n");
			strcat(szBuffer, "DEBUG:22  IO DAC clk, SCK\n");
			TxString(nType, szBuffer);
			goto DecipherRet;
		}
		nPN = sscanf(&szCommand[LEN_DEBUG], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_DEBUG_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 22)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_DEBUG_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bDebug = (BYTE)nV1;
		goto DecipherOK;
	}
#define CMD_DELAY		"DELAY:"
#define LEN_DELAY		(sizeof(CMD_DELAY)-1)
	if (strncmp(szCommand, CMD_DELAY, LEN_DELAY) == 0)	{
		if (szCommand[LEN_DELAY] == '?') {
			//pixel dwelling time
			printf("DELAY:1:DESCENT:PIXEL %d:%d\n", NV_Settings.nDelayDescent, NV_Settings.nPixelDwellTime);
			//auto calculation according to pixel number
			printf("DELAY:2:AUTOXLINEDELAY:0 %d\n", (int)NV_Settings.bAutoXLineDelay);
			fV = (float)SysParam.wXLineDelay * 4.0 / (float)NV_Settings.nSysClk / 1000.0;
			printf("DELAY:3:XLINEDELAY:0 %d,%.3f ms\n", SysParam.wXLineDelay, fV);
			printf("DELAY:4:OVERSCAN:0 %d\n", (int)NV_Settings.wDelayOverscan);
			printf("DELAY:5:PKTDELAY1:PKTDELAY2 %d:%d\n", nPktDelay1, nPktDelay2);
			printf("DELAY:8:DURA:HWUSDLY %d (%d MHz),J2[12]\n", (int)bHWUSDelay, NV_Settings.nSysClk);
			printf("DELAY:9:LB:0 %d\n", NV_Settings.nDelayLB); //delay at left bottom corner
			printf("DELAY:10:PKTDELAY3:PKTDELAY4 %d:%d\n", nPktDelay3, nPktDelay4); //delay for EXT_FOCUS
			printf("DELAY:11:PWRSAVE:0 %d\n", (int)NV_Settings.bScanPowerSave); //delay for EXT_FOCUS
			printf("DELAY:12:DURA:0 CON52.8,J85-J86\n");
			goto DecipherRet;
		}
		//ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_DELAY], "%d:%d:%d", &nB, &nV1, &nV2);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_DELAY_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nB < 1)||(nB > 12)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_DELAY_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nV1 < 0)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_DELAY_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if( (nV2 < 0)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_DELAY_INV_PARAM3_VAL));
			goto DecipherFail;
		}
		switch (nB) {
			case 1: //nB=1 //DELAY:1:DESCENT:PIXEL
				if (bFastScan) {
					NV_Settings.nDelayDescent = nV1;
					NV_Settings.nPixelDwellTime = nV2;
					pwFpga[SET_DELAY_DESCENT] = (WORD)NV_Settings.nDelayDescent; //descent delay
					pwFpga[SET_DELAY_PIXEL] = (WORD)NV_Settings.nPixelDwellTime; //delay per pixel
				}
				else {
					nDelayIDec = nV1; //the larger, the longer
					//nDelayIDecStep = nV2; //the larger, the slower
					NV_Settings.nPixelDwellTime = nV2; //the larger, the slower
				}
				break;
			case 2:
				if( (nV1 > 1)){
					sprintf(szValue, ErrorMsg::GetErrorText(ERR_DELAY_INV_PARAM2_VAL));
					goto DecipherFail;
				}
				NV_Settings.bAutoXLineDelay = nV1;
				//nDelayPerLine = nV1; //FW modulated FastScan()
				break;
			case 3: //DELAY:3:12000:0
				wV = (WORD)nV1; //time unit 200 ns(20 MHz)
				if (wV < 1000) wV = 1000; //minimum 1000=200 usec
				SysParam.wXLineDelay = wV;
				pwFpga[SET_X_LINE_DELAY] = SysParam.wXLineDelay;
				break;
			case 4: //DELAY:4:20:0
				NV_Settings.wDelayOverscan = (WORD)nV1;
				pwFpga[SET_DELAY_OVERSCAN] = NV_Settings.wDelayOverscan;
				break;
			case 5: //delay:5:0:200,  delay:5:1:200
				nPktDelay1 = nV1;
				nPktDelay2 = nV2;
				break;
			case 8:
				if( (nV2 > 1)){
					sprintf(szValue, ErrorMsg::GetErrorText(ERR_DELAY_INV_PARAM3_VAL));
					goto DecipherFail;
				}
				bHWUSDelay = (nV2 == 1) ? 1 : 0; //don't care
				LED_YELLOW_ON; //J2[12]
				delay_us(nV1); //debug
				LED_YELLOW_OFF;
				break;
			case 9:
				if (nV1 < 0) nV1 = 0; //at least 1, 0 is for test only
				NV_Settings.nDelayLB = nV1;
				break;
			case 10:
				nPktDelay3 = nV1;
				nPktDelay4 = nV2;
				break;
			case 11: //obsolete
				if( (nV1 > 1)){
					sprintf(szValue, ErrorMsg::GetErrorText(ERR_DELAY_INV_PARAM2_VAL));
					goto DecipherFail;
				}
				NV_Settings.bScanPowerSave = (BYTE)nV1;
				break;
			case 12: //hardware delay test
				bHWUSDelay = 1;
				LED_YELLOW_ON; //J12
				delay_us(nV1); //debug
				LED_YELLOW_OFF;
				bHWUSDelay = 0;
				break;
		}
		goto DecipherOK;
	}
#define CMD_ECHO		"ECHO"
#define LEN_ECHO		(sizeof(CMD_ECHO)-1)
	if (strncmp(szCommand, CMD_ECHO, LEN_ECHO) == 0) {
		if (bInitOK == 0) goto DecipherRet;
		nEchoTimeout = ECHO_TIMEOUT;
		TxString(nType, "ECHO"); //connection test
		goto DecipherRet; //no OK return
	}
#define CMD_ENEDS	"ENEDS:"
#define LEN_ENEDS	(sizeof(CMD_ENEDS)-1)
	if (strncmp(szCommand, CMD_ENEDS, LEN_ENEDS) == 0) {	//get image ADC
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_ENEDS], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENEDS_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENEDS_INV_PARAM_VAL));
			goto DecipherFail;
		}
		if (nV1 == 2) { //for debug of OINA interface
			SetEDSOn(1);
		}
		else if (NV_Settings.bEDSOnByHW == 0)
			EnableEDS((BYTE)nV1);		//0:disable, 1:firmware on
		goto DecipherOK;
	}
#define CMD_ENFIFO	"ENFIFO:"
#define LEN_ENFIFO	(sizeof(CMD_ENFIFO)-1)
	if (strncmp(szCommand, CMD_ENFIFO, LEN_ENFIFO) == 0) {	//get image ADC
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_ENFIFO], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENFIFO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENFIFO_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bEnableFIFO = (BYTE)nV1;
		goto DecipherOK;
	}
#define CMD_EDSDWT	"EDSDWT:"	//EDS dwelling time in msec
#define LEN_EDSDWT	(sizeof(CMD_EDSDWT)-1)
	if (strncmp(szCommand, CMD_EDSDWT, LEN_EDSDWT) == 0)	//get image ADC
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_EDSDWT], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_EDSDWT_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( nV1 < 0){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_EDSDWT_INV_PARAM_VAL));
			goto DecipherFail;
		}
		nEDSDWT = nV1;
		pwFpga[DELAY_MS_NUM] = nEDSDWT; //time unit: ms
		delay_void();
		pwFpga[DELAY_MS_NUM] = nEDSDWT; //write twice for redundancy
		goto DecipherOK;
	}
#define CMD_EDSHS	"EDSHS:"	//EDS handshake signal output
#define LEN_EDSHS	(sizeof(CMD_EDSHS)-1)
	if (strncmp(szCommand, CMD_EDSHS, LEN_EDSHS) == 0) {
		nPN = sscanf(&szCommand[LEN_EDSHS], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_EDSHS_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_EDSHS_INV_PARAM_VAL));
			goto DecipherFail;
		}
		//0: no handshake wait
		//1: handshake, output ext_x_pulse<->n_ext_x_ack, ext_y_pulse signal
		nEDSHandshake = nV1;
		pwFpga[EN_EDS_HANDSHAKE] = nEDSHandshake;
		delay_void();
		pwFpga[EN_EDS_HANDSHAKE] = nEDSHandshake;  //write twice for redundancy
		delay_void();
		pwFpga[DELAY_MS_NUM] = nEDSDWT;
		delay_void();
		pwFpga[DELAY_MS_NUM] = nEDSDWT; //write twice for redundancy
		goto DecipherOK;
	}
#define CMD_EDSONHW		"EDSONHW:"	//EDS dwelling time in msec
#define LEN_EDSONHW		(sizeof(CMD_EDSONHW)-1)
	if (strncmp(szCommand, CMD_EDSONHW, LEN_EDSONHW) == 0)	{ //get image ADC
		nPN = sscanf(&szCommand[LEN_EDSONHW], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_EDSONHW_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_EDSONHW_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bEDSOnByHW = (nV1 == 1) ? 1 : 0;
		goto DecipherOK;
	}
//
#define CMD_EDS_START	"EDS_START"
#define LEN_EDS_START	(sizeof(CMD_EDS_START)-1)
	if (strncmp(szCommand, CMD_EDS_START, LEN_EDS_START) == 0) {
		if (SysParam.bVacuumState != VAC_ST_READY){
			nErrCode = ERR_INV_VAC_ST;
//			sprintf(szValue, ErrorMsg::GetErrorText(ERR_INV_VAC_ST));
		}
		goto DecipherFail;
		//
		SetVOPPower(1); //turn on VOPA and VOPB
		OSTimeDly(4);
		SetHVPower(1);	//turn on HV power
		OSTimeDly(2);
		SetObjOn(1);
		//
		fV = PercentToVolt(fBrightness[0], DAC_CH_BR);
		SetVoltage(BOARD_SCAN, DAC_CH_BR0, fV);
		fV = PercentToVolt(fContrast[0], DAC_CH_CO);
		SetVoltage(BOARD_SCAN, DAC_CH_CO, fV);
		//
		SetRotationAngle(SysParam.fRotateAngle); //unit degree
		//
		goto DecipherOK;
	}
#define CMD_EDS_STOP	"EDS_STOP"
#define LEN_EDS_STOP	(sizeof(CMD_EDS_STOP)-1)
	if (strncmp(szCommand, CMD_EDS_STOP, LEN_EDS_STOP) == 0)	//get image ADC
	{
		SetObjOn(0);
		OSTimeDly(1);
		SetHVPower(0);	//turn off HV power
		OSTimeDly(1);
		SetVOPPower(1); //turn off VOPA and VOPB
		goto DecipherOK;
	}
#define CMD_ENCL		"ENCL:"
#define LEN_ENCL		(sizeof(CMD_ENCL)-1)
	if (strncmp(szCommand, CMD_ENCL, LEN_ENCL) == 0) {//scan stop
		p = strchr(szCommand, ':');
		if (p == NULL)
			goto DecipherRet;
		nPN = sscanf(&szCommand[LEN_ENCL], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENCL_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENCL_INV_PARAM_VAL));
			goto DecipherFail;
		}
		if (nV == 1) {
			SysParam.bPrevScanning = SysParam.bScanning;
			SysParam.bScanning = OP_ENABLE_CL;
		}
		else if (nV == 0) {
			SysParam.bScanning = SysParam.bPrevScanning;
		}
		goto DecipherRet;
	}
#define CMD_ENCOILSW	"ENCOILSW:"	//enable coil switch function
#define LEN_ENCOILSW	(sizeof(CMD_ENCOILSW)-1)
	if (strncmp(szCommand, CMD_ENCOILSW, LEN_ENCOILSW) == 0)	//get image ADC
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_ENCOILSW], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENCOILSW_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 3)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENCOILSW_INV_PARAM_VAL));
			goto DecipherFail;
		}
		//enable/disable coil switch at high magnification
		if (nV1 <= 1) //ENCOILSW:0 or ENCOILSW:1
			NV_Settings.bEnCoilSW = (nV1 == 1) ? 1 : 0;
		else if (nV1 == 2) { //for test only
			SetDigitalOutputBit(BOARD_IO, 10, 0);
		}
		else if (nV1 == 3) { //for test only
			SetDigitalOutputBit(BOARD_IO, 10, 1);
		}
		goto DecipherOK;
	}
#define CMD_ENVENT	"ENVENT:"
#define LEN_ENVENT	(sizeof(CMD_ENVENT)-1)
	if (strncmp(szCommand, CMD_ENVENT, LEN_ENVENT) == 0)	//get image ADC
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_ENVENT], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENVENT_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENVENT_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bEnableVent = nV1 & 0x0001;
		goto DecipherOK;
	}
#define CMD_ENVIDEO		"ENVADC:" 		//debug
#define LEN_ENVIDEO		(sizeof(CMD_ENVIDEO)-1)
	if (strncmp(szCommand, CMD_ENVIDEO, LEN_ENVIDEO) == 0)	//get image ADC
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_ENVIDEO], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENVADC_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENVADC_INV_PARAM_VAL));
			goto DecipherFail;
		}
		EnableVideoADC(nV1);
		goto DecipherOK;
	}
#define CMD_ENWDT		"ENWDT:"
#define LEN_ENWDT		(sizeof(CMD_ENWDT)-1)
	if (strncmp(szCommand, CMD_ENWDT, LEN_ENWDT) == 0) //ENWDT test
	{
		nV1 = atoi(&szCommand[LEN_ENWDT]);
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ENWDT_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bEnableWDT = nV1; //(nV == 1) ? 1 : 0;
		/*if (NV_Settings.bEnableWDT == 1)
			EnableWDT(1, WDT_TIMEOUT);
		else if (NV_Settings.bEnableWDT == 0)
			EnableWDT(0, WDT_TIMEOUT);
		*/
		goto DecipherOK;
	}
#define CMD_ERROR	"ERROR:"
#define LEN_ERROR	(sizeof(CMD_ERROR)-1)
	if (strncmp(szCommand, CMD_ERROR, LEN_ERROR) == 0)	//get error message
	{	//"ERROR:?"
		if (szCommand[LEN_ERROR] == '?') //ERROR:?
			DisplayErrorMessage(nType);
		else if (szCommand[LEN_ERROR] == '0') //ERROR:0
			ClearErrorMessage();
		else{
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ERROR_NOT_QUESTIONMARK_OR_0));
			goto DecipherFail;
		}
		goto DecipherRet;
	}
#if (SCAN_PCB_VER >= 10 && SCAN_PCB_VER <= 15)
#define CMD_FASTCLOCK	"FASTCLOCK:" //oscillator=20 MHz(0) or 30 MHz(1)
#define LEN_FASTCLOCK	(sizeof(CMD_FASTCLOCK)-1)
	if (strncmp(szCommand, CMD_FASTCLOCK, LEN_FASTCLOCK) == 0) //set contrast
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_FASTCLOCK], "%d", &nV1); //fV is percentage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_FASTCLOCK_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_FASTCLOCK_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bFastClock = (nV1 == 1) ? 1 : 0;
		pwFpga[SET_FAST_CLOCK] = nV1;
		goto DecipherOK;
	}
#endif
//
#define CMD_FOCUS		"FOCUS:"
#define LEN_FOCUS		(sizeof(CMD_FOCUS)-1)
	if (strncmp(szCommand, CMD_FOCUS, LEN_FOCUS) == 0) //set contrast
	{	//
		if (szCommand[LEN_FOCUS] == '?') {
			printf("FOCUS:0:SET %.3f\n", fFocus);
			printf("FOCUS:1:1 AF_RUN\n");
			printf("FOCUS:2:AFTHR %.2f\n", NV_Settings.fAutoFocusThreshold);
			printf("FOCUS:3:ROTADJ %d\n", (int)NV_Settings.bObjRotateAdj);
			printf("FOCUS:4:Y_NDX_INC %d\n", nYNdxInc);
			printf("FOCUS:5:RANGE %.3f, ranged auto focus\n", fFocusRange);
			printf("FOCUS:6:AF_PIXNUM %ld\n", NV_Settings.lAFPixelNum);
			printf("FOCUS:7:GRAD_DIFF %d\n", NV_Settings.nEdgeDiff);
			printf("FOCUS:8:AF_MIN %.1f\n", NV_Settings.fFocusMin);
			printf("FOCUS:9:AF_MAX %.1f\n", NV_Settings.fFocusMax);
			printf("FOCUS:10:AF_LIMNUM %d\n", NV_Settings.nAFLimitNum);
			printf("FOCUS:11:FOV %d\n", (int)NV_Settings.bFocusFOV);
			printf("FOCUS:12:AF_SW\n");
			printf("FOCUS:13:X_NDX_INC %d,%d\n", nXNdxInc, NV_Settings.sXNdxInc);
			printf("FOCUS:14:OBJTURN %d\n", NV_Settings.nObjTurn);
			printf("FOCUS:15:USE_IN %d\n", (int)NV_Settings.bUseIN);
			printf("FOCUS_VC = %.3f, FOCUS_VF = %.3f\n", fFocusVC, fFocusVF);
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_FOCUS], "%d %f", &nV1, &fV); //fV is percentage
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_FOCUS_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( ( nV1 < 0)||(nV1 > 15)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_FOCUS_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		nV2 = (int)fV;
		if (nV1 == 0) { //set focus value
			if( ( fV< 0)||(fV > 100)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_FOCUS_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			//fV is the percentage from fObjIMin to fObjIMax
			SetFocus(fV);
			bCommandDelay = 1;	//wait until objective current is stable
		}
		else if (nV1 == 1) { //FOCUS:1:1 (fast auto focus)
			if( ( nV2< 0)||(nV2 > 1)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_FOCUS_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			if ((nV2 == 1) && (SysParam.bScanning == OP_SEM_SCAN)) { //restart
				SysParam.bScanning = OP_AUTO_FOCUS; //auto focus operation
				pwFpga[SET_X_NDX_INC] = nXNdxInc; //default = 2
				EnablePLSync(0);
				bAutoFocusStart = 1;
				EnableAutoFocus(1);
				goto DecipherOK;
			}
			else { //if (nV2 == 0) {//FOCUS:1:0, leave auto focus mode
				bAutoFocusStart = 0;
				if (bPLSync == 1)
					EnablePLSync(1);
				EnableAutoFocus(0);
				goto DecipherOK;
			}
		}
		else if (nV1 == 2) { //set loop out threshold
			NV_Settings.fAutoFocusThreshold = fV; //Omar need to be defined
		}
		else if (nV1 == 3) { //focus rotation angle adjustment
			//FOCUS:3:1 (enable), FOCUS:3:0 (disable)
			if( ( fV< 0)||(fV > 1)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_FOCUS_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			NV_Settings.bObjRotateAdj = (fV == 1) ? 1 : 0;
		}
		else if (nV1 == 4) { //focus rotation angle adjustment
			//FOCUS:4:Y_NDX_INC
			nV2 = (int)fV; //Omar need to be defined
			nYNdxInc = nV2;
			pwFpga[SET_Y_NDX_INC] = nV2;
		}
		else if (nV1 == 5) { //FOCUS:5:4.0, auto focus around current operation point
			if (SysParam.bScanning == OP_SEM_SCAN) {
				fFocusRange = fV; //Omar need to be defined
				SysParam.bScanning = OP_AUTO_FOCUS; //auto focus operation
				bAutoFocusStart = 2;
				EnableAutoFocus(1);
				goto DecipherOK;
			}
		}
		else if (nV1 == 6) {
			NV_Settings.lAFPixelNum = (long)fV; //Omar need to be defined
		}
		else if (nV1 == 7) { //gradient difference threshold
			//only sharp edges are calculated for gradient, DELAY:7:48:0
			wV = (WORD)fV; //Omar need to be defined
			pwFpga[SET_GRADIENT_DIFF] = wV;
			NV_Settings.nEdgeDiff = wV; //calculate gradient by FW
		}
		else if (nV1 == 8) {
			NV_Settings.fFocusMin = fV; //Omar need to be defined
		}
		else if (nV1 == 9) {
			NV_Settings.fFocusMax = fV; //Omar need to be defined
		}
		else if (nV1 == 10) {
			nV2 = (int)fV;
			if (nV2 > AF_LIMIT_NUM) nV2 = AF_LIMIT_NUM;
			NV_Settings.nAFLimitNum = nV2;
		}
		else if (nV1 == 11) {
			if( ( fV< 0)||(fV > 1)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_FOCUS_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			NV_Settings.bFocusFOV = (BYTE)fV;
		}
		else if (nV1 == 12) { //FOCUS:12:1
			if( ( nV2< 0)||(nV2 > 2)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_FOCUS_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			if (nV2 == 1) {
				if (SysParam.bPrevScanning != OP_AUTO_FOCUS_SW)
					SysParam.bPrevScanning = SysParam.bScanning;
				SysParam.bScanning = OP_AUTO_FOCUS_SW; //auto focus operation
				bAutoFocusStart = 1; //initialization
				EnableAutoFocus(1);
				goto DecipherOK;
			}
			else if (nV2 == 2) { //FOCUS:12:2
				bAutoFocusStart = 2;
			}
			else { //FOCUS:12:0, leave auto focus mode
				EnableAutoFocus(0);
				SysParam.bScanning = SysParam.bPrevScanning;
			}
		}
		else if (nV1 == 13) {	//FOCUS:13:X_NDX_INC
			nV2 = (int)fV; //Omar need to be defined
			nXNdxInc = nV2;
			NV_Settings.sXNdxInc = (short)nV2;
			pwFpga[SET_X_NDX_INC] = nV2;
		}
		else if (nV1 == 14) {
			NV_Settings.nObjTurn = nV2; //Omar need to be defined
			SetObjR((int)NV_Settings.bObjR); //set fINT[]
		}
		else if (nV1 == 15) {
			if( ( nV2< 0)||(nV2 > 1)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_FOCUS_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			NV_Settings.bUseIN = (BYTE)nV2; //Omar need to be defined
		}
		goto DecipherOK;
	}
	/*if (strncmp(szCommand, "FHPDP:", 6) == 0)	//Footer/Header Per Data Packet
	{
		nPN = sscanf(&szCommand[6], "%d", &nN);
		if (nPN != 1) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}
		//2:header,1:footer,0:none
		bFooterHeader = (nN == 2) ? 2 :
			(nN == 0) ? 0 : 1;
		goto DecipherOK;
	}*/
#define CMD_FREQGEN		"FREQGEN:"
#define LEN_FREQGEN		(sizeof(CMD_FREQGEN)-1)
	if (strncmp(szCommand, CMD_FREQGEN, LEN_FREQGEN) == 0)
	{
		if (szCommand[LEN_FREQGEN] == '?') {
			printf("FREQGEN:0:FREQ  STOP SINE\n");
			printf("FREQGEN:1:FREQ  START SINE\n");
			printf("FREQGEN:2:VOLT  SET V START\n");
			printf("FREQGEN:3:VOLT  SET V STOP\n");
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_FREQGEN], "%d %f", &nV1, &fV);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_FREQGEN_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 3)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_FREQGEN_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if (nV1 == 0) {
			//generate simulation pattern DAC output
			//SetFreqGenerator(0, fV); //disable sine generator
		}
		else if (nV1 == 1) {
			//SetFreqGenerator(1, fV); //enable sine generator
			GenerateSinePattern();
		}
		else if (nV1 == 2) {
			fDACXFSet = fV;
			nEnableDACXFSet = 1;
			if (SysParam.bScanning == OP_SEM_SCAN) { //scan again
				bScanAbort = 1; //stop scanning
				OSTimeDly(2);
				SetViewArea(SysParam.fMagNow, nScanPixelNumX, nScanPixelNumY);
				SetDescentCurve(nScanPixelNumX, nScanPixelNumY);
				SysParam.bScanning = OP_SEM_SCAN; //scan again
			}
		}
		else if (nV1 == 3) {
			nEnableDACXFSet = 0;
			//restore original pattern
			if (SysParam.bScanning == OP_SEM_SCAN) { //scan again
				bScanAbort = 1; //stop scanning
				OSTimeDly(2);
				SetViewArea(SysParam.fMagNow, nScanPixelNumX, nScanPixelNumY);
				SetDescentCurve(nScanPixelNumX, nScanPixelNumY);
				SysParam.bScanning = OP_SEM_SCAN; //scan again
			}
		}
		goto DecipherOK;
	}
#define CMD_GETGENPAR	"GETGENPAR"		//get general parameter
#define LEN_GETGENPAR	(sizeof(CMD_GETGENPAR)-1)
	if (strncmp(szCommand, CMD_GETGENPAR, LEN_GETGENPAR) == 0)
	{
		SetParam.bHeader1 = SET_HEADER_1;
		SetParam.bHeader2 = SET_HEADER_2;
		SetParam.bTurboType = NV_Settings.nTurboType;
		for (i = 0; i < OBJ_ENERGY_NUM; i++) {
			SetParam.fObjIMax[i] = ByteSwapFloat(NV_Settings.fObjIMax[i]);
			SetParam.fObjIMin[i] = ByteSwapFloat(NV_Settings.fObjIMin[i]);
		}
		SetParam.bScaleMode = NV_Settings.bScaleMode;
		SetParam.bHWUSDelay = bHWUSDelay;
		SetParam.fMagMinimum = ByteSwapFloat(fMagMinimum);
		SetParam.fAPerMM[0] = ByteSwapFloat(NV_Settings.fAPerMM[0][0]);
		SetParam.fAPerMM[1] = ByteSwapFloat(NV_Settings.fAPerMM[0][1]);
		//
		//SetParam.bEnablePLModX = NV_Settings.bEnablePLModX;		//26, enable PL moduation X
		//SetParam.bEnablePLModY = NV_Settings.bEnablePLModY;
		//SetParam.fPLAmp = ByteSwapFloat(NV_Settings.fPLAmp);
		//SetParam.fPLPhase = ByteSwapFloat(NV_Settings.fPLPhase);
		//
		SetParam.bPLFreq = NV_Settings.bPLFreq;
		SetParam.bPLSyncAuto = NV_Settings.bPLSyncAuto;
		//
		wV = NV_Settings.nTurboReadySpeed[NV_Settings.nTurboType];
		SetParam.wTurboReadySpeed = ByteSwapWord(wV);
		SetParam.bBinPkt = NV_Settings.bBinPkt;
		TxData(nType, (BYTE *)&SetParam, sizeof(SET_PARAM));
		goto DecipherOK;
	}
#define CMD_GETAI	"GETAI:"		//get general parameter
#define LEN_GETAI	(sizeof(CMD_GETAI)-1)
	if (strncmp(szCommand, CMD_GETAI, LEN_GETAI) == 0)
	{	//<B> is not used now
		ReplaceChar(szCommand, ':', ' '); //GETAI:<B>:<C><CR><LF>
		nPN = sscanf(&szCommand[LEN_GETAI], "%d %d", &nB, &nCh);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GETAI_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nCh < 0)||(nCh > 23)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GETAI_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		fV = GetVoltage(nCh); //nCh = channel index, 0-based
		sprintf(szBuffer, "AI[%d]=%.3f V\n", nCh, fV);
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
#define CMD_GETAIALL	"GETAIALL"
#define LEN_GETAIALL	(sizeof(CMD_GETAIALL)-1)
	if (strncmp(szCommand, CMD_GETAIALL, LEN_GETAIALL) == 0)
	{
		if (bDB3Busy == 1) {
			PushCommandToQueue(szCommand, nType);
			goto DecipherRet;
		}
		if (szCommand[LEN_GETAIALL] == ':') //GETAIALL:1
			GetAllVoltage(fADC); //get new ADC values
		else
			GetAllVoltageNoEx(fADC); //get current values
		sprintf(szBuffer, "AIALL=");
		for (nCh = 0; nCh < ADC_CH_NUM_MAX; nCh++) {
			sprintf(szValue, "%.3f,", SysParam.fADC[nCh]);
			strcat(szBuffer, szValue);
			if (nCh % 4 == 3)
				strcat(szBuffer, "\n");
		}
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
#define CMD_GETAII	"GETAII"
#define LEN_GETAII	(sizeof(CMD_GETAII)-1)
	if (strncmp(szCommand, CMD_GETAII, LEN_GETAII) == 0)	//get image ADC
	{
		if (bDB3Busy == 1) {
			PushCommandToQueue(szCommand, nType);
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[7], "%d", &nN);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GETAII_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nN < 0)||(nN > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GETAII_INV_PARAM_VAL));
			goto DecipherFail;
		}
		if ((nN == 1) || (nN == 2)) { //GETAII:1
			szBuffer[0] = 0;
			for (i = 0; i < VIDEO_ADC_CH_NUM; i++) {
				sprintf(szValue, "AII[%d]=%.3f V,0X%04X\n", i, SysParam.fVADC[i], 0);
				strcat(szBuffer, szValue);
			}
			sprintf(szValue, "VSUM=%.3f V,0X%04X\n", SysParam.fVADC[5], 0);
			strcat(szBuffer, szValue);
			sprintf(szValue, "HDIFF=%.3f V,0X%04X\n", SysParam.fVADC[6], 0);
			strcat(szBuffer, szValue);
			sprintf(szValue, "VDIFF=%.3f V,0X%04X\n", SysParam.fVADC[7], 0);
			strcat(szBuffer, szValue);
		}
		else { //GETAII:0 or GETAII
			wV = GetVideoADC(); //return value will depend on shadow mode
			fV = WordToVolt(wV, ADC_16BIT, 5.0, 0.0);
			sprintf(szBuffer, "AII=%.3f V,0X%04X\n", fV, wV);
		}
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
	//GETCI:<B>:<C>:<CR><LF>, get current value
#define CMD_GETCI	"GETCI"
#define LEN_GETCI	(sizeof(CMD_GETCI)-1)
	if (strncmp(szCommand, CMD_GETCI, LEN_GETCI) == 0)
	{
		//ReplaceChar(szCommand, ':', ' ');
		//nPN = sscanf(&szCommand[LEN_GETCI], "%d %d", &nB, &nN);
		//ISEN_CH_DEFX(0),ISEN_CH_DEFY(1),ISEN_CH_STIGX(2),ISEN_CH_STIGY(3)
		//ISEN_CH_OBJ(4),ISEN_CH_AL0~ISEN_CH_AL3
		//fI = GetCoilCurrent(nN); //nN=ISEN_CH_XXX
		//sprintf(szBuffer, "%.3f mA\n", fI);
		//TxString(nType, szBuffer);
		fI = GetCoilCurrent(0); //nN=ISEN_CH_XXX
		sprintf(szBuffer, "DEFX = %.3f mA\n", fI * 1000.0);
		TxString(nType, szBuffer);
		fI = GetCoilCurrent(1); //nN=ISEN_CH_XXX
		sprintf(szBuffer, "DEFY = %.3f mA\n", fI * 1000.0);
		TxString(nType, szBuffer);
		fI = GetCoilCurrent(2);
		sprintf(szBuffer, "STIGX = %.3f mA\n", fI * 1000.0);
		TxString(nType, szBuffer);
		fI = GetCoilCurrent(3);
		sprintf(szBuffer, "STIGY = %.3f mA\n", fI * 1000.0);
		TxString(nType, szBuffer);
		fI = GetCoilCurrent(4);
		sprintf(szBuffer, "OBJ = %.3f mA\n", fI * 1000.0);
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
#define CMD_GETDI	"GETDI"
#define LEN_GETDI	(sizeof(CMD_GETDI)-1)
	if (strncmp(szCommand, CMD_GETDI, LEN_GETDI) == 0)
	{
		if (bDB3Busy == 1) {
			PushCommandToQueue(szCommand, nType);
			goto DecipherRet;
		}
		//ReplaceChar(szCommand, ':', ' ');
		//sscanf(&szCommand[LEN_GETDI], "%d", &nB);	//fV: current in unit mA
		wV = GetDigitalInput(0);
		WordToBinaryString(wV, szValue); //bbbb_bbbb_bbbb_bbbb_
		sprintf(szBuffer, "DI=%s,%04X\n", szValue, wDI);
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
#define CMD_GETDO	"GETDO"
#define LEN_GETDO	(sizeof(CMD_GETDO)-1)
	if (strncmp(szCommand, CMD_GETDO, LEN_GETDO) == 0)
	{
		if (bDB3Busy == 1) {
			PushCommandToQueue(szCommand, nType);
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		sscanf(&szCommand[LEN_GETDO + 1], "%d", &nB);
		wV = GetDOSetting(szBuffer, nB); //nB is not used
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
	//GET setting parameters (values that will not be changed during operation)
#define CMD_GETSET	"GETSET"
#define LEN_GETSET	(sizeof(CMD_GETSET)-1)
	if (strncmp(szCommand, CMD_GETSET, LEN_GETSET) == 0)
	{
		if (IsCommandEnabled(nType) != SUCCESS)
			goto DecipherRet;
		//wait until power on process is complete
		if (bInitOK == 0) goto DecipherRet;
		//
		if (bDB3Busy == 1) {
			PushCommandToQueue(szCommand, nType);
			goto DecipherRet;
		}
		szBuffer[0] = 0;
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_GETSET + 1], "%d", &nV);	//fV: current in unit mA
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GETSET_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV < 0)||(nV > 8)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GETSET_INV_PARAM_VAL));
			goto DecipherFail;
		}
		if (szCommand[LEN_GETSET] == 0) //all
			nV = 0x07;
		else if (nPN >= 2)
			goto DecipherFail;
		//
		if (nV == 8) { //GETSET:8
			TxSetParam(nType);
			goto DecipherRet;
		}
		if (nV & 0x1) {
			sprintf(szValue,"AFTHR=%.3f\n", NV_Settings.fAutoFocusThreshold);
			strcat(szBuffer, szValue);
			//
			sprintf(szValue,"APERMM[0]=%.3f,%.3f,\n", NV_Settings.fAPerMM[0][0], NV_Settings.fAPerMM[0][1]);
			strcat(szBuffer, szValue);
			//
			for (i = 0; i < 4; i++) { //only one coil pair is used
				sprintf(szValue,"BAUD[%d]=%d,\n", i, NV_Settings.nBaudrate[i]);
				strcat(szBuffer, szValue);
			}
			for (i = 0; i < 2; i++) { //only one coil pair is used
				sprintf(szValue,"BAUD[%d]=%d,\n", i + 4, NV_Settings.nPIDBaudrate[i]);
				strcat(szBuffer, szValue);
			}
			sprintf(szValue,"BINPKT=%d\n", (int)NV_Settings.bBinPkt);
			strcat(szBuffer, szValue);
			for (i = 0; i < BR_NUM; i++) {
				sprintf(szValue,"BR=%d,%.1f%c\n", i, fBrightness[i], '%');
				strcat(szBuffer, szValue);
			}
			for (i = 0; i < 4; i++) {
				sprintf(szValue,"CO=%.1f%c\n", fContrast[i], '%');
				strcat(szBuffer, szValue);
			}
			sprintf(szValue,"BSEMODE=%d (1:4,2:2)\n", (int)NV_Settings.bBSEMode);
			strcat(szBuffer, szValue);
			//sprintf(szValue,"DWTIME=%d,", nPixelDwellingTime); //pixel dwelling time (not used)
			//strcat(szBuffer, szValue);
			sprintf(szValue,"CHKTP=%d\n", bChkTP); //check turbo pump status for vacuum state machine change
			strcat(szBuffer, szValue);
			sprintf(szValue,"CSHUNTRATIO=%.3f,%.3f,\n", NV_Settings.fCoilShuntRatio[0], NV_Settings.fCoilShuntRatio[1]);
			strcat(szBuffer, szValue);
			sprintf(szValue,"CSHUNTR=%.3f,%.3f,\n", NV_Settings.fShuntR[0], NV_Settings.fShuntR[1]);
			strcat(szBuffer, szValue);
			sprintf(szValue,"CDEFSENR=%.3f,%.3f,\n", NV_Settings.fSenRCalc[0], NV_Settings.fSenRCalc[1]);
			strcat(szBuffer, szValue);
			sprintf(szValue,"CSENR=%.3f,%.3f\n", NV_Settings.fSenR[0], NV_Settings.fSenR[1]);
			strcat(szBuffer, szValue);
			//
			sprintf(szValue,"DEBUG=%d\n", bDebug); //pixel dwelling time (not used)
			strcat(szBuffer, szValue);
			sprintf(szValue,"DELAYAUTO=%d\n", (int)NV_Settings.bAutoXLineDelay); //FASTSCAN=1
			strcat(szBuffer, szValue);
			sprintf(szValue,"DELAYDESCENT=%d\n", NV_Settings.nDelayDescent); //FASTSCAN=1
			strcat(szBuffer, szValue);
			sprintf(szValue,"DELAYPIXEL=%d\n", 	NV_Settings.nPixelDwellTime); //FASTSCAN=1
			strcat(szBuffer, szValue);
			//sprintf(szValue,"EN_GRADIENT=%d\n", bEnableGradientByFW); //always 0
			//strcat(szBuffer, szValue);
			sprintf(szValue,"EDSONHW=%d\n", (int)NV_Settings.bEDSOnByHW); //EDS switch on control
			strcat(szBuffer, szValue);
			sprintf(szValue,"EDSDWT=%d ms\n", nEDSDWT); //EDS dwelling time
			strcat(szBuffer, szValue);
			sprintf(szValue,"EDSHS=%d\n", nEDSHandshake); //EDS handshake mode
			strcat(szBuffer, szValue);
			sprintf(szValue,"ENCOILSW=%d\n", (int)NV_Settings.bEnCoilSW);
			strcat(szBuffer, szValue);
			sprintf(szValue,"ENEDS=%d\n", (int)bEnableEDS);
			strcat(szBuffer, szValue);
			sprintf(szValue,"ENVENT=%d\n", (int)NV_Settings.bEnableVent);
			strcat(szBuffer, szValue);
			sprintf(szValue,"ENWDT=%d\n", (int)NV_Settings.bEnableWDT);
			strcat(szBuffer, szValue);
			sprintf(szValue,"FOCUS=%.5f%c\n", fFocus, '%');
			strcat(szBuffer, szValue);
			//sprintf(szValue,"HWUSDLY=%d\n", bHWUSDelay); //use hardware to generate usec delay
			//strcat(szBuffer, szValue);
			TxString(nType, szBuffer);
			szBuffer[0] = 0;
			sprintf(szValue,"HVMAX=0,%.1f,%.1f,%.1f,(A,F,B)\n", NV_Settings.fAccMaxV,
				NV_Settings.fFilaMaxV, NV_Settings.fBiasMaxV);
			strcat(szBuffer, szValue);
			sprintf(szValue,"HVPARAM=%.2f,%.2f,%.3f,%.3f,(A,B,F,R)\n", NV_Settings.fVAccRatio,
				NV_Settings.fVBiasRatio, NV_Settings.fFilaRatio, NV_Settings.fFilaResistance); //use hardware to generate usec delay
			strcat(szBuffer, szValue);
			//sprintf(szValue,"LEDON=%d\n", bEnableLED); //use hardware to generate usec delay
			//strcat(szBuffer, szValue);
			sprintf(szValue,"HISTO=%d\n", (int)bEnableHisto);
			strcat(szBuffer, szValue);
			sprintf(szValue,"HVTYPE=%d (0:S,1:M,2:C)\n", (int)NV_Settings.bHVType);
			strcat(szBuffer, szValue);
			sprintf(szValue,"IOINV=%d\n", (int)NV_Settings.bIOINV); //always 1
			strcat(szBuffer, szValue);
			sprintf(szValue,"LEDAUTO=%d\n", (int)NV_Settings.bLEDAuto);
			strcat(szBuffer, szValue);
			sprintf(szValue,"MAGMIN=%.1fX\n", fMagMinimum);
			strcat(szBuffer, szValue);
			strcat(szBuffer, "OBJIMAX=");
			for (i = 0; i < OBJ_ENERGY_NUM; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fObjIMax[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			strcat(szBuffer, "OBJIMIN=");
			for (i = 0; i < OBJ_ENERGY_NUM; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fObjIMin[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			sprintf(szValue,"OBJMODE=%d\n", (int)NV_Settings.bObjMode);
			strcat(szBuffer, szValue);
			sprintf(szValue,"OBJROTADJ=%d\n", (int)NV_Settings.bObjRotateAdj);
			strcat(szBuffer, szValue);
			TxString(nType, szBuffer); //string too long, separate the packet
			szBuffer[0] = 0;
			//
			//sprintf(szValue,"PKTNUM=%d\n", bPktNum);
			//strcat(szBuffer, szValue);
			sprintf(szValue,"ONEPKT=%d\n", (int)NV_Settings.bOnePkt);
			strcat(szBuffer, szValue);
			sprintf(szValue,"PIXELX=%d\n", nScanPixelNumX);
			strcat(szBuffer, szValue);
			sprintf(szValue,"PIXELY=%d\n", nScanPixelNumY);
			strcat(szBuffer, szValue);
			//
			sprintf(szValue, "PLAUTO=%d\n", (int)NV_Settings.bPLSyncAuto);
			strcat(szBuffer, szValue);
			sprintf(szValue, "PLFREQ=%d\n", (int)NV_Settings.bPLFreq);
			strcat(szBuffer, szValue);
			for (i = 0; i < BR_NUM; i++) {
				sprintf(szValue,"BASE_BR=%d,%.1f\n", i, NV_Settings.fBaseBR[i]);
				strcat(szBuffer, szValue);
				sprintf(szValue,"RANGE_BR=%d,%.1f\n", i, NV_Settings.fRangeBR[i]);
				strcat(szBuffer, szValue);
			}
			for (i = 0; i < CO_NUM; i++) {
				sprintf(szValue,"BASE_CO=%d,%.1f\n", i, NV_Settings.fBaseCO[i]);
				strcat(szBuffer, szValue);
				sprintf(szValue,"RANGE_CO=%d,%.1f\n", i, NV_Settings.fRangeCO[i]);
				strcat(szBuffer, szValue);
			}
			sprintf(szValue,"SCALE=%d(1:W_ROT,2:DACVID,3:WO_ROT,4:DAC16b)\n", NV_Settings.bScaleMode);
			strcat(szBuffer, szValue);
			sprintf(szValue,"SHADOW=%d(0~3:1CH,4:2CH,5:4CH,6:HD,7:VD,8:M2,9:M3)\n", nShadowMode); //Video average mode
			strcat(szBuffer, szValue);
			sprintf(szValue, "SYSCLK=%d MHz\n", NV_Settings.nSysClk);
			strcat(szBuffer, szValue);
			for (i = 0; i < TEMP_SENSOR_NUM_MAX; i++) {
				sprintf(szValue,"TEMPHI%d=%.3f,TEMPLO%d=%.3f\n", i, NV_Settings.fTempHiLimit[i], i, NV_Settings.fTempLoLimit[i]);
				strcat(szBuffer, szValue);
			}
			sprintf(szValue,"TURBOT=%d (0:Cu,1:Pf,2:Ag)\n", NV_Settings.nTurboType);
			strcat(szBuffer, szValue);
			for (i = 0; i <= TURBO_TYPE_MAX; i++) {
				sprintf(szValue,"TURBOSP=%d,%d,%d\n", i, NV_Settings.nTurboReadySpeed[i], NV_Settings.nTurboLowSpeed[i]);
				strcat(szBuffer, szValue);
			}
			sprintf(szValue,"UP2DN=%d\n", (int)NV_Settings.bUP2DN);
			strcat(szBuffer, szValue);
			//
			//sprintf(szValue,"VADCCLK=%d\n", bVADCClock); //vadc_conv clock number
			//strcat(szBuffer, szValue);
			sprintf(szValue, "VACERR0=%d,VACERR1=%d,\n", nVacErrToleranceNum[0], nVacErrToleranceNum[1]);
			strcat(szBuffer, szValue);
			for (i = 0; i < VAC_GAUGE_NUM_MAX; i++) {
				sprintf(szValue, "VACBYP=%d,%d,", i, bVACBypassOK[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			//sprintf(szValue, "VAC_RATE_MAX=%.4f,VAC_RATE_MIN=%.4f\n",
			//	NV_Settings.fVACChangeMax, NV_Settings.fVACChangeMin);
			//strcat(szBuffer, szValue);
			//
			sprintf(szValue, "VIDAVG=%d\n", nVADCAvgNum); //Video signal average times
			strcat(szBuffer, szValue);
			sprintf(szValue, "VIEWW=%.0f mm\n", NV_Settings.fImageWidthOnScreen);
			strcat(szBuffer, szValue);
			strcat(szBuffer, "V2IR=");
			for (i = 0; i < ISEN_CH_NUM; i++) {
				sprintf(szValue, "%.2f,", NV_Settings.fV2IR[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			nV1 = (NV_Settings.wWaitEOC & 0x8000) ? 1 : 0;	//wait_2nd_eoc
			nV2 = (NV_Settings.wWaitEOC & 0x00FF);			//wait eoc clock
			sprintf(szValue, "WAITEOC=%d,%d\n", nV1, nV2); //Video signal average times
			strcat(szBuffer, szValue);
			TxString(nType, szBuffer); //string too long, separate the packet
		}
		if (nV & 0x2) {
			szBuffer[0] = 0;
			strcat(szBuffer, "VDAC_MAX=");
			for (i = 0; i < VDAC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", fVDACVMax[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			strcat(szBuffer, "VDAC_MIN=");
			for (i = 0; i < VDAC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", fVDACVMin[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			for (j = 0; j < BOARD_NUM; j++) {
				sprintf(szValue, "DAC_MAX=%d,", j);
				strcat(szBuffer, szValue);
				for (i = 0; i < DAC_CH_NUM_MAX; i++) {
					sprintf(szValue, "%.3f,", fDACVMax[j][i]);
					strcat(szBuffer, szValue);
				}
				strcat(szBuffer, "\n");
			}
			for (j = 0; j < BOARD_NUM; j++) {
				sprintf(szValue, "DAC_MIN=%d,", j);
				strcat(szBuffer, szValue);
				for (i = 0; i < DAC_CH_NUM_MAX; i++) {
					sprintf(szValue, "%.3f,", fDACVMin[j][i]);
					strcat(szBuffer, szValue);
				}
				strcat(szBuffer, "\n");
			}
			/*strcat(szBuffer, "LOADR=");
			for (i = 0; i < ISEN_CH_NUM; i++) {
				sprintf(szValue, "%.2f,", NV_Settings.fLoadR[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n"); */
			strcat(szBuffer, "OFS_ADC=");
			for (i = 0; i < ADC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fADCOffset[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			strcat(szBuffer, "OFS_DAC=0,");
			for (i = 0; i < DAC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fDACOffset[0][i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			strcat(szBuffer, "OFS_DAC=1,");
			for (i = 0; i < DAC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fDACOffset[1][i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			strcat(szBuffer, "OFS_VADC="); //video ADC offset
			for (i = 0; i < VADC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fScanADCOffset[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			strcat(szBuffer, "OFS_VDAC="); //video DAC offset
			for (i = 0; i < VDAC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fScanDACOffset[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			strcat(szBuffer, "SLP_DAC=0,");
			for (i = 0; i < DAC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fDACSlope[0][i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			strcat(szBuffer, "SLP_DAC=1,");
			for (i = 0; i < DAC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fDACSlope[1][i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			/*strcat(szBuffer, "\n");
			strcat(szBuffer, "SLP_VADC="); //video ADC slope
			for (i = 0; i < VADC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fScanADCSlope[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n"); */
			strcat(szBuffer, "SLP_VDAC="); //video DAC slope
			for (i = 0; i < VDAC_CH_NUM_MAX; i++) {
				sprintf(szValue, "%.3f,", NV_Settings.fScanDACSlope[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			TxString(nType, szBuffer);
		}
		if (nV & 0x4) { //GETSET:4
			szBuffer[0] = 0;
			sprintf(szValue, "ACCKV=0,%d\n", (int)NV_Settings.bAcckVAdjust);
			strcat(szBuffer, szValue);
			sprintf(szValue, "ACCKV=1,%.1f kV\n", fAcckV);
			strcat(szBuffer, szValue);
			sprintf(szValue, "ACCKV=2,%d (0:15,12,10,8,5)\n", nObjTableNdx);
			strcat(szBuffer, szValue);
			sprintf(szValue, "AUTOVAC=%d\n", (int)NV_Settings.bAutoVAC);
			strcat(szBuffer, szValue);
			sprintf(szValue, "DACT=0,%d (0:5328,1:5648,2:5668)\n", (int)NV_Settings.bDACType[0]);
			strcat(szBuffer, szValue);
			sprintf(szValue, "DACT=1,%d (0:5328,1:5648,2:5668)\n", (int)NV_Settings.bDACType[1]);
			strcat(szBuffer, szValue);
			sprintf(szValue, "ENABLE_IP=%d\n", ENABLE_IP);
			strcat(szBuffer, szValue);
			sprintf(szValue, "ENFIFO=%d\n", (int)NV_Settings.bEnableFIFO);
			strcat(szBuffer, szValue);
			sprintf(szValue, "IDLET=%d\n", NV_Settings.nIdleTimeout);
			strcat(szBuffer, szValue);
			strcat(szBuffer, "LOADR=");
			for (i = 0; i < ISEN_CH_NUM; i++) {
				sprintf(szValue, "%.2f,", NV_Settings.fLoadR[i]);
				strcat(szBuffer, szValue);
			}
			strcat(szBuffer, "\n");
			sprintf(szValue, "NOKTH=0,%d (VAC state change threshold)\n", NV_Settings.sVACNOKTh[0]);
			strcat(szBuffer, szValue);
			sprintf(szValue, "NOKTH=1,%d (PG broken threshold)\n", NV_Settings.sVACNOKTh[1]);
			strcat(szBuffer, szValue);
			sprintf(szValue, "IGHI=%.3f\n", NV_Settings.fIGHITh);
			strcat(szBuffer, szValue);
			sprintf(szValue, "IGUH=%.3f\n", NV_Settings.fIGUHTh);
			strcat(szBuffer, szValue);
			sprintf(szValue, "IPTYPE=%d\n", (int)NV_Settings.bIPType);
			strcat(szBuffer, szValue);
			sprintf(szValue, "OBJR=%d (1:17,2:14,3:12)\n", (int)NV_Settings.bObjR);
			strcat(szBuffer, szValue);
			sprintf(szValue, "OBJSTBYR=%.3f\n", NV_Settings.fObjStandbyRatio);
			strcat(szBuffer, szValue);
			sprintf(szValue, "OVERSCAN=%.3f,%.3f\n", NV_Settings.fOverscan, fOverscan);
			strcat(szBuffer, szValue);
			sprintf(szValue, "NV_SETTINGS=%d bytes\n", (int)sizeof(NV_SETTINGS));
			strcat(szBuffer, szValue);
			sprintf(szValue, "SYS_PARAM_2=%d bytes\n", (int)sizeof(SYS_PARAM_2));
			strcat(szBuffer, szValue);
			sprintf(szValue, "SYS_PARAM_3=%d bytes\n", (int)sizeof(SYS_PARAM_3));
			strcat(szBuffer, szValue);
			sprintf(szValue, "UDPPORT=%d\n", (int)NV_Settings.bUDPPort); //for CL control
			strcat(szBuffer, szValue);
			TxString(nType, szBuffer);
		}
		goto DecipherRet; //goto DecipherOK;
	}
	if (strncmp(szCommand, "GETTIME", 7) == 0)
	{
		GetSystemTime();
		goto DecipherOK;
	}
	if (strncmp(szCommand, "GETVAC", 6) == 0)
	{
		szBuffer[0] = 0;
		//vacuum status VAC0, VAC1
		for (i = 0; i < 2; i++) {
			nV = GetGaugeStatus(i, szValue); //return VAC_OK or VAC_NOT_OK
			strcat(szBuffer, szValue);
		}
		strcat(szBuffer, "\n");
		IsChamberClose(szValue);
		strcat(szBuffer, szValue);
		IsCaseClose(szValue);
		strcat(szBuffer, szValue);
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
#define CMD_GETVER	"GETVER"
#define LEN_GETVER	(sizeof(CMD_GETVER)-1)
	if (strncmp(szCommand, CMD_GETVER, LEN_GETVER) == 0)	//get version
	{
		if (IsCommandEnabled(nType) != SUCCESS) goto DecipherRet;
		ShowTitle(nType);
		if ((SysParam.bErrorHWVersion & ERR_HW_PCBID) != 0) {
			nErrCode = ERR_INV_PID;
			goto DecipherFail;
		}
		else if (SysParam.bErrorHWVersion != 0) {
			nErrCode = ERR_INV_VERSION;
			goto DecipherFail;
		}
		else {
			goto DecipherRet;
		}
	}
/*#define CMD_GRADIENT	"GRADIENT:"
#define LEN_GRADIENT	(sizeof(CMD_GRADIENT)-1)
	if (strncmp(szCommand, CMD_GRADIENT, LEN_GRADIENT) == 0)
	{	// obsolete, always by hardware
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_GRADIENT], "%d", &nV1);
		if (nPN != 1) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}
		bEnableGradientByFW = (BYTE)nV1; //GRADIENT=1, used for auto focus procedure; GRADIENT=0
		goto DecipherOK;
	}*/
#define CMD_GV	"GV:"
#define LEN_GV	(sizeof(CMD_GV)-1)
	if (strncmp(szCommand, CMD_GV, LEN_GV) == 0)
	{	//<NV>:0-based index, <NV2>: open(1)/close(0)
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_GV], "%d %d", &nV1, &nV2);	//fV: current in unit mA
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GV_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 4)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GV_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nV2 < 0)||(nV2> 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_GV_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		SetGateValve(nV1, nV2); //nIndex, open/close
		goto DecipherOK;
	}
#define CMD_HELP	"HELP"
#define LEN_HELP	(sizeof(CMD_HELP)-1)
	if (strncmp(szCommand, CMD_HELP, LEN_HELP) == 0)
	{
		if (IsCommandEnabled(nType) != SUCCESS) goto DecipherRet;
		szBuffer[0] = 0;
		strcat(szBuffer, "--------------------------------------------------\n");
		//strcat(szBuffer, "ABORT                  stop scan process\n");
		strcat(szBuffer, "CLEARSET               clear NV settings\n");
		strcat(szBuffer, "CSHUNT:<NV>:<FV>       see CSHUNT:?\n");
		strcat(szBuffer, "DELAY:<B>:<NV>:<NV2>   see DELAY:?\n");
		strcat(szBuffer, "ENVENT:<NV>            enable(1)/disable(0) venting\n");
#if SCAN_PCB_VER >= 7
		strcat(szBuffer, "FASTSCAN:<NV>          enable(1)/disable(0) fast scan\n");
#endif
		strcat(szBuffer, "FOCUS:<NV>:<FV>        focus tuning, fV = 0 ~ 100\n");
		strcat(szBuffer, "GETAI:<B>:<C>          get input voltage\n");
		strcat(szBuffer, "GETAIALL               get all ADC\n");
		strcat(szBuffer, "GETAII                 get video ADC\n");
		strcat(szBuffer, "GETCI:<B>:<C>          get coil current\n");
		strcat(szBuffer, "GETDI:<B>              get digital input\n");
		strcat(szBuffer, "GETSET                 display settings\n");
		strcat(szBuffer, "GETSYS                 get system status\n");
		//strcat(szBuffer, "GETTIME                get system time\n");
		//strcat(szBuffer, "GRADIENT:<NV>          enable gradient calc. by fw\n");
		strcat(szBuffer, "GRADN2M:<NV>           set grad_n2m\n");
		strcat(szBuffer, "GV:<NV>:<NV2>          gate valve on/off(NV2), NV=0:TP_AIR,1:SCROLL\n");
		//strcat(szBuffer, "HVFREQ:<NV>:<FV>       set HV freq\n");
		strcat(szBuffer, "HVON:<NV>              high voltage on/off\n");
		strcat(szBuffer, "LED:<NV>:<NV2>         LED G(0),R(1),B(2),Y(3) dark(0)/solid(1)/blink(2)\n");
		strcat(szBuffer, "LEDON:<NV>             LED on/off\n");
		strcat(szBuffer, "LEDTEST:<NV>           LED test\n");
		strcat(szBuffer, "MAG:<NV>               set magnification\n");
		strcat(szBuffer, "OBJON:<NV>             turn on/off obj current source\n");
		strcat(szBuffer, "OBJIMAX:<FV>           set obj current maximum\n");
		strcat(szBuffer, "OBJIMIN:<FV>           set obj current minimum\n");
		strcat(szBuffer, "POINTMODE:<NV>         set point for EDS operation\n");
		strcat(szBuffer, "POINTSET:<FV1>:<FV2>   set point position (mm)\n");
		strcat(szBuffer, "ROTATE:<FV>            set rotate mode and angle (degree)\n");
#if IO_PCB_VER >= 2
		strcat(szBuffer, "PUMPON:<NV>:<NV2>      TP(0),SP(1),IP(2) on/off\n");
#else
		strcat(szBuffer, "PUMPON:<NV>:<NV2>      TP(0),SP(1) on/off\n");
#endif
		strcat(szBuffer, "PWRON:<NV>:<NV2>       HV(0),GV(1),VOPAB(2),VR(3),SENR(4),SHUNT(5) on/off\n");
#if SCAN_PCB_VER >= 7
		strcat(szBuffer, "RAMDUMP:<A>:<NV>       ram dump\n");
		strcat(szBuffer, "RAMSET:<A>:<NV>:<V>    ram set\n");
		strcat(szBuffer, "RAMTEST:<RW>:<CHIP>    ram test, RAMTEST:3:5\n");
#endif
		strcat(szBuffer, "SO:<B>:<NV>:XXXXXX     RS232(B=0),RS485(B=1),port=NV=0~3\n");
		strcat(szBuffer, "SAVESET                save NV settings\n");
		//strcat(szBuffer, "SCANPIX:<FVX>:<FVY>    fix pixel focus\n");
		strcat(szBuffer, "SCANSTART:<NR>:<T>     start scan\n");
		strcat(szBuffer, "SCANSTOP:<NV>          stop scan\n");
#if SCAN_PCB_VER >= 11
		strcat(szBuffer, "SENRSEL:<NV>           sense resistor select\n");
#endif
		strcat(szBuffer, "SETAO:<B>:<C>:<FV>     set output voltage\n");
		strcat(szBuffer, "SETAVGNUM:<NV>         set video avg time\n");
		strcat(szBuffer, "SETCI:<B>:<C>:<I>      set coil current\n");
#if SCAN_PCB_VER < 11
		strcat(szBuffer, "SENRSEL:<NV>         	  set deflector X10\n");
#endif
		strcat(szBuffer, "SETDO:<B>:<NV>         set digital output\n");
		strcat(szBuffer, "SETDOB:<B>:<C>:<NV>    set digital output bit\n");
		//TxString(nType, "SETDWTIME:<V>          set pixel dwelling time\n");
		sprintf(szValue, "SETLDR:<V1>...:<V%d>   set coil resistance\n", ISEN_CH_NUM);
		strcat(szBuffer, szValue);
		strcat(szBuffer, "SETOFSAD:<B>:<C>:<FV>  set ADC offset\n");
		strcat(szBuffer, "SETOFSDA:<B>:<C>:<FV>  set DAC offset\n");
		strcat(szBuffer, "SETSLPDA:<B>:<C>:<FV>  set DAC slope\n");
		sprintf(szValue, "SETV2IR:<V1>...:<V%d>  set I-to-V ratio\n", ISEN_CH_NUM);
		strcat(szBuffer, szValue);
		//TxString(nType, "SETTIME: YYYY-MM-DD hh:mm:ss   set system date and time\n");
		strcat(szBuffer, "SHADOW:<NV>            set shadow mode\n");
		strcat(szBuffer, "SHUTDOWN               shutdown the system\n");
		strcat(szBuffer, "TURBO:<NV>:<NV2>       turbo pump running,1/0=ON/OFF,1/0:Vent/NoVent\n");
		strcat(szBuffer, "TURBOSP:<NV>:<NV2>     show/set turbo speed\n");
		strcat(szBuffer, "VACON:<NV>             set vacuum to AIR(0, vent) or READY(1, pump)\n");
		//strcat(szBuffer, "VACUUM:<NV>            set vacuum state machine\n");
		//strcat(szBuffer, "--------------------------------------------------\n");
		TxString(nType, szBuffer);
		ShowTitle(nType);
		goto DecipherRet;
	}
#define CMD_HISTO		"HISTO:"
#define LEN_HISTO		(sizeof(CMD_HISTO)-1)
	if (strncmp(szCommand, CMD_HISTO, LEN_HISTO) == 0)
	{	//enable/disable hitogram transmission
		nPN = sscanf(&szCommand[LEN_HISTO], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HISTO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HISTO_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bEnableHisto = (BYTE)nV1; //enable histogram calculation
		goto DecipherOK;
	}
#define CMD_HISTOCH		"HISTOCH:"
#define LEN_HISTOCH		(sizeof(CMD_HISTOCH)-1)
	if (strncmp(szCommand, CMD_HISTOCH, LEN_HISTOCH) == 0) {
		nPN = sscanf(&szCommand[LEN_HISTOCH], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HISTOCH_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HISTOCH_INV_PARAM_VAL));
			goto DecipherFail;
		}
		SetHistoChannel(nV1);
		goto DecipherOK;
	}
/*#if SCAN_PCB_VER >= 11
#define CMD_HVFREQ		"HVFREQ:"
#define LEN_HVFREQ		(sizeof(CMD_HVFREQ)-1)
	if (strncmp(szCommand, CMD_HVFREQ, LEN_HVFREQ) == 0)
	{	//0:ACC, 1:FILA, 2:BIAS
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_HVFREQ], "%d %f", &nV1, &fV);	//fV: voltage
		if (nPN != 2) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}
		//nV1=0,ACC; 1,FILA; 2,BIAS
		if (SetIO_HV_Freq(nV1, fV) < 0)
			goto DecipherFail;
		else
			goto DecipherOK;
	}
#endif */
//
#define CMD_HVACC	"HVACC:" //unit volt
#define LEN_HVACC	(sizeof(CMD_HVACC)-1)
	//iprintf("%s",szCommand); //Omar
	if (strncmp(szCommand, CMD_HVACC, LEN_HVACC) == 0)
	{	// HV button control
		//ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_HVACC], "%f", &fV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVACC_INV_PARAM_NUM));
			goto DecipherFail;
		}
		/*
		if( (fV1 < 0)||(fV1 > 15000)){  //need to be verified
			sprintf(szValue, "HVACC_INV_PARAM_VAL");
			goto DecipherFail;
		}
		*/
		//fV1 unit volt
		SetHVAccelerator(fV1/NV_Settings.fVAccRatio);
	}
#define CMD_HVBIAS		"HVBIAS:" //unit volt
#define LEN_HVBIAS	(sizeof(CMD_HVBIAS)-1)
	if (strncmp(szCommand, CMD_HVBIAS, LEN_HVBIAS) == 0)
	{	// HV button control
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_HVBIAS], "%f", &fV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVBIAS_INV_PARAM_NUM));
			goto DecipherFail;
		}
		/*
		if( (fV1 < 0)||(fV1 > 3500)){  //need to be verified
			sprintf(szValue, "HVBIAS_INV_PARAM_VAL");
			goto DecipherFail;
		}
		*/
		//fV1 unit volt
		SetHVBias(fV1/NV_Settings.fVBiasRatio);
	}
/*
#define CMD_HVFILA	"HVFILA:" //unit watt
#define LEN_HVFILA	(sizeof(CMD_HVFILA)-1)
	if (strncmp(szCommand, CMD_HVFILA, LEN_HVFILA) == 0)
	{	// HV button control
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_HVFILA], "%f", &fV1);
		if (nPN != 1) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}
		//fV1 unit watt
		//DAC output = fW / fFilaRatio
		fV1 = fV1 / NV_Settings.fFilaRatio;
		//DAC output = fFilaRatio * sqrt(W*R)
		//fV2 = sqrt(fV1 * NV_Settings.fFilaResistance); //unit amperes
		//fV1 = fV2 * NV_Settings.fFilaRatio;
		SetHVFilament(fV1);
	}
*/
#define CMD_HVON	"HVON:"
#define LEN_HVON	(sizeof(CMD_HVON)-1)
	if (strncmp(szCommand, CMD_HVON, LEN_HVON) == 0)
	{	// HV button control
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_HVON], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVON_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVON_INV_PARAM_VAL));
			goto DecipherFail;
		}
		nV1 = (nV1 == 1) ? 1 : 0;
		if (nV1 == 0) {
			SetHVSwitch(nV1); //switch off
			goto DecipherOK;
		}
		else if (nV1 == 1) {
			if (IsInterlockSafe(ACT_HV_ON) == SUCCESS) { //nV1=1
				if (SetHVSwitch(nV1) == ERROR_FAIL)
					goto DecipherFail;
				else
					goto DecipherOK;
			}
			else {
				nErrCode = ERR_INTLK_FAIL;
				goto DecipherFail;
			}
		}
		else if (nV1 == 2) {
			HV_GetSN(); //get HV module serial number
		}
		else {
			nErrCode = ERR_INTLK_FAIL;
			goto DecipherFail;
		}
	}
#define CMD_HVMAX		"HVMAX:"
#define LEN_HVMAX		(sizeof(CMD_HVMAX)-1)
	if (strncmp(szCommand, CMD_HVMAX, LEN_HVMAX) == 0)
	{	// HV button control
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_HVMAX], "%d %f %f %f", &nV1, &fV1, &fV2, &fV3);
		if (nPN != 4) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVMAX_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 != 0)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVMAX_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if (nV1 == 0) {
			NV_Settings.fAccMaxV = fV1;
			NV_Settings.fFilaMaxV = fV2;
			NV_Settings.fBiasMaxV = fV3;
		}
		goto DecipherOK;
	}
#define CMD_HVPARAM		"HVPARAM:"
#define LEN_HVPARAM		(sizeof(CMD_HVPARAM)-1)
	if (strncmp(szCommand, CMD_HVPARAM, LEN_HVPARAM) == 0)	{	// HV button control
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_HVPARAM], "%f %f %f %f", &fV1, &fV2, &fV3, &fV4);
		if (nPN != 4) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVPARAM_INV_PARAM_NUM));
			goto DecipherFail;
		} //A:B:F:R
		NV_Settings.fVAccRatio = fV1; 	//real voltage = DAC voltage * fVAccratio
		NV_Settings.fVBiasRatio = fV2;	//
		NV_Settings.fFilaRatio = fV3;	//
		NV_Settings.fFilaResistance = fV4;
		goto DecipherOK;
	}
#define CMD_HVPARAM2		"HVPARAM2:"
#define LEN_HVPARAM2		(sizeof(CMD_HVPARAM2)-1)
	if (strncmp(szCommand, CMD_HVPARAM2, LEN_HVPARAM2) == 0) 	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_HVPARAM2], "%d %f", &nV1, &fV1);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVPARAM2_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 != 0)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVPARAM2_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if (nV1 == 0)
			NV_Settings.fAccMonI = fV;
	}
#define CMD_HVTYPE	"HVTYPE:"
#define LEN_HVTYPE	(sizeof(CMD_HVTYPE)-1)
	if (strncmp(szCommand, CMD_HVTYPE, LEN_HVTYPE) == 0) {
		nPN = sscanf(&szCommand[LEN_HVTYPE], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVTYPE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_HVTYPE_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bHVType = (BYTE)nV1; //0:Spellmen,1:Matsusada,2:Custom
		goto DecipherOK;
	}
#define CMD_IDLET	"IDLET:"
#define LEN_IDLET	(sizeof(CMD_IDLET)-1)
	if (strncmp(szCommand, CMD_IDLET, LEN_IDLET) == 0) {
		nPN = sscanf(&szCommand[LEN_IDLET], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IDLET_INV_PARAM_NUM));
			goto DecipherFail;
		}
		NV_Settings.nIdleTimeout = nV1;
		goto DecipherOK;
	}
#define CMD_IMGNUM	"IMGNUM:"
#define LEN_IMGNUM	(sizeof(CMD_IMGNUM)-1)
	if (strncmp(szCommand, CMD_IMGNUM, LEN_IMGNUM) == 0) {
		if (szCommand[LEN_IMGNUM] == '?') {
			printf("IMGNUM 1\n");
			printf("IMGNUM 3,2ND=10\n");
			printf("IMGNUM 4-6,2ND=14-16\n");
			printf("IMGNUM 7-8,2ND=s4-s5\n");
			//printf("IMGNUM 10,2ND=s4-s5,3RD=SEI\n");
			goto DecipherRet;
		}
		nPN = sscanf(&szCommand[LEN_IMGNUM], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IMGNUM_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 1)||(nV1 > 8)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IMGNUM_INV_PARAM_VAL));
			goto DecipherFail;
		}
		if (nV1 == 1) {
			bImageMask = 0x01;  //send 1st image only
			Set2ndImageType(0);	//two_image=0
			SetHistoChannel(0); //histo_ch=0
			//pwFpga[READ_ALL_VADC] = 0;
		}
		else if (nV1 == 2) { //send 2nd image only, for debug
			bImageMask = 0x02;
			Set2ndImageType(1);	//two_image=1
			SetHistoChannel(1); //histo_ch=1
			//pwFpga[READ_ALL_VADC] = 1;
		}
		else if (nV1 == 3) {
			bImageMask = 0x03;		//send both images
			bSecondImage = 10; 		//channel 3, SEI
			Set2ndImageType(1); 	//two_image=1
			//SetHistoChannel(1); 	//histo_ch=1
			//pwFpga[READ_ALL_VADC] = 1;
		}
		//second image
		//two_image = 2, s2+s3 (nV1=4)
		//two_image = 3, s2-s3 (nV1=5)
		//two_image = 4, s3-s2 (nV1=6)
		else if ((nV1 >= 4) && (nV1 <= 6)) {
			bImageMask = 0x03;		//send two images
			//SHADOW_2CH_P2(14), SHADOW_2CH_M3(15), SHADOW_2CH_M4(16)
			bSecondImage = (BYTE)(nV1 + 10);
			Set2ndImageType(nV1 - 2);	//two_image=2,3,4
			//pwFpga[READ_ALL_VADC] = 1;
		}
		//two_image = 5, s4 (nV1=7)
		//two_image = 6, s5 (nV1=8)
		else if ((nV1 >= 7) && (nV1 <= 8)) {
			bImageMask = 0x03;
			//SHADOW_1CH_4(17), SHADOW_1CH_5(18)
			bSecondImage = (BYTE)(nV1 + 10); //17,18
			Set2ndImageType(nV1 - 2);	//two_image=5,6
			//pwFpga[READ_ALL_VADC] = 1;
		}
		else {
			goto DecipherFail;
		}
		InitFastScan(); //change line delay
		goto DecipherOK;
	}
#define CMD_IO		"IO:"
#define LEN_IO		(sizeof(CMD_IO)-1)
	if (strncmp(szCommand, CMD_IO, LEN_IO) == 0)
	{	// HV button control
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_IO], "%d %d %d", &nB, &nV1, &nV2);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (nB == 3) //IO:3:1:0(enable), IO:3:0:0(disable)
			bEnableIO = (BYTE)nV1;
		else
			SetIOState(nB, nV1, nV2);	//board,channel, on/off
		goto DecipherOK;
	}
#define CMD_IOINV		"IOINV:"
#define LEN_IOINV		(sizeof(CMD_IOINV)-1)
	if (strncmp(szCommand, CMD_IOINV, LEN_IOINV) == 0)
	{
		nPN = sscanf(&szCommand[LEN_IOINV], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IOINV_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IOINV_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bIOINV = (BYTE)nV1;
		goto DecipherOK;
	}
#define CMD_IPLEAK		"IPLEAK:"
#define LEN_IPLEAK		(sizeof(CMD_IPLEAK)-1)
	if (strncmp(szCommand, CMD_IPLEAK, LEN_IPLEAK) == 0)
	{	//
		if (szCommand[LEN_IPLEAK] == '?') {
			printf("IPLEAK:0:NV:0      IP leak num,%d\r\n", NV_Settings.sIPLeakNum);
			printf("IPLEAK:1:NV1:NV2   NV1=0~3,NV2=on time (s)\r\n");
			printf("IPLEAK:2:NV1:NV2   NV1=0~3,NV2=off time (s)\r\n");
			for (i = 0; i < CHECK_VOLT_NUM; i++) {
				printf("IPLEAK ON TIME[%d] = %d\r\n", i, NV_Settings.sIPLeakOnTime[i]);
				printf("IPLEAK OFF TIME[%d] = %d\r\n", i, NV_Settings.sIPLeakOffTime[i]);
				printf("VACPARA:%d:<FV> IP_LEAK%d=%.3f\n", 16 + i, i, NV_Settings.fIPLeakCheckV[i]);
			}
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_IPLEAK], "%d %d %d", &nV1, &nV2, &nV3);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IPLEAK_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IPLEAK_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if (nV1 == 0) {
			NV_Settings.sIPLeakNum = nV2;
		}
		else if (nV1 == 1) {
			if( (nV2 < 0)||(nV2 > 3)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_IPLEAK_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			NV_Settings.sIPLeakOnTime[nV2] = nV3;
		}
		else if (nV1 == 2) {
			if( (nV2 < 0)||(nV2 > 3)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_IPLEAK_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			NV_Settings.sIPLeakOffTime[nV2] = nV3;
		}
		goto DecipherOK;
	}
	//--------------------------------------------------
	// IP_PWR_CTRL board test command begins
	//--------------------------------------------------
#define CMD_IPON		"IPON:"
#define LEN_IPON		(sizeof(CMD_IPON)-1)
	if (strncmp(szCommand, CMD_IPON, LEN_IPON) == 0) {	//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_IPON], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IPON_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IPON_INV_PARAM_VAL));
			goto DecipherFail;
		}
		SetIonPump(nV1);
		goto DecipherOK;
	}
/*#define CMD_IPINFO	"IPINFO"
#define LEN_IPINFO	(sizeof(CMD_IPINFO)-1)
	if (strncmp(szCommand, CMD_IPINFO, LEN_IPINFO) == 0)
	{
		sprintf(szBuffer, "$%d,IPINFO\n", IP_MOTOR_ADDR);
		RS485_WriteString(2, szBuffer);
		goto DecipherOK;
	}
*/
#define CMD_MOTOGV		"MOTOGV:"
#define LEN_MOTOGV		(sizeof(CMD_MOTOGV)-1)
	if (strncmp(szCommand, CMD_MOTOGV, LEN_MOTOGV) == 0) {	//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_MOTOGV], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_MOTOGV_INV_PARAM_NUM));
			goto DecipherFail;
		}
		SetMotorGateValve(nV1);
		goto DecipherOK;
	}
#define CMD_GETHVC		"GETHVC"
#define LEN_GETHVC		(sizeof(CMD_GETHVC)-1)
	if (strncmp(szCommand, CMD_GETHVC, LEN_GETHVC) == 0) {	//
		GetHVC();
		goto DecipherOK;
	}
	//--------------------------------------------------
	// IP_PWR_CTRL board test command ends
	//--------------------------------------------------
#define CMD_IPTYPE "IPTYPE:"
#define LEN_IPTYPE (sizeof(CMD_IPTYPE) - 1)
	if (strncmp(szCommand, CMD_IPTYPE, LEN_IPTYPE) == 0) {	//nV1 = 0,1,2
		nPN = sscanf(&szCommand[LEN_IPTYPE], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IPTYPE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IPTYPE_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bIPType = (BYTE)nV1;
		goto DecipherOK;
	}
#define CMD_IRQACK	"IRQACK:"
#define LEN_IRQACK	(sizeof(CMD_IRQACK)-1)
	if (strncmp(szCommand, CMD_IRQACK, LEN_IRQACK) == 0) {
		nPN = sscanf(&szCommand[LEN_IRQACK], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IRQACK_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_IRQACK_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bIRQACK = (nV1 == 1) ? 1 : 0;
		goto DecipherOK;
	}
#define CMD_LEDAUTO		"LEDAUTO:"
#define LEN_LEDAUTO		(sizeof(CMD_LEDAUTO)-1)
	if (strncmp(szCommand, CMD_LEDAUTO, LEN_LEDAUTO) == 0)	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_LEDAUTO], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_LEDAUTO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_LEDAUTO_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bLEDAuto = (nV1 == 1) ? 1 : 0;
		SetIO_LED_Auto(nV1);
		goto DecipherOK;
	}
#define CMD_LED		"LED:"
#define LEN_LED		(sizeof(CMD_LED)-1)
	if (strncmp(szCommand, CMD_LED, LEN_LED) == 0)
	{	// LED pattern control
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_LED], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_LED_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 >= LED_NUM)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_LED_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nV2 < 0)||(nV2 > ST_LED_MAX)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_LED_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		//nV1=RED(0),GREEN(1),BLUE(2),YELLOW(3)
		//nV2=ST_LED_DARK(0),ST_LED_SOLID(1),ST_LED_BLINK(2)
		//bLEDState[nV1] = nV2;
		SetLEDState(nV1, nV2);
		goto DecipherOK;
	}
#define CMD_LEDON	"LEDON:"
#define LEN_LEDON	(sizeof(CMD_LEDON)-1)
	if (strncmp(szCommand, CMD_LEDON, LEN_LEDON) == 0)
	{	// HV button control
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_LEDON], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_LEDON_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_LEDON_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bEnableLED = (BYTE)nV1;
		goto DecipherOK;
	}
#define CMD_LEDTEST		"LEDTEST:"
#define LEN_LEDTEST		(sizeof(CMD_LEDTEST)-1)
	if (strncmp(szCommand, CMD_LEDTEST, LEN_LEDTEST) == 0)
	{	//
		nPN = sscanf(&szCommand[LEN_LEDTEST], "%d", &nV1); //fV is percentage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_LEDTEST_INV_PARAM_NUM));
			goto DecipherFail;
		}
		/* Omar
		if( (nV1 < 3)||(nV1 > 20)){
			sprintf(szValue, "LEDTEST_INV_PARAM_VAL");
			goto DecipherFail;
		}
		*/
		if ((nV1 > 20) || (nV1 < 3)) nV1 = 3;
		bLEDTest = (BYTE)nV1;
		LEDTest(1);
		goto DecipherOK;
	}
#define CMD_MAG		"MAG:"
#define LEN_MAG		(sizeof(CMD_MAG)-1)
	if (strncmp(szCommand, CMD_MAG, LEN_MAG) == 0) //set contrast
	{	////magnification
		if (szCommand[LEN_MAG] == '?') {
			PrintZoomCtrl();
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_MAG], "%f", &fV);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_MAG_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fV < MAG_MIN)||(fV > MAG_MAX)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_MAG_INV_PARAM_VAL));
			goto DecipherFail;
		}
//		if (fV < MAG_MIN) {
//			nErrCode = ERR_MAG_TOO_SMALL;
//			goto DecipherFail;
//		}
//		if (fV > MAG_MAX) {
//			nErrCode = ERR_MAG_TOO_LARGE;
//			goto DecipherFail;
//		}
		nRet = SetMagnification(fV);
		if (nRet == ERROR_FAIL) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_MAG_SETMAGNIFICATION_FAIL));
			goto DecipherFail;
		}
		if (bDebug == 3) PrintZoomCtrl();
		printf("MAG:%.0f\r\n", fV);
		goto DecipherOK;
	}
	//-----------------------------------------------------
	//motor control command begin
	//-----------------------------------------------------
/*
#define CMD_MC		"MC:"		//motor command
#define LEN_MC		(sizeof(CMD_MC)-1)
	if (strncmp(szCommand, CMD_MC, LEN_MC) == 0)
	{
		Motor_Command(szCommand + LEN_MC);
		goto DecipherRet;
	}
#define CMD_MJOG		"MJOG:"
#define LEN_MJOG		(sizeof(CMD_MJOG)-1)
	if (strncmp(szCommand, CMD_MJOG, LEN_MJOG) == 0)
	{	//MJOG:1:1:100 //axis, direction, speed
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_MJOG], "%d %d %f", &nV1, &nV2, &fV);
		if ((nPN != 3) || (nV1 >= AXIS_NUM_MAX)) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}
		//--------------------------------------------------
		// nV1=0,1,2, axis index
		//--------------------------------------------------
		Motor_Jog(nV1, nV2, (int)fV);
		goto DecipherOK;
	}
#define CMD_MMOVE		"MMOVE:"
#define LEN_MMOVE		(sizeof(CMD_MMOVE)-1)
	if (strncmp(szCommand, CMD_MMOVE, LEN_MMOVE) == 0)
	{	//MMOVE:R:0:100 ,relative or absolute
		ReplaceChar(szCommand, ':', ' ');
		//fV in unit mm
		nPN = sscanf(&szCommand[LEN_MMOVE], "%c %d %f", &c, &nV1, &fV);
		if ((nPN != 3) || (nV1 >= AXIS_NUM_MAX)) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}
		//--------------------------------------------------
		// nV1=0,1,2, axis index
		//--------------------------------------------------
		if (c == 'R') //relative
			Motor_MoveRelative(nV1, fV);
		else if (c == 'A') //absolute
			Motor_MoveAbsolute(nV1, fV);
		else
			goto DecipherFail;
		goto DecipherOK;
	}
#define CMD_MSTOP		"MSTOP:"
#define LEN_MSTOP		(sizeof(CMD_MSTOP)-1)
	if (strncmp(szCommand, CMD_MSTOP, LEN_MSTOP) == 0)
	{	//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_MSTOP], "%d", &nV1);
		if (nPN != 1) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}
		Motor_Stop(nV1);
		goto DecipherOK;
	}
#define CMD_MGET		"MGET:"
#define LEN_MGET		(sizeof(CMD_MGET)-1)
	if (strncmp(szCommand, CMD_MGET, LEN_MGET) == 0)
	{	//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_MGET], "%d", &nV1);
		if (nPN != 1) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}
		//Motor_GetStatus(nV1);
		//Motor_GetPosition(nV1); //axis
		//wait for response from motor control board
		//OSTimeDly(1);
		Motor_GetAllInfo(nV1, szBuffer); //POS=,CW=,CCW=,MOVING=
		strcat(szBuffer, "\n");
		TxString(nType, szBuffer);
		goto DecipherRet; //goto DecipherOK;
	}
*/
/*
#define CMD_PULSEMM		"PULSEMM:"
#define LEN_PULSEMM		(sizeof(CMD_PULSEMM)-1)
	if (strncmp(szCommand, CMD_PULSEMM, LEN_PULSEMM) == 0) //set contrast
	{	//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_PULSEMM], "%f", &fV); //pulse per mm
		//400(2.5 um) or 1000 (1 um)
		if (fV < 10 || fV > 10000) {
			nErrCode = ERR_OVER_RANGE;
			goto DecipherFail;
		}
		NV_Settings.fPulsePerMM = fV;
		goto DecipherOK;
	}
*/
#define CMD_MRESET		"MRESET:"
#define LEN_MRESET		(sizeof(CMD_MRESET)-1)
	if (strncmp(szCommand, CMD_MRESET, LEN_MRESET) == 0)
	{	//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_MRESET], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_MRESET_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_MRESET_INV_PARAM_VAL));
			goto DecipherFail;
		}
		Motor_ResetPos(nV1);
		goto DecipherOK;
	}
#define CMD_NOKTH	"NOKTH:"
#define LEN_NOKTH	(sizeof(CMD_NOKTH)-1)
	if (strncmp(szCommand, CMD_NOKTH, LEN_NOKTH) == 0)
	{	//NOK threshold, 0: vacuum, 1: broken
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_NOKTH], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_NOKTH_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_NOKTH_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		NV_Settings.sVACNOKTh[nV1] = nV2;
		goto DecipherOK;
	}
	//-----------------------------------------------------
	//motor control command ends
	//-----------------------------------------------------
#define CMD_OBJMODE		"OBJMODE:"
#define LEN_OBJMODE		(sizeof(CMD_OBJMODE)-1)
	if (strncmp(szCommand, CMD_OBJMODE, LEN_OBJMODE) == 0)
	{	//
		nPN = sscanf(&szCommand[LEN_OBJMODE], "%d", &nN);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJMODE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nN< 0)||(nN > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJMODE_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bObjMode = (nN == 1) ? 1 : 0;
		goto DecipherOK;
	}
#define CMD_OBJON		"OBJON:"
#define LEN_OBJON		(sizeof(CMD_OBJON)-1)
	if (strncmp(szCommand, CMD_OBJON, LEN_OBJON) == 0)
	{	//
		nPN = sscanf(&szCommand[LEN_OBJON], "%d", &nN);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJON_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nN< 0)||(nN > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJON_INV_PARAM_VAL));
			goto DecipherFail;
		}
		SetObjOn(nN);
		goto DecipherOK;
	}
#define CMD_OBJIMAX		"OBJIMAX:"
#define LEN_OBJIMAX		(sizeof(CMD_OBJIMAX)-1)
	if (strncmp(szCommand, CMD_OBJIMAX, LEN_OBJIMAX) == 0) //set contrast
	{	//
		nPN = sscanf(&szCommand[LEN_OBJIMAX], "%d:%f", &nV1, &fI); //fV is percentage
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJIMAX_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if((nV1 < 0) || ( nV1 >= OBJ_ENERGY_NUM)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJIMAX_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if((fI < OBJIMIN_LO) || (fI > OBJIMAX_HI)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJIMAX_INV_PARAM2_VAL));
			goto DecipherFail;
		}

		NV_Settings.fObjIMax[nV1] = fI;
		fObjStandbyV = CalculateObjStandbyV();
		goto DecipherOK;
	}
#define CMD_OBJIMIN		"OBJIMIN:"
#define LEN_OBJIMIN		(sizeof(CMD_OBJIMIN)-1)
	if (strncmp(szCommand, CMD_OBJIMIN, LEN_OBJIMIN) == 0) //set contrast
	{	//
		nPN = sscanf(&szCommand[LEN_OBJIMIN], "%d:%f", &nV1, &fI); //fV is percentage
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJIMIN_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if((nV1 < 0) || ( nV1 >= OBJ_ENERGY_NUM)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJIMIN_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if((fI < OBJIMIN_LO) || (fI > OBJIMAX_HI)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJIMIN_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		NV_Settings.fObjIMin[nV1] = fI;
		fObjStandbyV = CalculateObjStandbyV();
		goto DecipherOK;
	}
#define CMD_OBJR		"OBJR:"
#define LEN_OBJR		(sizeof(CMD_OBJR)-1)
	if (strncmp(szCommand, CMD_OBJR, LEN_OBJR) == 0) {	//
		nPN = sscanf(&szCommand[LEN_OBJR], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJR_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1< 1)||(nV1 > 3)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJR_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bObjR = (BYTE)nV1; //1:17,2:14,3:12
		SetObjR(nV1);
		goto DecipherOK;
	}
#define CMD_OBJSTBYR		"OBJSTBYR:"
#define LEN_OBJSTBYR		(sizeof(CMD_OBJSTBYR)-1)
	if (strncmp(szCommand, CMD_OBJSTBYR, LEN_OBJSTBYR) == 0) {	//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_OBJSTBYR], "%f", &fV); //fV is percentage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJR_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (fV< 0.0)||(fV > 1.0)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OBJR_INV_PARAM_VAL));
			goto DecipherFail;
		}

		NV_Settings.fObjStandbyRatio = fV;
		fObjStandbyV = CalculateObjStandbyV();
		SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fObjStandbyV);
		goto DecipherOK;
	}
#define CMD_ONEPKT	"ONEPKT:"
#define LEN_ONEPKT	(sizeof(CMD_ONEPKT)-1)
	if (strncmp(szCommand, CMD_ONEPKT, LEN_ONEPKT) == 0) //ampere per mm
	{	//ONEPKT:0
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_ONEPKT], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ONEPKT_INV_PARAM_NUM));
			goto DecipherFail;
		}
		NV_Settings.bOnePkt = nV1;
		goto DecipherOK;
	}
#define CMD_OVERSCAN "OVERSCAN:"
#define LEN_OVERSCAN (sizeof(CMD_OVERSCAN) - 1)
	if (strncmp(szCommand, CMD_OVERSCAN, LEN_OVERSCAN) == 0)	{
		nPN = sscanf(&szCommand[LEN_OVERSCAN], "%d:%f", &nV1, &fV1);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OVERSCAN_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1< 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OVERSCAN_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (fV1< 0)||(fV1 > OVERSCAN_MAX)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_OVERSCAN_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if (fV1 > OVERSCAN_MAX)
			fV1 = OVERSCAN_MAX;
		else if (fV1 < 0)
			fV1 = 0.0;
		fV2 = 1.0 + fV1;
		if ((int)(fV2 * (float)nScanPixelNumX) > PIXEL_NUM_X_MAX)
			goto DecipherFail;
		if (nV1 == 0) {
			NV_Settings.fOverscan = fV1;
			fOverscan = fV1;
		}
		else if (nV1 == 1) {
			fOverscan = fV1;
		}
		sprintf(szBuffer, "SETPIXN:%d:%d", nScanPixelNumX, nScanPixelNumY);
		PushCommandToQueue(szBuffer, nType);
		goto DecipherOK;
	}
#define CMD_PAUSE	"PAUSE"
#define LEN_PAUSE	(sizeof(CMD_PAUSE)-1)
	if (strncmp(szCommand, CMD_PAUSE, LEN_PAUSE) == 0)	{
		ReplaceChar(szCommand, ':', ' ');
		SetHVSwitch(0);
		SetHVFilament(0);
		SetHVBias(0);
		SetHVAccelerator(0);
		SetObjOn(0);
		goto DecipherOK;
	}
//-----------------------------------------------------
// PID = PCB_ID(1), HV_ID(2)
//-----------------------------------------------------
#define CMD_PIDGET	"PIDGET:"
#define LEN_PIDGET	(sizeof(CMD_PIDGET)-1) //power line synchronization
	if (strncmp(szCommand, CMD_PIDGET, LEN_PIDGET) == 0) {	//
		p = &szCommand[LEN_PIDGET];
		nV = atoi(p);
		if( (nV< 0)||(nV > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PIDGET_INV_PARAM_VAL));
			goto DecipherFail;
		}
		if (nV == 0) { //PIDGET:0
			//remove error flag
			//bErrorHWVersion &= (~ERR_HW_PCBID);
		}
		else if (nV == 1) { //PIDGET:1
			printf("NV=");
			for (i = 0; i < UUID_NUM; i++) {
				printf("%02X-", NV_Settings.bUUID[i]);
			}
			printf("\r\n");
			nV2 = UUID_Init(szBuffer);
			printf("ID=");
			for (i = 0; i < UUID_NUM; i++) {
				printf("%02X-", bUUID_PID[i]);
			}
			printf("\r\n");
			if (nV2 > 0)
				printf("ID OK\r\n");
			else
				printf("ID FAIL\r\n");
		}
		/*else if (nV == 2) { //PIDGET:2
			printf("HVID=");
			for (i = 0; i < HV_ID_NUM; i++) { //HV_ID_NUM=PCB_ID_NUM=6
				printf("%02X", NV_Settings.bHVID[i]);
			}
			printf("\n");
		}*/
		goto DecipherOK;
	} //
#define CMD_PIDSET	"PIDSET:"
#define LEN_PIDSET	(sizeof(CMD_PIDSET)-1) //power line synchronization
	if (strncmp(szCommand, CMD_PIDSET, LEN_PIDSET) == 0)
	{	//PIDSET:0:0534A6B7C4E2
		p = &szCommand[LEN_PIDSET];
		nV = atoi(p);
		if( (nV< 1)||(nV > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PIDSET_INV_PARAM_VAL));
			goto DecipherFail;
		}
		p = p + 2;
		nV2 = atoi(p);
		strupr(p);
		nPN = sscanf(p, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
			&nID[0],&nID[1],&nID[2],&nID[3],&nID[4],&nID[5],&nID[6],&nID[7],
			&nID[8],&nID[9],&nID[10],&nID[11],&nID[12],&nID[13],&nID[14],&nID[15]);
		/*if (nPN != UUID_NUM) {
			nErrCode = ERR_INV_PARAM;
			goto DecipherFail;
		}*/
		if (nV == 1) { //PIDSET:1:XXXXXX
			for (i = 0; i < UUID_NUM; i++) {
				NV_Settings.bUUID[i] = (BYTE)nID[i];
				printf("%02X-", NV_Settings.bUUID[i]);
			}
			printf("\n");
		}
		else if ((nV == 2) && (nV2 == 18)) { //PIDSET:2:18
			for (i = 0; i < UUID_NUM; i++) { //HV_ID_NUM=PCB_ID_NUM=6
				NV_Settings.bUUID[i] = bUUID_PID[i];
			}
		}
		CheckPID();
		goto DecipherOK;
	} //
#define CMD_PIDURD	"PIDURD:"
#define LEN_PIDURD	(sizeof(CMD_PIDURD)-1) //power line synchronization
	if (strncmp(szCommand, CMD_PIDURD, LEN_PIDURD) == 0)
	{	//PIDURD:0
		p = &szCommandOrg[LEN_PIDURD];
		nV = atoi(p);
		nN = SPI_GetRxQueNum(nV); //nV=0,1
		if (nN == 0) { //no data
			printf("PID UART%d has no data\r\n", nV);
		}
		else if (SPI_ReadUART(nV, szBuffer) > 0) {
			printf(szBuffer);
		}
		goto DecipherOK;
	}
#define CMD_PIDUWR	"PIDUWR:"
#define LEN_PIDUWR	(sizeof(CMD_PIDUWR)-1) //power line synchronization
	if (strncmp(szCommand, CMD_PIDUWR, LEN_PIDUWR) == 0)
	{	//PIDUWR:0:0534A6B7C4E2
		p = &szCommandOrg[LEN_PIDUWR];
		nV = atoi(p);
		p = p + 2;
		SPI_WriteUART(nV, p);
		goto DecipherOK;
	}
#define CMD_PLSET	"PLSET:"
#define LEN_PLSET	(sizeof(CMD_PLSET)-1) //power line synchronization
	if (strncmp(szCommand, CMD_PLSET, LEN_PLSET) == 0)
	{	//
		if (szCommand[LEN_PLSET] == '?') { //PLSET:?
			printf("0:PLSYNC=%d\r\n", (int)bPLSync); //bPLSync = 0,1,2,3
			printf("3:PLFREQ=%d\r\n", (int)NV_Settings.bPLFreq);
			printf("4:X GET PL_INTERVAL,PL_ADDR\r\n");
			printf("6:PLAUTO=%d\r\n", (int)NV_Settings.bPLSyncAuto);
			printf("8:PLSAMPNO=%d\r\n", (int)NV_Settings.wPLAddrNum);
			//printf("9:PLAMP=%.3f\r\n", NV_Settings.fPLAmp);
			//printf("10:PLPHASE=%.3f\r\n", NV_Settings.fPLPhase);
			printf("PARAMETER=0X%04X\r\n", wParameter);
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_PLSET], "%d %f", &nV1, &fV1);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PLSET_INV_PARAM_NUM));
			goto DecipherFail;
		}
		nV2 = (int)fV1;
		if (nV1 == 0) { //PLSET:0:1, 16.67 ms/20.00 ms scan per line
			if( (nV2 < 0)||(nV2 > 1)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_PLSET_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			EnablePLSync(nV2);
			bPLSync = (nV2 == 1) ? 1 : 0; //0,1
		}
		else if (nV1 == 3) { //PLSET:3:60, set power line frequency
			if( (nV2 != 50)&&(nV2 != 60)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_PLSET_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			NV_Settings.bPLFreq = (nV2 == 50) ? 50 : 60;
			SetPLPeriod(); //unit us
		}
		else if (nV1 == 4) { //PLSET:4:1
			wV = pwFpga[GET_PL_NDX];
			sprintf(szBuffer, "PL_ADDR_NDX=0X%04X\r\n", wV); //sample number index
			printf(szBuffer);
		}
		else if (nV1 == 6) { //auto PL sync modulation
			if( (nV2 < 0)||(nV2 > 1)){
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_PLSET_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			NV_Settings.bPLSyncAuto = (BYTE)nV2;
		}
		else if (nV1 == 8) { //PL address number
			NV_Settings.wPLAddrNum = (WORD)nV2;
			SetPLParameter(1);
		}
		/*else if (nV1 == 9) { //set amplitude
			NV_Settings.fPLAmp = fV1; //volt
			SetPLParameter(1); //PL DAC modulation amplitude
		}
		else if (nV1 == 10) { //set phase
			NV_Settings.fPLPhase = fV1; //degree
			SetPLParameter(1); //PL DAC modulation phase
		}*/
		else{
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PLSET_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		goto DecipherOK;
	}
//POINTSET:<FV1>:<FV2><CR><LF>, set to specified point, unit mm
#define CMD_POINTSET		"POINTSET:"
#define LEN_POINTSET		(sizeof(CMD_POINTSET)-1)
	if (strncmp(szCommand, CMD_POINTSET, LEN_POINTSET) == 0)
	{	//<T>: interval in msec
		//e.g. POINTSET:0.011:0.034
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_POINTSET], "%f %f", &fV1, &fV2);	//fV1, fV2 unit mm
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_POINTSET_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (fV1 < -fImageWidthReal/2)||(fV1 > fImageWidthReal/2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_POINTSET_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (fV2 < -fImageWidthReal/2)||(fV2 > fImageWidthReal/2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_POINTSET_INV_PARAM2_VAL));
			goto DecipherFail;
		}

		fV1 = fV1 / (fImageWidthReal/2) * 5.0;
		fV2 = fV2 / (fImageWidthReal/2) * 5.0;
		SetDeflectorFineV(DAC_CH_DEFX_F, fV1);
		SetDeflectorFineV(DAC_CH_DEFY_F, fV2);
		goto DecipherOK;
	}
#define CMD_POS		"POS:"
#define LEN_POS		(sizeof(CMD_POS)-1)
	if (strncmp(szCommand, CMD_POS, LEN_POS) == 0)
	{	//e.g. POS:10:0
		nPN = sscanf(&szCommand[LEN_POS], "%d:%d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_POS_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 >= PIXEL_NUM_X_MAX)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_POS_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nV2 < 0)||(nV2 >= PIXEL_NUM_Y_MAX)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_POS_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		SetDACPos(nV1, nV2);
		goto DecipherOK;
	}
//POINTMODE:<NV><CR><LF>, set to specified point
#define CMD_POINTMODE		"POINTMODE:"
#define LEN_POINTMODE		(sizeof(CMD_POINTMODE)-1)
	if (strncmp(szCommand, CMD_POINTMODE, LEN_POINTMODE) == 0)	//scan start
	{	//<T>: interval in msec
		//e.g. POINTMODE:10:400
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_POINTMODE], "%d", &nV1);	//nN times, nB: interval in msec
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_POINTMODE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_POINTMODE_INV_PARAM_VAL));
			goto DecipherFail;
		}
		/*if (SysParam.bVacuumState != VAC_ST_READY) {
			nErrCode = ERR_INV_VAC_ST;
			goto DecipherFail;
		} */
		printf("POINTMODE %d\n", nV1);
		//
		bACK = 0;
		bScanAbort = 0;
		bScanPause = 0;
		bScanRestart = 0;
		if (nV1 == 0) { //POINTMODE:0
			//SetVOPPower(0); //VOPC, VOPB off
			SysParam.bScanning = SysParam.bPrevScanning;
			SetDeflectorFineV(DAC_CH_DEFX_F, 0);
			SetDeflectorFineV(DAC_CH_DEFY_F, 0);
		}
		else if (nV1 == 1) { //POINTMODE:1
			//SetVOPPower(1); //VOPC, VOPB on
			SysParam.bPrevScanning = SysParam.bScanning;
			SysParam.bScanning = OP_POINT;
			SetSenrSel(nSenRSet); //0:larger ohm, 1: smaller resistor
			SetCoilShunt(nShuntSet);
		}
		goto DecipherOK;
	}
#define CMD_PUMPON		"PUMPON:"
#define LEN_PUMPON		(sizeof(CMD_PUMPON)-1)
	if (strncmp(szCommand, CMD_PUMPON, LEN_PUMPON) == 0)
	{	//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_PUMPON], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PUMPON_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PUMPON_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nV2 != 0)&&(nV2 != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PUMPON_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if (nV1 == 0) //PUMPON:0:x
			SetTurboPump(nV2);	//
		else if (nV1 == 1) //PUMPON:1:x
			SetScrollPump(nV2);	//
#if ENABLE_IP == 1
		else if (nV1 == 2) //PUMPON:2:x
			SetIonPump(nV2);	//
#endif
		goto DecipherOK;
	}
#define CMD_PWRON		"PWRON:"
#define LEN_PWRON		(sizeof(CMD_PWRON)-1)
	if (strncmp(szCommand, CMD_PWRON, LEN_PWRON) == 0)
	{	//PWRON:0:ON/OFF HV_PWR, PWRON:1:ON/OFF GV_PWR, PWRON:2:ON/OFF VOP(deflector, stigmator)
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_PWRON], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PWRON_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > 9)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PWRON_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nV2 != 0)&&(nV2 != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_PWRON_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if (nV1 == 0)
			SetHVPower(nV2);	//DOUT4, PWRON:0:X
		/*else if (nV1 == 1)
			SetGVPower(nV2); 	//DOUT5, PWRON:1:X */
		else if (nV1 == 2)
			SetVOPPower(nV2);	//DOUT8, PWRON:2:X, control VOP power
		else if (nV1 == 3)
			SetRelayOn(nV2);	//RELAY_ON, PWRON:3:X, control relay power, +12VR
		else if (nV1 == 4)
			SetSenrSel(nV2);	//SENR_SEL
		else if (nV1 == 5)
			SetCoilShunt(nV2);	//SHUNT
		else if (nV1 == 6) { //PWRON:6:1
			SetRelayOn(nV2);
			SetVOPPower(nV2);
		}
#if GAUGE_POWER_CTRL == 1
		else if (nV1 == 7) {	//vac gauge power on/off control
			SetIO_VAC_Gauges(nV2);
		}
#endif
		else if (nV1 == 8) {
			SetTurboPump(nV2);
		}
		else if (nV1 == 9) {
			SetScrollPump(nV2);
		}
//		else {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
		goto DecipherOK;
	}
#if SCAN_PCB_VER >= 7
#define CMD_RAMDUMP		"RAMDUMP:"
#define LEN_RAMDUMP		(sizeof(CMD_RAMDUMP)-1)
	if (strncmp(szCommand, CMD_RAMDUMP, LEN_RAMDUMP) == 0) 	{ //RAMDUMP:X1000:256:2
		//64 Mb, nCS is determined by bit 7
		ReplaceChar(szCommand, ':', ' ');
		nV2 = 2;
		if (szCommand[LEN_RAMDUMP] == 'X')
			nPN = sscanf(&szCommand[LEN_RAMDUMP + 1], "%lX %d %d", &dwAddr, &nV1, &nV2);
		else
			nPN = sscanf(&szCommand[LEN_RAMDUMP], "%ld %d %d", &dwAddr, &nV1, &nV2);
		if ((nPN != 2) && (nPN != 3)) {
			printf(szValue, ErrorMsg::GetErrorText(ERR_RAMDUMP_INV_PARAM_NUM));
			goto DecipherFail;
		}
		DumpRAM(dwAddr, nV1, nV2); //nV1=length; nV2=1,BYTE;nV2=2,WORD
		goto DecipherOK;
	}
#define CMD_RAMSET		"RAMSET:"
#define LEN_RAMSET		(sizeof(CMD_RAMSET)-1)
	if (strncmp(szCommand, CMD_RAMSET, LEN_RAMSET) == 0) {
		//RAMSET:X1000:0:256 //memset(addr, val, num)
		//64 Mb, nCS is determined by bit 7
		ReplaceChar(szCommand, ':', ' ');
		if (szCommand[LEN_RAMSET] == 'X')
			nPN = sscanf(&szCommand[LEN_RAMSET + 1], "%lX %d %d", &dwAddr, &nV1, &nV2);
		else
			nPN = sscanf(&szCommand[LEN_RAMSET], "%ld %d %d", &dwAddr, &nV1, &nV2);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_RAMSET_INV_PARAM_NUM));
			goto DecipherFail;
		}
		SetBaseAddr(dwAddr);
		for (i = 0; i < nV1; i++) { //nV2=length, nV1=value
			pwSram[i] = nV2;
		}
		goto DecipherOK;
	}
	//nV1=RW control,0x01:write,0x02:read,0x03:rad and write
	//nV2=chip 0 only
	//e.g. RAMTEST:3:0 test read and write
#define CMD_RAMTEST		"RAMTEST:"
#define LEN_RAMTEST		(sizeof(CMD_RAMTEST)-1)
	if (strncmp(szCommand, CMD_RAMTEST, LEN_RAMTEST) == 0) //Y, row index
	{
		//64 Mb, nCS is determined by bit 7
		WORD wWriteBuf[RAM_ADDR_BITN];
		//
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_RAMTEST], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_RAMTEST_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (nV2 > RAM_TEST_NUM) nV2 = RAM_TEST_NUM;
		else if (nV2 < 1) nV2 = 1;
		nTestNum = nV2; //RAMTEST:0:1
		for (nTestNdx = 0; nTestNdx < nTestNum; nTestNdx++) {
			szBuffer[0] = 0;
			nCh = 0;
			dwAddr = 0x00000001; //0x0040_0000
			wV1 = (nCh == 0) ? 0x5555 : 0x3698;
			for (i = 0; i < RAM_ADDR_BITN; i++) {
				dwCS = (nCh == 0) ? dwAddr : (dwAddr + 0x00400000); //chip 0,1
				SetBaseAddr(dwCS);
				wWriteBuf[i] = wV1;
				pwSram[0] = wV1;
				sprintf(szValue, "X%08lX,%04X(W)\n", dwCS, wV1);
				wV1 = wV1 ^ 0xFFFF;
				strcat(szBuffer, szValue);
				dwAddr = dwAddr << 1;
			}
			TxString(nType, szBuffer);
			TxString(nType, "\r\n");
			//
			szBuffer[0] = 0;
			nPN = 0;
			dwAddr = 0x00000001; //0x0040_0000
			for (i = 0; i < RAM_ADDR_BITN; i++) {
				dwCS = (nCh == 0) ? dwAddr : (dwAddr + 0x00400000);
				SetBaseAddr(dwCS);
				wV2 = pwSram[0];
				if (wV2 != wWriteBuf[i]) {
					sprintf(szValue, "X%08lX,%04X(R, ERROR)\n", dwCS, wV2);
					nPN++;
				}
				else
					sprintf(szValue, "X%08lX,%04X(R)\n", dwCS, wV2);
				strcat(szBuffer, szValue);
				dwAddr = dwAddr << 1;
			}
			TxString(nType, szBuffer);
			bTestOK[nTestNdx] = nPN;
			TxString(nType, "\r\n");
		}
		for (nTestNdx = 0; nTestNdx < nTestNum; nTestNdx++) {
			if (bTestOK[nTestNdx] == 0)
				sprintf(szBuffer, "%02d: RAMTEST PASS\n", nTestNdx + 1);
			else
				sprintf(szBuffer, "%02d: RAMTEST FAIL\n", nTestNdx + 1);
			TxString(nType, szBuffer);
		}
		goto DecipherRet;
	}
#endif
//
#define CMD_REGION	"REGION:"
#define LEN_REGION	(sizeof(CMD_REGION)-1)
	if (strncmp(szCommand, CMD_REGION, LEN_REGION) == 0)
	{	//region scan, fV1 = 0.2 ~ 0.8
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_REGION], "%d %f", &nV1, &fV1);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_REGION_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 != 0)&&(nV1 != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_REGION_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if ((nV1 == 1) && ((fV1 > 0.9) || (fV1 < 0.1))) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_REGION_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		//REGION:1:0.3 //enable region scan
		bRegionScan = (nV1 == 1) ? 1 : 0;
		fRegion = fV1;
		fRegionStart =  0.5 - fRegion / 2;
		nRegionYStart = (float)nScanPixelNumY * (0.5 - fV1 / 2);
		nRegionYEnd = (float)nScanPixelNumY * (0.5 + fV1 / 2);
		pwFpga[SET_REGION_SCAN] = (nV1 == 1) ? 1 : 0;
		pwFpga[SET_Y_START] = (WORD)nRegionYStart;
		pwFpga[SET_Y_END] = (WORD)nRegionYEnd;
		goto DecipherOK;
	}
//
#define CMD_RETURN	"RETURN:"
#define LEN_RETURN	(sizeof(CMD_RETURN)-1)
	if (strncmp(szCommand, CMD_RETURN, LEN_RETURN) == 0)
	{	//
		nPN = sscanf(&szCommand[LEN_RETURN], "%d", &nN);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_RETURN_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nN != 0)&&(nN != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_RETURN_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bReturn = (nN == 1) ? 1 : 0;
		goto DecipherOK;
	}
#define CMD_RESEND		"RESEND:"	//serial output
#define LEN_RESEND		(sizeof(CMD_RESEND)-1)
	if (strncmp(szCommand, CMD_RESEND, LEN_RESEND) == 0) //UR1, UR2 output test
	{
		nPN = sscanf(&szCommand[LEN_RESEND], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_RESEND_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (nResendNum < RESEND_NUM_MAX)
			nResendY[nResendNum++] = nV1; //nV1 = Y position
		goto DecipherRet;
	}
#define CMD_RESET	"RESET:"
#define LEN_RESET	(sizeof(CMD_RESET)-1)
	if (strncmp(szCommand, CMD_RESET, LEN_RESET) == 0)
	{	//
		if (szCommand[LEN_RESET] == '?') {
			printf("RESET:1  reset IO, HV\n");
			printf("RESET:2  reset VADC process\n");
			printf("RESET:3  reset VADC\n");
			printf("RESET:4  reset vac gauge power\n");
			printf("RESET:5  reset FPGA\n");
			goto DecipherRet;
		}
		nPN = sscanf(&szCommand[LEN_RESET], "%d", &nN);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_RESET_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nN< 1)||(nN > 5)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_RESET_INV_PARAM_VAL));
			goto DecipherFail;
		}
		if (nN == 1) { //RESET:1
			Reset_IO(); //reset I/O board
			SetHVSwitch(0); //turn off HV
		}
		else if (nN == 2) {//RESET:2, reset scan board and video ADC
			SetImageInit();
		}
#if ENABLE_RESET_VADC == 1
		else if (nN == 3) { //RESET:3
			ResetVADC();
		}
#endif
#if GAUGE_POWER_CTRL == 1
		else if (nN == 4) { //RESET:4
			ResetVacPower();
		}
#endif
		else if (nN == 5)
			Reset_FPGA();
		ClearTurboCommand();
		goto DecipherOK;
	}
#define CMD_ROTATE	"ROTATE:"
#define LEN_ROTATE	(sizeof(CMD_ROTATE)-1)
	if (strncmp(szCommand, CMD_ROTATE, LEN_ROTATE) == 0)	//get image ADC
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_ROTATE], "%f", &fV);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ROTATE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (fV < 0)||(fV > 359)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_ROTATE_INV_PARAM_VAL));
			goto DecipherFail;
		}
		SysParam.fRotateAngle = fV;
		SetRotationAngle(fV); //unit degree
		goto DecipherOK;
	}
#define CMD_SAVESET		"SAVESET"
#define LEN_SAVESET		(sizeof(CMD_SAVESET)-1)
	if (strncmp(szCommand, CMD_SAVESET, LEN_SAVESET) == 0) {
		SaveUserParameters(&NV_Settings, sizeof(NV_SETTINGS));
		goto DecipherOK;
	}
#define CMD_SCALE	"SCALE:"
#define LEN_SCALE	(sizeof(CMD_SCALE)-1)
	if (strncmp(szCommand, CMD_SCALE, LEN_SCALE) == 0)	{
		if (szCommand[LEN_SCALE] == '?') {
			printf("1:SCALE_BY_TRIG_W_ROT, rotation\n");
			printf("2:SCALE_BY_DACVID, SEM\n");
			printf("3:SCALE_BY_TRIG_WO_ROT, EDS\n");
			printf("4:SCALE_BY_DAC16B\n");
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SCALE], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCALE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1< 1)||(nV1 > 4)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCALE_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bScaleMode = nV1; 	//rotate mode
		goto DecipherOK;
	}
#define CMD_SCANSTART		"SCANSTART:"
#define LEN_SCANSTART		(sizeof(CMD_SCANSTART)-1)
	if (strncmp(szCommand, CMD_SCANSTART, LEN_SCANSTART) == 0) {	//<T>: interval in msec
		//e.g. SCANSTART:<N>:<T><CR><LF>
		if (bInitOK == 0) goto DecipherRet;
		if (IsCommandEnabled(nType) != SUCCESS) goto DecipherRet;
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SCANSTART], "%d %d", &nN, &nT);	//nN times, nB: interval in msec

		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCANSTART_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nN < 0)||(nN > 65535)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCANSTART_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nT < 0)||(nT > 60000)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SCANSTART_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if ((SysParam.bErrorHWVersion & ERR_HW_PCBID) != 0) {
			nErrCode = ERR_INV_PID;
			goto DecipherFail;
		}
		if (SysParam.bErrorHWVersion != 0) {
			nErrCode = ERR_INV_VERSION; //Hardware version error
			goto DecipherFail;
		}
		/*if (bEnableEDS == 1) {
			nErrCode = ERR_EDS_ONLY;
			goto DecipherFail;
		}*/
		/*if (SysParam.bVacuumState != VAC_ST_READY) {
			nErrCode = ERR_INV_VAC_ST;
			goto DecipherFail;
		}*/
		/*
		if (SysParam.bScanning != OP_IDLE) {
			nErrCode = ERR_REENTRY;
			goto DecipherFail; //can't reentry
		}*/
		//
		SetVOPPower(1); //turn on VOPA and VOPB
		OSTimeDly(4);
		//SetHVPower(1);	//turn on HV power
		//OSTimeDly(TICKS_PER_SECOND);
		//
		fV = PercentToVolt(fBrightness[0], DAC_CH_BR0);
		SetVoltage(BOARD_SCAN, DAC_CH_BR0, fV);
		fV = PercentToVolt(fBrightness[1], DAC_CH_BR1);
		SetVoltage(BOARD_SCAN, DAC_CH_BR1, fV);
		fV = PercentToVolt(fBrightness[2], DAC_CH_BR2);
		SetVoltage(BOARD_SCAN, DAC_CH_BR2, fV);
		fV = PercentToVolt(fBrightness[3], DAC_CH_BR3);
		SetVoltage(BOARD_SCAN, DAC_CH_BR3, fV);
		//
		fV = PercentToVolt(fContrast[0], DAC_CH_CO);
		SetVoltage(BOARD_SCAN, DAC_CH_CO, fV);
		//
		bACK = 0;
		bReturn = 0;
		bScanAbort = 0;
		bScanPause = 0;
		SysParam.bScanning = OP_SEM_SCAN;
		nEchoTimeout = ECHO_TIMEOUT;
		nIdleTimeout = IDLE_TIMEOUT;
		//
		bScanRestart = 0;
		nScanImageNum = nN;	//start ScanMonitorTask
		nScanImageNdx = 0;
		if (nT <= 0) nT = PITR_INTERVAL; //as fast as possible
		//nScanInterval = (int)(nT/PITR_INTERVAL);	//unit PITR_INTERVAL msec
		//nScanIntervalTick = 0;
		ClearCommandInQueue();
		//SetupPITR(1, PITR_FREQ);	//10 Hz
		if (bDebug == 0) printf("SCANSTART\n");
		SetRotationAngle(SysParam.fRotateAngle); //unit degree
		//
		//disable region mode
		bRegionScan = 0;
		pwFpga[SET_REGION_SCAN] = 0;
		goto DecipherOK;
	}
#define CMD_SETAO		"SETAO:"
#define LEN_SETAO		(sizeof(CMD_SETAO)-1)
	//SETAO:<B>:<C>:<FV><CR><LF>
	if (strncmp(szCommand, CMD_SETAO, LEN_SETAO) == 0)
	{	//<B>:BID, <C>:0-base channel, <FV>: voltage
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETAO], "%d %d %f", &nB, &nCh, &fV);	//fV: current in unit mA

		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nB < 0)||(nB >= BOARD_NUM)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAO_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nCh < 0)||(nCh >= DAC_CH_NUM_MAX)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAO_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if( (fV > fDACVMax[nB][nCh])||(fV < fDACVMin[nB][nCh]) ){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAO_INV_PARAM3_VAL));
			goto DecipherFail;
		}

		SetVoltage(nB, nCh, fV);
		goto DecipherOK;
	}
#define CMD_SETAOIW		"SETAOIW:"
#define LEN_SETAOIW		(sizeof(CMD_SETAOIW)-1)
	if (strncmp(szCommand, CMD_SETAOIW, LEN_SETAOIW) == 0) //set video DAC output
	{	////SETAOIW:<C>:<V>, <C>:0-base channel, <V>: voltage
		if (bDB3Busy == 1) //don't process during scanning
			goto DecipherOK;
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETAOIW], "%d %d", &nCh, &nV1);	//fV: current in unit mA
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAOIW_INV_PARAM_NUM));
			goto DecipherFail;
		}

		//nCh = (nCh == 0) ? DAC_CH_DEFX_F : DAC_CH_DEFY_F;
		wV = (WORD)nV1;
		if (nCh == 0) SetFineDACX(wV);
		else SetFineDACY(wV);
		goto DecipherOK;
	}
#define CMD_SETAOI	"SETAOI:"
#define LEN_SETAOI	(sizeof(CMD_SETAOI)-1)
	if (strncmp(szCommand, CMD_SETAOI, LEN_SETAOI) == 0) //set video DAC output
	{	////SETAOI:<C>:<V>, <C>:0-base channel, <V>: voltage
		if (bDB3Busy == 1) //don't process during scanning
			goto DecipherOK;
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETAOI], "%d %f", &nCh, &fV);	//fV: current in unit mA
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAOI_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nCh !=  0)&&(nCh != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAOI_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if ((fV > DACF_OUT_MAX) || (fV < DACF_OUT_MIN)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAOI_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		nCh = (nCh == 0) ? DAC_CH_DEFX_F : DAC_CH_DEFY_F;
		SetDeflectorFineV(nCh, fV);
		goto DecipherOK;
	}
	//N=1,4.5;2,6.6;3,9.4;4,10.476;5:13.1 (usec)
	//6,15.1;7,17.1;8,18.44;9,
	//N=1,4.1;2,4.8;3,7.0;4,7.1;5:8.9 (usec) (channel 1)
	//6,9.9;7,11.1;8,11.4;9,13.1;16,13.136
#define CMD_SETAVGNUM	"SETAVGNUM:"
#define LEN_SETAVGNUM	(sizeof(CMD_SETAVGNUM)-1)
	if (strncmp(szCommand, CMD_SETAVGNUM, LEN_SETAVGNUM) == 0)
	{	//SETAVGNUM:<N>, default=1
		nPN = sscanf(&szCommand[LEN_SETAVGNUM], "%d", &nN);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAVGNUM_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nN !=  1)&&(nN != 2)&&(nN !=  4)&&(nN != 8)&&(nN !=  16)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETAVGNUM_INV_PARAM_VAL));
			goto DecipherFail;
		}
//		if ((nN < AVGNUM_MIN) || (nN > AVGNUM_MAX)) {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
#if USE_AD7655 == 1
		if (nN >= 32) { //for old settings
			nV = (nN <= 32) ? 1 :	//5 us
				(nN == 48) ? 2 :	//7 us
				(nN == 64) ? 3 :	//9 us
				(nN == 128) ? 4 :	//11
				(nN == 256) ? 5 :	//13
				(nN == 384) ? 6 :	//15
				(nN == 512) ? 7 :	//17
				(nN == 1024) ? 8 : 2;
			nN = nV;
		}
#endif
		nVADCAvgNum = nN;
		GetXScanTime(nScanPixelNumX, nVADCAvgNum);
		pwFpga[SET_AVG_NUM] = nN; //nN=1,2,4,8,16
		pwFpga[SET_MULTIPLY] = MULTIPLY_NUM / nN;
		printf("SETAVGNUM %d\r\n", nN);
		goto DecipherOK;
	}
#define CMD_SETBRX	"SETBRX:"
#define LEN_SETBRX	(sizeof(CMD_SETBRX)-1)
	if (strncmp(szCommand, CMD_SETBRX, LEN_SETBRX) == 0) //set brightness, BOARD_SCAN
	{	//SETBRX:<NV>:<BR><CR><LF>
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETBRX], "%d %f", &nV1, &fV); //percentage
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETBRX_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 > BR_NUM)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETBRX_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if ((fV > 100) || (fV < 0)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETBRX_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		//fV = 0 ~ 100
		fBrightness[nV1] = NV_Settings.fBaseBR[nV1] + fV * NV_Settings.fRangeBR[nV1] / 100.0; //50% ~ 83%
		fV = PercentToVolt(fBrightness[nV1], nDACChBR[nV1]);
		SetVoltage(BOARD_SCAN, nDACChBR[nV1], fV);
		goto DecipherOK;
	}
#define CMD_SETBR	"SETBR:"
#define LEN_SETBR	(sizeof(CMD_SETBR)-1)
	if (strncmp(szCommand, CMD_SETBR, LEN_SETBR) == 0) //set brightness, BOARD_SCAN
	{	//SETBR:<BR><CR><LF>
		//if (bDB3Busy == 1) {
		//	PushCommandToQueue(szCommand, nType);
		//	goto DecipherRet;
		//}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETBR], "%f", &fV); //percentage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETBR_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fV > 100) || (fV < 0)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETBR_INV_PARAM_VAL));
			goto DecipherFail;
		}
		//--------------------------------
		// fV = 0 ~ 100
		//--------------------------------
		fBR = NV_Settings.fBaseBR[0] + fV * NV_Settings.fRangeBR[0] / 100.0; //50% ~ 66% (0 ~ 1.65 volts)
		fV = PercentToVolt(fBrightness[0], DAC_CH_BR);
		for (i = 0; i < 4; i++) {
			SetVoltage(BOARD_SCAN, nDACChBR[i], fV);
			fBrightness[i] = fBR;
		}
		goto DecipherOK;
	}
	//------------------------------------------------------
	//SETCI:<B>:<C>:<I><CR><LF>
	//set coil current
	//------------------------------------------------------
	if (strncmp(szCommand, "SETCI:", 6) == 0) //set coil current(I)
	{	//nN=<C>=SIG_CH_DEFX(0),SIG_CH_DEFY(1),SIG_CH_STIGX(2),SIG_CH_STIGY(3),SIG_CH_OBJ(4)
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[6], "%d %d %f", &nB, &nN, &fI);	//fV: current in unit mA
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETCI_INV_PARAM_NUM));
			goto DecipherFail;
		}
		switch (nN)
		{
			/*
			case ISEN_CH_DEFX:	//0
			case ISEN_CH_DEFY:	//1
				SetDeflectorCurrent(nN, fI); //nN =
				break;
			case SIG_CH_OBJ:	//4
				SetObjectiveCurrent(fI);
				break;*/
			case ISEN_CH_STIGX:	//2
			case ISEN_CH_STIGY:	//3
			case ISEN_CH_AL0:	//5
			case ISEN_CH_AL1:	//6
			case ISEN_CH_AL2:	//7
			case ISEN_CH_AL3:	//8
				SetCoilCurrent(nN, fI);
				break;
		}
		goto DecipherOK;
	}
#define CMD_SETCOX	"SETCOX:"
#define LEN_SETCOX	(sizeof(CMD_SETCOX)-1)
	if (strncmp(szCommand, CMD_SETCOX, LEN_SETCOX) == 0) //set contrast
	{	//SETCOX:<NV>:<CO><CR><LF>
		//ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETCOX], "%d:%f", &nV1, &fV); //fV is percentage
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETCOX_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 < 0)||(nV1 >2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETCOX_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if ((fV > 100) || (fV < 0)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETCOX_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		//nV1=0,1,2
		fContrast[nV1] = NV_Settings.fBaseCO[nV1] + fV * NV_Settings.fRangeCO[nV1] / 100.0; //fV=0~100, avoid video output over flow
		fV = PercentToVolt(fContrast[nV1], nDACChCO[nV1]);
		SetVoltage(BOARD_SCAN, nDACChCO[nV1], fV);
		goto DecipherOK;
	}
#define CMD_SETCO	"SETCO:"
#define LEN_SETCO	(sizeof(CMD_SETCO)-1)
	if (strncmp(szCommand, CMD_SETCO, LEN_SETCO) == 0) //set contrast
	{	//SETCO:<CO><CR><LF>
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETCO], "%f", &fV); //fV is percentage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETCO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fV > 100) || (fV < 0)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETCO_INV_PARAM_VAL));
			goto DecipherFail;
		}
		fContrast[0] = NV_Settings.fBaseCO[0] + fV * NV_Settings.fRangeCO[0] / 100.0; //fV=0~100%, avoid video output over flow
		//0~50% --> 0 ~ 5 volts
		fV = PercentToVolt(fContrast[0], DAC_CH_CO);
		SetVoltage(BOARD_SCAN, DAC_CH_CO, fV);
		goto DecipherOK;
	}
#define CMD_SENRSEL	"SENRSEL:"
#define LEN_SENRSEL	(sizeof(CMD_SENRSEL)-1)
	if (strncmp(szCommand, CMD_SENRSEL, LEN_SENRSEL) == 0) //set coil current(I)
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SENRSEL], "%d", &nV1); //percentage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SENRSEL_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nV1 !=  0)&&(nV1 != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SENRSEL_INV_PARAM_VAL));
			goto DecipherFail;
		}
		SetSenrSel(nV1);
		goto DecipherOK;
	}
#define CMD_SETDO		"SETDO:"
#define LEN_SETDO		(sizeof(CMD_SETDO)-1)
	if (strncmp(szCommand, CMD_SETDO, LEN_SETDO) == 0)
	{	//SETDO:<B>:<N>
		ReplaceChar(szCommand, ':', ' ');
		if (szCommand[LEN_SETDO+2] == 'X') {
			szCommand[LEN_SETDO+2] = ' ';
			nPN = sscanf(&szCommand[LEN_SETDO], "%d %X", &nB, &nN);
		}
		else
			nPN = sscanf(&szCommand[LEN_SETDO], "%d %d", &nB, &nN);

		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETDO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nB !=  0)&&(nB != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETDO_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		wV = (WORD)nN;
		SetDigitalOutput(nB, wV); //nB=0, SCAN; 1, IO
		goto DecipherOK;
	}
#define CMD_SETDOB		"SETDOB:"
#define LEN_SETDOB		(sizeof(CMD_SETDOB)-1)
	if (strncmp(szCommand, CMD_SETDOB, LEN_SETDOB) == 0)
	{	//SETDO:<B>:<C>:<NV>
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETDOB], "%d %d %d", &nB, &nV1, &nV2);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETDOB_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (nB !=  0)&&(nB != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETDOB_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if( (nV1 <  0)||(nV1 > 15)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETDOB_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		if( (nV2 !=  0)&&(nV2 != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETDOB_INV_PARAM3_VAL));
			goto DecipherFail;
		}
		SetDigitalOutputBit(nB, nV1, nV2);
		//---------------------------------------------------------
		//can't be set every time
		//this is a bug of verilog code, the setting is not stable
		//schmitt trigger problem (?) --> FPGA abnormal reset
		//---------------------------------------------------------
		goto DecipherOK;
	}
#define CMD_SETHV		"SETHV"
#define LEN_SETHV		(sizeof(CMD_SETHV)-1)
	if (strncmp(szCommand, CMD_SETHV, LEN_SETHV) == 0)
	{	//HVA, HVB, HVF, HVG SETHVF:
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETHV+2], "%f", &fV);	//fV: unit voltage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETHV_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fV != 0) && (IsInterlockSafe(ACT_SET_HV) != SUCCESS))
		{	//check interlock only when fV is not zero
			nErrCode = ERR_INTLK_FAIL;
			goto DecipherFail;
		}
		switch (szCommand[LEN_SETHV]) //[6] =':'
		{
			case 'F':	//HV filament
				SetHVFilament(fV); //unit volt
				break;
			case 'G':
				SetHVFEG(fV); //unit volt
				break;
			case 'B':	//HV bias
				SetHVBias(fV); //unit volt
				break;
			case 'A':	//HV acceleration
				SetHVAccelerator(fV);
				bCommandDelay = 1;
				break;
			default:
				goto DecipherFail;
		}
		goto DecipherOK;
	}
	if (strncmp(szCommand, "SETLDR:", 7) == 0)
	{	//current sense resistor value
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[7], "%f %f %f %f %f %f %f %f %f", &fVR[0], &fVR[1], &fVR[2], &fVR[3], &fVR[4],
			&fVR[5], &fVR[6], &fVR[7], &fVR[8]);

		if (nPN != ISEN_CH_NUM) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETLDR_INV_PARAM_NUM));
			goto DecipherFail;
		}
		for (nV = 0; nV < ISEN_CH_NUM; nV++)
			NV_Settings.fLoadR[nV] = fVR[nV];
		goto DecipherOK;
	}
	if (strncmp(szCommand, "SETOBJ:", 7) == 0) //set OBJ coarse, fine, max, min
	{	//DAC_OBJ_C*fCoarseRatio[SIG_CH_OBJ]+DAC_OBJ_F*fFineRatio[SIG_CH_OBJ]
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[7], "%f", &fV); //fV = current
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOBJ_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if( (fV <  0)||(fV > 100)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOBJ_INV_PARAM_VAL));
			goto DecipherFail;
		}

		fObjPercent = fV / 100; //used for simulation, change to 0 ~ 0.99
		//
		goto DecipherOK;
	}
	if (strncmp(szCommand, "SETOFSAD", 8) == 0) //set DAC offset
	{	//SETOFSAD:<B>:<C>:<FV>
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[9], "%d %d %f", &nB, &nCh, &fV);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSAD_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fV < -3) || (fV > 3)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSAD_INV_PARAM3_VAL));
			goto DecipherFail;
		}
		if (nB == 2)
			if (nCh < VADC_CH_NUM_MAX)
				NV_Settings.fScanADCOffset[nCh] = fV;	//video ADC
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSAD_INV_PARAM2_VAL));
				goto DecipherFail;
			}
		else if (nB == 1) //IO board
			if (nCh < ADC_CH_NUM_MAX)
				NV_Settings.fADCOffset[nCh] = fV;
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSAD_INV_PARAM2_VAL));
				goto DecipherFail;
			}
		else {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSAD_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		goto DecipherOK;
	}
#define CMD_SETOFSDA		"SETOFSDA:"
#define LEN_SETOFSDA		(sizeof(CMD_SETOFSDA)-1)
	if (strncmp(szCommand, CMD_SETOFSDA, LEN_SETOFSDA) == 0) //set DAC offset
	{	//SETOFSDA:<B>:<C>:<FV>
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETOFSDA], "%d %d %f", &nB, &nCh, &fV);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSDA_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fV < -3) || (fV > 3)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSDA_INV_PARAM3_VAL));
			goto DecipherFail;
		}
		if (nB < BOARD_NUM)
			if (nCh < DAC_CH_NUM_MAX)
				NV_Settings.fDACOffset[nB][nCh] = fV;
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSDA_INV_PARAM2_VAL));
				goto DecipherFail;
			}
		else if (nB == 6) //IO board
			if (nCh < VDAC_CH_NUM_MAX)
				NV_Settings.fScanDACOffset[nCh] = fV;	//video DAC
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSDA_INV_PARAM2_VAL));
				goto DecipherFail;
			}
		else {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETOFSDA_INV_PARAM1_VAL));
			goto DecipherFail;
		}
//		if (nPN != 3) {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
//		if ((fV < -3) || (fV > 3) || (nCh > DAC_CH_NUM_MAX)) {
//			nErrCode = ERR_OVER_RANGE;
//			goto DecipherFail;
//		}
//		if (nB < BOARD_NUM && nCh < DAC_CH_NUM_MAX)
//			NV_Settings.fDACOffset[nB][nCh] = fV;
//		else if (nB == 6 && nCh < VDAC_CH_NUM_MAX) //set fine DAC offset
//			NV_Settings.fScanDACOffset[nCh] = fV;	//video DAC
		goto DecipherOK;
	}
#define CMD_SETSLPDA		"SETSLPDA:"
#define LEN_SETSLPDA		(sizeof(CMD_SETSLPDA)-1)
	if (strncmp(szCommand, CMD_SETSLPDA, LEN_SETSLPDA) == 0) //set DAC offset
	{	//SETSLPDA:<B>:<C>:<FV>
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETSLPDA], "%d %d %f", &nB, &nCh, &fV);
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSLPDA_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fV < 0.5) || (fV > 2.0)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSLPDA_INV_PARAM3_VAL));
			goto DecipherFail;
		}
		if (nB < BOARD_NUM)
			if (nCh < DAC_CH_NUM_MAX)
				NV_Settings.fDACSlope[nB][nCh] = fV;
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSLPDA_INV_PARAM2_VAL));
				goto DecipherFail;
			}
		else if (nB == 6) //IO board
			if (nCh < VDAC_CH_NUM_MAX)
				NV_Settings.fScanDACSlope[nCh] = fV;	//video DAC
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSLPDA_INV_PARAM2_VAL));
				goto DecipherFail;
			}
		else {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSLPDA_INV_PARAM1_VAL));
			goto DecipherFail;
		}
//		if (nPN != 3) {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
//		if ((fV < 0.5) || (fV > 2.0) || (nCh > DAC_CH_NUM_MAX)) {
//			nErrCode = ERR_OVER_RANGE;
//			goto DecipherFail;
//		}
//		if ((nB < BOARD_NUM) && (nCh < DAC_CH_NUM_MAX))
//			NV_Settings.fDACSlope[nB][nCh] = fV;
//		else if (nB == 6 && nCh < VDAC_CH_NUM_MAX) //set fine DAC slope
//			NV_Settings.fScanDACSlope[nCh] = fV;	//video DAC
		goto DecipherOK;
	}
#define CMD_SETPITR		"SETPITR:"
#define LEN_SETPITR		(sizeof(CMD_SETPITR)-1)
	if (strncmp(szCommand, CMD_SETPITR, LEN_SETPITR) == 0) { //SETPITR:<T>, interval in msec
		nPN = sscanf(&szCommand[LEN_SETPITR], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETPITR_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (nV1 < 0) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETPITR_INV_PARAM_VAL));
			goto DecipherFail;
		}

		if (nV1 == 0) { //disable PIT
			fV = 0;
			StopPITR();
		}
		else {
			fV = 1000 / (float)nV1; //change interval to frequency
			SetupPITR(1, fV);
		}
		goto DecipherOK;
	}
#define CMD_SETPIXN		"SETPIXN:" //set pixel number
#define LEN_SETPIXN		(sizeof(CMD_SETPIXN)-1)
	if (strncmp(szCommand, CMD_SETPIXN, LEN_SETPIXN) == 0)
	{	//SETPIXN:<NX>:<NY>
		if (bDB3Busy == 1) {
			PushCommandToQueue(szCommand, nType);
			goto DecipherRet;
		}
		if (szCommand[LEN_SETPIXN] == '?') { //SETPIXN:?
			nV1 = pwFpga[GET_X_NUM];
			nV2 = pwFpga[GET_Y_NUM];
			printf("XN=%d,YN=%d\n", nV1, nV2);
			goto DecipherRet;
		}
		nPN = sscanf(&szCommand[LEN_SETPIXN], "%d:%d", &nNX, &nNY);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETPIXN_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nNX > PIXEL_NUM_X_MAX)||(nNX * (1.0 + fOverscan) > PIXEL_NUM_X_MAX)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETPIXN_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if (nNY > PIXEL_NUM_Y_MAX) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETPIXN_INV_PARAM2_VAL));
			goto DecipherFail;
		}

//		if (nPN != 2) {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
//		if ((nNX > PIXEL_NUM_X_MAX) || (nNY > PIXEL_NUM_Y_MAX)) { //fXRange
//			nErrCode = ERR_OVER_RANGE;
//			goto DecipherFail;
//		}
//		if (nNX * (1.0 + fOverscan) > PIXEL_NUM_X_MAX) {
//			nErrCode = ERR_OVER_RANGE;
//			goto DecipherFail;
//		}
		if (SetPixelNum(nNX, nNY) != SUCCESS) { //sOverscan
			nErrCode = ERR_OVER_RANGE; // Omar never Fail
			goto DecipherFail;
		}
		GetXScanTime(nScanPixelNumX, nVADCAvgNum);
		nScanImageNdx = 0;
		SetDescentCurve(nNX, nNY); //sOverscan
		InitFastScan();
		bCommandDelay = 1;
		printf("SETPIXN:%d:%d\r\n", nNX, nNY);
		goto DecipherOK;
	}
#define CMD_SETR	"SETR:"
#define LEN_SETR	(sizeof(CMD_SETR)-1)
	if (strncmp(szCommand, CMD_SETR, LEN_SETR) == 0) { //set brightness, BOARD_SCAN
		//SETR:<BR><CR><LF>
		if (szCommand[LEN_SETR] == '?') {
			printf("0:%.2f brightness baseline 0\n", NV_Settings.fBaseBR[0]);
			printf("1:%.2f brightness baseline 1\n", NV_Settings.fBaseBR[1]);
			printf("2:%.2f brightness baseline 2\n", NV_Settings.fBaseBR[2]);
			printf("3:%.2f brightness baseline 3\n", NV_Settings.fBaseBR[3]);
			printf("4:%.2f brightness range 0\n", NV_Settings.fRangeBR[0]);
			printf("5:%.2f brightness range 1\n", NV_Settings.fRangeBR[1]);
			printf("6:%.2f brightness range 2\n", NV_Settings.fRangeBR[2]);
			printf("7:%.2f brightness range 3\n", NV_Settings.fRangeBR[3]);
			printf("8:%.2f contrast baseline 0-3\n", NV_Settings.fBaseCO[0]);
			printf("9:%.2f contrast range 0-3\n", NV_Settings.fRangeCO[0]);
			printf("SETR:10:0 reset to default\n");
			printf("11:%.2f brightness baseline 4\n", NV_Settings.fBaseBR[4]);
			printf("12:%.2f brightness baseline 5\n", NV_Settings.fBaseBR[5]);
			printf("13:%.2f brightness range 4\n", NV_Settings.fRangeBR[4]);
			printf("14:%.2f brightness range 5\n", NV_Settings.fRangeBR[5]);
			printf("15:%.2f contrast baseline 1\n", NV_Settings.fBaseCO[1]);
			printf("16:%.2f contrast baseline 2\n", NV_Settings.fBaseCO[2]);
			printf("17:%.2f contrast range 1\n", NV_Settings.fRangeCO[1]);
			printf("18:%.2f contrast range 2\n", NV_Settings.fRangeCO[2]);
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETR], "%d %f", &nV1, &fV); //percentage
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETR_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fV < 0.0) || (fV > 100.0)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETR_INV_PARAM2_VAL));
			goto DecipherFail;
		}

		if ((nV1 >= 0) && (nV1 <= 3))
			NV_Settings.fBaseBR[nV1] = fV;
		else if ((nV1 >= 4) && (nV1 <= 7))
			NV_Settings.fRangeBR[nV1 - 4] = fV;
		else if (nV1 == 8)
			NV_Settings.fBaseCO[0] = fV; //SETR:3:BASEBR
		else if (nV1 == 9)
			NV_Settings.fRangeCO[0] = fV; //SETR:1:COR (0~100)
		else if ((nV1 == 10) && (fV == 0.0)) { //set to default
			ResetBRCOToDefault();
		}
		else if ((nV1 >= 11) && (nV1 <= 12))
			NV_Settings.fBaseBR[nV1 - 7] = fV; //[4],[5]
		else if ((nV1 >= 13) && (nV1 <= 14))
			NV_Settings.fRangeBR[nV1 - 9] = fV; //[4],[5]
		else if ((nV1 >= 15) && (nV1 <= 16))
			NV_Settings.fBaseCO[nV1 - 14] = fV; //[1],[2]
		else if ((nV1 >= 17) && (nV1 <= 18))
			NV_Settings.fRangeCO[nV1 - 16] = fV; //[1],[2]
		else{
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETR_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		goto DecipherOK;
	}
#define CMD_SETSTIGX	"SETSTIGX:"
#define LEN_SETSTIGX	(sizeof(CMD_SETSTIGX)-1)
	if (strncmp(szCommand, CMD_SETSTIGX, LEN_SETSTIGX) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETSTIGX], "%f", &fV); //percentage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSTIGX_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (fV < -50 || fV > 50) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSTIGX_INV_PARAM_VAL));
			goto DecipherFail;
		}
		fV = fV / 10;
		SetVoltage(BOARD_IO, DAC_CH_STIGX, fV);
		goto DecipherOK;
	}
#define CMD_SETSTIGY	"SETSTIGY:"
#define LEN_SETSTIGY	(sizeof(CMD_SETSTIGY)-1)
	if (strncmp(szCommand, CMD_SETSTIGY, LEN_SETSTIGY) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETSTIGY], "%f", &fV); //percentage
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSTIGY_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (fV < -50 || fV > 50) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETSTIGY_INV_PARAM_VAL));
			goto DecipherFail;
		}
		fV = fV / 10;
		SetVoltage(BOARD_IO, DAC_CH_STIGY, fV);
		goto DecipherOK;
	}
	if (strncmp(szCommand, "SETV2IR:", 8) == 0)
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[8], "%f %f %f %f %f %f %f %f %f", &fVR[0], &fVR[1], &fVR[2], &fVR[3], &fVR[4],
			&fVR[5], &fVR[6], &fVR[7], &fVR[8]);
		if (nPN != ISEN_CH_NUM) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETV2IR_INV_PARAM_NUM));
			goto DecipherFail;
		}
		for (nV = 0; nV < ISEN_CH_NUM; nV++)
			NV_Settings.fV2IR[nV] = fVR[nV];
		//example
		//1V -> 0.1A, V2IR = 10 (fDefSenR = 1), APerMM[] no change
		//1V -> 0.2A, V2IR = 5  (fDefSenR = 2), APerMM[] no change
		//GetScanRangeMax(0, 1);
		//GetMagnificationMin();
		goto DecipherOK;
	}
#define CMD_SETZOOM		"SETZOOM:" //set screen view width
#define LEN_SETZOOM		(sizeof(CMD_SETZOOM)-1)
	if (strncmp(szCommand, CMD_SETZOOM, LEN_SETZOOM) == 0)
	{	//SETZOOM:<XC>,<YC>,<FV>,<NX>,<NY>
		//FV: range of X, view area = (XC-FV, YC-FV) to (XC+FV, YC+FV)
		if (IsScanning() == 1)	{
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETZOOM_INV_PARAM_IS_SCANNING));
			goto DecipherFail;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SETZOOM], "%f %f %f %d %d", &fXC, &fYC, &fMag, &nNX, &nNY);
		if (nPN != 5) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETZOOM_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((fMag < MAG_MIN) || (fMag > MAG_MAX) || (SetMagnification(fMag) != SUCCESS)) { //fXRange
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETZOOM_INV_PARAM3_VAL));
			goto DecipherFail;
		}
		if (nNX > PIXEL_NUM_X_MAX) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETZOOM_INV_PARAM4_VAL));
			goto DecipherFail;
		}
		if (nNY > PIXEL_NUM_Y_MAX) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SETZOOM_INV_PARAM5_VAL));
			goto DecipherFail;
		}
//		if (nPN != 5) {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
//		//nNY = nNX*3/4
//		if ((nNX > PIXEL_NUM_X_MAX) || (nNY > PIXEL_NUM_Y_MAX)) { //fXRange
//			nErrCode = ERR_OVER_RANGE;
//			goto DecipherFail;
//		}
//		if ((fMag < MAG_MIN) || (fMag > MAG_MAX)) { //fXRange
//			nErrCode = ERR_MAG;
//			goto DecipherFail;
//		}
//		if (SetMagnification(fMag) != SUCCESS) {
//			nErrCode = ERR_OVER_RANGE;
//			goto DecipherFail;
//		}
//		if (SetPixelNum(nNX, nNY) != SUCCESS) { //sOverscan
//			nErrCode = ERR_OVER_RANGE;// Omar never Fail
//			goto DecipherFail;
//		}
		GetXScanTime(nScanPixelNumX, nVADCAvgNum);
		nScanImageNdx = 0;
		SetDescentCurve(nNX, nNY); //sOverscan
		InitFastScan();
		bCommandDelay = 1;
		printf("SETZOOM:%d:%d OK\r\n",nNX,nNY);
		goto DecipherRet;
	}
#define CMD_SHADOW		"SHADOW:" //set screen view width
#define LEN_SHADOW		(sizeof(CMD_SHADOW)-1)
	if (strncmp(szCommand, CMD_SHADOW, LEN_SHADOW) == 0) //set shadow mode
	{
		/*if (bDB3Busy == 1) {
			PushCommandToQueue(szCommand, nType);
			goto DecipherRet;
		}*/
		if (szCommand[LEN_SHADOW] == '?') {
			printf("0-3: CH0-3\n");
			printf("5:   sum of CH0-3\n");
			printf("10:  SEI\n");
			printf("11:  CH0+CH1\n");
			printf("12:  CH0-CH1+0x7FFF\n");
			printf("13:  CH1-CH0+0x7FFF\n");
			printf("14:  CH2+CH3\n");
			printf("15:  CH2-CH3+0x7FFF\n");
			printf("16:  CH3-CH2+0x7FFF\n");
			printf("17:  CH4\n");
			printf("18:  CH5\n");
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SHADOW], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SHADOW_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (nV1 < 0 || nV1 > 18) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SHADOW_INV_PARAM_VAL));
			goto DecipherFail;
		}

		nShadowMode = nV1;
		/*if (bImageMask == 0x03) //send two image
			SelectVideoChip(0); //select all chips
		else if ((nShadowMode >= SHADOW_2CH_P1) && (nShadowMode <= SHADOW_2CH_M2))
			SelectVideoChip(PARA_CHIP0_ONLY); //speed up process
		else if ((nShadowMode >= SHADOW_2CH_P2) && (nShadowMode <= SHADOW_2CH_M4)) {
			SelectVideoChip(PARA_CHIP1_ONLY); //speed up process
		}
		else if ((nShadowMode >= SHADOW_1CH_4) && (nShadowMode <= SHADOW_1CH_5)) {
			SelectVideoChip(PARA_CHIP2_ONLY); //speed up process
		}
		else
			SelectVideoChip(0); //select all chips*/
		pwFpga[SET_SHADOW_MODE] = nShadowMode;
		goto DecipherOK;
	}
	if (strncmp(szCommand, "SHUTDOWN", 8) == 0)
	{
		if (SysParam.bScanning != OP_IDLE) //stop scanning process
		{
			bScanAbort = 1;
			OSTimeDly(TICKS_PER_SECOND);
		}
		//close all gate valves, BOARD_CPU
		TxString(nPrevConn, "CLOSE ALL GATE VALVES...\n");
		//SetGateValve(0, VALVE_CLOSE);	//close GV0
		nVacOff = 1; //go to VAC_ST_STANDBY or VAC_ST_AIR
		//StopTurbo(NV_Settings.nEnableVent);
		//SetTurboPump(0); 	//turn off turbo pump power
		//SetScrollPump(0);	//turn off scroll pump power
		//SetGateValve(1, VALVE_CLOSE);	//close GV1
		//
		//turn off HV module, BOARD_CPU
		TxString(nPrevConn, "TRUN OFF HV MODULE...\n");
		SetHVSwitch(0);	//DOUT15, HV_ON
		SetHVFilament(0);
		SetHVBias(0);
		SetHVAccelerator(0);
		//
		SetHVPower(0);	//DOUT4
		SetGVPower(0); 	//DOUT5
		SetVOPPower(0);	//DOUT8
		//turn off coil current
		TxString(nPrevConn, "TRUN OFF COIL CURRENT...\n");
		SetDeflectorCurrent(ISEN_CH_DEFX, 0);
		SetDeflectorCurrent(ISEN_CH_DEFY, 0);
		SetCoilCurrent(ISEN_CH_STIGX, 0);
		SetCoilCurrent(ISEN_CH_STIGY, 0);
		SetCoilCurrent(ISEN_CH_AL0, 0);
		SetCoilCurrent(ISEN_CH_AL1, 0);
		SetCoilCurrent(ISEN_CH_AL2, 0);
		SetCoilCurrent(ISEN_CH_AL3, 0);
		SetObjOn(0); //turn off obj relay
		//
		goto DecipherOK;
	}
#define CMD_SN	"SN:" //serial number
#define LEN_SN	(sizeof(CMD_SN)-1)
	if (strncmp(szCommand, CMD_SN, LEN_SN) == 0) {	//SN:<CH>:<FV>
		if (IsCommandEnabled(nType) != SUCCESS) goto DecipherRet;
		nV1 = atoi(&szCommand[LEN_SN]);
		if (nV1 == 0) { //SN:0:0
			sprintf(szBuffer, "SN=%s\r\n", NV_Settings.szSN);
			TxString(nType, szBuffer);
		}
		else if (nV1 == 1) { //SN:1:1234567890AB
			p = &szCommand[LEN_SN];
			p += 2;
			memmove(NV_Settings.szSN, p, 12);
			p[12] = 0;
		}
		else goto DecipherFail;
		goto DecipherOK;
	}
#define CMD_SIMUVAC	 "SIMUVAC:"
#define LEN_SIMUVAC	 (sizeof(CMD_SIMUVAC) - 1)
	if (strncmp(szCommand, CMD_SIMUVAC, LEN_SIMUVAC) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SIMUVAC], "%d %f", &nV1, &fV1);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SIMUVAC_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (nV1 < 0 || nV1 > 5) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SIMUVAC_INV_PARAM1_VAL));
			goto DecipherFail;
		}

		if (nV1 == 0) { //SIMUVAC:0:1
			SysParam.bSimuVAC = (fV1 == 1.0) ? 1 : 0;
		}
		else if (nV1 == 1) { //set turbo speed
			nSimuTPSpeed = (int)fV1;
		}
		else if ((nV1 >= 2) && (nV1 <= 4)) { //set gauge 0~2 voltage
			//vac0, vac1, vac2
			SysParam.fADC[ADC_CH_VAC0 + nV1 - 2] = fV1;
		}
		else if (nV1 == 5) { //set ion gauge voltage
			SysParam.fADC[ADC_CH_IG] = fV1;
		}
		goto DecipherOK;
	}
#define CMD_SIMU "SIMU:"
#define LEN_SIMU (sizeof(CMD_SIMU) - 1)
	if (strncmp(szCommand, CMD_SIMU, LEN_SIMU) == 0)	{
		if (szCommand[LEN_SIMU] == '?') {
			siprintf(szBuffer, "SIMU=%d\n", SysParam.bSimuMode);
			TxString(nType, szBuffer);
			goto DecipherRet;
		}
		nPN = sscanf(&szCommand[LEN_SIMU], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SIMU_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 != 0) && (nV1 != 1)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SIMU_INV_PARAM_VAL));
			goto DecipherFail;
		}
		EnableSimuMode(nV1);
		SysParam.bSimuMode = (BYTE)nV1;
		nSimuShift = 0;
		goto DecipherOK;
	}
#define CMD_SO	"SO:"	//serial output
#define LEN_SO	(sizeof(CMD_SO)-1)
	if (strncmp(szCommand, CMD_SO, LEN_SO) == 0) //UR1, UR2 output test
	{
		//ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_SO], "%d:%d", &nB, &nV1);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		//SO:<B>:<PORT>:STRING
		strcpy(szBuffer, &szCommandOrg[LEN_SO + 4]);
		if ((nV1 >= 0) && (nV1 <= 3)) { //port number
			AppendLFCR(szBuffer);
			if (nB == 0) { //RS232 output
				//port 1, SO:0:1:STRING
				RS232_WriteString(nV1, szBuffer);
			}
			else if (nB == 1) { //RS485 output
				//port 1, SO:1:1:abc,
				//port 2, SO:1:2:$1,MOVE:1
				RS485_WriteString(nV1, szBuffer);
			}
			else if (nB == 2) { //send to turbo pump, add CR
				//SO:2:1:0011001206111111017
				sprintf(szBuffer, "%s\r", &szCommandOrg[LEN_SO + 4]);
				RS485_WriteString(nV1, szBuffer);
				printf(szBuffer);
			}
			else {
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_SO_INV_PARAM1_VAL));
				goto DecipherFail;
			}
		}
		else if ((nV1 >= 4) && (nV1 <= 5)) { //PID port number
			nV1 = nV1 - 4;
			AppendLFCR(szBuffer); //append LF
			if (nB == 0) { //RS232 output
				//port 0, SO:0:4:<STRING>
				SPI_WriteUART(nV1, szBuffer);
			}
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_SO_INV_PARAM1_VAL));
				goto DecipherFail;
			}
		}
		else{
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SO_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		goto DecipherRet;
	}
#define CMD_SOE	"SOE:"	//serial output
#define LEN_SOE	(sizeof(CMD_SOE)-1)
	if (strncmp(szCommand, CMD_SOE, LEN_SOE) == 0) //UR1, UR2 output test
	{
		nPN = sscanf(&szCommand[LEN_SOE], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SOE_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 != 0) && (nV1 != 1)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SOE_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bSerDebug = (BYTE)nV1; //(nV1 == 1) ? 1 : 0;
		ClearSerialBuffer();
		goto DecipherRet;
	}
#define CMD_START "START" //obsolete command
#define LEN_START (sizeof(CMD_START) - 1)
	if (strncmp(szCommand, CMD_START, LEN_START) == 0)	{
		if (IsInterlockSafe(ACT_SCROLL_ON) == FALSE) {
			nErrCode = ERR_INTLK_FAIL;
			goto DecipherFail;
		}
		nVacOn = 1; //go to VAC_ST_READY
		SetVOPPower(1);
		OSTimeDly(4);
		SetHVSwitch(0);	//DOUT15, HV_ON, on/off relay
		SetHVFilament(HVF_DEFAULT);
		SetHVBias(HVB_DEFAULT);
		SetHVAccelerator(HVA_DEFAULT);
		//
		OSTimeDly(1);
		SetHVPower(1);	//DOUT14, HV power
		nV = ObjI2V(OBJI_DEFAULT, &fV1, &SysParam.fObjAdjAngle); //current(fV1) and rotation angle(fV2)
		if (nV != SUCCESS) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_START_INV_PARAM_VAL));
			goto DecipherFail;
		}
		SetObjOn(1); //set focus current
		SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fV1);
		SetVoltage(BOARD_IO, DAC_CH_OBJ_F, 0);
		SetRotationAngle(SysParam.fRotateAngle);
		goto DecipherOK;
	}
#define CMD_SYSCLK "SYSCLK:"
#define LEN_SYSCLK (sizeof(CMD_SYSCLK) - 1)
	if (strncmp(szCommand, CMD_SYSCLK, LEN_SYSCLK) == 0)
	{
		nPN = sscanf(&szCommand[LEN_SYSCLK], "%d", &nNX);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SYSCLK_INV_PARAM_NUM));
			goto DecipherFail;
		}

		if (nNX == 20) { //20 MHz
			//pwFpga[SET_1US_CLK] = 9;
			NV_Settings.nSysClk = 20;
		}
		else if (nNX == 40) {
			//pwFpga[SET_1US_CLK] = 19;
			NV_Settings.nSysClk = 40;
		}
		else{
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_SYSCLK_INV_PARAM_VAL));
			goto DecipherFail;
		}
		Set1USClkNum();
		goto DecipherOK;
	}
#define CMD_TEST	"TEST:"
#define LEN_TEST	(sizeof(CMD_TEST)-1)
	if (strncmp(szCommand, CMD_TEST, LEN_TEST) == 0)
	{
		if (szCommand[LEN_TEST] == '?') {
			printf("2 deflector\n");
			printf("3 stigmator\n");
			printf("4 objective\n");
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_TEST], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TEST_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 != 0) && (nV1 != 1)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TEST_INV_PARAM_VAL));
			goto DecipherFail;
		}
		bTestMode = (BYTE)nV1;
		nConnTest = nConnType;
		goto DecipherOK;
	}
#define CMD_TEMPLIM		"TEMPLIM:"	//set temperature limit
#define LEN_TEMPLIM		(sizeof(CMD_TEMPLIM)-1)
	if (strncmp(szCommand, CMD_TEMPLIM, LEN_TEMPLIM) == 0) //set brightness, BOARD_SCAN
	{	//nV1: sensor index, 0:case,1:heat sink 1,2:TP_START,3:heat sink 2,4:TP_RUN
		//nV2: high low index, 1:high limit, 0:low limit
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_TEMPLIM], "%d %d %f", &nV1, &nV2, &fV); //percentage
		if (nPN != 3) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TEMPLIM_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 < 0) || (nV1 >= TEMP_SENSOR_NUM_MAX)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TEMPLIM_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if ((nV2 != 0) && (nV2 != 1)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TEMPLIM_INV_PARAM2_VAL));
			goto DecipherFail;
		}
//		if ((nPN != 3) || (nV1 >= TEMP_SENSOR_NUM_MAX) || (nV2 > 1)) {
//			nErrCode = ERR_OVER_RANGE;
//			goto DecipherFail;
//		}
		if (CheckTempSetLimit(nV1, fV) == 0) { //out of limit
			//nErrCode = ERR_OVER_RANGE;
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TEMPLIM_INV_PARAM3_VAL));
			goto DecipherFail;
		}
		if (nV2 == 1) //high limit, TEMPLIM:<NV1>:<NV2>:<FV>
			NV_Settings.fTempHiLimit[nV1] = fV;
		else if (nV2 == 0) //low limit
			NV_Settings.fTempLoLimit[nV1] = fV;
		goto DecipherOK;
	}
#define CMD_TURBO "TURBO:"
#define LEN_TURBO (sizeof(CMD_TURBO) - 1)
	if (strncmp(szCommand, CMD_TURBO, LEN_TURBO) == 0)
	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_TURBO], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TURBO_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 < 0) || (nV1 > 4)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TURBO_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if (nV1 == 1) {
			if (IsInterlockSafe(ACT_TURBO_ON) == SUCCESS) {//check VAC1
				//StartTurbo(); //TURBO:1:0
				PushTurboCommand(TP_CMD_START);
			}
			else {
				nErrCode = ERR_INTLK_FAIL;
				goto DecipherFail;
			}
		}
		else if (nV1 == 0) {//TURBO:0:1(VENT), TURBO:0:0(No VENT)
			//StopTurbo(nV2); //nV2=1,Venting;nV2=0,No Venting
			if (nV2 == 1) //OmarBug
				PushTurboCommand(TP_CMD_STOP_VENT);
			else if (nV2 == 0)
				PushTurboCommand(TP_CMD_STOP_NO_VENT);
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_TURBO_INV_PARAM2_VAL));
				goto DecipherFail;
			}
		}
		else if (nV1 == 2) //TURBO:2:0
			GetTurboRotationSpeed(1);
		else if (nV1 == 3) //TURBO:3:0
			GetTurboMotorTemp(1);
		else if (nV1 == 4) { //TURBO:4:0
			GetTurboErrorMsg(szBuffer);
			printf("%s\n", szBuffer);
		}
		goto DecipherOK;
	}
#define CMD_TURBOSP "TURBOSP:"
#define LEN_TURBOSP (sizeof(CMD_TURBOSP) - 1)
	if (strncmp(szCommand, CMD_TURBOSP, LEN_TURBOSP) == 0)
	{
		if (szCommand[LEN_TURBOSP] == '?') {
			printf("0:%4d  custom ready speed\n", NV_Settings.nTurboReadySpeed[0]);
			printf("1:%4d  Pfeiffer ready speed\n", NV_Settings.nTurboReadySpeed[1]);
			printf("2:%4d  Agilent ready speed\n", NV_Settings.nTurboReadySpeed[2]);
			printf("3:%4d  custom low speed\n", NV_Settings.nTurboLowSpeed[0]);
			printf("4:%4d  Pfeiffer low speed\n", NV_Settings.nTurboLowSpeed[1]);
			printf("5:%4d  Agilent low speed\n", NV_Settings.nTurboLowSpeed[2]);
			printf("6:%4d  turbo brake speed\n", (int)NV_Settings.sTurboBrakeSpeed);
			printf("TURBOT=%d (0:Cu,1:Pf,2:Ag)\n", NV_Settings.nTurboType);
			printf("turbo ready speed = %d\n", nTurboReadySpeed);
			printf("turbo low speed = %d\n", nTurboLowSpeed);
			printf("current turbo speed = %d\n", nTurboRotationSpeed);
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_TURBOSP], "%d %d", &nV1, &nV2);
		//nV1 = 0,1,2
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TURBOSP_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 < 0) || (nV1 > 6)) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TURBOSP_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if (nV1 < 3) { //0,1,2
			NV_Settings.nTurboReadySpeed[nV1] = nV2;
			nTurboReadySpeed = nV2;
		}
		else if (nV1 == 6) {
			NV_Settings.sTurboBrakeSpeed = nV2;
		}
		else { //3,4,5
			nV1 -= 3;
			NV_Settings.nTurboLowSpeed[nV1] = nV2;
			nTurboLowSpeed = nV2;
		}
		goto DecipherOK;
	}
#define CMD_TURBOT "TURBOT:"
#define LEN_TURBOT (sizeof(CMD_TURBOT) - 1)
	if (strncmp(szCommand, CMD_TURBOT, LEN_TURBOT) == 0) {	//nV1 = 0,1,2
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_TURBOT], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TURBOT_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 < 0) || (nV1 > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_TURBOT_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.nTurboType = nV1;
		goto DecipherOK;
	}
#define CMD_UP2DN "UP2DN:"
#define LEN_UP2DN (sizeof(CMD_UP2DN) - 1)
	if (strncmp(szCommand, CMD_UP2DN, LEN_UP2DN) == 0)	{
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_UP2DN], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_UP2DN_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 != 0) && (nV1 != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_UP2DN_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bUP2DN = nV1;
		goto DecipherOK;
	}
#define CMD_UDPPORT		"UDPPORT:"
#define LEN_UDPPORT		(sizeof(CMD_UDPPORT)-1)
	if (strncmp(szCommand, CMD_UDPPORT, LEN_UDPPORT) == 0) {	//UDPPORT:0
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_UDPPORT], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_UDPPORT_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 != 0) && (nV1 != 1)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_UDPPORT_INV_PARAM_VAL));
			goto DecipherFail;
		}
		NV_Settings.bUDPPort = (BYTE)nV1;
		goto DecipherOK;
	}
#define CMD_UUID "UUID"
#define LEN_UUID (sizeof(CMD_UUID) - 1)
	if (strncmp(szCommand, CMD_UUID, LEN_UUID) == 0) {
		SPI_ReadUUID(bUUID);
		szBuffer[0] = 0;
		for (i = 0 ; i < UUID_NUM; i++) {
			sprintf(szValue, "%02X-", bUUID[i]);
			strcat(szBuffer, szValue);
		}
		strcat(szBuffer, "\r\n");
		TxString(nType, szBuffer);
		goto DecipherOK;
	}
#define CMD_VACERR "VACERR:"
#define LEN_VACERR (sizeof(CMD_VACERR) - 1)
	if (strncmp(szCommand, CMD_VACERR, LEN_VACERR) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_VACERR], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VACERR_INV_PARAM_NUM));
			goto DecipherFail;
		}
		nVacErrToleranceNum[0] = nV1; //scanning=0, different error tolerance
		nVacErrToleranceNum[1] = nV2; //scanning=1, different error tolerance
		goto DecipherOK;
	}
#define CMD_VACOK "VACOK:"
#define LEN_VACOK (sizeof(CMD_VACOK) - 1)
	if (strncmp(szCommand, CMD_VACOK, LEN_VACOK) == 0) {	//VACOK:0:1
		if (szCommand[LEN_VACOK] == '?') {
			printf("NDX=0,1,2 pirani gauge, 3:IG\r\n");
			printf("VACOK:<NDX>:<STATE>\r\n");
			printf("STATE= 2, NONE\r\n"); 		//VAC_NONE
			printf("STATE= 3, BROKEN\r\n"); 	//VAC_BROKEN
			printf("STATE= 4, AIR\r\n");		//VAC_AIR
			printf("STATE= 8, LO\r\n");			//VAC_LO
			printf("STATE=16, HI\r\n");			//VAC_HI
			printf("STATE=32, MH\r\n");
			printf("STATE=64, UH\r\n");
			for (i = 0; i < VAC_GAUGE_NUM_MAX; i++) {
				printf("VACOK=%d,%d\r\n", i, bVACBypassOK[i]);
			}
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_VACOK], "%d %d", &nV1, &nV2);
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VACOK_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 < 0) || (nV1 >= VAC_GAUGE_NUM_MAX)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VACOK_INV_PARAM1_VAL));
			goto DecipherFail;
		}
//		if ((nPN != 2) || (nV1 >= VAC_GAUGE_NUM_MAX)) {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
		if ((nV2 < 0) || (nV2 > 2)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VACOK_INV_PARAM2_VAL));
			goto DecipherFail;
		}
		bVACBypassOK[nV1] = nV2; //0:NOK, 1:OK, 2:NONE
		goto DecipherOK;
	}
#define CMD_VACON "VACON:"
#define LEN_VACON (sizeof(CMD_VACON) - 1)
	if (strncmp(szCommand, CMD_VACON, LEN_VACON) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_VACON], "%d", &nV1);
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VACON_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 < 0) || (nV1 > 3)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VACON_INV_PARAM_VAL));
			goto DecipherFail;
		}
		if (nV1 == 0) { //VACON:0
			nVacOff = 1;
			nVacOffSent = 1; //STANDBY or AIR change during AIR_WAIT process
			nVacOn = 0;
			nGotoAir = 0;
			lVacTotalTime = 0;
		}
		else if (nV1 == 1) { //VACON:1
			if (IsInterlockSafe(ACT_SCROLL_ON) == SUCCESS) {
				nVacOn = 1;
				nVacOff = 0;
				lVacTotalTime = 0;
			}
			else {
				printf("VACON:1 INTLK FAIL\r\n");
				nErrCode = ERR_INTLK_FAIL;
				goto DecipherFail;
			}
		}
#if ENABLE_IG == 1
		else if (nV1 == 2) { //VACON:2
			nVacOff = 2; //STANDBY0 ->STANDBY_WAIT -> STANDBY1(VENT)
		}
#endif
		else if (nV1 == 3) { //ERROR2 -> INT_LO_WAIT
			nVacOff = 1;
			nVacOn = 0;
			nGotoAir = 1;
			lVacTotalTime = 0;
		}
		ResetVACCounter();
#if GAUGE_POWER_CTRL == 1
		ResetVacPower();
#endif
		goto DecipherOK;
	}
#define CMD_VACUUM 		"VACUUM:"
#define LEN_VACUUM 		(sizeof(CMD_VACUUM) - 1)
	if (strncmp(szCommand, CMD_VACUUM, LEN_VACUUM) == 0) { //query vacuum state
		if (szCommand[LEN_VACUUM] == '?') { //VACUUM:?
			szBuffer[0] = 0;
			for (i = 0; i < VAC_GAUGE_NUM_MAX; i++) {
				sprintf(szValue, "GST=%d,%d,\r\n", i, SysParam.sGaugeStatus[i]);
				strcat(szBuffer, szValue);
			}
			TxString(nType, szBuffer);
			sprintf(szBuffer, "TPGV timeout = %d s\n", SysParam.sTPGVTimeout);
			TxString(nType, szBuffer);
			sprintf(szBuffer, "VAC timeout = %d min\n", SysParam.sVacTimeout);
			TxString(nType, szBuffer);
			sprintf(szBuffer, "IG timeout = %d min\n", SysParam.sIGTimeout);
			TxString(nType, szBuffer);
			ShowVacuumStatus(nType);
			PrintVacStLog();
			goto DecipherRet;
		}
		goto DecipherFail;
	}
#define CMD_VACPARA		"VACPARA:"
#define LEN_VACPARA		(sizeof(CMD_VACPARA)-1)
	if (strncmp(szCommand, CMD_VACPARA, LEN_VACPARA) == 0)	{
		if (IsCommandEnabled(nType) != SUCCESS) goto DecipherRet;
		if (szCommand[LEN_VACPARA] == '?') {
			printf("VACPARA:0:<NV> LO_WAIT_TIMEOUT=%d s\n", NV_Settings.sLowWaitTimeout);
			printf("VACPARA:1:<NV> INT_HI_WAIT_TIMEOUT=%ld s\n", NV_Settings.lHighWaitTimeout);
			printf("VACPARA:2:<NV> UH_WAIT_TIMEOUT=%ld s\n", NV_Settings.lUHighWaitTimeout);
			printf("VACPARA:3:<FV> LOW_CHANGE_RATE=%.4f V/s\n", NV_Settings.fLowChangeRate);
			printf("VACPARA:4:<FV> HIGH_CHANGE_RATE=%.4f V/s\n", NV_Settings.fHighChangeRate);
			printf("VACPARA:5:<FV> IG,LOTH=%.3f V\n", NV_Settings.fIGLOTh); //larger
			printf("VACPARA:6:<FV> IG,HITH=%.3f V\n", NV_Settings.fIGHITh);
			printf("VACPARA:7:<FV> IG,MHTH=%.3f V\n", NV_Settings.fIGMHTh);
			printf("VACPARA:8:<FV> IG,UHTH=%.3f V\n", NV_Settings.fIGUHTh);
			printf("VACPARA:9:<FV> PG,LOTH=%.3f V\n", NV_Settings.fPGLOTh);
			printf("VACPARA:10:<FV> PG,HITH=%.3f V\n", NV_Settings.fPGHITh);
			printf("VACPARA:11:<NV> AIR_WAIT_TIMEOUT=%d s\n", NV_Settings.sAirWaitTimeout);
			printf("VACPARA:12:<NV> MH_WAIT_TIMEOUT=%d s\n", NV_Settings.sMHighWaitTimeout);
			printf("VACPARA:13:<NV> IG_WAIT_TIMEOUT=%d s\n", NV_Settings.sIGWaitTimeout);
			printf("VACPARA:14:<NV> VENT_WAIT_TIMEOUT=%d s\n", NV_Settings.sVentWaitTimeout);
			printf("VACPARA:15:<NV> STANDBY_WAIT_TIMEOUT=%d s\n", NV_Settings.sStandbyWaitTimeout);
			printf("VACPARA:16:<FV> IP_LEAK0=%.3f V\n", NV_Settings.fIPLeakCheckV[0]);
			printf("VACPARA:17:<FV> IP_LEAK1=%.3f V\n", NV_Settings.fIPLeakCheckV[1]);
			printf("VACPARA:18:<FV> IP_LEAK2=%.3f V\n", NV_Settings.fIPLeakCheckV[2]);
			printf("VACPARA:19:<FV> IP_LEAK3=%.3f V\n", NV_Settings.fIPLeakCheckV[3]);
			printf("VACPARA:20:<NV> TPGV_TIMEOUT=%d s\n", NV_Settings.sTPGVTimeout);
			printf("VACPARA:21:<NV> HI_WAIT_TIMEOUT=%d s\n", NV_Settings.sHighWaitTimeout);
			printf("VACPARA:22:<NV> IG,INT2TH=%.3f V\n", NV_Settings.fIGINT2Th);
			PrintVacTimeout();
			goto DecipherRet;
		}
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_VACPARA], "%d %f", &nV1, &fV1); //fV is percentage
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VACPARA_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if ((nV1 < 0) || (nV1 > 22)){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VACPARA_INV_PARAM1_VAL));
			goto DecipherFail;
		}
		if (nV1 == 0)
			NV_Settings.sLowWaitTimeout = (short int)fV1;
		else if (nV1 == 1)
			NV_Settings.lHighWaitTimeout = (long)fV1;
		else if (nV1 == 2)
			NV_Settings.lUHighWaitTimeout = (long)fV1;
		else if (nV1 == 3)
			NV_Settings.fLowChangeRate = fV1;
		else if (nV1 == 4)
			NV_Settings.fHighChangeRate = fV1;
		else if (nV1 == 5) {
			//if (fV1 < 2.0) fV1 = TorrToVolt(fV1);
			NV_Settings.fIGLOTh = fV1; //IG, volt
		}
		else if (nV1 == 6) {
			if (fV1 < 1e-1) fV1 = TorrToVolt(fV1);
			NV_Settings.fIGHITh = fV1; //IG
		}
		else if (nV1 == 7) {
			if (fV1 < 1e-1) fV1 = TorrToVolt(fV1);
			NV_Settings.fIGMHTh = fV1; //IG
		}
		else if (nV1 == 8) {
			if (fV1 < 1e-2) fV1 = TorrToVolt(fV1);
			NV_Settings.fIGUHTh = fV1; //IG
		}
		else if (nV1 == 9)
			NV_Settings.fPGLOTh = fV1; //PG
		else if (nV1 == 10)
			NV_Settings.fPGHITh = fV1; //PG
		else if (nV1 == 11)
			NV_Settings.sAirWaitTimeout = (short int)fV1;
		else if (nV1 == 12)
			NV_Settings.sMHighWaitTimeout = (short int)fV1;
		else if (nV1 == 13)
			NV_Settings.sIGWaitTimeout = (short int)fV1;
		else if (nV1 == 14)
			NV_Settings.sVentWaitTimeout = (short int)fV1;
		else if (nV1 == 15)
			NV_Settings.sStandbyWaitTimeout = (short int)fV1;
		else if ((nV1 >= 16) && (nV1 <= 19))
			NV_Settings.fIPLeakCheckV[nV1 - 16] = fV1;
		else if (nV1 == 20)
			NV_Settings.sTPGVTimeout = (short int)fV1;
		else if (nV1 == 21)
			NV_Settings.sHighWaitTimeout = (short int)fV1;
		else if (nV1 == 22) {
			if (fV1 < 10.0) fV1 = TorrToVolt(fV1);
			NV_Settings.fIGINT2Th = fV1; //IG
		}
		goto DecipherOK;
	}
//
#define CMD_VIEWW	"VIEWW:" 			//set screen view width, unit mm
#define LEN_VIEWW	(sizeof(CMD_VIEWW)-1)
	if (strncmp(szCommand, CMD_VIEWW, LEN_VIEWW) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_VIEWW], "%f", &fV); //fV unit: mm
		if (nPN != 1) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VIEWW_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (fV < 10.0){
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_VIEWW_INV_PARAM_VAL));
			goto DecipherFail;
		}
//		if ((nPN != 1) || (fV < 10.0)) {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
		NV_Settings.fImageWidthOnScreen = fV; //image width on screen
		CalculateMagMin();
		goto DecipherOK;
	}
#define CMD_WAITEOC		"WAITEOC:"
#define LEN_WAITEOC		(sizeof(CMD_WAITEOC)-1)
	if (strncmp(szCommand, CMD_WAITEOC, LEN_WAITEOC) == 0) {
		ReplaceChar(szCommand, ':', ' ');
		nPN = sscanf(&szCommand[LEN_WAITEOC], "%d %d", &nV1, &nV2); //percentage
		if (nPN != 2) {
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_WAITEOC_INV_PARAM_NUM));
			goto DecipherFail;
		}
		if (nV1 == 0) { //WAITEOC:0:4
			wV = (WORD)nV2 & 0x00FF; //(0~255)
			NV_Settings.wWaitEOC &= 0xFF00;
			NV_Settings.wWaitEOC += wV;
			pwFpga[WAIT_EOC] = NV_Settings.wWaitEOC;
		}
		else if (nV1 == 1) { //enable wait 2nd eoc
			if (nV2 == 1) //WAITEOC:1:1, enable
				NV_Settings.wWaitEOC = NV_Settings.wWaitEOC | 0x8000; //wait_2nd_eoc = 1
			else if (nV2 == 0)//WAITEOC:1:0, disable
				NV_Settings.wWaitEOC = NV_Settings.wWaitEOC & (~0x8000); //wait_2nd_eoc = 0
			else{
				sprintf(szValue, ErrorMsg::GetErrorText(ERR_WAITEOC_INV_PARAM2_VAL));
				goto DecipherFail;
			}
			pwFpga[WAIT_EOC] = NV_Settings.wWaitEOC;
		}
		else{
			sprintf(szValue, ErrorMsg::GetErrorText(ERR_WAITEOC_INV_PARAM1_VAL));
			goto DecipherFail;
		}
//		if (nPN != 2) {
//			nErrCode = ERR_INV_PARAM;
//			goto DecipherFail;
//		}
//		if (nV1 == 0) { //WAITEOC:0:4
//			wV = (WORD)nV2 & 0x00FF; //(0~255)
//			NV_Settings.wWaitEOC &= 0xFF00;
//			NV_Settings.wWaitEOC += wV;
//			pwFpga[WAIT_EOC] = NV_Settings.wWaitEOC;
//		}
//		else if (nV1 == 1) { //enable wait 2nd eoc
//			if (nV2 == 1) //WAITEOC:1:1, enable
//				NV_Settings.wWaitEOC = NV_Settings.wWaitEOC | 0x8000; //wait_2nd_eoc = 1
//			else //WAITEOC:1:0, disable
//				NV_Settings.wWaitEOC = NV_Settings.wWaitEOC & (~0x8000); //wait_2nd_eoc = 0
//			pwFpga[WAIT_EOC] = NV_Settings.wWaitEOC;
//		}
		goto DecipherOK;
	}
	//invalid command
	sprintf(szBuffer, "ERROR:INV_CMD,C:%s\n", szCommand);
	if ((bReturn == 1) || (bDebug == 1))
		TxString(nType, szBuffer);
	SaveErrorMessage(szBuffer, 0);
	nReentry = 0;
	return ERROR_FAIL;	//FAIL
DecipherFail:
	//GetErrorMessage(nErrCode, szValue);  //Omar
	sprintf(szBuffer, "ERROR:%s,%s\n", szCommandOrg, szValue);
	if ((bReturn == 1) || (bDebug == 1))
		TxString(nType, szBuffer);
	SaveErrorMessage(szBuffer, 0);
	nReentry = 0;
	return ERROR_FAIL;	//FAIL
DecipherOK:
	if ((bReturn == 1) || (bDebug == 1))
		TxString(nType, "OK\n");
	nReentry = 0;
	return SUCCESS;	//SUCCESS
DecipherRet:
	nReentry = 0;
	return SUCCESS;
}

void DumpRAM(DWORD dwAddr, int nLength, int nDataType)
{
	//WORD wV;
	int i;
	WORD pwBuffer[20000];
	BYTE *pbBuffer;
	int nModulus = 10;
	int nModulus2 = 20;
	//
	if (nLength % 16 == 0) {
		nModulus = 16;
		nModulus2 = 32;
	}
	if (nLength > 1024) nLength = 1024;
	SetBaseAddr(dwAddr);
	//
	memmove((BYTE *)pwBuffer, (BYTE *)pwSram, nLength*sizeof(WORD));
	if ((nDataType & 2) != 0) { //WORD
		for (i = 0; i < nLength; i++)
		{
			if ((i % nModulus == 0) && (nModulus % 10 != 0))
				printf("0X%06lX:", dwAddr+i);
			else if ((i % nModulus == 0) && (nModulus % 10 == 0))
				printf("%08ld:", dwAddr+i);
			else if ((nModulus == 16) && (i % 16 == 8))
				printf("- ");
			printf("%04X ", pwBuffer[i]);
			if (i % nModulus == (nModulus-1))
				printf("\n");
		}
		if (i % nModulus != 0)
			printf("\n");
	}
	//
	//Intel CPUs are little-endian, while Motorola 680x0,freescale CPUs are big-endian
	if ((nDataType & 1) != 0) { //BYTE
		pbBuffer = (BYTE *)pwBuffer;
		for (i = 0; i < nLength * 2; i++)
		{
			if ((i % (nModulus2) == 0) && (nModulus % 10 != 0))
				printf("0X%06lX:", dwAddr+i);
			else if ((i % (nModulus2) == 0) && (nModulus % 10 == 0))
				printf("%08ld:", dwAddr+i);
			else if ((nModulus == 16) && (i % (nModulus2) == 16))
				printf("- ");
			printf("%02X ", pbBuffer[i]); //LSB first --> PC
			if (i % (nModulus2) == (nModulus2-1))
				printf("\n");
		}
		if (i % (nModulus2) != 0)
			printf("\n");
	}
}

void SelectVideoChip(int nSelect)
{
	WORD wV = 0;
	if (nSelect == PARA_CHIP0_ONLY) //0100_0000_0000
		wV = PARA_CHIP0_ONLY;
	else if (nSelect == PARA_CHIP1_ONLY) //1000_0000_0000
		wV = PARA_CHIP1_ONLY;
	else if (nSelect == PARA_CHIP2_ONLY) //0001_0000_0000_0000
		wV = PARA_CHIP2_ONLY;
	wParameter &= (~PARA_CHIP_SEL); //set all bits = 0
	wParameter |= wV;
	//
	pwFpga[SET_PARAMETER] = wParameter;
}
//---------------------------------------
//nType, second image
//1:s2+s3
//2:s2
//3:s3
//4:ch4
//5:ch5
//---------------------------------------
void Set2ndImageType(int nType)
{
	WORD wV;
	wV = (WORD)(nType & 0x07);
	wV = wV << 7;
	wParameter &= (~PARA_2IMG_MASK); //bit[9:7]
	wParameter |= wV;
	pwFpga[SET_PARAMETER] = wParameter;
}
//nCh=0, nShowdowMode
//nCh=1, output SEI signal histogram
void SetHistoChannel(int nCh)
{
	if (nCh == 1)
		wParameter |= PARA_HISTO_CH;
	else
		wParameter &= (~PARA_HISTO_CH); //default
	pwFpga[SET_PARAMETER] = wParameter;
}
//enable simulation mode
void EnableSimuMode(int nEnable)
{
	if (nEnable == 1)
		wParameter |= PARA_SIMU_MODE;
	else
		wParameter &= (~PARA_SIMU_MODE);
	pwFpga[SET_PARAMETER] = wParameter;
}
//
//enable power line synchronization
//nEnable = 0,1,2,3
//0: no PL modulation
//1: 1 interrupt per PL cycle
//2: 2 interrupt per PL cycle (obsolete)
//3: 4 interrupt per PL cycle (obsolete)
//
void EnablePLSync(int nEnable)
{
	WORD wV;
	if (nEnable > 3)
		return;
	wV = (WORD)nEnable & 0x0003; //phase_reg[1:0]
	wV = wV << 4; //bit4, wait_pl_reg
	if (nEnable != 0) {
		wParameter &= (~0x0030); //0011_0000
		wParameter |= wV; //phase_reg = 2'b11
	}
	else
		wParameter &= (~0x0030);
	pwFpga[SET_PARAMETER] = wParameter;
}
// enable auto focus process
void EnableAutoFocus(int nEnable)
{
	if (nEnable == 1)
		wParameter |= PARA_AUTO_FOCUS;
	else
		wParameter &= (~PARA_AUTO_FOCUS);
	pwFpga[SET_PARAMETER] = wParameter;
}

void EnableIRQ3(int nEnable)
{
	/*if (nEnable == 1) {
		wParameter |= PARA_EN_IRQ3;
	}
	else {
		wParameter &= (~PARA_EN_IRQ3);
	}
	//bEnableIRQ3 = (BYTE)nEnable;
	pwFpga[SET_PARAMETER] = wParameter;
	*/
}

#if USE_FIFO == 1
void ResetFIFO(void)
{
	wSetFIFO |= PARA_FIFO_RESET;	//bit 13
	pwFpga[SET_FIFO] = wSetFIFO;
	delay_void();
	wSetFIFO &= (~PARA_FIFO_RESET);
	pwFpga[SET_FIFO] = wSetFIFO;
}

void EnableFIFO(int nEnable)
{
	if (nEnable == 1)
		wSetFIFO |= PARA_FIFO_ENABLE; //bit 14, 0x4000
	else
		wSetFIFO &= (~PARA_FIFO_ENABLE);
	pwFpga[SET_FIFO] = wSetFIFO;
}
#endif

#define DEG_PER_RADIAN	(360/6.2831852)
// set power line parameter
void SetPLParameter(int nSet)
{
	//int i;
	//float fV, fRadian;
	//float fPhase = 0;
	WORD wPeriod = 16667;
	WORD wPLSampInterval;
	WORD wPLAddrNum = 32;
	//
	if (NV_Settings.bPLFreq == 60) wPeriod = 16667; //16.667 ms, 32*521=16672
	else wPeriod = 20000; //20 ms, 32*625=20,000, 32*626=20,032
	//
	wPLAddrNum = NV_Settings.wPLAddrNum;
	if (wPLAddrNum < 16)
		wPLAddrNum = 16;
	else if (wPLAddrNum > 128)
		wPLAddrNum = 128;
	//
	if (nSet & 0x01) { //calculate PL modulation value, sinewave
		//fV = (float)wPeriod / wPLAddrNum;
		//if ((fV - (int)fV) > 0.5)
		//	wPLSampInterval = wPeriod / wPLAddrNum + 1; //pl_interval_num
		//else
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//important trick !!! 2014.3.15
		//wPLSampleInterval* wPLAddrNum must be larger than wPeriod !!!
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		wPLSampInterval = wPeriod / wPLAddrNum + 1;
		//
		//pwFpga[SET_PL_INTERVAL] = wPLSampInterval;
		//pwFpga[SET_PL_ADDR_NUM] = wPLAddrNum;
		SetPLPeriod();
		//pwFpga[SET_PL_PERIOD] = wPeriod + 32; //32 usec
		if (bDebug == 13) {
			printf("INTERVAL=%d,ADDR_NUM=%d\r\n",
				(int)wPLSampInterval, (int)wPLAddrNum);
		}
	}
	/*else if (nSet & 0x02) { //set all values equal to middle value
		for (i = 0; i <= wPLAddrNum; i++) {
			wPLSet[i] = PL_DAC_WMID; //voltage = 0;
		}
	}*/
}

extern int nVACCounter;
void ShowVacuumStatus(int nType)
{
	char szValue[128] = "";
	char szBuffer[256] = "";
	int nV;
	float fV;
	int nUpdate = 0;
	//
	GetVacuumStateName(SysParam.bVacuumState, szValue);
	sprintf(szBuffer, "----------------\nVAC_STATE=%d,%s\n", SysParam.bVacuumState, szValue);
	TxString(nType, szBuffer);
	sprintf(szBuffer, "VAC counter = %d s\n", nVACCounter);
	TxString(nType, szBuffer);
	szBuffer[0] = 0;
	fV = GetPiraniGauge(0);
	sprintf(szValue, "PG0=%.3f volts\n", fV);
	strcat(szBuffer, szValue);
	fV = GetPiraniGauge(1);
	sprintf(szValue, "PG1=%.3f volts\n", fV);
	strcat(szBuffer, szValue);
	nV = GetTurboMotorTemp(nUpdate); //update=0
	sprintf(szValue, "TPTMP=%d deg C\n", nV);
	strcat(szBuffer, szValue);
	GetTurboRotationSpeed(nUpdate);
	sprintf(szValue, "TPSP=%d Hz\n", nTurboRotationSpeed);
	strcat(szBuffer, szValue);
	TxString(nType, szBuffer);
#if ENABLE_IG == 1
	fV = GetIonGauge();
	sprintf(szBuffer, "IG=%.3f volts\n", fV);
	TxString(nType, szBuffer);
#endif
}
