#include <predef.h>
#include <stdio.h>
#include <stdlib.h> //atoi, rand, srand
#include <string.h>
#include <startnet.h>
#include <nbtime.h>
#include <math.h>
#include <serial.h>
//
//#include <autoupdate.h>
//#include <startnet.h>
//#include <dhcpclient.h>
//#include <tcp.h>
//#include <udp.h>
//#include <serial.h>
//#include <NetworkDebug.h>
#include "main.h"
#include "cpu_cmd.h"
#include "data_struct.h"
#include "rtc.h"
#include "function.h"
//#include <ucos.h>
//
#define TEMP_TABLE_NUM		19
#define OBJ_TURN_NUM		900
//
extern float fObjPercent;
extern int nScanPixelNumX;
extern int nScanPixelNumY;
extern int nScanPixelNumXDV;
extern int nScanPixelNumYDV;
extern int nSimuShift;
extern WORD wBrightness;	//used in simulation
extern WORD wContrast;		//0,1,2, used in simulation
extern volatile PWORD pwSram;	//nCS1
extern volatile PWORD pwFpga;	//nCS2
extern BYTE bScanAbort;
extern float fRatioDefSenR;
extern int nCoilSwitch;
extern BYTE bEnableGradientByFW;
extern BYTE bDebug;
extern BYTE bHWUSDelay; //hardware usec delay
extern int nObjTableNdx;
//x scan time in usec
float fXScanTime = 10000.0;
//PL line period in usec
float fPLPeriod = 16666.7; //us
extern BYTE bPLSync;
extern BYTE bZoomCtrl;
extern BYTE bEnablePLModX;
extern BYTE bEnablePLModY;
extern SYS_PARAM_3 SysParam;
extern SYS_PARAM_2 SysParam2;
//
extern BYTE bEnableEDS;
extern BYTE bIOFPGAMajorVersion;
extern BYTE bIOFPGAMinorVersion;
//
//0: debug, command
//1: turbo pump
//2: RS485, IP_PWR_CTRL and custom HV control
//3: reserved high vacuum gauge
extern int fdSerial[4];
/*
#include "cardtype.h"
#include <effs_fat/fat.h>
#ifdef USE_MMC
#include <effs_fat/mmc_mcf.h>
#endif
#ifdef USE_CFC
#include <effs_fat/cfc_mcf.h>
#endif
#include "FileSystemUtils.h"
*/
//
// This key can be updated if you modify your structure. For example,
// you may need to do something special if you load a new version
// of s/w in a device that has the old structure values.
#define VERIFY_KEY (0x48666051)  // NV Settings key code
NV_SETTINGS NV_Settings;
DWORD dwGradientSum = 0;
//int nEdgeDiff = 32;
int nGradientXStart, nGradientXEnd;
int nCommandInQueue = 0;
int nCmdQueType[CMD_IN_QUEUE_NUM];
char szCommandInQueue[CMD_IN_QUEUE_NUM][CMD_SIZE];
//float fMaxXAtDEFX0 = 3, fMaxYAtDEFX0 = 3;
//float fRangeMax[2] = {3, 3};
//float fRangeMin;
//
void delay_void(void)
{
	// do nothing
	// shortest time delay, about 1.5 usec
	asm("nop"); //300 ns
	asm("nop");
/*
#if HW_US_DELAY == 1
	WORD wV;
	int i = 0;
	pwFpga[DELAY_NUM] = FPGA_SYSCLK;
	pwFpga[DELAY_START] = 1;
	pwFpga[DELAY_START] = 0;
	do {
		wV = pwFpga[GET_STATUS] & 0x0004; //delay timeout
		i = i + 1;
		if (i > 1000) break;
	}
	while (wV == 0);
#endif
*/
}
//------------------------------------------------
// delay nus microsecond
//------------------------------------------------
void delay_us(int nus)
{
	WORD wV;
	int i, j, jmax;
	//
	if (bHWUSDelay == 1) {
		pwFpga[DELAY_START] = 1; //set delay_us_timeout=0
		//pwFpga[DELAY_START] = 1; //set delay_us_timeout=0
		pwFpga[DELAY_NUM] = nus; //nus max = 65535
		delay_void(); //must be longer than 1 us
		delay_void();
		//delay_void();
		pwFpga[DELAY_START] = 0;
		jmax = 8;
		while (1) {
			wV = pwFpga[GET_STATUS] & 0x0004; //wait until delay_us_timeout=1
			if (wV != 0) break;
			nus--;
			for (j = 0; j < jmax; j++)
				asm("nop");
			if (nus < 0) break; //timeout
		}
	}
	else { //delay by software
		jmax = 16; //(nus < 10) ? 10 : (nus < 20) ? 12 : 14;
		for (i = 0; i < nus; i++) {
			for (j = 0; j < jmax; j++)
				asm("nop");
		}
	}
}

/*DWORD delay_100us(int nTimes)
{
	int i;
	DWORD dwV;
	dwV = 0;
	for (i = 0; i < nTimes; i++)
		dwV = dwV + (DWORD)((float)0x100 * (float)rand()/RAND_MAX); //100 usec
	return dwV;
}*/
/*-------------------------------------------------------------------
 Convert IP address to a string
 -------------------------------------------------------------------*/
void IPtoString(IPADDR ia, char *s)
{
	PBYTE ipb = (PBYTE)&ia;
	siprintf(s, "%d.%d.%d.%d",(int)ipb[0],(int)ipb[1],(int)ipb[2],(int)ipb[3]);
}
// change cOld to cNew
void ReplaceChar(char *szCommand, char cOld, char cNew)
{
	int i, nLen;
	nLen = strlen(szCommand);
	for (i = 0; i < nLen; i++)
	{
		if (szCommand[i] == cOld) szCommand[i] = cNew;
	}
}

void PackChar(char *szBuffer, char *szPack)
{
	char *p1, *p2;
	p2 = p1 = szBuffer;
	while (*p1 != 0) {
		if (strchr(szPack, *p1) != NULL) {	//if *p1 is in szPack
			p1++;
			continue;
		}
		*p2 = *p1;
		p2++;
		p1++;
	}
	*p2 = 0;
}

/* Change wV to "0000_0000_0000_0000_" string */
void WordToBinaryString(WORD wV, char *szBuffer)
{
	int i;
	WORD wMask = 0x8000;
	szBuffer[0] = 0;
	for (i = 0; i < 16; i++)
	{
		if ((wV & wMask) != 0)
			strcat(szBuffer, "1");
		else
			strcat(szBuffer, "0");
		if ((i % 4) == 3)
			strcat(szBuffer, "_");
		wMask = wMask >> 1;
	}
}

void GetSystemTime(void)
{
	time_t t;
	/*struct tm bts;
	if ( RTCGetTime( bts ) == 0 )
	{
		t = mktime( &bts );
		iprintf( "RTC Time = %s\n", ctime( &t ) );
	}
	else
	{ */
		t = time(NULL);
		//printf( "Failed to get RTC time\n" );
		iprintf( "SYS Time = %s\n", ctime( &t ) );
	//}
}

void SetSystemTime(int nYear, int nMon, int nDay, int nHour, int nMin, int nSec)
{
	struct tm bts;
	time_t t;
	bts.tm_isdst = 0;
	bts.tm_year = nYear - 1900;
	bts.tm_mon = ( BYTE ) nMon - 1;
	bts.tm_mday = ( BYTE ) nDay;
	bts.tm_hour = nHour;
	bts.tm_min = nMin;
	bts.tm_sec = nSec;
	//
	t = mktime(&bts);
	set_time(t); //set system time

	/*if ( RTCSetTime( bts ) == 0 )
	{
		iprintf("Time is correctly set..\n");
	}*/
}

WORD AutoFocusSimuData(int nX, int nY)
{
	float fV, fR;
	WORD wV;
	int i, j;
	WORD wStepValX[PIXEL_NUM_X_MAX_PI];
	WORD wStepValY[PIXEL_NUM_Y_MAX];
	//
	//nX = nX - X_SKIP_NUM;
	if ((nX == 0) && (nY == 0))
	{
		if (fObjPercent > 0.5) fV = (1 - fObjPercent);
		else fV = fObjPercent;
		//
		for (i = 0; i < nScanPixelNumX/2; i++) {
			fR = (float)i / (nScanPixelNumX / 2); //fR = 0 ~ 1
			wStepValX[i] = VIDEO_ADC_BOTTOM + fV * (VIDEO_ADC_MAX - VIDEO_ADC_BOTTOM) * fR;
		}
		for (i = nScanPixelNumX; i >= nScanPixelNumX/2; i--) {
			j = (nScanPixelNumX - i);
			fR = (float)j / (nScanPixelNumX / 2);
			wStepValX[i] = VIDEO_ADC_BOTTOM + fV * (VIDEO_ADC_MAX - VIDEO_ADC_BOTTOM) * fR;
		}
		for (i = 0; i < nScanPixelNumY/2; i++) {
			fR = (float)i / (nScanPixelNumY / 2); //fR = 0 ~ 1
			wStepValY[i] = VIDEO_ADC_BOTTOM + fV * (VIDEO_ADC_MAX - VIDEO_ADC_BOTTOM) * fR;
		}
		for (i = nScanPixelNumY; i >= nScanPixelNumY/2; i--) {
			j = (nScanPixelNumY - i);
			fR = (float)j / (nScanPixelNumY / 2);
			wStepValY[i] = VIDEO_ADC_BOTTOM + fV * (VIDEO_ADC_MAX - VIDEO_ADC_BOTTOM) * fR;
		}
	}
	if (nX < 0) wV = 0;
	else wV = (wStepValX[nX] + wStepValY[nY])/2;
	return wV;
}
//
#define PI       3.1415926
float fRadiusX, fRadiusY, fRadius;
//
WORD GenerateCircleGridPattern(int nX, int nY)
{
    float fTheta;
    float fRadiusRatio;
    float fXR, fYR;
    //
    int nTheta;

	if (nX == 0 && nY == 0)
    {
       fRadiusX = (float)nScanPixelNumX / 2;
       fRadiusY = (float)nScanPixelNumY / 2;
       fRadius = sqrt(fRadiusX * fRadiusX + fRadiusY * fRadiusY);
    }
    fXR = (float)nX - fRadiusX;
    fYR = (float)nY - fRadiusY;
    fRadiusRatio = sqrt(fXR * fXR + fYR * fYR) / fRadius;
    fTheta = atan(fYR/fXR);
    if (fXR == 0 && fYR > 0)
       fTheta = PI / 2;
    else if (fXR == 0 && fYR < 0)
       fTheta = -PI / 2;
    else if (fXR > 0 && fYR < 0)
       fTheta += 2 * PI;
    else if (fXR < 0 && fYR < 0)
       fTheta += PI;
    else if (fXR < 0 && fYR > 0)
       fTheta += PI;
    if (fRadiusRatio < 0.1 || fRadiusRatio > 0.9)
       return 0xC00;
    fTheta = fTheta + (float)nSimuShift * 0.1;
    nTheta = (int)(fTheta * 24 / PI); //48 parts
    if (nTheta % 12 == 0)
       return 0x0;   //black
    else
       return 0xC00;
}

WORD GetGridPattern(int nShift, int nRandom, int nX, int nY)
{
	int nX1, nY1, nMXY;
	WORD wV;

	nX1 = (int)((nX + nShift)/nScanPixelNumXDV) % 2; //nX1 = 0 or 1
	nY1 = (int)(nY/nScanPixelNumYDV) % 2;
	nMXY =  nX1 + (nY1 * 2) ;
#if HIS_SHIFT == 3
	wV = (nMXY % 4 == 1) ? 0x200 :
		(nMXY % 4 == 2) ? 0x500 :
		(nMXY % 4 == 3) ? 0x800 : 0xB00;
	if (nRandom != 0)
		wV = wV + (WORD)((float)0x100 * (float)rand()/RAND_MAX);
#else
	wV = (nMXY % 4 == 1) ? 0x2000 :
		(nMXY % 4 == 2) ? 0x5000 :
		(nMXY % 4 == 3) ? 0x8000 : 0xB000;
	if (nRandom != 0)
		wV = wV + (WORD)((float)0x1000 * (float)rand()/RAND_MAX);
#endif
	//wV =  wV << wContrast; //0,1,2 (768*4=3072)
	//wV += wBrightness; //0x200 ~ 0x800
	//wV = wV & 0xFFF;	//12-bit
	return wV;
}

void ResetBRCOToDefault(void)
{
	int i;
	for (i = 0; i < BR_NUM; i++) {
		NV_Settings.fBaseBR[i] = 50;
		NV_Settings.fRangeBR[i] = 25;
	}
	for (i = 0; i < CO_NUM; i++) {
		NV_Settings.fBaseCO[i] = 0;
		NV_Settings.fRangeCO[i] = 100.0;
	}
}

void ClearNVSettings(void)
{
	int i, nB;
	for (nB = 0; nB < BOARD_NUM; nB++) {
		for (i = 0; i < DAC_CH_NUM_MAX; i++) {
			NV_Settings.fDACSlope[nB][i] = 1;
			NV_Settings.fDACOffset[nB][i] = 0;
		}
	}
	for (i = 0; i < VDAC_CH_NUM_MAX; i++) {
		NV_Settings.fScanDACSlope[i] = 1;
		NV_Settings.fScanDACOffset[i] = 0;	//X, Y direction
	}
	for (i = 0; i < ADC_CH_NUM_MAX; i++) {
		NV_Settings.fADCOffset[i] = 0;
		//NV_Settings.bADCEnable[i] = 1;
	}
	/*NV_Settings.bADCEnable[ADC_CH_AL0] = 0;
	NV_Settings.bADCEnable[ADC_CH_AL1] = 0;
	NV_Settings.bADCEnable[ADC_CH_AL2] = 0;
	NV_Settings.bADCEnable[ADC_CH_AL3] = 0;
	NV_Settings.bADCEnable[ADC_CH_RESV8] = 0;
	NV_Settings.bADCEnable[ADC_CH_RESV9] = 0;
	NV_Settings.bADCEnable[ADC_CH_RESV10] = 0;
	NV_Settings.bADCEnable[ADC_CH_RESV11] = 0;
	*/
	//
	for (i = 0; i < VADC_CH_NUM_MAX; i++) //0,1,2,3,2CH,VSUM,HDIFF,VDIFF
		NV_Settings.fScanADCOffset[i] = 0;	//0
	// V to I ratio, total ISEN_CH_NUM channels
	NV_Settings.fV2IR[0] = 10;		//deflector X, -5 ~ +5V, max 0.5A*10=5
	NV_Settings.fV2IR[1] = 10;		//deflector Y
	NV_Settings.fV2IR[2] = 10;		//stigmator X, -5 ~ +5V, max 0.3A*16.5=5
	NV_Settings.fV2IR[3] = 10;		//stigmator Y
	NV_Settings.fV2IR[4] = 8.2;		//objective, 10V <-> 1.17A
	NV_Settings.fV2IR[5] = 10;		//aligner 0, -5 ~ +5V, max 0.5A*10=5
	NV_Settings.fV2IR[6] = 10;		//aligner 1
	NV_Settings.fV2IR[7] = 10;		//aligner 2, -5 ~ +5V, max 0.3A*16.5=5
	NV_Settings.fV2IR[8] = 10;		//aligner 3
	//coil resistance, unit ohm
	//Vsense = I * Rload
	//ISEN_CH_NUM=9,index=0~8
	NV_Settings.fLoadR[0] = DEF_SEN_R0; 	//deflector X sense resistance
	NV_Settings.fLoadR[1] = DEF_SEN_R0;		//deflector Y sense resistance
	NV_Settings.fLoadR[2] = 10;		//stigmator X sense resistance
	NV_Settings.fLoadR[3] = 10;		//stigmator Y sense resistance
	NV_Settings.fLoadR[4] = 1;		//objective sense resistance
	NV_Settings.fLoadR[5] = 10;		//aligner 0
	NV_Settings.fLoadR[6] = 10;		//aligner 1
	NV_Settings.fLoadR[7] = 10;		//aligner 2
	NV_Settings.fLoadR[8] = 10;		//aligner 3
	//
	//NV_Settings.fScanScale[0] = SCAN_C2FR_1;
	//NV_Settings.fScanScale[1] = SCAN_C2FR_2;
	//NV_Settings.fScanScale[2] = SCAN_C2FR_3;
	//NV_Settings.fScanScale[3] = SCAN_C2FR_4;
	//NV_Settings.fScanScale[4] = SCAN_C2FR_5;
	//NV_Settings.fScanScale[5] = SCAN_C2FR_6;
	//
	//NV_Settings.fObjScale[0] = OBJ_C2FR_1;
	//NV_Settings.fObjScale[1] = OBJ_C2FR_2;
	//NV_Settings.fObjScale[2] = OBJ_C2FR_3;
	//NV_Settings.fObjScale[3] = OBJ_C2FR_4;
	//
	NV_Settings.fAPerMM[0][0] = A_PER_MM_X0;	//X=0.15, 1*0.15*3 = 5 V
	NV_Settings.fAPerMM[0][1] = A_PER_MM_Y0;	//X=0.121, 20.6*0.121*2 = 5 V
	NV_Settings.fAPerMM[1][0] = A_PER_MM_X1;	//X=0.15, 1*0.15*3 = 5 V
	NV_Settings.fAPerMM[1][1] = A_PER_MM_Y1;	//X=0.121, 20.6*0.121*2 = 5 V
	//NV_Settings.nXSkipNum = 0;
	NV_Settings.bEnableVent = 0; //no vent
	//
	//GetScanRangeMax(0, 1); //3 mm
	//
	/*for (i = 0; i < OBJ_ENERGY_NUM; i++) {
		NV_Settings.fObjIMax[i] = OBJIMAX_DEFAULT; 	//smallest obj current
		NV_Settings.fObjIMin[i] = OBJIMIN_DEFAULT;	//largest obj current
	}*/
	NV_Settings.fObjIMax[0] = 1.3558;
	NV_Settings.fObjIMin[0] = 1.0329;
	NV_Settings.fObjIMax[1] = 1.14;
	NV_Settings.fObjIMin[1] = 0.8838;
	NV_Settings.fObjIMax[2] = 1.0071;
	NV_Settings.fObjIMin[2] = 0.7904;
	NV_Settings.fObjIMax[3] = 0.8792;
	NV_Settings.fObjIMin[3] = 0.6833;
	NV_Settings.fObjIMax[4] = 0.6694;
	NV_Settings.fObjIMin[4] = 0.5373;
	//
	NV_Settings.fTempHiLimit[0] = TEMP0_HI_LIMIT;	//INCASE
	NV_Settings.fTempLoLimit[0] = TEMP0_LO_LIMIT;
	NV_Settings.fTempHiLimit[1] = TEMP1_HI_LIMIT;	//HS_1
	NV_Settings.fTempLoLimit[1] = TEMP1_LO_LIMIT;
	NV_Settings.fTempHiLimit[2] = TEMP2_HI_LIMIT;	//TP_START
	NV_Settings.fTempLoLimit[2] = TEMP2_LO_LIMIT;
	NV_Settings.fTempHiLimit[3] = TEMP3_HI_LIMIT;	//HS_2
	NV_Settings.fTempLoLimit[3] = TEMP3_LO_LIMIT;
	NV_Settings.fTempHiLimit[4] = TEMP4_HI_LIMIT; 	//TP_RUN
	NV_Settings.fTempLoLimit[4] = TEMP4_LO_LIMIT;
	//
	NV_Settings.fImageWidthOnScreen = 143; //155 mm
	//
	//fRangeMax[0] = ZOOM_RANGE_MAX;
	//fRangeMax[1] = ZOOM_RANGE_MAX;
	//fRangeMin = ZOOM_RANGE_MIN;
	NV_Settings.nTurboType = TURBO_PFEIFFER;
	NV_Settings.fCoilShuntRatio[0] = DEF_SHUNT_RATIO1; //100 uH, 50 ohm
	NV_Settings.fCoilShuntRatio[1] = DEF_SHUNT_RATIO2; //20 uH, 250 ohm
	NV_Settings.fSenRCalc[0] = DEF_SEN_R0;
	NV_Settings.fSenRCalc[1] = DEF_SEN_R0R1;
	NV_Settings.fSenR[0] = DEF_SEN_R0;
	NV_Settings.fSenR[1] = DEF_SEN_R1;
	NV_Settings.fShuntR[0] = DEF_SHUNT_R0;
	NV_Settings.fShuntR[1] = DEF_SHUNT_R1;
	//400, 0.25 um, 1.8 degree, 0.5 mm pitch
	//1000, 0.1 um, 0.72 degre, 0.5 mm pitch
	NV_Settings.fPulsePerMM = 400;
	//NV_Settings.bRS232Enable[0] = 1;
	//NV_Settings.bRS232Enable[1] = 1; //varian use RS232
	//NV_Settings.bRS232Enable[2] = 0; //motion control use RS485
	//NV_Settings.bRS232Enable[3] = 1; //vacuum gauge
	//NV_Settings.fVACChangeMax = 0.006;
	//NV_Settings.fVACChangeMin = 0.002;
	NV_Settings.fAutoFocusThreshold = 0.2;
	NV_Settings.bScaleMode = SCALE_BY_TRIG_W_ROT;
	for (i = 0; i < PCB_ID_NUM; i++) {
		//NV_Settings.bPCBID[i] = 0;
		NV_Settings.bHVID[i] = 0;
	}
	NV_Settings.bObjRotateAdj = 0;
	NV_Settings.nBaudrate[0] = 115200;
	NV_Settings.nBaudrate[1] = 9600; //turbo pump control
	NV_Settings.nBaudrate[2] = 9600; //IP, gate valve 2(1), HV(2)
	NV_Settings.nBaudrate[3] = 9600; //reserved
	NV_Settings.nPIDBaudrate[0] = 9600;
	NV_Settings.nPIDBaudrate[1] = 9600;
	NV_Settings.sAirWaitTimeout = AIR_WAIT_TIMEOUT; //scroll on
	NV_Settings.sLowWaitTimeout = LOW_WAIT_TIMEOUT; //scroll on
	NV_Settings.lHighWaitTimeout = INT_HIGH_WAIT_TIMEOUT; //turbo on
	NV_Settings.sHighWaitTimeout = HIGH_WAIT_TIMEOUT;
	NV_Settings.sMHighWaitTimeout = MHIGH_WAIT_TIMEOUT; //ip on, HI to MH
	NV_Settings.lUHighWaitTimeout = UHIGH_WAIT_TIMEOUT; //ip on, MH to UH
	NV_Settings.sIGWaitTimeout = 30; //wait until IG is stable
	NV_Settings.sVentWaitTimeout = 40;
	NV_Settings.sStandbyWaitTimeout = STANDBY_WAIT_TIEMOUT;
	NV_Settings.sTPGVTimeout = TPGV_TIMEOUT;
	NV_Settings.fLowChangeRate = -0.07;	//volt/s
	NV_Settings.fHighChangeRate = -0.07; //volt/s
	//
	NV_Settings.fIMaxFactor = 0.99; //0.8* fIMaxAbs=maximal current
	for (i = 0; i < VADC_CH_NUM; i++) {
		NV_Settings.fCalVideo[i] = 0;
	}
#if FPGA_SYSCLK == 20
	NV_Settings.nSysClk = 20; //20 MHz
	NV_Settings.wDelayOverscan = 20; // 1 usec
#else
	NV_Settings.nSysClk = 40; //40 MHz
	NV_Settings.wDelayOverscan = 40; // 1 usec
#endif
	//
#if HV_TYPE == HV_SPELLMEN
	NV_Settings.fVAccRatio = 4096; //real voltage = DAC voltage * fVAccratio
	NV_Settings.fVBiasRatio = 600;
	NV_Settings.fFilaRatio = 0.856;	//watt = fV * fFilaRatio
	NV_Settings.fFilaResistance = 0.5;
	NV_Settings.fAccMonI = 33.3;    //7.65V --> 255 uA
#elif HV_TYPE == HV_MATSUSADA
	NV_Settings.fVAccRatio = 1500; //real voltage = DAC voltage * fVAccratio
	NV_Settings.fVBiasRatio = 350;
	NV_Settings.fFilaRatio = 2.075;	//watt = fV * fFilaRatio
	NV_Settings.fFilaResistance = 0.5;
	NV_Settings.fAccMonI = 100.0;
#else 	//custom
	NV_Settings.fVAccRatio = 1500; //real voltage = DAC voltage * fVAccratio
	NV_Settings.fVBiasRatio = 350;
	NV_Settings.fFilaRatio = 2.075;	//watt = fV * fFilaRatio
	NV_Settings.fFilaResistance = 0.5;
	NV_Settings.fAccMonI = 100.0;
#endif
	//PL modulation parameters
	NV_Settings.bEnablePLModX = 0;	//enable PL modulation
	NV_Settings.bEnablePLModY = 0;
	//NV_Settings.fPLAmp = 0.4;
	//NV_Settings.fPLPhase = 312; 	//degree
	NV_Settings.wPLAddrNum = 32;	//sample number per PL cycle
	NV_Settings.bPLFreq = 60; 		//default 60 Hz
	SetPLPeriod(); 					//set fPLPeriod
	NV_Settings.bPLSyncAuto = 0; 	//auto PL sync function
	NV_Settings.bBinPkt = 1;		//GETSYS binary packet
	NV_Settings.bUP2DN = 1;			//scan from up to down
	NV_Settings.bAutoXLineDelay = 1; 	//calculate delay time automatically
#if USE_AD7655 == 1 //20 MHz
	NV_Settings.nDelayDescent = 2; 		//200 ns;
	NV_Settings.nPixelDwellTime = 2; 	//100 ns
#else
	NV_Settings.nDelayDescent = 4; 		//= 4;
	NV_Settings.nPixelDwellTime = 20; 	//= 20;
#endif
	NV_Settings.nDelayLB = 2;
	NV_Settings.fAccMaxV = 10;			//volt
	NV_Settings.fFilaMaxV = 10;
	NV_Settings.fBiasMaxV = 10;
	NV_Settings.nTurboReadySpeed[0] = 1500; //custom
	NV_Settings.nTurboReadySpeed[1] = 1500; //Pfeiffer
	NV_Settings.nTurboReadySpeed[2] = 1350; //Agilent
	NV_Settings.nTurboLowSpeed[0] = 100; //custom
	NV_Settings.nTurboLowSpeed[1] = 100; //Pfeiffer
	NV_Settings.nTurboLowSpeed[2] = 100; //Agilent
	NV_Settings.sTurboBrakeSpeed = 1050;
	NV_Settings.bEnableWDT = 1;
#if IO_74HC14 == 1
	NV_Settings.bIOINV = 1;
#else
	NV_Settings.bIOINV = 0; //0:74HC07(OC), 1:74HC04(NOT)
#endif
	NV_Settings.bOnePkt = 0;
	NV_Settings.sVACNOKTh[0] = 3; //VAC NOK threshold
	NV_Settings.sVACNOKTh[1] = 3; //VAC BNOK threshold, broken
	NV_Settings.fIGLOTh = 7.3; //TorrToVolt(1); //1 torr
	NV_Settings.fIGINT2Th = 5.673; //TorrToVolt(1e-2); //5.673,INT2 threshold
	NV_Settings.fIGHITh = 3.7; //TorrToVolt(1e-4);	//3.7,ion gauge
	NV_Settings.fIGMHTh = 3.5; TorrToVolt(2e-6);	//3.5,ion gauge
	NV_Settings.fIGUHTh = 3.1; //TorrToVolt(8e-7);	//3.1,ion gauge
	NV_Settings.fPGLOTh = 3.55; //pirani gauge
	NV_Settings.fPGHITh = 1.2; //pirani gauge
#if IO_PCB_VER >= 4
	NV_Settings.bLEDAuto = 1;
#else
	NV_Settings.bLEDAuto = 0;
#endif
	NV_Settings.bEnCoilSW = 0;
	NV_Settings.lAFPixelNum = 80000;
	NV_Settings.nEdgeDiff = 12;
	NV_Settings.fCoilRatio = 0.280;	//inductance ratio of small and whole coil
	NV_Settings.fIThRatio = 0.70;	//I aspect ratio 1/1.25=0.8
	NV_Settings.fFocusMin = 35.0;	//auto focus
	NV_Settings.fFocusMax = 90.0;
	NV_Settings.nAFLimitNum = 28;
	NV_Settings.bFocusFOV = 1;
	strcpy(NV_Settings.szSN, "88635737030");
	//NV_Settings.fRatioBR = 6.0;
	//NV_Settings.fRatioCO = 2.0;
	//
	//NV_Settings.bUUID[0] = 0x01;
	//NV_Settings.bUUID[1] = 0x81;
	//NV_Settings.bUUID[2] = 0xAA;
	NV_Settings.wWaitEOC = 4; 		//4x50=300 ns, wait second eoc=0
	NV_Settings.bEDSOnByHW = 0; 	//EDS switch by hardware
	NV_Settings.sXNdxInc = 2;
	NV_Settings.bObjMode = 0; 		//1: power saving
	//NV_Settings.wXInit = 0; 		//
	NV_Settings.fOverscan = 0.06;	//default
	ResetBRCOToDefault();
	NV_Settings.bBSEMode = 2; //1:4 in 1, 2: 4 in 2
	//
	NV_Settings.sIPLeakNum = 3;
	NV_Settings.sIPLeakOnTime[0] = 90;
	NV_Settings.sIPLeakOnTime[1] = 180;
	NV_Settings.sIPLeakOnTime[2] = 120;
	NV_Settings.sIPLeakOnTime[3] = 60;
	NV_Settings.sIPLeakOffTime[0] = 180;
	NV_Settings.sIPLeakOffTime[1] = 300;
	NV_Settings.sIPLeakOffTime[2] = 180;
	NV_Settings.sIPLeakOffTime[3] = 210;
	NV_Settings.fIPLeakCheckV[0] = 3.92;
	NV_Settings.fIPLeakCheckV[1] = 3.88;
	NV_Settings.fIPLeakCheckV[2] = 3.6;
	NV_Settings.fIPLeakCheckV[3] = 3.59;
	NV_Settings.bScanPowerSave = 0;
	NV_Settings.bHVType = HV_TYPE;
	NV_Settings.bUDPPort = 0;	//0: return to CMD_PORT and DATA_PORT, 1:return to source port
	NV_Settings.bAutoVAC = 0;
	NV_Settings.bObjR = OBJ_CTRL_R;
	NV_Settings.fObjStandbyRatio = 0.8;
	NV_Settings.bAcckVAdjust = 0;
	NV_Settings.nObjTurn = OBJ_TURN_NUM;
	NV_Settings.bDACType[0] = DAC_AD5328; //SCAN
	NV_Settings.bDACType[1] = DAC_AD5328; //IO
	NV_Settings.bIPType = 0; //0:VARIAN, 1:EDWARDS
	NV_Settings.bUseIN = USE_IN; //ampere-turn
	NV_Settings.bEnableFIFO = 0;
	NV_Settings.nIdleTimeout = 3600;
}

/*void GetScanRangeMax(float fRange, int nDEFX)
{
	//fV2IR=10, 1V --> 0.1A
	fMaxXAtDEFX0 = (float)(DACF_P5V) / NV_Settings.fV2IR[0] / NV_Settings.fAPerMM[nCoilSwitch][0];
	fMaxYAtDEFX0 = (float)(DACF_P5V) / NV_Settings.fV2IR[1] / NV_Settings.fAPerMM[nCoilSwitch][1];
	//
	// deflector resistor ratio 10/5 ohm = 2
	if (nDEFX == 1)
		fRatioDefSenR = 2; //use 5 ohm deflector current sense resistor
	else if ((fRange > fMaxXAtDEFX0) && (fRange < fMaxXAtDEFX0 * DEF_R1_R2_RATIO)) {
		fRatioDefSenR = 2; //use 5 ohm deflector current sense resistor
	}
	else {
		fRatioDefSenR = 1; //use 10 ohm deflector current sense resistor
	}
	//scan from -fRangeMax to +fRangeMax
	fRangeMax[0] = fMaxXAtDEFX0 * fRatioDefSenR;
	fRangeMax[1] = fMaxYAtDEFX0 * fRatioDefSenR;
	//e.g. fRangeMax[0] = 5/10/0.15 = 3.33 mm
	//     fRangeMax[1] = 5/10/0.18 = 2.78 mm
	fRangeMin = 0.004 * fRatioDefSenR; //4 um/256 -> 40 nm/pixel
}*/
//

BYTE CheckTempSetLimit(int nNdx, float fSet)
{
	BYTE bValid = 1;
	float fHiLimit[TEMP_SENSOR_NUM_MAX] = {TEMP0_HI_LIMIT, TEMP1_HI_LIMIT, TEMP2_HI_LIMIT, TEMP3_HI_LIMIT, TEMP4_HI_LIMIT};
	float fLoLimit[TEMP_SENSOR_NUM_MAX] = {TEMP0_LO_LIMIT, TEMP1_LO_LIMIT, TEMP2_LO_LIMIT, TEMP3_LO_LIMIT, TEMP4_LO_LIMIT};
	if (fSet > fHiLimit[nNdx]) {		//70 degree centigrade
		bValid = 0;
	}
	else if (fSet < fLoLimit[nNdx]) {		//70 degree centigrade
		bValid = 0;
	}
	return bValid;
}

void CheckNVSettings(void)
{
	BYTE bValid = 1;
	int i;
	BYTE bUUID[UUID_NUM];
	float fHiLimit[TEMP_SENSOR_NUM_MAX] = {TEMP0_HI_LIMIT, TEMP1_HI_LIMIT, TEMP2_HI_LIMIT, TEMP3_HI_LIMIT, TEMP4_HI_LIMIT};
	float fLoLimit[TEMP_SENSOR_NUM_MAX] = {TEMP0_LO_LIMIT, TEMP1_LO_LIMIT, TEMP2_LO_LIMIT, TEMP3_LO_LIMIT, TEMP4_LO_LIMIT};
	//
	iprintf("Checking NV_Settings if Flash...\r\n");
	iprintf("Size of NV Structure: %d\r\n", (int)sizeof(NV_Settings));
	iprintf("Size of SYS_PARAM_2: %d\r\n", (int)sizeof(SYS_PARAM_2));
	iprintf("Size of SYS_PARAM_3: %d\r\n", (int)sizeof(SYS_PARAM_3));

	NV_SETTINGS *pData = (NV_SETTINGS *) GetUserParameters();
	iprintf("Verify key = 0x%lX\r\n", pData->dwVerifyKey);
	//
	for (i = 0; i < 2; i++) {
		if (pData->fAPerMM[i][0] > 20 || pData->fAPerMM[i][0] < 0.01)
			bValid = 0;
		if (pData->fAPerMM[i][1] > 20 || pData->fAPerMM[i][1] < 0.01)
			bValid = 0;
	}
	if (pData->dwVerifyKey != VERIFY_KEY) {
		iprintf("invalid verify key\n");
		bValid = 0;
	}
	for (i = 0; i < OBJ_ENERGY_NUM; i++) {
		if ((pData->fObjIMax[i] > OBJIMAX_HI) || (pData->fObjIMax[i] < OBJIMIN_LO)) {
			iprintf("invalid OBJ I max\n");
			bValid = 0;
		}
		if ((pData->fObjIMin[i] > OBJIMAX_HI) || (pData->fObjIMin[i] < OBJIMIN_LO)) {
			iprintf("invalid OBJ I min\n");
			bValid = 0;
		}
	}
	//
	for (i = 0; i < TEMP_SENSOR_NUM_MAX; i++) {
		if (pData->fTempHiLimit[i] > fHiLimit[i]) {		//70 degree centigrade
			iprintf("invalid temp hi limit %d\n", i);
			bValid = 0;
			break;
		}
		if (pData->fTempLoLimit[i] < fLoLimit[i]) {		//70 degree centigrade
			iprintf("invalid temp lo limit %d\n", i);
			bValid = 0;
			break;
		}
	}
	for (i = 0; i < TURBO_TYPE_NUM_MAX; i++) {
		if ((pData->nTurboReadySpeed[i] > 8000) || (pData->nTurboReadySpeed[i] < 20)) {
			bValid = 0;
			break;
		}
	}
	if ((pData->fImageWidthOnScreen < 5) || (pData->fImageWidthOnScreen > 500)) { //screen size, unit mm
		bValid = 0;
	}
	if ((pData->nTurboType > TURBO_TYPE_MAX) || (pData->nTurboType < TURBO_TYPE_MIN))
		bValid = 0;
	if (pData->bBinPkt > 3)
		bValid = 0;
	if ((pData->fIGMHTh > 10.0) || (pData->fIGMHTh < 0.0))
		bValid = 0;
	if ((pData->fIGLOTh > 10.0) || (pData->fIGLOTh < 0.0))
		bValid = 0;
	if (bValid == 0) {
		iprintf("Flash verification is INVALID - Initializing Flash\r\n");
		memmove(bUUID, NV_Settings.bUUID, UUID_NUM);
		memset(&NV_Settings, 0, sizeof(NV_SETTINGS));
		NV_Settings.dwVerifyKey = VERIFY_KEY;
		memmove(NV_Settings.bUUID, bUUID, UUID_NUM);
		ClearNVSettings();
		SaveUserParameters(&NV_Settings, sizeof(NV_SETTINGS));
	}
	else {
		memmove((BYTE *)&NV_Settings, (BYTE *)pData, sizeof(NV_SETTINGS));
		iprintf("Flash verification is VALID\r\n");
	}
	//avoid illegal value
	if ((NV_Settings.fObjStandbyRatio > 1.0) || (NV_Settings.fObjStandbyRatio < 0.0))
		NV_Settings.fObjStandbyRatio = 0.8;
	if ((NV_Settings.nObjTurn > 4000) || (NV_Settings.nObjTurn < 50))
		NV_Settings.nObjTurn = OBJ_TURN_NUM;
	SetPLPeriod(); //calculate fPLPeriod;
}
/*
 * Histogram operation
 */
#define HIS_BASE_NUM		512
#define HIS_SHIFT			7	//3(4096),7(65536)	//8*512=4096
#define HIS_BASE_NUM_FPGA	1024
#define HIS_FPGA_RATIO		2
#define HIS_HEADER_SIZE		18
#define HIS_FOOTER_SIZE		32
#define HIS_PACKET_SIZE		(HIS_HEADER_SIZE+HIS_BASE_NUM+HIS_FOOTER_SIZE)	//512+26=538, 292
//
long lHistogram[HIS_BASE_NUM];

void ClearHistogram(void)
{
	memset(&lHistogram, 0, HIS_BASE_NUM * sizeof(long));
	//there are 1024 histogram valuses in FPGA
	memset((BYTE *)&pwSram[ADDR_HISTO_START], 0, (HIS_BASE_NUM_FPGA + 10) * sizeof(WORD));
}

void SetHistogram(WORD wV)
{
	int nBase;
	nBase = wV >> HIS_SHIFT;	//65536/128=512
	if (nBase > HIS_BASE_NUM)
		return;
	lHistogram[nBase]++;
}

int GetHistogramPacketFromFPGA(BYTE *pData)
{
	int i, j, nBytes;
	long lMax, lSum, lTotal20, lTotal80;
	int nBaseMin, nBaseMax, nBase20, nBase80;
	float fRatio;
	WORD wHisBuffer[HIS_BASE_NUM_FPGA];
	WORD wVH, wVL;
	DWORD dwV;
	//
	memmove((BYTE *)wHisBuffer, (BYTE *)&pwSram[ADDR_HISTO_START], HIS_BASE_NUM_FPGA * sizeof(WORD));

	lTotal20 = (long)nScanPixelNumX * (long)nScanPixelNumY * 0.2;
	lTotal80 = (long)nScanPixelNumX * (long)nScanPixelNumY * 0.8;
	nBaseMin = -1;
	nBaseMax = -1;
	nBase20 = -1;
	nBase80 = -1;
	//
	lMax = 0;
	//base number reduce from 1024 to 512
	for (i = 0, j = 0; i < HIS_BASE_NUM; i++) {
		lHistogram[i] = (long)wHisBuffer[j] + (long)wHisBuffer[j + 1];
		j += 2;
		if (lHistogram[i] > lMax) lMax = lHistogram[i];
	}
	fRatio = (float)255 / lMax;
	pData[0] = 0x56;
	pData[1] = 0x6A;
	pData[2] = (lMax & 0x000000FF);			//LSB
	pData[3] = (lMax & 0x0000FF00) >> 8;
	pData[4] = (lMax & 0x00FF0000) >> 16;
	pData[5] = (lMax & 0xFF000000) >> 24;
	pData[6] = (HIS_BASE_NUM & 0x00FF);
	pData[7] = (HIS_BASE_NUM & 0xFF00) >> 8;
	for (i = 0, lSum = 0; i < HIS_BASE_NUM; i++)
	{
		lSum += lHistogram[i];
		if (lSum >= lTotal20 && nBase20 == -1) nBase20 = i;
		if (lSum >= lTotal80 && nBase80 == -1) nBase80 = i;
		if (lHistogram[i] != 0 && nBaseMin == -1) nBaseMin = i;
		if (lHistogram[i] != 0) nBaseMax = i;
	}
	pData[8] = nBase20 & 0x00FF;
	pData[9] = (nBase20 & 0xFF00) >> 8;
	pData[10] = nBase80 & 0x00FF;
	pData[11] = (nBase80 & 0xFF00) >> 8;
	pData[12] = nBaseMin & 0x00FF;
	pData[13] = (nBaseMin & 0xFF00) >> 8;
	pData[14] = nBaseMax & 0x00FF;
	pData[15] = (nBaseMax & 0xFF00) >> 8;
	for (i = 0, j = HIS_HEADER_SIZE; i < HIS_BASE_NUM; i++, j++)
		pData[j] = (BYTE)(lHistogram[i] * fRatio);
	//65536*65536(=4,294,967,296) --> 4096*1024*1024(=4,294,967,296)
	if (bEnableGradientByFW == 1) {
		wVL = (WORD)(dwGradientSum & 0xFFFF);
		wVH = (WORD)((dwGradientSum & 0xFFFF0000) >> 16);
		dwV = (DWORD)wVH;
		dwV = dwV << 16;
		dwV += (DWORD)wVL;
		dwGradientSum = dwV;
	}
	else {
		wVL = pwFpga[GET_GRADIENT_L];
		wVH = pwFpga[GET_GRADIENT_H];
		dwV = (DWORD)wVH;
		dwV = dwV << 16;
		dwV += (DWORD)wVL;
		dwGradientSum = dwV;
	}
	if (bDebug == 5) printf("GRADIENT=0X%04X_%04X,%ld\n", wVH, wVL, dwGradientSum);

	pData[HIS_PACKET_SIZE - 6] = wVL & 0x00FF;
	pData[HIS_PACKET_SIZE - 5] = (wVL & 0xFF00) >> 8;
	pData[HIS_PACKET_SIZE - 4] = wVH & 0x00FF;
	pData[HIS_PACKET_SIZE - 3] = (wVH & 0xFF00) >> 8;
	//
	pData[HIS_PACKET_SIZE - 2] = 0xA6;
	pData[HIS_PACKET_SIZE - 1] = 0x65;
	nBytes = HIS_PACKET_SIZE;
	return nBytes;
}

int GetHistogramPacket(BYTE *pData)
{
	int i, j, nBytes;
	long lMax, lSum, lTotal20, lTotal80;
	int nBaseMin, nBaseMax, nBase20, nBase80;
	float fRatio;
	//
	lTotal20 = (long)nScanPixelNumX * (long)nScanPixelNumY * 0.2;
	lTotal80 = (long)nScanPixelNumX * (long)nScanPixelNumY * 0.8;
	nBaseMin = -1;
	nBaseMax = -1;
	nBase20 = -1;
	nBase80 = -1;
	//
	lMax = 0;
	for (i = 0; i < HIS_BASE_NUM; i++)
		if (lHistogram[i] > lMax) lMax = lHistogram[i];
	fRatio = (float)255 / lMax;
	pData[0] = 0x56;
	pData[1] = 0x6A;
	pData[2] = (lMax & 0x000000FF);
	pData[3] = (lMax & 0x0000FF00) >> 8;
	pData[4] = (lMax & 0x00FF0000) >> 16;
	pData[5] = (lMax & 0xFF000000) >> 24;
	pData[6] = (HIS_BASE_NUM & 0x00FF);
	pData[7] = (HIS_BASE_NUM & 0xFF00) >> 8;
	for (i = 0, lSum = 0; i < HIS_BASE_NUM; i++)
	{
		lSum += lHistogram[i];
		if (lSum >= lTotal20 && nBase20 == -1) nBase20 = i;
		if (lSum >= lTotal80 && nBase80 == -1) nBase80 = i;
		if (lHistogram[i] != 0 && nBaseMin == -1) nBaseMin = i;
		if (lHistogram[i] != 0) nBaseMax = i;
	}
	pData[8] = nBase20 & 0x00FF;
	pData[9] = (nBase20 & 0xFF00) >> 8;
	pData[10] = nBase80 & 0x00FF;
	pData[11] = (nBase80 & 0xFF00) >> 8;
	pData[12] = nBaseMin & 0x00FF;
	pData[13] = (nBaseMin & 0xFF00) >> 8;
	pData[14] = nBaseMax & 0x00FF;
	pData[15] = (nBaseMax & 0xFF00) >> 8;
	for (i = 0, j = HIS_HEADER_SIZE; i < HIS_BASE_NUM; i++, j++)
		pData[j] = (BYTE)(lHistogram[i] * fRatio);
	//65536*65536(=4,294,967,296) --> 4096*1024*1024(=4,294,967,296)
	pData[HIS_PACKET_SIZE - 6] = dwGradientSum & 0x000000FF;
	pData[HIS_PACKET_SIZE - 5] = (dwGradientSum & 0x0000FF00) >> 8;
	pData[HIS_PACKET_SIZE - 4] = (dwGradientSum & 0x00FF0000) >> 16;
	pData[HIS_PACKET_SIZE - 3] = (dwGradientSum & 0xFF000000) >> 24;
	//
	pData[HIS_PACKET_SIZE - 2] = 0xA6;
	pData[HIS_PACKET_SIZE - 1] = 0x65;
	nBytes = HIS_PACKET_SIZE;
	return nBytes;
}

void ClearGradientX(void)
{
	dwGradientSum = 0;
	nGradientXStart = 16; //nScanPixelNumX / 4;
	nGradientXEnd = nScanPixelNumX - 16; //nScanPixelNumX * 3 / 4;
}

void SetGradientX(WORD *pwVal, int nPixelNum)
{
	int i;
	WORD wV[2] = {0, 0}; //both are positive
	int nSign[2] = {-1, 1};
	//
	for (i = nGradientXStart; i < nGradientXEnd; i++) {
		wV[1] = wV[0];
		nSign[1] = nSign[0];
		//
		wV[0] = pwVal[i + 1] - pwVal[i];
		if (wV[0] & 0x8000) {
			wV[0] ^= 0xFFFF; //get absolute value
			nSign[0] = -1;
		}
		else
			nSign[0] = 1;
		if ((nSign[0] == nSign[1]) && (wV[0] > NV_Settings.nEdgeDiff) && (wV[1] > NV_Settings.nEdgeDiff))
			dwGradientSum += (DWORD)wV[0];
	}
}
//-----------------------------------------------------------
// return temperature in degree centigrade
//-----------------------------------------------------------
int VoltToTemperature(float fVolt, float *pfTemp)
{
	int i;
	float fTemp = 0;
	//pull down 4.7K
	float fTempTable[TEMP_TABLE_NUM] = {-20, -15, -10, -5, 0,
		5, 10, 15, 20, 25, 30,
		35, 40, 45, 50, 55, 60,
		65, 70};
	float fVoltTable[TEMP_TABLE_NUM] = { 0.152194605, 0.1994291, 0.25797545, 0.329306355, 0.414705882,
		0.515282392, 0.630487805, 0.759400705, 0.901901494, 1.055102041, 1.21608907,
		1.381491048, 1.54759529, 1.711165049, 1.86912509, 2.018742679, 2.158363485,
		2.287273264, 2.404278406 };
	if (fVolt < fVoltTable[0]) {
		*pfTemp = fTempTable[0];
		return -1;
	}
	if (fVolt > fVoltTable[TEMP_TABLE_NUM-1]) {
		*pfTemp = fTempTable[TEMP_TABLE_NUM-1];
		return -1;
	}
	for (i = 0; i < TEMP_TABLE_NUM - 1; i++) {
		if (fVolt >= fVoltTable[i] && fVolt < fVoltTable[i+1]) {
			fTemp = fTempTable[i] + (fVolt - fVoltTable[i])/(fVoltTable[i+1] - fVoltTable[i])*(fTempTable[i+1]-fTempTable[i]);
			*pfTemp = fTemp;
			return 1;
		}
	}
	return -1;
}
//-------------------------------------------------------------
// command queue process
//-------------------------------------------------------------
void ClearCommandInQueue(void)
{
	nCommandInQueue = 0;
}
//
int GetCommandNumInQueue(void)
{
	return nCommandInQueue;
}

void PushCommandToQueue(char *szCommand, int nConnType)
{
	if (nCommandInQueue < CMD_IN_QUEUE_NUM) {
		strcpy(szCommandInQueue[nCommandInQueue], szCommand);
		nCmdQueType[nCommandInQueue] = nConnType;
		nCommandInQueue++;
	}
}
//
//return communication interface type
int PopCommandFromQueue(char *szCommand, int nNdx)
{
	strcpy(szCommand, szCommandInQueue[nNdx]);
	return nCmdQueType[nNdx];
}

float ByteSwapFloat(float fVIn)
{
	BYTE *pVIn, *pVOut;
	float fVOut;
	pVIn = (BYTE *)&fVIn;
	pVOut = (BYTE *)&fVOut;
	pVOut[0] = pVIn[3];
	pVOut[1] = pVIn[2];
	pVOut[2] = pVIn[1];
	pVOut[3] = pVIn[0];
	return fVOut;
}

DWORD ByteSwapDWord(DWORD dwVIn)
{
	BYTE *pVIn, *pVOut;
	DWORD dwVOut;
	pVIn = (BYTE *)&dwVIn;
	pVOut = (BYTE *)&dwVOut;
	pVOut[0] = pVIn[3];
	pVOut[1] = pVIn[2];
	pVOut[2] = pVIn[1];
	pVOut[3] = pVIn[0];
	return dwVOut;
}

WORD ByteSwapWord(WORD wVIn)
{
	BYTE *pVIn, *pVOut;
	WORD wVOut;
	pVIn = (BYTE *)&wVIn;
	pVOut = (BYTE *)&wVOut;
	pVOut[0] = pVIn[1];
	pVOut[1] = pVIn[0];
	return wVOut;
}

void SetPLPeriod(void)
{
	fPLPeriod = 1e6 / (float)NV_Settings.bPLFreq; //usec
	//must be longer for timeout
	pwFpga[SET_PL_PERIOD] = (WORD)fPLPeriod + 32;
}

void GetXScanTime(int nX, int nAvgNum)
{
	//32: 5.195 us
	//48: 6.125 us
	//64: 6.318 us
	//128: 9.676 us
	//256: 16.132 us
	//384: 22.597 us
	//512: 28.704 us
	//1024: 53.699 us
	fXScanTime = (3.22 + 0.0504375 * (float)nAvgNum) * (float)nX;
	bEnablePLModX = 0;
	bEnablePLModY = 0;
	if (NV_Settings.bPLSyncAuto == 0) {
		//sync state no change
		return;
	}
	if (SysParam.bZoomCtrl <= 1) { //no sync when magnification is small
		EnablePLSync(0);
		return;
	}
	EnablePLSync(1); //1 interrupt per power period
	//SetPLDAC(0, 0.0);
	//SetPLDAC(1, 0.0);
	//
	if (bDebug == 13) {
		printf("AUTO=%d,SYNC=%d,Period=%.1f,XScanTime=%.1f us\r\n",
			(int)NV_Settings.bPLSyncAuto, (int)bPLSync, fPLPeriod, fXScanTime);
	}
}

DWORD GetGradient(void)
{
	DWORD dwV;
	WORD wVL, wVH;
	wVL = pwFpga[GET_GRADIENT_L];
	wVH = pwFpga[GET_GRADIENT_H];
	dwV = (DWORD)wVH;
	dwV = dwV << 16;
	dwV += (DWORD)wVL;
	return dwV;
}

int OpenSerialPort(int nPort)
{
	int fdS;
	SerialClose( nPort ); // close UART 2
	fdS = OpenSerial( nPort, NV_Settings.nBaudrate[nPort], 1, 8, eParityNone );
	if (fdS < 0) {
		printf("serial%d open error\n", nPort);
		return -1;
	}
	if ((nPort >= 3) || (nPort < 0))
		return -1;
	fdSerial[nPort] = fdS;
	return 0;
}

//
//$2,GETSYS:1
//$2,SYS=8,bFilaOK,bFilaWDT,fFilaI,fFilaV,ACC_MON,ACC_FB,HV_ON,HV_ON_SW
//I_ACC_MON = ACC_MON * 39 uA
//ACC = ACC_FB * (5000/4.5)
void ParseHVCMessage(char *szCommand)
{
	char *p, *p1;
	int nNdx;
	int nPN = 0;
	int nV1, nV2, nV3, nV4 = 0, nV5 = 0;
	float fV1, fV2, fV3, fV4, fV5 = 0, fV6 = 0, fV7 = 0;
	int nParamNum;
	char szBuffer[256];
#define CMD_SYS		"SYS="
#define LEN_SYS		(sizeof(CMD_SYS)-1)
	if (strncmp(szCommand, CMD_SYS, LEN_SYS) == 0) {
		p = &szCommand[LEN_SYS];
		nParamNum = atoi(p);
		p1 = strchr(p, ',');
		p = p1 + 1;
		if (nParamNum == 8)
			nPN = sscanf(p, "%d,%d,%f,%f,%f,%f,%d,%d", &nV1, &nV2, &fV1, &fV2, &fV3, &fV4, &nV3, &nV4);
		else if (nParamNum == 9)
			nPN = sscanf(p, "%d,%d,%f,%f,%f,%f,%d,%d,%f", &nV1, &nV2, &fV1, &fV2, &fV3, &fV4, &nV3, &nV4, &fV5);
		else if (nParamNum == 10)
			nPN = sscanf(p, "%d,%d,%f,%f,%f,%f,%d,%d,%f,%d", &nV1, &nV2, &fV1, &fV2, &fV3, &fV4, &nV3, &nV4, &fV5, &nV5);
		else if (nParamNum == 11)
			nPN = sscanf(p, "%d,%d,%f,%f,%f,%f,%d,%d,%f,%d,%f", &nV1, &nV2, &fV1, &fV2, &fV3, &fV4, &nV3, &nV4, &fV5, &nV5, &fV6);
		else if (nParamNum == 12)
			nPN = sscanf(p, "%d,%d,%f,%f,%f,%f,%d,%d,%f,%d,%f,%f", &nV1, &nV2, &fV1, &fV2, &fV3, &fV4,
				&nV3, &nV4, &fV5, &nV5, &fV6, &fV7);
		if (nPN != nParamNum)
			return;
		SysParam.bFilaOK = (BYTE)nV1;
		SysParam.bFilaWDT = (BYTE)nV2;
		SysParam.fFilaI = fV1;
		SysParam.fFilaV = fV2;
		SysParam.bCusHVON = (BYTE)nV3; //HV ON control
		SysParam.bHVONSW = (BYTE)nV4; //HV ON switch
		SysParam.fBiasV = fV5;
		//
		SysParam2.bFilaOK = SysParam.bFilaOK;
		SysParam2.bFilaWDT = SysParam.bFilaWDT;
		SysParam2.fFilaI = SysParam.fFilaI;
		SysParam2.fFilaV = SysParam.fFilaV;
		SysParam2.bCusHVON = SysParam.bCusHVON;
		SysParam2.bHVONSW = SysParam.bHVONSW;
		SysParam2.fBiasV = SysParam.fBiasV;
		SysParam2.bExtCtrl = (BYTE)nV5;		//external control
		SysParam2.fTempe[0] = fV6;
		SysParam2.fTempe[1] = fV7;
		SysParam2.fAccCurrent = fV3; // * 39; //unit uA
		SysParam2.fAccVoltage = fV4; // * (15000/4.5); //unit kV
		if (bDebug == 21) {
			sprintf(szBuffer, "FilaOk=%d\r\nFilaWDT=%d\r\nFilaI=%.4f\r\nFilaV=%.4f\r\nHVON=%d\r\nBiasV=%.2f\r\n",
					SysParam.bFilaOK, SysParam.bFilaWDT, SysParam.fFilaI, SysParam.fFilaV,
					SysParam.bCusHVON, SysParam.fBiasV);
			printf(szBuffer);
			printf("ACCKV=%.3f kV\n", SysParam2.fAccVoltage);
			printf("ACCI=%.3f uA\n", SysParam2.fAccCurrent);
			printf("EXTCTRL=%d\n", (int)SysParam2.bExtCtrl);
			printf("TEMPE=0,%.1f deg C\n", SysParam2.fTempe[0]);
			printf("TEMPE=1,%.1f deg C\n", SysParam2.fTempe[1]);
		}
	}
#define CMD_VADC		"VADC="
#define LEN_VADC		(sizeof(CMD_VADC)-1)
	if (strncmp(szCommand, CMD_VADC, LEN_VADC) == 0) {//scan stop
		p = &szCommand[LEN_VADC];
		for (nNdx = 0; nNdx < 8; nNdx++) {
			p1 = strchr(p, ',');
			if (p1 == NULL)
				break;
			//fHVCADC[nNdx] = atof(p);
			SysParam2.fADCV[nNdx] = atof(p); //fHVCADC[nNdx];
			p = p1 + 1;
		}
		goto ParseRet;
	}
ParseRet:
	return;
}

void ParseIPCMessage(char *szCommand)
{
	char *p, *p1;
	int nV1 ;
	WORD wkV ;
	WORD wAmp ;
	char szValue[128] ;
	//$1,
#define CMD_GETAMP	"AMP="
#define LEN_GETAMP	(sizeof(CMD_GETAMP)-1)
	if (strncmp(szCommand, CMD_GETAMP, LEN_GETAMP) == 0) {//IP CURRENT
		p = &szCommand[LEN_GETAMP];
		ReplaceChar(szCommand, '=', ' ');
		sscanf(&szCommand[LEN_GETAMP], "%d", &nV1);
		wAmp = ByteSwapWord(nV1) ;
		SysParam.fIpAmp = WordToVolt(wAmp, 676, 3.3, 0) ;
		sprintf(szValue, "IP CURRENT=%f\r\n", SysParam.fIpAmp); //IP current info
		//printf(szValue) ;
		goto ParseRet;
	}
#define CMD_BOUND	"BOUNDARY="
#define LEN_BOUND	(sizeof(CMD_BOUND)-1)
	if (strncmp(szCommand, CMD_BOUND, LEN_BOUND) == 0) {//motor limit
		p = &szCommand[LEN_BOUND];
		ReplaceChar(szCommand, '=', ' ');
		sscanf(&szCommand[LEN_BOUND], "%d", &nV1);
		sprintf(szValue, "BOUNDARY at %d steps\r\n", nV1); //boundary reached at counter moving steps
		//printf(szValue) ;
		goto ParseRet;
	}
#define CMD_GETKV	"KV="
#define LEN_GETKV	(sizeof(CMD_GETKV)-1)
	if (strncmp(szCommand, CMD_GETKV, LEN_GETKV) == 0) {//IP HV
		p = &szCommand[LEN_GETKV];
		ReplaceChar(szCommand, '=', ' ');
		sscanf(&szCommand[LEN_GETKV], "%d", &nV1);
		wkV = ByteSwapWord(nV1) ;
		SysParam.fIpHV = WordToVolt(wkV, 676, 3.3, 0) ; ;
		sprintf(szValue, "IP HV=%f\r\n", SysParam.fIpHV); //IP HV infor
		//printf(szCommand) ;
		goto ParseRet;
	}
//IPINFO=<IPHV>,<IPAMP>,<IP status>,<Motor status>
#define CMD_INFO	"IPINFO="
#define LEN_INFO	(sizeof(CMD_INFO)-1)
	if (strncmp(szCommand, CMD_INFO, LEN_INFO) == 0) {
		p = &szCommand[LEN_INFO];
		ReplaceChar(szCommand, '=', ' ');
		sscanf(&szCommand[LEN_INFO], "%d", &nV1);
		wkV = nV1; //ByteSwapWord(nV1);
		SysParam.fIpAmp = WordToVolt(wkV, 1024, 5.0, 0) ; //0-10mA => 0-3.3V
		SysParam.fIpAmp /= 3.3; //unit mA
		if (bDebug == 20)
			printf("IP Current=%.4f mA\r\n", SysParam.fIpAmp); //IP Current info
		p1 = strchr(p, ',');
		if (p1 == NULL) return;
		p = p1 + 1;
		sscanf(p, "%d", &nV1);
		wAmp = nV1; //ByteSwapWord(nV1);
		SysParam.fIpHV = WordToVolt(wAmp, 1024, 5.0, 0) ;//0-5 kV => 0-3.3V
		SysParam.fIpHV *= 1515.0; //unit volt
		if (bDebug == 20)
			printf("IP HV=%.1f V\r\n", SysParam.fIpHV); //IP HV info
		p1 = strchr(p, ',');
		if (p1 == NULL) return;
		p = p1 + 1;
		sscanf(p, "%d", &nV1);
		SysParam.bMotorMove = (BYTE)nV1; //0,1,2
		if (bDebug == 20)
			printf("MOVING=%d\r\n", SysParam.bMotorMove); //motor moving status
		p1 = strchr(p, ',');
		if (p1 == NULL) return;
		p = p1 + 1;
		sscanf(p, "%d", &nV1);
		SysParam.bIPON = (BYTE)nV1;
		if (bDebug == 20)
			printf("IPON=%d\r\n", SysParam.bIPON); //IP on off status
		goto ParseRet;
	}
#define CMD_OVSTP	"OVERSTEP="
#define LEN_OVSTP	(sizeof(CMD_OVSTP)-1)
	if (strncmp(szCommand, CMD_OVSTP, LEN_OVSTP) == 0) {//motor oversteps
		p = &szCommand[LEN_OVSTP];
		ReplaceChar(szCommand, '=', ' ');
		sscanf(&szCommand[LEN_OVSTP], "%d", &nV1);
		sprintf(szValue, "OVER STEP at %d steps\r\n", nV1); //move over specified steps
		//printf(szValue) ;
		goto ParseRet;
	}
#define CMD_MVTMO	"TIMEOUT"
#define LEN_MVTMO	(sizeof(CMD_MVTMO)-1)
	if (strncmp(szCommand, CMD_MVTMO, LEN_MVTMO) == 0) {//move timeout
		sprintf(szValue, "MOVE TIMEOUT"); //
		//printf(szValue) ;
		goto ParseRet;
	}
ParseRet:
	return;
}


void EnableEDS(BYTE bEnable)
{
	static BYTE bPrevState = OP_IDLE;
	//
	SetEDSOn(bEnable); //set relay on/off
	if (bEnable == 1) {
		if (SysParam.bScanning != OP_EDS)
			bPrevState = SysParam.bScanning;
		SysParam.bScanning = OP_EDS;
	}
	else {
		//if (SysParam.bScanning != OP_IDLE) //system is stop
		SysParam.bScanning = bPrevState;
	}
	bEnableEDS = (bEnable == 1) ? 1 : 0;
}

void AppendLFCR(char *szText)
{
	int nLen;
	nLen = strlen(szText) - 1;
	if (szText[nLen] == '\r')
		return;
	if (szText[nLen] == '\n')
		return;
	strcat(szText, "\n\r"); //LF+CR
}

void RemoveLFCR(char *szText)
{
	int nLen;
	char c;
	nLen = strlen(szText);
	c = szText[nLen - 1];
	while ((c == '\r') || (c == '\n')) {
		szText[nLen - 1] = 0;
		nLen = strlen(szText);
		if (nLen < 1) break;
		c = szText[nLen - 1];
	}
	return;
}

float CalculateObjStandbyV(void)
{
	float fI;
	float fV;
	fI = NV_Settings.fObjIMax[nObjTableNdx] * NV_Settings.fObjStandbyRatio + NV_Settings.fObjIMin[nObjTableNdx] * (1 - NV_Settings.fObjStandbyRatio);
	ObjI2V(fI, &fV, &SysParam.fObjAdjAngle);
	return fV;
}
