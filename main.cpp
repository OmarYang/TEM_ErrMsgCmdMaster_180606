/******************************************************************************
 * Copyright 1998-2008 NetBurner, Inc.  ALL RIGHTS RESERVED
 *   Permission is hereby granted to purchasers of NetBurner Hardware
 *   to use or modify this computer program for any use as long as the
 *   resultant program is only executed on NetBurner provided hardware.
 *
 *   No other rights to use this program or it's derivitives in part or
 *   in whole are granted.
 *
 *   It may be possible to license this or other NetBurner software for
 *   use on non NetBurner Hardware. Please contact Licensing@Netburner.com
 *   for more information.
 *
 *   NetBurner makes no representation or warranties with respect to the
 *   performance of this computer program, and specifically disclaims any
 *   responsibility for any damages, special or consequential, connected
 *   with the use of this program.
 *
 *   NetBurner, Inc
 *   5405 Morehouse Drive
 *   San Diego Ca, 92121
 *   http://www.netburner.com
 *
 *****************************************************************************/
#include <predef.h>
#include <stdio.h>
#include <stdlib.h> //atoi, srand
#include <string.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <tcp.h>
#include <udp.h>
#include <serial.h>
#include <NetworkDebug.h>
#include "mod_sel.h"
#include <cfinter.h>
#include <pins.h>
#include "main.h"
#include "cpu_cmd.h"
#include "data_struct.h"
#include "rtc.h"
#include "function.h"
#include "Turbo_Control_Protocol.h"
#include <ucos.h>
#include "motor.h"
#include "hv_ctrl.h"
#include "ErrorMsg.h"
//
#include "cardtype.h"	//define USE_MMC
#include <effs_fat/fat.h>
#ifdef USE_MMC
#include <effs_fat/mmc_mcf.h>
#endif
#ifdef USE_CFC
#include <effs_fat/cfc_mcf.h>
#endif
//
#define DEBUG_TCP		1
#define SINGLE_CH		0	//1: use ADC0 only, 0: use 4 ADCs
#define ENABLE_IRQ1 	1
//
#if SCAN_PCB_VER >= 14
#define ENABLE_IRQ3		0
#else
#define ENABLE_IRQ3		0
#endif
//
const char *AppName = APP_NAME;
//IPADDR 	IpAddrFrom = 0; //command from
UDP_CLIENT UdpClient;
//0: debug, command
//1: turbo pump
//2: RS485, IP_PWR_CTRL and custom HV control
//3: reserved high vacuum gauge
int fdSerial[4] = {-1, -1, -1, -1};
//
BYTE bSerial1Got = 0;
int nSerial1Ndx = 0;
BYTE bSerial2Got = 0;
BYTE bSerial3Got = 0;
//BYTE bPktOrderNdx = 0;
BYTE bDB3Busy = 0;
//
int nPrevConn = 0;
BYTE bACK = 0;
//int bSimuMode = 0;
int nSimuShift = 0;
//WORD wAFYStart = 0; //auto focus Y start
BYTE bEnablePLModX = 0;
BYTE bEnablePLModY = 0;
BYTE bEnableIO = 1;
BYTE bInitOK = 0;
BYTE bLEDTest = 0;
DWORD dwGradient = 0;
//
int nResendNum = 0;
int nResendY[RESEND_NUM_MAX];
float fObjStandbyV = OBJ_STANDBY_V;
//
extern BYTE bEnableEDS;
extern BYTE bEnableCommand;
//
#if USE_AD7655 == 1
int nVADCAvgNum = 1; //25*100 ns=2.5 us
#elif SCAN_PCB_VER >= 10
int nVADCAvgNum = 64; //25*100 ns=2.5 us
#else
int nVADCAvgNum = 2;
#endif
//
int nASCIICode = 0;
//
//OS_CRIT OsCritical;
//
extern BYTE bVACBypassOK[VAC_GAUGE_NUM_MAX];
//extern int nGaugeStatus[VAC_GAUGE_NUM_MAX]; //gauge status
extern float fVACChangeRate[VAC_GAUGE_NUM_MAX];
extern float fVACLevel[VAC_GAUGE_NUM_MAX];
extern float fOverscan;
//
extern BYTE bDebug;
extern BYTE bSerDebug;
extern BYTE bBlanking;
extern int nOverscan;
//
extern BYTE bHVCommand;
extern BYTE bFilaConstMode; //FILACNST, 0: NONE, 1:CP, 2:CC
//
extern volatile PWORD pwSram;	//nCS1
extern volatile PWORD pwFpga;	//nCS2
extern int nShadowMode;
BYTE bImageMask = 0x01; //0x01:BEI, 0x02:SEI, 0x03:BEI+SEI
BYTE bSecondImage = 10;
//extern int nObjOn;
extern WORD wSCAN_DO;	//16-bit DO buffer
extern WORD wIO_DO;		//IO board output
extern WORD wDI;
extern int nConnType;
extern BYTE bPLSync;
//
extern int nPktDelay1;
extern int nPktDelay2;
extern int nPktDelay3;
extern float fFocus;
extern float fFocusRange;
extern int nEDSDWT;
extern int nEDSHandshake;
extern BYTE bTestMode;
//
WORD wDACX[PIXEL_NUM_X_MAX_PI];		//ascent DAC table
WORD wDACY[PIXEL_NUM_Y_MAX];
WORD wDACDX[PIXEL_NUM_X_MAX_PI];	//descent DAC table
WORD wDACDY[PIXEL_NUM_Y_MAX];
DWORD dwImageAddr = 0;
//int nQueAddrBuf[PIXEL_NUM_Y_MAX];
//
int nScanDelay[PIXEL_NUM_X_MAX_PI];
int nIdleTimeout = IDLE_TIMEOUT;   //shutdown timer
int nEchoTimeout = 0;
int nDelayIDec = 1; 	//delay increase with time
int nDelayIDecStep = 1;	//the larger, the slower
//int nDelayDescent = 4;	//4 x 50 ns = 200 ns
int nDelayPerLine = 200; //unit microsecond
//int hdlTxUDP = -1;
extern float fObjV;
float fObjVOP = 3;
extern float fInitialX;	//volt
extern float fInitialY;	//volt
extern float fFinalY; 		//volt
//
int nReadyToErrorNdx[3] = {0, 0, 0};
int nReadyToErrorNum = 5;
int nReadyCount = 0;
extern int nVacErrToleranceNum[2];
//
extern BYTE bVOPPower;
//
BYTE bFastScan = 0x01; //always 1
BYTE bRegionScan = 0;
float fRegion = 1.0;
float fRegionStart = 0.25;
int nRegionYStart;
int nRegionYEnd;
//
BYTE bEnableGradientByFW = 0;
//BYTE bFastClock = 1;
//
int nDelayEdge = 200;
int nXNumMStep = 4;
int nXNumPStep = 1;
WORD wZMax = 0;
WORD wZMin = 0xFFFF;
WORD wZMaxFW[2] = {0};
WORD wZMinFW[2] = {0xFFFF};
//
//OS_SEM semUdpTransfer;
//
//----- Global Vars -----
BYTE pRxBuffer[RX_BUFSIZE];  //4096 bytes
char szSerial1Buffer[MAX_STRING_SIZE];
char szSerial2Buffer[MAX_STRING_SIZE];
char szSerial3Buffer[MAX_STRING_SIZE];
//
#define CMD_QUE_NUM 10
char szCmdQueue[CMD_QUE_NUM][32];
int nCmdQueNdx = 0;
int nCmdQuePrevNdx = 0;
//
void SetupIRQ1(void);
void SetupIRQ3(void); //power line modulation
//
extern "C" {
   void UserMain(void * pd);
#if MODULE_TYPE == MT_MOD5270B
   void SetIntc( long func, int vector, int level, int prio );
#elif MODULE_TYPE == MT_MOD5282
   void SetIntc( int intc, long func, int vector, int level, int prio );
#endif
}
//------------------------------------------------------------------
// Declare a task stack for the UDP Reader task
DWORD UdpReaderStack[USER_TASK_STK_SIZE];
DWORD TcpListenStack[USER_TASK_STK_SIZE];
DWORD UdpTransmitStack[USER_TASK_STK_SIZE];
DWORD ScanMonitorTaskStack[USER_TASK_STK_SIZE];
//DWORD TimerTaskStack[USER_TASK_STK_SIZE];
DWORD Serial0TaskStack[USER_TASK_STK_SIZE];
DWORD Serial1TaskStack[USER_TASK_STK_SIZE];
DWORD Serial2TaskStack[USER_TASK_STK_SIZE];
DWORD VacuumStateTaskStack[USER_TASK_STK_SIZE];
//
BYTE bTimerStop = 0;
BYTE bScanAbort = 0;
BYTE bScanPause = 0;
//BYTE bScanning = OP_IDLE;
BYTE bScanRestart = 0;
BYTE bAutoFocusStart = 0;
//BYTE bAutoFocusMode = 0;
//BYTE bAutoFocusRefresh = 0;
//
BYTE bStopType = 1;
int nYNdxInc = 1;  //auto focus y address increment
int nXNdxInc = 2;  //auto focus x address increment
BYTE bFooterPerUDP = 1;
BYTE bIRQACK = 0;
int nTxTestNum;
//int nFfsTest;		//file system test
//
int nScanImageNum = 0;		//10
int nScanImageNdx = 0;	//0 ~ 9
//int nScanInterval = 0;	//msec, 1000
//int nScanIntervalTick = 0; // 200,400,600,800,1000
long lSysclkTick = 0; //unit 0.1 msec
long lTick1S = 0;
int nScanPixelNumX = 400;
int nScanPixelNumY = 300;
int nScanPixelNumXDV = 50;
int nScanPixelNumYDV = 37;
BYTE bPktNum = 1;
int nDataBytesPerLine = 800;
//int nTotBytePerPacket[PKT_NUM_MAX] = {816, 816, 816}; //=nDataBytePerPacket+sizeof(HEADER_BEGIN_
int nDataBytePerPacket[PKT_NUM_MAX] = {800, 800, 800};
int nBytePerPixel = 2;
int nPixelPerPacket[PKT_NUM_MAX] = {400, 400, 400};
//float fTimeXAsc = 0.01; //10 ms
//float fTimeX = 0.01;
//
extern volatile PWORD pwFifo;
extern NV_SETTINGS NV_Settings;
extern SYS_PARAM_3 SysParam;
extern SYS_PARAM_2 SysParam2;
extern SET_PARAM SetParam;
//
extern int nTurboRotationSpeed;
extern int nTurboReadySpeed;
extern int nTurboLowSpeed;
//
extern float fADCVal[ADC_CH_NUM_MAX];
//
//TCP/IP connection
DCFD_SET fdSet[NFDS];
//BYTE bTcpConnected = 0;
//
BYTE bEnableHisto = 0; //default no histogram
//LED control
BYTE bEnableLED = 1;
BYTE bLEDState[LED_NUM] = {ST_LED_DARK, ST_LED_DARK, ST_LED_DARK, ST_LED_DARK};	//0:GREEN,1:RED,2:BLUE,3:YELLOW
int nPITRCount;
extern int nVacOn;
extern int nVacOff;
extern BYTE bChkTP;
extern BYTE bEnableEDX;
//
int nBytesIn = QUE_DATA_START;
int nBytesOut = QUE_DATA_START;
int nQueStart = QUE_DATA_START;
int nQueSize = 32768;
//1520 pixel x 2 byte/pixel * 300 line = 912,000 bytes
BYTE pbTxBuffer[2][UDP_QUEUE_SIZE]; //[0]:BEI,ADC1; [1]:SEI,ADC3
BYTE bFirstPacket = 0;
BYTE bFinalPacket = 0;
//
BYTE bFPGAMajorVersion = 0x00;
BYTE bFPGAMinorVersion = 0x00;
BYTE bIOFPGAMajorVersion = IO_PCB_VER;
BYTE bIOFPGAMinorVersion = 0x00;
BYTE bFPGADateMonth = 0x00;
BYTE bFPGADateDay = 0x00;
BYTE bPIDMajorVersion = 1;
BYTE bPIDMinorVersion = 1;
//
// Parallel shows
void ShowTitle(int nConnType)
{
	char szValue[128];
	char szBuffer[512];
	//
	strcpy(szBuffer, "---------------------------------------\n");
	sprintf(szValue, "%s SEM Controller Program\n", MODEL_NAME);
	strcat(szBuffer, szValue);
	sprintf(szValue, "FW_VER=%d.%d.%d, %4d/%02d/%02d \n", MAJOR_VER, MINOR_VER, BUILD_VER,
			FW_DATE_Y, FW_DATE_M, FW_DATE_D);
	strcat(szBuffer, szValue);
	//
	GetVersion();
	//
	sprintf(szValue, "SCAN_PCB_VER=%d\n", SCAN_PCB_VER);
	strcat(szBuffer, szValue);
	sprintf(szValue, "SCAN_FPGA_VER=%02d.%02d, %02d/%02d\n", bFPGAMajorVersion, bFPGAMinorVersion,
		bFPGADateMonth, bFPGADateDay);
	strcat(szBuffer, szValue);
#if IO_PCB_VER >= 1
	sprintf(szValue, "IO_PCB_VER=%d\n", IO_PCB_VER);
	strcat(szBuffer, szValue);
	sprintf(szValue, "IO_FPGA_VER=%02d.%02d\n",
		bIOFPGAMajorVersion, bIOFPGAMinorVersion);
	strcat(szBuffer, szValue);
#endif
	sprintf(szValue, "COIL_PCB_VER=%d\n", COIL_PCB_VER);
	strcat(szBuffer, szValue);
	SysParam.bErrorHWVersion &= (~ERR_HW_SCAN_VER);
	SysParam.bErrorHWVersion &= (~ERR_HW_IO_VER);
	if (bFPGAMajorVersion != SCAN_PCB_VER) {
		strcat(szBuffer, "ERROR: SCAN FPGA AND PCB MISMATCH!\n");
		SysParam.bErrorHWVersion |= ERR_HW_SCAN_VER;
	}
	if (bIOFPGAMajorVersion != IO_PCB_VER) {
		strcat(szBuffer, "ERROR: IO FPGA AND PCB MISMATCH!\n");
		SysParam.bErrorHWVersion |= ERR_HW_IO_VER;
	}
#if SCAN_PCB_VER >= 17
	SPI_GetVersion();
	sprintf(szValue, "PID_VER=%d.%d\n", (int)bPIDMajorVersion,
		(int)bPIDMinorVersion);
	strcat(szBuffer, szValue);
#endif
	strcat(szBuffer, "---------------------------------------\n");
	TxString(nConnType, szBuffer);
}

void TxSpeedTest(void*)
{
	int i, nTxTestNdx;
	char szNum[10];
	BYTE pbBuffer[TX_TEST_SIZE]; //=1024
	float fTime1, fTime2;
	//
	fTime1 = (float)TimeTick / TICKS_PER_SECOND;
	printf("Tx Speed Test, Start time = %8.4f\n", fTime1);
	//
	ShowIP( UdpClient.iaSource );
	iprintf("\n");
	bTimerStop = 1;
	for (nTxTestNdx = 0; nTxTestNdx < nTxTestNum; nTxTestNdx++)
	{
		sprintf(szNum, "%08d", nTxTestNdx);
		for (i = 0; i < TX_TEST_SIZE; i += 8)
		{
			memmove(pbBuffer + i, szNum, 8); //
		}
		for (i = 0; i < TX_TEST_NUM; i++)	//TX_TEST_NUM=1024, transmit 1 Mbytes data
			TxData(CONN_UDP, pbBuffer, TX_TEST_SIZE);
		iprintf("%d", nTxTestNdx%10);
	}
	bTimerStop = 0;
	fTime2 = (float)TimeTick / TICKS_PER_SECOND;
	printf("\nEnd time = %8.4f, %8.4f, %8.4f MB/s\n", fTime2, fTime2 - fTime1, (float)nTxTestNum/(fTime2 - fTime1));
}

/*
void FfsTest(void*)
{
	printf("FFS Test Start ------------------------\n");
	// The following call to f_enterFS() must be called in every task that accesses
	//	the file system.  This must only be called once in each task and must be done before
	//	any other file system functions are used.  Up to 10 tasks can be assigned to use
	//	the file system. Any task may also be removed from accessing the file system with a
	//	call to the function f_releaseFS().
	//
	f_enterFS();
	InitExtFlash();  // Initialize the CFC or SD/MMC external flash drive
	//
	DisplayEffsSpaceStats();  // Display file space usage
	switch (nFfsTest)
	{
		case 1:
		ReadWriteTest();		//read write test
		break;
		case 2:
		DumpDir();              // Display flash card files and directories
		break;
	}
	UnmountExtFlash();
	f_releaseFS();
	printf("FFS Test End ------------------------\n");
}
*/

void TxSetParam(int nType)
{
	NV_SETTINGS NVSetTemp;
	int i, nB;
	memmove(&NVSetTemp, &NV_Settings, sizeof(NV_SETTINGS));
	for (nB = 0; nB < BOARD_NUM; nB++) {
		for (i = 0; i < DAC_CH_NUM_MAX; i++) {
			NVSetTemp.fDACSlope[nB][i] = ByteSwapFloat(NV_Settings.fDACSlope[nB][i]);
			NVSetTemp.fDACOffset[nB][i] = ByteSwapFloat(NV_Settings.fDACOffset[nB][i]);
		}
	}
	for (i = 0; i < VDAC_CH_NUM_MAX; i++) {
		NVSetTemp.fScanDACOffset[i] = ByteSwapFloat(NV_Settings.fScanDACOffset[i]);
		NVSetTemp.fScanDACSlope[i] = ByteSwapFloat(NV_Settings.fScanDACSlope[i]);
	}
	for (i = 0; i < VADC_CH_NUM_MAX; i++) {
		NVSetTemp.fScanADCOffset[i] = ByteSwapFloat(NV_Settings.fScanADCOffset[i]);
	}
	for (i = 0; i < ADC_CH_NUM_MAX; i++) {
		NVSetTemp.fADCOffset[i] = ByteSwapFloat(NV_Settings.fADCOffset[i]);
	}
	for (i = 0; i < ISEN_CH_NUM; i++) {
		NVSetTemp.fV2IR[i] = ByteSwapFloat(NV_Settings.fV2IR[i]);
		NVSetTemp.fLoadR[i] = ByteSwapFloat(NV_Settings.fLoadR[i]);
	}
	for (i = 0; i < TURBO_TYPE_NUM_MAX; i++) {
		NVSetTemp.nTurboReadySpeed[i] = ByteSwapFloat(NVSetTemp.nTurboReadySpeed[i]);
	}
	for (i = 0; i < 4; i++) {
		NVSetTemp.fBaseBR[i] = ByteSwapFloat(NV_Settings.fBaseBR[i]);
		NVSetTemp.fRangeBR[i] = ByteSwapFloat(NV_Settings.fRangeBR[i]);
	}
	//----------------------------------------------------------
	for (i = 0; i < CO_NUM; i++) {
		NVSetTemp.fBaseCO[i] = ByteSwapFloat(NV_Settings.fBaseCO[i]);
		NVSetTemp.fRangeCO[i] = ByteSwapFloat(NV_Settings.fRangeCO[i]);
	}
	//[0]: major coil, 1: minor coil
	NVSetTemp.fAPerMM[0][0] = ByteSwapFloat(NV_Settings.fAPerMM[0][0]);
	NVSetTemp.fAPerMM[0][1] = ByteSwapFloat(NV_Settings.fAPerMM[0][1]);
	NVSetTemp.fAPerMM[1][0] = ByteSwapFloat(NV_Settings.fAPerMM[1][0]);
	NVSetTemp.fAPerMM[1][1] = ByteSwapFloat(NV_Settings.fAPerMM[1][1]);
	for (i = 0; i < OBJ_ENERGY_NUM; i++) {
		NVSetTemp.fObjIMax[i] = ByteSwapFloat(NV_Settings.fObjIMax[i]);
		NVSetTemp.fObjIMin[i] = ByteSwapFloat(NV_Settings.fObjIMin[i]);
	}
	for (i = 0; i < TEMP_SENSOR_NUM_MAX; i++) {
		NVSetTemp.fTempHiLimit[i] = ByteSwapFloat(NV_Settings.fTempHiLimit[i]);
		NVSetTemp.fTempLoLimit[i] = ByteSwapFloat(NV_Settings.fTempLoLimit[i]);
	}
	//----------------------------------------------------------
	NVSetTemp.fImageWidthOnScreen = ByteSwapFloat(NV_Settings.fImageWidthOnScreen);
	NVSetTemp.nTurboType = ByteSwapDWord(NV_Settings.nTurboType);
	NVSetTemp.fCoilShuntRatio[0] = ByteSwapFloat(NV_Settings.fCoilShuntRatio[0]);
	NVSetTemp.fCoilShuntRatio[1] = ByteSwapFloat(NV_Settings.fCoilShuntRatio[1]);
	NVSetTemp.fSenRCalc[0] = ByteSwapFloat(NV_Settings.fSenRCalc[0]);
	NVSetTemp.fSenRCalc[1] = ByteSwapFloat(NV_Settings.fSenRCalc[1]);
	NVSetTemp.fShuntR[0] = ByteSwapFloat(NV_Settings.fShuntR[0]);
	NVSetTemp.fShuntR[1] = ByteSwapFloat(NV_Settings.fShuntR[1]);
	NVSetTemp.fSenR[0] = ByteSwapFloat(NV_Settings.fSenR[0]);
	NVSetTemp.fSenR[1] = ByteSwapFloat(NV_Settings.fSenR[1]);
	NVSetTemp.fAutoFocusThreshold = ByteSwapFloat(NV_Settings.fAutoFocusThreshold);
	NVSetTemp.sLowWaitTimeout = ByteSwapWord(NV_Settings.sLowWaitTimeout);
	NVSetTemp.lHighWaitTimeout = ByteSwapDWord(NV_Settings.lHighWaitTimeout);
	NVSetTemp.sAirWaitTimeout = ByteSwapWord(NV_Settings.sAirWaitTimeout);
	NVSetTemp.lUHighWaitTimeout = ByteSwapDWord(NV_Settings.lUHighWaitTimeout);
	NVSetTemp.sMHighWaitTimeout = ByteSwapWord(NV_Settings.sMHighWaitTimeout);	//7200, HI to MH
	NVSetTemp.sIGWaitTimeout = ByteSwapWord(NV_Settings.sIGWaitTimeout);
	NVSetTemp.sVentWaitTimeout = ByteSwapWord(NV_Settings.sVentWaitTimeout);
	NVSetTemp.fLowChangeRate = ByteSwapFloat(NV_Settings.fLowChangeRate);
	NVSetTemp.fHighChangeRate = ByteSwapFloat(NV_Settings.fHighChangeRate);
	//
	NVSetTemp.fVAccRatio = ByteSwapFloat(NV_Settings.fVAccRatio); //real voltage = DAC voltage * fVAccratio
	NVSetTemp.fVBiasRatio = ByteSwapFloat(NV_Settings.fVBiasRatio);
	NVSetTemp.fFilaRatio = ByteSwapFloat(NV_Settings.fFilaRatio);	//watt = fV * fFilaRatio
	NVSetTemp.fFilaResistance = ByteSwapFloat(NV_Settings.fFilaResistance);
	NVSetTemp.nDelayLB = ByteSwapDWord(NV_Settings.nDelayLB);
	NVSetTemp.fAccMaxV = ByteSwapFloat(NV_Settings.fAccMaxV);
	NVSetTemp.fFilaMaxV = ByteSwapFloat(NV_Settings.fFilaMaxV);
	NVSetTemp.fBiasMaxV = ByteSwapFloat(NV_Settings.fBiasMaxV);
	NVSetTemp.fAccMonI = ByteSwapFloat(NV_Settings.fAccMonI);
	//
	NVSetTemp.fIGHITh = ByteSwapFloat(NV_Settings.fIGHITh);		//IG: HVAC threshold voltage
	NVSetTemp.fIGUHTh = ByteSwapFloat(NV_Settings.fIGUHTh);		//IG: UHVAC threshold voltage
	NVSetTemp.fPGLOTh = ByteSwapFloat(NV_Settings.fPGLOTh);		//pirani low vacuum threshold, 2.8
	NVSetTemp.fPGHITh = ByteSwapFloat(NV_Settings.fPGHITh);		//pirani high vacuum threshold, 1.0
	NVSetTemp.fIGLOTh = ByteSwapFloat(NV_Settings.fIGLOTh);		//IG: LOW threshold voltage
	NVSetTemp.fIGMHTh = ByteSwapFloat(NV_Settings.fIGMHTh);		//IG: medium high threshold voltage
	//
	NVSetTemp.fFocusMax = ByteSwapFloat(NV_Settings.fFocusMax);
	NVSetTemp.fFocusMin = ByteSwapFloat(NV_Settings.fFocusMin);
	NVSetTemp.lAFPixelNum = ByteSwapDWord(NV_Settings.lAFPixelNum); //AF=Auto Focus
	NVSetTemp.nAFLimitNum = ByteSwapDWord(NV_Settings.nAFLimitNum);
	NVSetTemp.nEdgeDiff = ByteSwapDWord(NV_Settings.nEdgeDiff); //used for auto focus calculation
	//
	NVSetTemp.fCoilRatio = ByteSwapFloat(NV_Settings.fCoilRatio);
	NVSetTemp.fIThRatio = ByteSwapFloat(NV_Settings.fIThRatio); //I threshold for mag switch
	NVSetTemp.fIMaxFactor = ByteSwapFloat(NV_Settings.fIMaxFactor);
	//
	for (i = 0; i < 2; i++)
		NVSetTemp.sVACNOKTh[i] = ByteSwapWord(NV_Settings.sVACNOKTh[i]);
	for (i = 0; i < 4; i++)
		NVSetTemp.nBaudrate[i] = ByteSwapDWord(NV_Settings.nBaudrate[i]);
	NVSetTemp.wWaitEOC = ByteSwapWord(NV_Settings.wWaitEOC);
	NVSetTemp.nSysClk = ByteSwapDWord(NV_Settings.nSysClk);
	NVSetTemp.nTurboType = ByteSwapDWord(NV_Settings.nTurboType);
	NVSetTemp.nDelayDescent = ByteSwapDWord(NV_Settings.nDelayDescent);
	NVSetTemp.nPixelDwellTime = ByteSwapDWord(NV_Settings.nPixelDwellTime);
	NVSetTemp.sIPLeakNum = ByteSwapWord(NV_Settings.sIPLeakNum);			//2
	for (i = 0; i < CHECK_VOLT_NUM; i++) {
		NVSetTemp.sIPLeakOnTime[i] = ByteSwapWord(NV_Settings.sIPLeakOnTime[i]);
		NVSetTemp.sIPLeakOffTime[i] = ByteSwapWord(NV_Settings.sIPLeakOffTime[i]);
		NVSetTemp.fIPLeakCheckV[i] = ByteSwapDWord(NV_Settings.fIPLeakCheckV[i]);
	}
	/*DWORD dwVerifyKey;
	int nXSkipNum;
	float fVACLevel[2]; 	//VAC0 and VAC1
	float fCalVideo[VADC_CH_NUM];	//video preamp output calibration
	float fPLAmp;
	float fPLPhase;
	WORD wPLAddrNum;		//addr number
	*/
	TxData(nType, (BYTE *)&NVSetTemp, sizeof(NV_SETTINGS));
}

void TxSysParam3(int nType)
{
	SYS_PARAM_3 SysParamTemp;
	int i, nSize;
	short int sV;
	//
	nSize = sizeof(SYS_PARAM_3);
	memmove(&SysParamTemp, &SysParam, nSize);
	SysParamTemp.bHeader1 = SYS3_HEADER_1;
	SysParamTemp.bHeader2 = SYS3_HEADER_2;
	SysParamTemp.bVersion = SYS_PARAM_VER;
	SysParamTemp.fDefSenR = ByteSwapFloat(SysParam.fDefSenR);
	for (i = 0; i < VAC_GAUGE_NUM_MAX; i++)
		SysParamTemp.sGaugeStatus[i] = (short)ByteSwapWord(SysParam.sGaugeStatus[i]);
	SysParam.fHVBiasV = GetHVBiasCurrentV(); //unit voltage
	SysParamTemp.fHVBiasV = ByteSwapFloat(SysParam.fHVBiasV);
	SysParamTemp.fHVBiasI = ByteSwapFloat(SysParam.fHVBiasI);
	SysParamTemp.lHVOnTime = (long)ByteSwapDWord(SysParam.lHVOnTime);
	SysParamTemp.fMagNow = ByteSwapFloat(SysParam.fMagNow);
	SysParam.fTempTP = (float)GetTurboMotorTemp(0); //nUpdate=0
	SysParamTemp.fTempTP = ByteSwapFloat(SysParam.fTempTP);
	SysParam.fTempIN = GetTemperature(TEMP_IN_CASE);
	SysParamTemp.fTempIN = ByteSwapFloat(SysParam.fTempIN);
	SysParam.fTempHS[0] = GetTemperature(TEMP_COIL_HS);
	SysParam.fTempHS[1] = GetTemperature(TEMP_POWER_HS);
	SysParam.fTempHS[2] = GetTemperature(TEMP_TEMPE3);
	for (i = 0; i < 3; i++)
		SysParamTemp.fTempHS[i] = ByteSwapFloat(SysParam.fTempHS[i]);
	SysParamTemp.fTimeXAsc = ByteSwapFloat(SysParam.fTimeXAsc);
	SysParamTemp.fTimeX = ByteSwapFloat(SysParam.fTimeX);
	SysParam.sTPSpeed = (short)GetTurboRotationSpeed(0);
	SysParamTemp.sTPSpeed = (short)ByteSwapWord(SysParam.sTPSpeed);
	SysParamTemp.wXLineDelay = ByteSwapWord(SysParam.wXLineDelay);
	SysParam.wDI = GetDigitalInput(0);
	SysParamTemp.wDI = ByteSwapWord(SysParam.wDI);
	SysParam.wDO[0] = wSCAN_DO;
	SysParamTemp.wDO[0] = ByteSwapWord(SysParam.wDO[0]);
	SysParam.wDO[1] = wIO_DO;
	SysParamTemp.wDO[1] = ByteSwapWord(SysParam.wDO[1]);
	for (i = 0; i < ADC_CH_NUM_MAX; i++)
		SysParamTemp.fADC[i] = ByteSwapFloat(SysParam.fADC[i]);
	//
	for (i = 0;  i < SYS_VADC_NUM; i++) //video ADC
		SysParamTemp.fVADC[i] = ByteSwapFloat(SysParam.fVADC[i]);
	SysParamTemp.fRotateAngle = ByteSwapFloat(SysParam.fRotateAngle);
	SysParamTemp.fObjAdjAngle = ByteSwapFloat(SysParam.fObjAdjAngle);
	SysParamTemp.sVacTimeout = (short)ByteSwapWord(SysParam.sVacTimeout);
	SysParamTemp.sTPGVTimeout = (short)ByteSwapWord(SysParam.sTPGVTimeout);
	SysParamTemp.sIGTimeout = (short)ByteSwapWord(SysParam.sIGTimeout);
	//
	SysParamTemp.bFilaOK = SysParam.bFilaOK;
	SysParamTemp.bFilaWDT = SysParam.bFilaWDT;
	SysParamTemp.fFilaI = ByteSwapFloat(SysParam.fFilaI);
	SysParamTemp.fFilaV = ByteSwapFloat(SysParam.fFilaV);
	SysParamTemp.fBiasV = ByteSwapFloat(SysParam.fBiasV);	//volts
	SysParamTemp.bCusHVON = SysParam.bCusHVON;
	//
	SysParamTemp.fIpHV = ByteSwapFloat(SysParam.fIpHV);
	SysParamTemp.fIpAmp = ByteSwapFloat(SysParam.fIpAmp);
	SysParamTemp.bMotorMove = SysParam.bMotorMove;
	SysParamTemp.sMotorTime = (short)ByteSwapWord(SysParam.sMotorTime);
	SysParamTemp.bIPON = SysParam.bIPON;
	TxData(nType, (BYTE *)&SysParamTemp, sizeof(SYS_PARAM_3));
	sV = nOverscan;
	SysParamTemp.sOverscan = (short)ByteSwapWord(sV);
}
//
void TxSysParam2(int nType)
{
	SYS_PARAM_2 SysParamTemp2;
	int i, nSize;
	//
	nSize = sizeof(SYS_PARAM_2);
	memmove(&SysParamTemp2, &SysParam2, nSize);
	SysParamTemp2.bHeader1 = SYS2_HEADER_1;
	SysParamTemp2.bHeader2 = SYS2_HEADER_2;
	SysParamTemp2.bVersion = SYS2_PARAM_VER;
	SysParamTemp2.fFilaI = ByteSwapFloat(SysParam2.fFilaI);
	SysParamTemp2.fFilaV = ByteSwapFloat(SysParam2.fFilaV);
	SysParamTemp2.fHVBiasV = ByteSwapFloat(SysParam2.fHVBiasV);
	SysParamTemp2.fHVBiasI = ByteSwapFloat(SysParam2.fHVBiasI);
	SysParamTemp2.fBiasV = ByteSwapFloat(SysParam2.fBiasV);
	for (i = 0; i < 8; i++)
		SysParamTemp2.fADCV[i] = ByteSwapFloat(SysParam2.fADCV[i]);
	SysParamTemp2.fTempe[0] = ByteSwapFloat(SysParam2.fTempe[0]);
	SysParamTemp2.fTempe[1] = ByteSwapFloat(SysParam2.fTempe[1]);
	SysParamTemp2.fAccVoltage = ByteSwapFloat(SysParam2.fAccVoltage); //unit kV
	SysParamTemp2.fAccCurrent = ByteSwapFloat(SysParam2.fAccCurrent); //uA
	TxData(nType, (BYTE *)&SysParamTemp2, sizeof(SYS_PARAM_2));
}
//
void TxData(int nConnType, BYTE *pData, int nLen)
{
	int i, nFdNdx = 0;
	char szBuffer[32];
	int nRet;
	int nPort = UDP_DATA_PORT;
	//
	if (nConnType == CONN_TCP) {
		//remove '\n' and '\r' for UDP transfer
		//for ASCII code only
		if (nASCIICode == 1)
		{
			if (pData[nLen - 1] == '\n')
				nLen--;
			if (pData[nLen - 1] == '\r')
				nLen--;
			nFdNdx = 0; //command port
		}
		else
			nFdNdx = 1; //data port
		if (SysParam.bTcpConnected == 1) { //TCP connected
			//write() is non-blocking function
			//writeall() is blocking function
			i = 0;
			do {
				nRet = writeall(fdSet[nFdNdx].fdConn, (char *)pData, nLen);
				if (nRet == nLen) //try three time
					break;
				i++;
			} while (i < 3);
			if ((bDebug == 4) && (i != 0)) { //write again
				printf("writeall() error, %d, %d\n", nLen, nRet);
			}
		}
		return;
	}
	else if (nConnType == CONN_UDP) {
		if (UdpClient.iaSource == 0) //NO UDP connection
			return;
		//remove '\n' and '\r' for UDP transfer
		//for ASCII code only
		if (nASCIICode == 1)
		{
			if (pData[nLen - 1] == '\n')
				nLen--;
			if (pData[nLen - 1] == '\r')
				nLen--;
			nPort = UDP_CMD_PORT;
		}
		UDPPacket UdpTxPkt;
		//UdpTxPkt.SetSourcePort( UdpClient.nDestPort );
		//UdpTxPkt.SetDestinationPort( UdpClient.nSrcPort );
		//
		UdpTxPkt.SetSourcePort( UDP_TALK_PORT );
		if (NV_Settings.bUDPPort == 1)
			UdpTxPkt.SetDestinationPort( UdpClient.nSrcPort );
		else
			UdpTxPkt.SetDestinationPort( nPort );  //UDP_CMD_PORT or UDP_DATA_PORT
		//UdpTxPkt.SetDataSize(nLen);
		UdpTxPkt.AddData(pData, nLen);
		UdpTxPkt.Send(UdpClient.iaSource);
		//hdlTxUDP = CreateTxUdpSocket(UdpClient.iaSource, nPort, UDP_TALK_PORT);
		//int sendto(int sock, PBYTE what_to_send, int len_to_send,IPADDR to_addr,  WORD remote_port)
		//sendto(hdlTxUDP, pData, nLen, UdpClient.iaSource, nPort);
		return;
	}
	else if (nConnType == CONN_UART0)
	{
		if (nASCIICode == 1)
			printf((char *)pData);
		else {
			for (i = 0; i < nLen; i++) {
				if ((i % 32) == 0) printf("%04d:", i);
				printf("%02X", pData[i]);
				if ((i % 32) == 15) printf("-");
				else if ((i % 32) == 31) printf("\n");
			}
			printf("\n");
		}
		return;
	}
	else if (nConnType == CONN_UART0_BIN) //debug used, ASCII binary code
	{
		for (i = 0; i < nLen; i+=2) {
			sprintf(szBuffer, "%02X%02X ", pData[i + 1], pData[i]);
			iprintf(szBuffer);
			//if ((i > 0) && (i % 64 == 0) && (i != nLen - 1))
			//	iprintf("\n");
		}
		return;
	}
}

void TxString(int nConnType, char *pStr)
{
	nASCIICode = 1;
	TxData(nConnType, (BYTE *)pStr, strlen(pStr));
	nASCIICode = 0;
}

void ResetQueIndex(void)
{
	int nQueNum;
	//nQueStart = (NV_Settings.bOnePkt == 1) ? sizeof(ONE_PKT_HEADER) : sizeof(DATA_HEADER);
	//1 Mbytes = 500 kpixels = 800x600
	nQueNum = (UDP_QUEUE_SIZE / nDataBytesPerLine) - 1;
	nQueSize = nDataBytesPerLine * nQueNum;
	nQueStart = 0;
	//
	nBytesIn = nQueStart;
	nBytesOut = nQueStart;
}
/*-------------------------------------------------------------------
 * This task will wait for incoming UDP packets from PC and process them.
 -------------------------------------------------------------------*/
void UdpReaderTask( void *pd )
{
	//int port = ( int ) pd;
	WORD nLen = 0;
	//iprintf( "UdpReaderTask monitoring port %d\n", port );
	iprintf( "UdpReaderTask monitor on port %d,%d\r\n", UDP_CMD_PORT, UDP_DATA_PORT);

	// Create FIFO to store incoming packets and initialize it
	OS_FIFO fifo_cmd;
	OSFifoInit( &fifo_cmd );
	OS_FIFO fifo_data;
	OSFifoInit( &fifo_data );

	// Register to listen for UDP packets on port number 'port'
	//RegisterUDPFifo( port, &fifo );
	RegisterUDPFifo( UDP_CMD_PORT, &fifo_cmd );
	RegisterUDPFifo( UDP_DATA_PORT, &fifo_data );

	while ( 1 )
	{
		// We construct a UDP packet object using the FIFO.
		// This constructor will only return when we have received a packet
		UDPPacket upkt( &fifo_cmd, 1 );  /* Replace this 0 with a tick count to have a time out delay */
		// Did we get a valid packet or just time out?
		if ( upkt.Validate() )	{
			nLen = upkt.GetDataSize();
			memmove(pRxBuffer, upkt.GetDataBuffer(), nLen);
			pRxBuffer[nLen] = 0;
			//iprintf( "\nReceived a UDP packet with %d bytes from :", ( int ) len );
			UdpClient.iaSource = upkt.GetSourceAddress();
			//iaDestination = upkt.GetDestinationAddress();
			UdpClient.nSrcPort = upkt.GetSourcePort();
			UdpClient.nDestPort = upkt.GetDestinationPort(); //*/
			DecipherMultipleCommand(CONN_UDP, (char *)pRxBuffer);
			memset(pRxBuffer, 0, RX_BUFSIZE);
		}
		UDPPacket upkt_data( &fifo_data, 1 );
		if ( upkt_data.Validate() ) {
			nLen = upkt_data.GetDataSize();
			memmove(pRxBuffer, upkt_data.GetDataBuffer(), nLen);
			pRxBuffer[nLen] = 0;
			UdpClient.iaSource = upkt_data.GetSourceAddress();
			//iaDestination = upkt.GetDestinationAddress();
			UdpClient.nSrcPort = upkt_data.GetSourcePort();
			UdpClient.nDestPort = upkt_data.GetDestinationPort(); //*/
			//SET_PARAM or NV_SETTINGS
			DecipherDataPacket(CONN_UDP, pRxBuffer, nLen);
		}
		OSTimeDly(1); //wait 50 msec
	}
}

void TcpListenTask(void *pd)
{
	//int port = ( int ) pd;
	int i, nLen;
	int fdListen[2];
	int nDestPort[2] = {TCP_CMD_PORT, TCP_DATA_PORT};
	IPADDR iaClient;
	WORD   wClientPort;
	int fdA;
	//
	//nDestPort = TCP_CMD_PORT; //5064 (S4)
	fdListen[0] = listen( 0, nDestPort[0], 5 ); //INADDR_ANY = 0
	fdListen[1] = listen( 0, nDestPort[1], 5 ); //INADDR_ANY = 0
	iprintf( "TCPListenTask Listening on port %d,%d\r\n", TCP_CMD_PORT, TCP_DATA_PORT);
	while ( 1 )
	{
		// Declare file descriptor sets for select()
		fd_set read_fds;
		fd_set error_fds;

		// Init the fd sets
		FD_ZERO( &read_fds );
		FD_ZERO( &error_fds );

		// Configure the fd sets so select() knows what to process. In
		// this case any fd data to be read, or an error.
		for (i = 0; i < NFDS; i++) {
			if ( fdSet[i].fdConn )
			{
				FD_SET( fdSet[i].fdConn, &read_fds );
				FD_SET( fdSet[i].fdConn, &error_fds );
			}
			// select() should also process the listen fd
			FD_SET( fdListen[i], &read_fds );
			FD_SET( fdListen[i], &error_fds );
		}

		/*
		select() will block until any fd has data to be read, or
		has an error. When select() returns, read_fds and/or
		error_fds variables will have been modified to reflect
		the events.
		*/
		select( FD_SETSIZE, &read_fds, ( fd_set * )0, &error_fds, 0 ); //nfds,*read_fds,*write_fds,*error_fds,timeout

		for (i = 0; i < NFDS; i++) {
			// If the listen fd has a connection request, accept it.
			if ( FD_ISSET( fdListen[i], &read_fds ) )
			{
				fdA = accept( fdListen[i], &iaClient, &wClientPort, 0 ); //timeout=forever

				// If accept() succeeded
				if ( fdA > 0 && (fdSet[i].fdConn == 0))
				{
					fdSet[i].fdConn = fdA;
					fdSet[i].iaClient = iaClient;
					fdSet[i].wClientPort = wClientPort;
					SysParam.bTcpConnected = 1;
					#if DEBUG_TCP == 1
					iprintf( "connection on fd[%d] = %d, ", i, fdA );
					ShowIP(iaClient);
					iprintf(":%d\n", wClientPort);
					#endif
					//fdA = 0;
				}
				// If no array positions are open, close the connection
				else //if ( fdA )
				{
					close( fdA );
					#if DEBUG_TCP == 1
					iprintf("socket[%d] full\r\n", i);
					#endif
				}
			}
			// If the listen fd has an error, close it and reopen
			if ( FD_ISSET( fdListen[i], &error_fds ) )
			{
				close( fdListen[i] );
				fdListen[i] = listen( 0, nDestPort[i], 5 ); //INADDR_ANY = 0
				#if DEBUG_TCP == 1
				iprintf( "re-listen on [%d]\r\n", i );
				#endif
			}

			// Process each fd array element and check it against read_fds
			// and error_fds.
			if ( fdSet[i].fdConn != 0) {
				// Check for data to be read
				if ( FD_ISSET( fdSet[i].fdConn, &read_fds ) )
				{
					while (dataavail(fdSet[i].fdConn))
					{	//480 is a multiple of 32 and 48
						nLen = read( fdSet[i].fdConn, (char *)pRxBuffer, RX_BUFSIZE - 1);	//REQ_BUF_SIZE );
						pRxBuffer[nLen] = 0;
						if (i == 0) //command port
							DecipherMultipleCommand(CONN_TCP, (char *)pRxBuffer);
						else if (i == 1)//data port
							DecipherDataPacket(CONN_TCP, pRxBuffer, nLen);
						memset(pRxBuffer, 0, RX_BUFSIZE);
					} //while (dataavail(fdSet[i].fdConn))
				} //if ( FD_ISSET( fdSet[i].fdConn, &read_fds ) )

				// Check for errors
				if ( FD_ISSET( fdSet[i].fdConn, &error_fds ) )
				{
					close( fdSet[0].fdConn );
					fdSet[0].fdConn = 0;
					close( fdSet[1].fdConn );
					fdSet[1].fdConn = 0;
					SysParam.bTcpConnected = 0;
					#if DEBUG_TCP == 1
					iprintf( "close connection fd[0], fd[1]\r\n");
					#endif
				}
			} //if ( fdSet[i].fdConn != 0)
		} //for (i = 0; i < NFDS; i++)
		OSTimeDly(1); //wait 50 msec
	} //while (1)
}
/*-------------------------------------------------------------------
 * This task will monitor scanning process
!important
The pipeline architecture allows a greater throughput rate at the
expense of pipeline delay or latency. This means that while the
converter is capable of capturing a new input sample every clock
cycle, it actually takes three clock cycles for the conversion to be
fully processed and appear at the output. This latency is not a
concern in most applications.
 -------------------------------------------------------------------*/
//
void X_Descent()
{
	int i, nDelayNdx;
	pwFpga[SET_DACX_F] = wDACDX[0];
	for (nDelayNdx = 0; nDelayNdx < nDelayIDec; nDelayNdx++) //DELAY:1:nDelayIDec:nPixelDwellTime
		delay_void();
	for (i = 0; i < nScanPixelNumX + nOverscan; i += 3) {
		pwFpga[SET_DACX_F] = wDACDX[i];
		for (nDelayNdx = 0; nDelayNdx < nDelayIDec; nDelayNdx++) //DELAY:1:nDelayIDec:nPixelDwellTime
			delay_void();
	}
	pwFpga[SET_DACX_F] = wDACX[0]; //last point
	for (nDelayNdx = 0; nDelayNdx < nDelayIDec; nDelayNdx++) //DELAY:1:nDelayIDec:nPixelDwellTime
		delay_void();
}
//
void Y_Descent()
{
	int i, nDelayNdx;
	pwFpga[SET_DACY_F] = wDACDY[0];
	for (nDelayNdx = 0; nDelayNdx < nDelayIDec; nDelayNdx++) //DELAY:1:nDelayIDec:nPixelDwellTime
		delay_void();
	for (i = 0; i < nScanPixelNumY; i += 3) {
		pwFpga[SET_DACY_F] = wDACDY[i];
		for (nDelayNdx = 0; nDelayNdx < nDelayIDec; nDelayNdx++) //DELAY:1:nDelayIDec:nPixelDwellTime
			delay_void();
	}
	pwFpga[SET_DACY_F] = wDACY[0]; //last point
	for (nDelayNdx = 0; nDelayNdx < nDelayIDec; nDelayNdx++) //DELAY:1:nDelayIDec:nPixelDwellTime
		delay_void();
}
//
void ScanMonitorTask(void * pd)
{
	int nBytes;
	//int nSetObj = 0;
	int nConnType = CONN_UDP;
	int i, nNdx = 0, nV;
	char szValue[64];
	char szBuffer[1024];
	float fV;
	//
	//DWORD dwV;
	DWORD dwFocusGradient[AF_LIMIT_NUM];
	float fFocusLimitSet[AF_LIMIT_NUM];
	int nFocusLimitSet[AF_LIMIT_NUM];
	//
	int nAFLimitNum = 4;
	BYTE bAFFirstCheck = 0;
	int nAFLineNum = 20;
	int nSum = 0;
	DWORD dwMax;
	int nMaxNdx = 0;
	int nTick = 0;
	float fFMax, fFMin;
	float fFSet, fFStep;
	float fAFStart = 0, fAFEnd = 0;
	//
	while (1)
	{
		if (NV_Settings.bEnableWDT == 1) //enable watch dog timer
			ResetWDT();
		//SysParam.bScanning
		//0: idle
		//1: SEM scan
		//2: EDS operation (HV on, vacuum ready)
		//3: auto focus mode
		//4: point mode
		//5: blanking
		if (bBlanking == 1) {
			nIdleTimeout = NV_Settings.nIdleTimeout; //reset shutdown timeout counter
			BlankingProcess(1);
		}
		else if (bBlanking == 3) { //fix point
			nIdleTimeout = NV_Settings.nIdleTimeout; //reset shutdown timeout counter
			BlankingProcess(3);
		}
		//--------------------------------------------------------------
		//OP_POINT: deflector control by POINTSET command, EEDS_ON=0
		//OP_EDS: deflector control by external waveform, EEDS_ON=1
		//--------------------------------------------------------------
		else if ((SysParam.bScanning == OP_POINT) || (SysParam.bScanning == OP_EDS)) {
			nIdleTimeout = NV_Settings.nIdleTimeout; //reset shutdown timeout counter
			//FastScan(nConnType); //only fast scan
		}
		else if (SysParam.bScanning == OP_ENABLE_CL) {
			nIdleTimeout = NV_Settings.nIdleTimeout; //reset shutdown timeout counter
			if (bScanAbort != 0) {	//don't sent histogram
				sprintf(szBuffer, "SCAN ABORTED...%d,%d\n", bScanAbort, bStopType);
				ScanOverProcess(szBuffer);
			}
		}
		//--------------------------------------------------------------
		// auto focus process controlled by software
		//--------------------------------------------------------------
		else if (SysParam.bScanning == OP_AUTO_FOCUS_SW) { //FOCUS:12:1
			nIdleTimeout = NV_Settings.nIdleTimeout; //reset VOP shutdown timeout counter
			if (bAutoFocusStart == 1) { //FOCUS:12:1
				nAFLineNum = NV_Settings.lAFPixelNum / nScanPixelNumX; //row number
				//
				if (nAFLineNum < 24) nAFLineNum = 24; //at least 24 lines
				else if (nAFLineNum > nScanPixelNumY / 2) nAFLineNum = nScanPixelNumY / 2;
				//
				nV = nScanPixelNumY / nAFLineNum;
				pwFpga[SET_Y_NDX_INC] = nV;
				nYNdxInc = nV;
				bAutoFocusStart = 0;
				bScanAbort = 0;
				bScanPause = 0;
				bScanRestart = 0;
				InitFastScan();
				SetDeflectorFineV(DAC_CH_DEFX_F, 0);
				SetDeflectorFineV(DAC_CH_DEFY_F, 0);
			}
			else if (bAutoFocusStart == 2) { //FOCUS:12:2
				//
				SetDeflectorFineV(DAC_CH_DEFX_F, fInitialX);
				fV = fInitialY * (float)nAFLineNum / (nScanPixelNumY / 2);
				SetDeflectorFineV(DAC_CH_DEFY_F, fV);
				//
				ClearGradientX();
				SetImageInit(); //reset wZMax, wZMin in FPGA
				ResetQueIndex();
				//
				bFirstPacket = 1;
				bFinalPacket = 0;
				//
				FastScan(nConnType); //bScanning
				//TxIdle(0); //nWait=0
				//
				SetDeflectorFineV(DAC_CH_DEFX_F, 0);
				SetDeflectorFineV(DAC_CH_DEFY_F, 0);
				//
				bAutoFocusStart = 0;
				dwGradient = GetGradient();
				sprintf(szBuffer, "EXF_OVER,%.4f,%ld\r\n", fFocus, dwGradient);
				TxString(nConnType, szBuffer);
				if (nConnType != CONN_UART0)
					printf(szBuffer);
			}
		}
		//----------------------------------------------------
		// auto focus process controlled by firmware
		//----------------------------------------------------
		else if (SysParam.bScanning == OP_AUTO_FOCUS) { //FOCUS:1:1
			if (bAutoFocusStart != 0) { //1 or 2, first execution
				if (bDebug == 7) {
					fAFStart = (float)lSysclkTick / 10; //unit second
				}
				//
				bAFFirstCheck = 1;
				nAFLimitNum = NV_Settings.nAFLimitNum;
				for (i = 0; i < nAFLimitNum; i++) {
					nFocusLimitSet[i] = 0;
					dwFocusGradient[i] = 0;
				}
				if (bAutoFocusStart == 1) { //full range search
					for (i = 0; i < nAFLimitNum; i++) { //from 4 to 96
						fFocusLimitSet[i] = NV_Settings.fFocusMin + (NV_Settings.fFocusMax - NV_Settings.fFocusMin) * (float)i / (nAFLimitNum - 1);
					}
				}
				//fast auto focus, search around current focus
				else if (bAutoFocusStart == 2) {
					nAFLimitNum = 8;
					fFMax = fFocus + fFocusRange;
					if (fFMax > 96.0) fFMax = 96;
					fFMin = fFocus - fFocusRange;
					if (fFMin < 4.0) fFMin = 4;
					fFStep = (fFMax - fFMin) / (nAFLimitNum - 1);
					for (i = 0; i < nAFLimitNum; i++) {
						fFocusLimitSet[i] = fFMin + (float)i * fFStep;
					}
				}
				bAutoFocusStart = 0;
				//
				nAFLineNum = NV_Settings.lAFPixelNum / nScanPixelNumX; //row number
				if (nAFLineNum < 24) nAFLineNum = 24; //at least 24 lines
				else if (nAFLineNum > nScanPixelNumY / 2) nAFLineNum = nScanPixelNumY / 2;
				//
				nV = nScanPixelNumY / nAFLineNum;
				pwFpga[SET_Y_NDX_INC] = nV;
				nYNdxInc = nV;
				//
				SetDeflectorFineV(DAC_CH_DEFX_F, fInitialX);
				fV = fInitialY * (float)nAFLineNum / (nScanPixelNumY / 2);
				SetDeflectorFineV(DAC_CH_DEFY_F, fV);
				//OSTimeDly(1); //wait until current is stable
			} //if (bAutoFocusStart != 0)
			//-----------------------------------------
			// find the foci not set yet
			//-----------------------------------------
			for (i = 0; i < nAFLimitNum; i++) {
				if (nFocusLimitSet[i] == 0) { //up limit
					SetFocus(fFocusLimitSet[i]);
					nNdx = i; //nNdx point to the data where gradient is not calculated yet
					break; //break for loop
				}
			}
			if (NV_Settings.bObjMode == 1)
				SetObjToMin(1);
			OSTimeDly(1); //wait until focus is stable
			if (NV_Settings.bObjMode == 1)
				SetObjToMin(0);
			//
			ClearGradientX();
			SetImageInit(); //reset wZMax, wZMin in FPGA
			ResetQueIndex();
			//
			bFirstPacket = 0;
			bFinalPacket = 0;
			nIdleTimeout = NV_Settings.nIdleTimeout; //reset VOP shutdown timeout counter
			//fast scan from y_start to y_end
			FastScan(nConnType);
			//TxIdle(0); //nWait=0
			//flag set after process complete
			nFocusLimitSet[nNdx] = 1;
			//get gradient
			dwGradient = GetGradient();
			dwFocusGradient[nNdx] = dwGradient;
			nSum = 0;
			for (i = 0; i < nAFLimitNum; i++)
				nSum += nFocusLimitSet[i];
			if (bDebug == 7) { //debug auto focus process
				printf("NDX=%d,SUM=%d,", nNdx, nSum);
				printf("G=0X%08lX_%ld_\n", dwGradient, dwGradient);
				for (i = 0; i < nAFLimitNum; i++) {
					szBuffer[0] = 0;
					sprintf(szValue, "(%d)%.3f", i, fFocusLimitSet[i]);
					strcat(szBuffer, szValue);
					sprintf(szValue, ",%ld", (long)dwFocusGradient[i]);
					strcat(szBuffer, szValue);
					printf(szBuffer);
				}
				printf("\n");
				//OSTimeDly(1); //MUST!
			}
			if (nSum == nAFLimitNum) { //all scan points are searched and calculated
				if (bAFFirstCheck == 1) { //full range search
					bAFFirstCheck = 0;
					dwMax = 0;
					nMaxNdx = 0;
					for (i = 0; i < nAFLimitNum; i++) {
						if (dwFocusGradient[i] > dwMax) {
							dwMax = dwFocusGradient[i];
							nMaxNdx = i; //find the max point
						}
					}
					if (nMaxNdx <= 2) {
						nNdx = 2; //left limit
					}
					else if (nMaxNdx >= nAFLimitNum - 2) {
						nNdx = nAFLimitNum - 2;
					}
					else {
						nNdx = nMaxNdx;
					}
					fFStep = (NV_Settings.fFocusMax - NV_Settings.fFocusMin) / (NV_Settings.nAFLimitNum - 1);
					fFStep *= 1.5;
					fFSet = fFocusLimitSet[nNdx];
					if (bDebug == 7) {
						printf("MAX=%ld, POS=%d\n", dwMax, nNdx);
					}
					for (i = 0; i < 4; i++) {
						/*if (i == 0) {
							dwFocusGradient[i] = dwFocusGradient[nNdx - 1];
						}
						else if (i == 3) {
							dwFocusGradient[i] = dwFocusGradient[nNdx + 1];
						}
						else {
							dwFocusGradient[i] = 0;
						}*/
						dwFocusGradient[i] = 0;
						nFocusLimitSet[i] = 0;
						fFocusLimitSet[i] = fFSet - fFStep + (float)i * 2 * fFStep / 3;
					}
					nAFLimitNum = 4;
				}
				else if (dwFocusGradient[0]+dwFocusGradient[1] < dwFocusGradient[2]+dwFocusGradient[3]) {
					//right side is larger
					fFocusLimitSet[0] = fFocusLimitSet[1];
					dwFocusGradient[0] = dwFocusGradient[1];
					fFocusLimitSet[1] = fFocusLimitSet[0] * 2 / 3 + fFocusLimitSet[3] / 3;
					fFocusLimitSet[2] = fFocusLimitSet[0] / 3 + fFocusLimitSet[3] * 2 / 3;
					dwFocusGradient[1] = 0;
					dwFocusGradient[2] = 0;
					nFocusLimitSet[1] = 0;
					nFocusLimitSet[2] = 0; //
					if (bDebug == 7) printf("shift to high side\n");
				}
				else {
					//left side is larger
					fFocusLimitSet[3] = fFocusLimitSet[2];
					dwFocusGradient[3] = dwFocusGradient[2];
					fFocusLimitSet[1] = fFocusLimitSet[0] * 2 / 3 + fFocusLimitSet[3] / 3;
					fFocusLimitSet[2] = fFocusLimitSet[0] / 3 + fFocusLimitSet[3] * 2 / 3;
					dwFocusGradient[1] = 0;
					dwFocusGradient[2] = 0;
					nFocusLimitSet[1] = 0; //
					nFocusLimitSet[2] = 0;
					if (bDebug == 7) printf("shift to low side\n");
				}
				fV = (fFocusLimitSet[1] + fFocusLimitSet[2]) / 2;
				sprintf(szBuffer, "FOCUS=%.3f", fV); //update focus value
				TxString(nConnType, szBuffer);
			}
			if (fFocusLimitSet[3] - fFocusLimitSet[0] < NV_Settings.fAutoFocusThreshold) {
				//pwFpga[SET_X_START] = 0;
				//pwFpga[SET_X_END] = (WORD)nPixelX;
				//pwFpga[SET_Y_START] = 0;
				//pwFpga[SET_Y_END] = (WORD)nScanPixelNumY;
				fV = (fFocusLimitSet[1] + fFocusLimitSet[2]) / 2;
				sprintf(szBuffer, "FOCUS:0:%.3f", fV);
				if (bDebug == 7) printf("%s\n", szBuffer);
				//
				SysParam.bScanning = OP_SEM_SCAN; //back to SEM scan mode
				sprintf(szBuffer, "FOCUS=%.3f", fV); //update focus value
				TxString(nConnType, szBuffer);
				SetFocus(fV);
				if (bDebug == 7) {
					fAFEnd = (float)lSysclkTick / 10;
					printf("AFT=%.1f s\n", fAFEnd - fAFStart);
				}
				if (bPLSync == 1)
					EnablePLSync(1);
				EnableAutoFocus(0); //FPGA leave auto focus mode
			}
		}
		else if (SysParam.bScanning == OP_SEM_SCAN) { //area scan
			//if nScanInterval == 0, device will scan as fast as possible
			//if nScanImageNum == 0, device will keep running forever, until bScanAbort=TRUE
			// set to left-bottom corner, starting point of scan
			nConnType = (SysParam.bTcpConnected == 1) ? CONN_TCP : CONN_UDP;
			//delay some time (10 msec) until current is stable
			//sprintf((char *)bBuffer, "SCAN %d/%d\n", nScanImageNdx + 1, nScanImageNum);
			//TxString(CONN_UART0, (char *)bBuffer); //1 char = 100 usec
			//dwImageAddr = (bRegionScan == 1) ? (nScanPixelNumX * nScanPixelNumY * (0.5 + fRegion/2)) : 0;
			//
			if (nScanImageNdx == 0) { //nScanImageNdx == 0, first image
				pwFpga[SET_SHADOW_MODE] = nShadowMode;	//four_vadc=0
				nReadyToErrorNum = nVacErrToleranceNum[1];
			}
			//
			ClearHistogram();
			ClearGradientX();
			//
			wZMax = 0;
			wZMin = 0xFFFF;
			bACK = 0;
			nSimuShift++;
			nIdleTimeout = NV_Settings.nIdleTimeout; //reset VOP shutdown timeout counter
			//
			bFirstPacket = 1;
			bFinalPacket = 0;
			if (bScanPause == 1) {
				//fObjVOP = fObjV;
				//SetObjectiveCoarseV(OBJ_ON_IMIN_V); //set objective current low
				do { //freeze
					if (bScanAbort == 1) break;
					SetDeflectorFineV(DAC_CH_DEFX_F, 0);
					SetDeflectorFineV(DAC_CH_DEFY_F, 0);
					OSTimeDly(1); //wait 50 msec
				} while (bScanPause == 1);
				//SetObjectiveCoarseV(fObjVOP);
			}
			// 1 mm -> 0.15 A, 0.016 mm -> 0.0024 A
			if (nScanImageNdx == 0) {
				InitFastScan(); //set pixel number
				if (bDebug == 11) printf("INITFASTSCAN,%d\n", nScanImageNdx);
			}
			SetImageInit(); //reset wZMax, wZMin in FPGA
			//
			ResetQueIndex(); //process one packet (bOnewPkt)
			SetDeflectorFineV(DAC_CH_DEFX_F, fInitialX);
			if (bRegionScan == 0) {
				SetDeflectorFineV(DAC_CH_DEFY_F, fInitialY);
			}
			else {
				fV = fInitialY * (1.0 - fRegionStart) + fFinalY * fRegionStart;
				SetDeflectorFineV(DAC_CH_DEFY_F, fV);
			}
			if (NV_Settings.nDelayLB > 0) { //keep at left-bottom point to overcome hysteresis
				if (NV_Settings.bObjMode == 1) {
					SetObjToMin(1);
					for (i = 0; i < NV_Settings.nDelayLB - 1; i++) {
						if (bScanAbort == 1) break;
						OSTimeDly(1); //nDelay at left bottom point
					}
					SetObjToMin(0); //save power
					OSTimeDly(1);
				}
				else { //bObjMode == 0
					for (i = 0; i < NV_Settings.nDelayLB; i++) {
						if (bScanAbort == 1) break;
						OSTimeDly(1); //nDelayLB at left bottom point, overcome hysteresis
					}
				}
			}
			//--------------------------------------------------------------
			if (bDebug == 11) printf("FASTSCAN() STARTS.\n");
			FastScan(nConnType); //only fast scan
			if (bDebug == 11) printf("FASTSCAN() ENDS.\n");
			//--------------------------------------------------------------
			//
			if (bDebug == 5) { //debug auto focus process
				printf("F=%.3f,G=0X%08lX_%ld_\n", fFocus, dwGradient, dwGradient);
			}
			//bACK = 2; //wait ScanResume() command
			//
			for (i = 0; i < 10; i++) { //wait 500 msec
				if (bScanRestart != 0) break;
				if (bScanAbort != 0) break;
				if (bACK == 0) break; //ScanResume() command received
				OSTimeDly(1); //MUST! for TCP, 500 msec
			}
			if (bScanRestart == 1) {
				bScanRestart = 0;
				nScanImageNdx = 0;
			}
			else
				nScanImageNdx++;
			if (bDebug == 11) printf("%d/%d\n", nScanImageNdx, nScanImageNum);
		} //if (SysParam.bScanning == OP_SEM_SCAN)
		//if (bRegionScan == 1)
		//	OSTimeDly(1);
		if (bScanAbort != 0)
		{	//don't sent histogram
			sprintf(szBuffer, "SCAN ABORTED...%d,%d\n", bScanAbort, bStopType);
			ScanOverProcess(szBuffer);
		}
		TxIdle(0);
		if (SysParam.bScanning == OP_SEM_SCAN) { //send histogram only when OP_SEM_SCAN
			if ((bEnableHisto == 1) && (bScanAbort == 0)) {	//send histogram data
				OSTimeDly(1); //give Windows AP some times to process image data
				//send histogram to PC
				if (bFastScan == 0)
					nBytes = GetHistogramPacket(pbTxBuffer[0]);
				else
					nBytes = GetHistogramPacketFromFPGA(pbTxBuffer[0]);
				TxData(nConnType, pbTxBuffer[0], nBytes);
			}
			/*if ((nScanImageNdx >= nScanImageNum) && (nScanImageNum > 0)) {
				//nScanImageNum = 0, endless scan
				ScanOverProcess("SCAN OVER...\n");
				OSTimeDly(1); //give Windows AP some times to process graph
			}*/
		}
		else if (SysParam.bScanning == OP_AUTO_FOCUS) {
			//do nothing
		}
		else {
			OSTimeDly(1); //wait 50 msec, next image
		}
		if (bInitOK == 1) {
			//important, code must be placed here to avoid db3 conflict
			if ((nTick % (TICKS_PER_SECOND/2)) == 0)
				GetAllVideoADC(1);
		}
		nTick++;
	} //while(1)
}
//
void ScanOverProcess(char *szMsg)
{
	AutoScanStop();
	bScanAbort = 0;
	//
	EnableAutoFocus(0);
	if (nPrevConn != CONN_UART0)
		printf(szMsg);
	TxString(nPrevConn, szMsg);
	nScanImageNdx = 0;
	nScanImageNum = 0;
	bScanRestart = 0;
	SysParam.bScanning = OP_IDLE;
	SysParam.bPrevScanning = OP_IDLE;
	//SetupPITR(1, 0);	//stop interrupt
	//
	nReadyToErrorNum = nVacErrToleranceNum[0]; //not scanning
	OSTimeDly(1);
	//
	if (bStopType == 0) //don't shutdown all power
		return;
	//
	SetHVSwitch(0);
	SetHVSwitch(0); //for safety reason, execute twice!!!, important
	//SetVOPPower(0);
	//
	SetVoltage(BOARD_SCAN, DAC_CH_BR0, 0);
	SetVoltage(BOARD_SCAN, DAC_CH_BR1, 0);	//set coarse DAC
	SetVoltage(BOARD_SCAN, DAC_CH_BR2, 0);
	SetVoltage(BOARD_SCAN, DAC_CH_BR3, 0);
	SetDeflectorFineV(DAC_CH_DEFX_F, 0);
	SetDeflectorFineV(DAC_CH_DEFY_F, 0);
	SetVoltage(BOARD_SCAN, DAC_CH_CO, 0); //make video output 0
	SetVoltage(BOARD_SCAN, DAC_CH_CO1, 0);
	SetVoltage(BOARD_SCAN, DAC_CH_CO2, 0);
	SetVoltage(BOARD_IO, DAC_CH_STIGX, 0);
	SetVoltage(BOARD_IO, DAC_CH_STIGY, 0);
	//
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fObjStandbyV); //OBJ_ON_IMIN_V);
}
//
//execute only when dDB2Busy = 0
void TxIdle(int nWait)
{
	int i, nType, nCmdQueNum;
	char szCommand[CMD_SIZE];
	//
	//----------------------------------------------
	nCmdQueNum = GetCommandNumInQueue(); //process queued command
	if (nCmdQueNum == 0)
		return;
	//----------------------------------------------
	if (nWait) OSTimeDly(nWait);
	for (i = 0; i < nCmdQueNum; i++) {
		nType = PopCommandFromQueue(szCommand, i);
		if (bDebug == 1)
			printf("TxIdle():%s\n", szCommand);
		DecipherCommand(nType, szCommand); //connection type
	}
	ClearCommandInQueue();
}
//----------------------------------------------------
// transmit data in pbUdpbuffer, size=nByteInQueue
// transmit data per line (nDataBytesPerLine)
//nNdx=0, BEI; nNdx=1, SEI (second image)
//----------------------------------------------------
void TransmitData(int nConnType, int nDataByteNum, int nDataNdx, int nNdx)
{
	DATA_BEGIN_HEADER DataBeginHeader;
	DATA_HEADER DataHeader;
	DATA_END_HEADER DataEndHeader;
	//ONE_PKT_HEADER OnePktHeader;
	//BYTE bTxData[3072];
	BYTE bPktNdx;
	//int nDataNum;
	int nBytes;
	int nPacketDelay;
	//BYTE *pData;
	WORD wV;
	BYTE bADCCh;
	//
	int nHeaderSize;
	BYTE bTxBuffer[ROW_BYTE_NUM_MAX];
	//
	//int i;
	//WORD *pwV, wVX[PIXEL_NUM_X_MAX_PI];
	//
	bADCCh = (nNdx == 0) ? nShadowMode : bSecondImage;
	nHeaderSize = sizeof(DATA_HEADER);
	//
	for (bPktNdx = 0; bPktNdx < bPktNum; bPktNdx++) {
		nBytes = nDataBytePerPacket[bPktNdx];
		//0.3 usec * 2000 = 0.6 msec, take a long time
		//pData = &pbTxBuffer[nNdx][nDataNdx] - sizeof(DATA_HEADER);
		memmove(&bTxBuffer[nHeaderSize], &pbTxBuffer[nNdx][nDataNdx], nBytes);
		nDataNdx += nBytes;
		if (bPktNdx == bPktNum - 1)
			nPacketDelay = nPktDelay2; //200;// + nDataBytePerPacket[0]; //unit usec, take longer for final packet
		else
			nPacketDelay = nPktDelay1; //200;// + nDataBytePerPacket[0] / 2;
		if (bFirstPacket) { //don't append footer for the first packet
			DataBeginHeader.bHeader1 = HEADER_1;
			DataBeginHeader.bHeader2 = HEADER_2;
			DataBeginHeader.bPixelX_LSB = (nScanPixelNumX & 0x00FF);
			DataBeginHeader.bPixelX_MSB = (nScanPixelNumX & 0xFF00)>>8;
			DataBeginHeader.bPixelY_LSB = (nScanPixelNumY & 0x00FF);
			DataBeginHeader.bPixelY_MSB = (nScanPixelNumY & 0xFF00)>>8;
			DataBeginHeader.bLen_LSB = (nBytes & 0x00FF);
			DataBeginHeader.bLen_MSB = (nBytes & 0xFF00) >>8;
			//fit intel CPU byte order
			DataBeginHeader.bAddr[0] = (dwImageAddr & 0x000000FF);
			DataBeginHeader.bAddr[1] = (dwImageAddr & 0x0000FF00)>>8;
			DataBeginHeader.bAddr[2] = (dwImageAddr & 0x00FF0000)>>16;
			DataBeginHeader.bAddr[3] = (dwImageAddr & 0xFF000000)>>24;
			//DataBeginHeader.bFooterHeader = bFooterHeader; //reserved
			DataBeginHeader.bPktNdx = bPktNdx;
			DataBeginHeader.bPktNum = bPktNum;
			DataBeginHeader.bACK = bACK;
			DataBeginHeader.bNdx = (BYTE)nNdx; //0 or 1
			DataBeginHeader.bADCCh = bADCCh;
			//memmove(pData, &DataBeginHeader, sizeof(DATA_BEGIN_HEADER));
			memmove(bTxBuffer, &DataBeginHeader, sizeof(DATA_BEGIN_HEADER));
		}
		else if (bFinalPacket) {
			DataEndHeader.bHeader1 = FOOTER_1;
			DataEndHeader.bHeader2 = FOOTER_2;
#if CALC_MAXMIN == 1
			wZMax = wZMaxFW[nNdx];
			wZMin = wZMinFW[nNdx];
#endif
			DataEndHeader.bZMax_LSB = (wZMax & 0x00FF);
			DataEndHeader.bZMax_MSB = (wZMax & 0xFF00)>>8;
			DataEndHeader.bZMin_LSB = (wZMin & 0x00FF);
			DataEndHeader.bZMin_MSB = (wZMin & 0xFF00)>>8;
			DataEndHeader.bLen_LSB = (nBytes & 0x00FF);
			DataEndHeader.bLen_MSB = (nBytes & 0xFF00) >>8;
			//fit intel CPU byte order
			DataEndHeader.bAddr[0] = (dwImageAddr & 0x000000FF);
			DataEndHeader.bAddr[1] = (dwImageAddr & 0x0000FF00)>>8;
			DataEndHeader.bAddr[2] = (dwImageAddr & 0x00FF0000)>>16;
			DataEndHeader.bAddr[3] = (dwImageAddr & 0xFF000000)>>24;
			DataEndHeader.bPktNdx = bPktNdx;
			DataEndHeader.bPktNum = bPktNum;
			wV = pwFpga[GET_TIME_X]; //get X scan time in usec = fPeriodX
			SysParam.fTimeX = (float)wV/1e6; //unit second
			wV = pwFpga[GET_TIME_ASC]; //get X ascending time in usec = fPeriodX
			SysParam.fTimeXAsc = (float)wV/1e6; //unit second
			DataEndHeader.bTimeAsc_LSB = (wV & 0xFF);
			DataEndHeader.bTimeAsc_MSB = (wV & 0xFF00) >> 8;
			DataEndHeader.bNdx = (BYTE)nNdx;
			DataEndHeader.bADCCh = bADCCh;
			//memmove(pData, &DataEndHeader, sizeof(DATA_END_HEADER));
			memmove(bTxBuffer, &DataEndHeader, sizeof(DATA_END_HEADER));
		}
		else {
			DataHeader.bHeader1 = DATA_1;
			DataHeader.bHeader2 = DATA_2;
			DataHeader.bLen_LSB = (nBytes & 0x00FF);
			DataHeader.bLen_MSB = (nBytes & 0xFF00) >>8;
			DataHeader.bAddr[0] = (dwImageAddr & 0x000000FF);
			DataHeader.bAddr[1] = (dwImageAddr & 0x0000FF00)>>8;
			DataHeader.bAddr[2] = (dwImageAddr & 0x00FF0000)>>16;
			DataHeader.bAddr[3] = (dwImageAddr & 0xFF000000)>>24;
			DataHeader.bPktNdx = bPktNdx;
			DataHeader.bPktNum = bPktNum;
			DataHeader.bACK = bACK;
			DataHeader.bNdx = (BYTE)nNdx;
			DataHeader.bADCCh = bADCCh;
			//memmove(pData, &DataHeader, sizeof(DATA_HEADER));
			memmove(bTxBuffer, &DataHeader, sizeof(DATA_HEADER));
		}
		TxData(nConnType, bTxBuffer, nBytes + sizeof(DATA_HEADER));
		//TxData(nConnType, pData, nBytes + sizeof(DATA_HEADER)); //index is the largest no.
		//
		dwImageAddr += nBytes;
		//----------------------------------------------------------
		//don't delete code below !!!
		//this is a MUST!
		//wait a moment for next packet, important!!!
		delay_us(nPacketDelay);
		//don't delete code above !!!
		//----------------------------------------------------------
	}
	return;
}
//
/*void TimerTask(void * pd)
{
	int i = 0;
	time_t t;
	//
	while (1)
	{
		// tick every 200 msec
		OSTimeDly(TICKS_PER_SECOND/5);
		nScanIntervalTick = nScanIntervalTick + 200;
		if ((i % 300 == 0) && (bTimerStop == 0)) //print per 60 seconds
		{
			iprintf("Timer task is running %d\n", i);
			t = time(NULL);
			iprintf( "SYS Time = %s\n", ctime( &t ) );
		}
		i++;
	}
}*/
// serial 0 is used for debug
void Serial0Task(void * pd)
{
	char szBuffer[MAX_STRING_SIZE_P1];
	int nNdx = 0;
	char c;
	int nTick = 0;
	int nRet = 0;
	//
#if ENABLE_PID_UART == 1
	int i, nNum, nLen;
	int nDataNdx[2] = {0, 0};
	char szUART[2][PID_UART_SIZE] = {"", ""};
#endif
	//
	while (1)
	{
		if (charavail()) {
			c = getchar();
			if (c == '\n') { //remove CR+LF
				szBuffer[nNdx] = 0;
				iprintf("\n");
				DecipherMultipleCommand(CONN_UART0, szBuffer);
				memset(szBuffer, 0, MAX_STRING_SIZE);
				nNdx = 0;
				strcpy(szCmdQueue[nCmdQueNdx], szBuffer);
				nCmdQuePrevNdx = nCmdQueNdx;
				nCmdQueNdx = (nCmdQueNdx + 1) % CMD_QUE_NUM;
			}
			else if (c == 21) { //CTRL+UP
				sprintf(szBuffer, "\r%s", szCmdQueue[nCmdQuePrevNdx]);
				iprintf(szBuffer);
				nCmdQuePrevNdx = nCmdQuePrevNdx - 1;
				nCmdQuePrevNdx = nCmdQuePrevNdx % CMD_QUE_NUM;
			}
			else if (c == 4) { //CTRL+DOWN
				sprintf(szBuffer, "\r%s", szCmdQueue[nCmdQuePrevNdx]);
				iprintf(szBuffer);
				nCmdQuePrevNdx = nCmdQuePrevNdx + 1;
				nCmdQuePrevNdx = nCmdQuePrevNdx % CMD_QUE_NUM;
			}
			else if (c != '\r') 	{
				if (nNdx < MAX_STRING_SIZE - 1)
					szBuffer[nNdx++] = c;
				else nNdx = 0;
			}
		}
		OSTimeDly(1); // 50 msec
		if (NV_Settings.bEnableWDT == 1) //enable watch dog timer
			ResetWDT();
		if (nTick < 10 * TICKS_PER_SECOND) {
			nTick++;
		}
		if (nTick == 3 * TICKS_PER_SECOND) {
			//check PCB ID at power on process, after loading NV_settings
#if SCAN_PCB_VER >= 17
			nRet = UUID_Init(szBuffer);
#else
			nRet = ds2401_init(szBuffer);
#endif
			if (nRet == -1) {
				printf("ID check fail\n");
			}
			else {
				printf("ID check pass %d\n", nRet);
			}
		}
		if (nTick == 8 * TICKS_PER_SECOND)
			ShowTitle(CONN_UART0);
#if ENABLE_PID_UART == 1
		if (bInitOK == 0)
			continue;
		if (PID_RX_DATA == 0)
			continue;
		//------------------------------------------
		for (i = 0; i < 2; i++) {
			nNum = SPI_GetRxQueNum(i);
			if (nNum > 0) {
				nLen = SPI_ReadUART(i, szBuffer);
				if (nDataNdx[i] + nLen >= PID_UART_SIZE) {
					nDataNdx[i] = 0;
					continue;
				}
				memmove(&szUART[i][nDataNdx[i]], szBuffer, nLen);
				nDataNdx[i] += nLen;
				c = szUART[i][nDataNdx[i] - 1];
				if ((c == '\n') || (c == '\r')) {
					szUART[i][nDataNdx[i]] = 0;
					if (bSerDebug == 1)
						printf("SI:%d:%s", i + 4, szUART[i]); //debug purpose
					nDataNdx[i] = 0;
				}
			}
		} //for (i = 0, loop)
#endif
	} //while (1)
}
//
//called by Turbo_Control_Protocol.cpp
//
void ClearSerialBuffer(void)
{
	szSerial1Buffer[0] = 0;
	bSerial1Got = 0;
	nSerial1Ndx = 0;
	szSerial2Buffer[0] = 0;
	bSerial2Got = 0;
}

void ClearSerial1Buffer(void)
{
	szSerial1Buffer[0] = 0;
	bSerial1Got = 0;
	nSerial1Ndx = 0;
}
// serial 1 is used for vacuum turbo pump communication
void Serial1Task(void * pd)
{
	char szBuffer[MAX_STRING_SIZE_P1];
	int rv;
	int nTextNdx = 0;
	char szText[128];
	char c;
	//
	szSerial1Buffer[0] = 0;
	while (1)
	{
		while (dataavail(fdSerial[1])) {
			rv = read(fdSerial[1], &c, 1);
			if (bSerDebug == 1) { //serial port debug, SOE:1
				if ((c == '\r') && (nTextNdx > 0))	{ //CR
				//if ((c == '\n' || c == '\r') && nTextNdx > 0)	{ //CR only, add LF
					szText[nTextNdx] = 0;
					printf("SI:1:%s\n", szText); //debug purpose
					nTextNdx = 0;
				}
				else {
					szText[nTextNdx] = c;
					nTextNdx = (nTextNdx + 1) % 127;
				}
				continue;
			}
			else if (NV_Settings.nTurboType == TURBO_PFEIFFER) {
				if ((c == ' ') || (c == 0) || c == '\n') {	//skip '\n' and 0x00
				}
				else if ((c & 0x80) != 0) {
					//0xFF, not ASCII code
				}
				else if (c == '\r') {
					szBuffer[nSerial1Ndx] = 0;
					strcpy(szSerial1Buffer, szBuffer);
					bSerial1Got = 1;
					nSerial1Ndx = 0;
				}
				else {
					if (nSerial1Ndx < MAX_STRING_SIZE)
						szBuffer[nSerial1Ndx++] = c;
					else nSerial1Ndx = 0;
				}
			} //if (NV_Settings.iTurboType == TURBO_PEIFFER)
			else { //02 80 32 30 34 30 30 30 30 32 37 03 CRC
				if (c == 0x02) nSerial1Ndx = 0; //STX
				else if (c == 0x03) {
					c = 0; //ETX
					szBuffer[nSerial1Ndx++] = 0;
					strcpy(szSerial1Buffer, szBuffer);
					bSerial1Got = 1;
				}
				else if ((c & 0x80) != 0) { //ADDR
				}
				else {
					if (nSerial1Ndx < MAX_STRING_SIZE)
						szBuffer[nSerial1Ndx++] = c;
					else nSerial1Ndx = 0;
				}
			}
		} //while (dataavail(fdSerial[1]))
		OSTimeDly(1); // 50 msec
	}
}
// motor communication interface
void Serial2Task(void * pd)
{
	char szBuffer[MAX_STRING_SIZE_P1];
	int rv, nNdx = 0;
	char c;
	char *p, *p1;
	BYTE bAddr;
	int nTextNdx = 0;
	char szText[128];
	int nTick = 0;
	//
	szSerial2Buffer[0] = 0;
	while (1)
	{
		while (dataavail(fdSerial[2])) {
			rv = read(fdSerial[2], &c, 1);
			if (bSerDebug == 1) { //SOE:1, Serial2 receives something
				if ((c == '\r') && (nTextNdx > 0))	{ //CR
				//if ((c == '\n' || c == '\r') && nTextNdx > 0)	{ //CR only, add LF
					szText[nTextNdx] = 0;
					printf("SI:2:%s\n", szText); //debug purpose
					nTextNdx = 0;
				}
				else {
					szText[nTextNdx] = c;
					nTextNdx = (nTextNdx + 1) % 127;
				}
				continue;
			}
			if (((c & 0x80) != 0) || (c < 0x05)) {
				//0xFF, not ASCII code
				nNdx = 0;
			}
			else if (c == '\r') { //CR
				//skip, do nothing
			}
			else if (c == '\n') { //LF
				szBuffer[nNdx] = 0;
				if (nNdx > 0) { //for test purpose
					strcpy(szSerial2Buffer, szBuffer);
					bSerial2Got = 1;
					if (bDebug == 19)
						printf("SI:2:%s\n", szBuffer);
				}
				nNdx = 0;
				//connect to custom HV module
				if (szBuffer[0] == '$') { //response from custom device
					p1 = &szBuffer[1];
					bAddr = (BYTE)atoi(p1);
					p = strchr(p1, ',');
					if (p == NULL) break;
					p++;
					if (bAddr == HV_ADDR) { //$2,HV module
						//SO:1:2:$2,GETSYS:1\n	//B=1,PORT=2,RS485 ADDR=2
						//$2,SYS:8,bFilaOK,bFilaWDT,fFilaI,fFilaV,fAccV,fAccI,HV_ON,HV_ON_SW
						//$2,SYS:9,bFilaOK,bFilaWDT,fFilaI,fFilaV,fAccV,fAccI,HV_ON,HV_ON_SW,fBiasV
						//$2,SYS:10,bFilaOK,bFilaWDT,fFilaI,fFilaV,fAccV,fAccI,HV_ON,HV_ON_SW,fBiasV,bExtCtrl
						//$2,SYS:11,bFilaOK,bFilaWDT,fFilaI,fFilaV,fAccV,fAccI,HV_ON,HV_ON_SW,fBiasV,bExtCtrl,fTempe
						//$2,SYS:12,bFilaOK,bFilaWDT,fFilaI,fFilaV,fAccV,fAccI,HV_ON,HV_ON_SW,
						//	fBiasV,bExtCtrl,fTempe[0],fTempe[1]
						ParseHVCMessage(p);
					}
					else if (bAddr == IP_MOTOR_ADDR) { //$1,IP PWR CTRL
						//SO:1:2:$1,IPINFO\n	//B=1,PORT=2,RS485 ADDR=1
						//$1,IPINFO:IpAmp,IpHV,bMoving,IPON
						ParseIPCMessage(p) ;
					}
				}
			}
			else {
				if (nNdx < MAX_STRING_SIZE) {
					szBuffer[nNdx++] = c;
				}
				else nNdx = 0;
			}
		} //while (dataavail(fdSerial[2]))
		OSTimeDly(1); // 50 msec
		nTick = (nTick + 1) % TICKS_PER_SECOND;
		if (bSerDebug == 1)
			continue;
		if (nTick == 0) { //get IP_PWR_CTRL information
#if ENABLE_IP == 1
			sprintf(szBuffer, "$%d,IPINFO\n", IP_MOTOR_ADDR);
			RS485_WriteString(2, szBuffer);
#endif
		}
		else if (nTick == 5) { //get HV information
			if (NV_Settings.bHVType == HV_CUSTOM) {
				sprintf(szBuffer, "$%d,GETSYS:1\n", HV_ADDR);
				RS485_WriteString(2, szBuffer);
			}
		}
		else if (nTick == 7) { //get HV information
			if (NV_Settings.bHVType == HV_CUSTOM) {
				switch (bHVCommand) {
					case 1: //COSNT //0:NONE,1:CP,2:CC
						sprintf(szBuffer, "$%d,CONST:%d\n", HV_ADDR, (int)bFilaConstMode);
						RS485_WriteString(2, szBuffer);
						break;
					case 2: //GETSYS:2 binary mode
						sprintf(szBuffer, "$%d,GETV\n", HV_ADDR);
						RS485_WriteString(2, szBuffer);
						break;
				}
				bHVCommand = 0;
			}
		}
	}
}
//
void Serial3Task(void * pd)
{
	char szBuffer[MAX_STRING_SIZE_P1];
	int rv, nNdx = 0;
	char c;
	int nTextNdx = 0;
	char szText[128];
	//
	szSerial3Buffer[0] = 0;
	while (1)
	{
		while (dataavail(fdSerial[3])) {
			rv = read(fdSerial[3], &c, 1);
			if (bSerDebug == 1) {
				if (c == '\n' && nTextNdx > 0)	{ //CR only, add LF
					szText[nTextNdx] = 0;
					printf("SI:3:%s\n", szText); //debug purpose
					nTextNdx = 0;
				}
				else {
					szText[nTextNdx] = c;
					nTextNdx = (nTextNdx + 1) % 127;
				}
				continue;
			}
			if ((c & 0x80) != 0) {
				//0xFF, not ASCII code
			}
			else if (c == '\r') {
				//do nothing
			}
			else if (c == '\n') {
				szBuffer[nNdx] = 0;
				if (nNdx > 0) {
					strcpy(szSerial3Buffer, szBuffer);
					bSerial3Got = 1;
				}
				nNdx = 0;
			}
			else {
				if (nNdx < MAX_STRING_SIZE)
					szBuffer[nNdx++] = c;
				else nNdx = 0;
			}
		} //while (dataavail(fdSerial[3]))
		OSTimeDly(1); // 50 msec
	}
}
//--------------------------------------------------
// wait until nWaitState is violate
// nVacTimeout in seconds
//--------------------------------------------------
/*BOOL WaitVacuumStatus(int nNdx, int nVacTimeout, int nWaitState)
{
	int nTick;
	char szValue[32];
	for (nTick = 0; (nTick < nVacTimeout) && (GetGaugeStatus(nNdx, szValue) == nWaitState); nTick++)
		OSTimeDly(TICKS_PER_SECOND);
	if (nTick == nVacTimeout)
		return FALSE;
	else
		return TRUE;
}*/
//
BOOL IsVacuumReady(void)
{
	int nRet;
	nRet = (SysParam.bVacuumState == VAC_ST_READY) ? TRUE : FALSE;
	return nRet;
}

BOOL IsNextVacuumStateOK(int nNext)
{
	//----------------------------------------------
	// nNext is the next vacuum state
	// nVacuumStae is the current vacuum state
	//----------------------------------------------
#if GAUGE_OP_MODE == 2
	if ((nNext == VAC_ST_ERROR1) || (nNext == VAC_ST_ERROR2))
		return TRUE;
#else
	if (nNext == VAC_ST_ERROR)
		return TRUE;
#endif
	switch (SysParam.bVacuumState) //current vacuum state
	{
		case VAC_ST_AIR:
			if (nNext == VAC_ST_LO_WAIT)
				return TRUE;
			break;
		case VAC_ST_STANDBY:
			if ((nNext == VAC_ST_AIR_WAIT) && (NV_Settings.bEnableVent == 1)) //go to AIR
				return TRUE;
			if (nNext == VAC_ST_LO_WAIT)
				return TRUE;
			break;
		case VAC_ST_LO: //go to AIR or go to READY
			if (IsInterlockSafe(ACT_SCROLL_ON) != SUCCESS) //is it safe to turn on scroll pump?
				return FALSE;
			if ((nNext == VAC_ST_AIR_WAIT) || (nNext == VAC_ST_HI_WAIT)) {
				return TRUE;
			}
			break;
		case VAC_ST_HI:
			if (IsInterlockSafe(ACT_TURBO_ON) != SUCCESS) //is it safe to turn on turbo pump?
				return FALSE;
			if ((nNext == VAC_ST_LO_WAIT) || (nNext == VAC_ST_AIR_WAIT))
				return TRUE;
			break;
#if GAUGE_OP_MODE == 2
		case VAC_ST_ERROR1:
			return TRUE;
		case VAC_ST_ERROR2:
			return TRUE;
#else
		case VAC_ST_ERROR:
			if ((nNext == VAC_ST_AIR_WAIT) || (nNext == VAC_ST_LO_WAIT)) {
				return TRUE;
			}
#endif
			break;
	}
	return FALSE;
}

void RedundantProcess(void)
{
	//redundant process to avoid trip
#if EN_REDUNDANT == 1
	SetRelayOn(1); //relay power on
	OSTimeDly(1);
	SetVOPPower(1); //power for deflector and stigmator
	OSTimeDly(4);
#endif
}
// AIR, STANDBY, ERROR
BOOL VentingProcess(void)
{
	char szState[2][30] = {"STANDBY", "AIR"};
#if SHOW_VACUUM_STATE == 1
	printf("Start venting process to %s\n", szState[(int)NV_Settings.bEnableVent]);
#endif
	//RedundantProcess();
	//
	SetHVPower(0);		//turn off high voltage power
	SetHVSwitch(0);		//turn on HV switch
	StopTurbo((int)NV_Settings.bEnableVent);
	SetGateValve(0, VALVE_CLOSE); //turbo pump, gate valve 0 close
	SetGateValve(1, VALVE_CLOSE); //external, gate valve 1 close
	SetScrollPump(0); //turn off scroll pump power control
	return TRUE;
}
//
void CloseAllValves(void)
{
	//for EM200
	//0:chamber, 1:IP, 2:pipe|chamber, 3:turbo
	SetGateValve(0, VALVE_CLOSE);
	SetGateValve(1, VALVE_CLOSE);
#if ENABLE_IG == 1
	SetGateValve(2, VALVE_CLOSE);
	SetGateValve(3, VALVE_CLOSE);
#endif
}

void VarianVentToAir(void)
{
	if (NV_Settings.nTurboType != TURBO_AGILENT)
		return;
	//---------------------------------------------------------
	// gate valve 0 is not under control for Pfeiffer type
	//---------------------------------------------------------
	SetGateValve(0, VALVE_OPEN); //open to air
	OSTimeDly(TICKS_PER_SECOND);
	SetGateValve(0, VALVE_CLOSE);
	OSTimeDly(TICKS_PER_SECOND * 3);
}
//
BYTE bGreen = 0, bRed = 0, bYellow = 0, bBlue = 0;
BYTE bLedSum = 0;
///////////////////////////////////////////////////////////////////////
// INTERRUPT - PIT interrupt service routine
//
#if MODULE_TYPE == MT_MOD5270B
INTERRUPT( my_pitr_func, 0x2600 )
#elif MODULE_TYPE == MT_MOD5282
INTERRUPT( my_pitr_func, 0x2200 )
#endif
{
#if IO_PCB_VER >= 1
	BYTE bV;
#endif
	WORD tmp = sim.pit[1].pcsr; // Get PIT1 Control & Status Register
	// data
	//
	// Clear PIT1 - Refer to table 19-3 for more information on what
	// bits are being cleared and set
	//
	tmp &= 0xFF0F; // Bits 4-7 cleared
	tmp |= 0x0F; // Bits 0-3 set, set EN(0)=PITE(3)=1
	//
	sim.pit[1].pcsr = tmp;
	//
	// You can add you ISR code here
	// - Do not call any RTOS function with pend or init in the function
	// name
	// - Do not call any functions that perform a system I/O read,
	// write, printf, iprintf, etc.
	//
	// LED control
	//nScanIntervalTick++; //100 ms
	lSysclkTick++;
	//
	if (bInitOK == 0)
		return;
	if (bEnableLED == 0)
		return;
	if (NV_Settings.bLEDAuto == 1)
		return;
	//--------------------------------------------
	//always auto mode!!!
	//--------------------------------------------
	// LED scenario
	// GREEN:
	//  power on
	// RED:
	// 	flash twice: HW version error FAST-FAST-SLOW
	// 	DARK: HV off
	// 	SOLID: HV on
	// BLUE:
	// 	DARK: STANDBY or AIR
	// 	flash 1 Hz: LOW_WAIT
	// 	flash 2 Hz: READY_WAIT
	// 	SOLID: READY
	// YELLOW:
	//	SOLID: HV power on
	//	DARK: HV power off
	//--------------------------------------------
	// red LED
	//--------------------------------------------
	if (SysParam.bErrorHWVersion != 0) { //red LED flash twice per second
		if ((nPITRCount % 10 == 0) || (nPITRCount % 10 == 4)) {
			LED_RED_ON;
			bRed = IO_LED_RED;
		}
		else if ((nPITRCount % 10 == 2) || (nPITRCount % 10 == 6)) {
			LED_RED_OFF;
			bRed = 0;
		}
	}
	else if (bLEDState[LEDR] == ST_LED_SOLID) {
		LED_RED_ON;
		bRed = IO_LED_RED;
	}
	else if (bLEDState[LEDR] == ST_LED_BLINK_1HZ) {
		if (nPITRCount % 10 == 5) {
			LED_RED_ON;
			bRed = IO_LED_RED;
		}
		else if (nPITRCount % 10 == 0) {
			LED_RED_OFF;
			bRed = 0;
		}
	}
	else if (bLEDState[LEDR] == ST_LED_BLINK_2HZ) {
		if (nPITRCount % 5 == 3) {
			LED_RED_ON;
			bRed = IO_LED_RED;
		}
		else if (nPITRCount % 5 == 0) {
			LED_RED_OFF;
			bRed = 0;
		}
	}
	else if (bLEDState[LEDR] == ST_LED_DARK) {
		LED_RED_OFF;
		bRed = 0;
	}
	//--------------------------------------------
	// green LED
	//--------------------------------------------
	if (bLEDState[LEDG] == ST_LED_SOLID) {
		LED_GREEN_ON;
		bGreen = IO_LED_GREEN;
	}
	else if (bLEDState[LEDG] == ST_LED_BLINK_1HZ) {
		if (nPITRCount % 10 == 5) {
			LED_GREEN_ON;
			bGreen = IO_LED_GREEN;
		}
		else if (nPITRCount % 10 == 0) {
			LED_GREEN_OFF;
			bGreen = 0;
		}
	}
	else if (bLEDState[LEDG] == ST_LED_DARK) {
		LED_GREEN_OFF;
		bGreen = 0;
	}
	//--------------------------------------------
	// blue LED
	//--------------------------------------------
	if (bLEDState[LEDB] == ST_LED_SOLID) {
		LED_BLUE_ON;
		bBlue = IO_LED_BLUE;
	}
	else if (bLEDState[LEDB] == ST_LED_BLINK_1HZ) {
		if (nPITRCount % 10 == 5) {
			LED_BLUE_ON;
			bBlue = IO_LED_BLUE;
		}
		else if (nPITRCount % 10 == 0) {
			LED_BLUE_OFF;
			bBlue = 0;
		}
	}
	else if (bLEDState[LEDB] == ST_LED_BLINK_2HZ) {
		if (nPITRCount % 5 == 3) {
			LED_BLUE_ON;
			bBlue = IO_LED_BLUE;
		}
		else if (nPITRCount % 5 == 0) {
			LED_BLUE_OFF;
			bBlue = 0;
		}
	}
	else if (bLEDState[LEDB] == ST_LED_DARK) {
		LED_BLUE_OFF;
		bBlue = 0;
	}
	//--------------------------------------------
	// yellow LED
	//--------------------------------------------
	if (bLEDState[LEDY] == ST_LED_SOLID) {
		LED_YELLOW_ON;
		bYellow = IO_LED_YELLOW;
	}
	else if (bLEDState[LEDY] == ST_LED_BLINK_1HZ) {
		if (nPITRCount % 10 == 5) {
			LED_YELLOW_ON;
			bYellow = IO_LED_YELLOW;
		}
		else if (nPITRCount % 10 == 0) {
			LED_YELLOW_OFF;
			bYellow = 0;
		}
	}
	else if (bLEDState[LEDY] == ST_LED_DARK) {
		LED_YELLOW_OFF;
		bYellow = 0;
	}
	//------------------------------------------------
	//int nG, int nR, int nY, int nB
	//------------------------------------------------
#if IO_PCB_VER >= 1
	bV = bGreen + bRed + bYellow + bBlue;
	//SetIO_LED(bGreen, bRed, bYellow, bBlue);
	if ((bLedSum != bV) && (bEnableIO == 1)) {
		SetIO_LED(bGreen, bRed, bYellow, bBlue);
		bLedSum = bV;
	}
	else if (bEnableIO == 2) { //for testing
		SetIO_LED(bGreen, bRed, bYellow, bBlue);
	}
#endif
	nPITRCount++;
}

/*
Declare our interrupt procedure....
name: our_irq1_pin_isr
masking level (The value of the ColdFire SR during the interrupt:

use 0x2700 to mask all interrupts.
    0x2500 to mask levels 1-5 etc...
    0x2100 to mask level 1
*/
//
INTERRUPT( out_irq1_pin_isr, 0x2100 )
{
	BYTE bOK = 0;
  /* WARNING WARNING WARNING
  Only a very limited set of RTOS functions can be called from
  within an interrupt service routine.
  Basically, only OS POST functions and LED functions should be used
  No I/O (read, write or printf may be called), since they can block. */
	//WORD wV1, wV2;
	sim.eport.epfr=0x02; /* Clear the interrupt edge 0 0 0 0 0 0 1 0 */
	//
	if (bInitOK == 0)
		return;
	if (SysParam.bScanning == OP_SEM_SCAN)
		bOK = 1;
	else if (SysParam.bScanning == OP_AUTO_FOCUS)
		bOK = 1;
	else if (SysParam.bScanning == OP_AUTO_FOCUS_SW) //auto focus controlled by SW
		bOK = 1;
	if (bOK == 0)
		return;
	J2[32].set(1);
	//move data from SRAM to NB RAM
	//0.6 us/pixel
	//1200 pixels takes 728 usec
	//1600 pixels takes 960 usec
	//2400 pixels takes 1440 usec
	//4000 pixels takes 2400 usec
	if (NV_Settings.bEnableFIFO) {
		if (bImageMask & 0x01) {
			memmove(&pbTxBuffer[0][nBytesIn], (BYTE *)&pwFifo[ADDR_BEI_START],
				nDataBytesPerLine);
		}
		if (bImageMask & 0x02) {
			memmove(&pbTxBuffer[1][nBytesIn], (BYTE *)&pwFifo[ADDR_SEI_START],
				nDataBytesPerLine);
		}
	}
	else { //bEnableFIFO = 0
		if (bImageMask & 0x01) {
			memmove(&pbTxBuffer[0][nBytesIn], (BYTE *)&pwSram[ADDR_BEI_START],
				nDataBytesPerLine);
		}
		if (bImageMask & 0x02) {
			memmove(&pbTxBuffer[1][nBytesIn], (BYTE *)&pwSram[ADDR_SEI_START],
				nDataBytesPerLine);
		}
	}
	nBytesIn = (nBytesIn + nDataBytesPerLine) % nQueSize + nQueStart;
	//OSSemPost(&IrqPostSem);
	J2[32].set(0);
	if (bIRQACK == 1) { //CPU send ACK to FPGA after data moved
		pwFpga[SET_IRQ_ACK] = 1;
		delay_void();
		pwFpga[SET_IRQ_ACK] = 0;
	}
	//if (bBlanking == 2) {
	//	BlankingProcess(1);
	//}
}

WORD wIRQ_EPPAR = 0x0;
WORD wIRQ_EPIER = 0x0;

void SetupIRQ1(int nEnable)
{
#if MODULE_TYPE == MT_MOD5270B
	/* First set up the Eport module to use IRQ1 as a falling-edge active  IRQ pin
	  (See the 5270 UM chapter 15). Set the pin assignment register irq1 pin falling
	  edge sensitive*/
	//10 falling edge, 01 rising edge
	wIRQ_EPPAR |= 0x0004;
	if (nEnable == 1)
		wIRQ_EPIER |= 0x0002;
	else
		wIRQ_EPIER |= (~0x0002);
	sim.eport.eppar = wIRQ_EPPAR; /* 00 00 00 00 00 00 01 00  see table 15-3 in UM */
	sim.eport.epddr = 0x0;    /* All edge port pins as inputs */
	sim.eport.epier = wIRQ_EPIER; /* Enable IRQ1 only 0 0 0 0 0 0 1 0 */
	SetIntc((long)&out_irq1_pin_isr, /* Our interrupt function */
	1, /* The vector number from the users manual table 10-13 */
	1, /* Set this to priority 1 but any value from 1 to 6 would be valid.*/
	1); /* The priority within the gross levels; see chapter 10, any value from 0 to 7 is ok */
#elif MODULE_TYPE == MT_MOD5282
	//00: level, 01 rising edge, 10:falling edge
	sim.eport.eppar = 0x0004; /* 00 00 00 00 00 00 01 00  see table 11-3 in UM */
	sim.eport.epddr = 0x0;    /* All edge port pins as inputs */
	sim.eport.epier = 0x0002; /* Enable IRQ1 only 0 0 0 0 0 0 1 0 */
	SetIntc(0, (long)&out_irq1_pin_isr, 1, 1, 1);
#endif
}
//
//0x2100 block all interrupts below level 2
//interrupt level see 5270 UM 13.2.1.6
INTERRUPT( out_irq3_pin_isr, 0x2400 )
{
  /* WARNING WARNING WARNING
  Only a very limited set of RTOS functions can be called from
  within an interrupt service routine.
  Basically, only OS POST functions and LED functions should be used
  No I/O (read, write or printf may be called), since they can block. */
	//ISR takes about 3 usec
	sim.eport.epfr = 0x08; /* Clear the irq3 interrupt flag 0 0 0 0 1 0 0 0 */
	//J2[11] = 1; //debug
	//J2[11] = 0;
}

void SetupIRQ3(int nEnable)
{
	#if MODULE_TYPE == MT_MOD5270B
	/* First set up the Eport module to use IRQ3 as a falling-edge active  IRQ pin
	  (See the 5270 UM chapter 15.4.1.1). Set the pin assignment register irq3 pin falling
	  edge sensitive*/
	//10 falling edge, 01 rising edge
	wIRQ_EPPAR |= 0x0080; //falling edge
	if (nEnable == 1) {
		wIRQ_EPIER |= 0x0008;
	}
	else {
		wIRQ_EPIER &= (~0x0008);
	}
	EnableIRQ3(nEnable);
	//
	sim.eport.eppar = wIRQ_EPPAR; /* 00 00 00 00 10 00 00 00  see table 15-3 in UM */
	sim.eport.epddr = 0x0;    /* All edge port pins as inputs */
	sim.eport.epier = wIRQ_EPIER; /* Enable IRQ3 0 0 0 0 1 0 0 0 */
	SetIntc((long)&out_irq3_pin_isr, /* Our interrupt function */
	3, /* The vector number from the 5282 users manual table 10-13, 5270 UM table 13-13 */
	3, /* Set this to priority 1 but any value from 1 to 6 would be valid.*/
	1); /* The priority within the gross levels; see chapter 10, any value from 0 to 7 is ok */
	#elif MODULE_TYPE == MT_MOD5282
	//00: level, 01 rising edge, 10:falling edge
	wIRQ_EPPAR |= 0x0040;
	sim.eport.eppar = wIRQ_EPPAR; /* 00 00 00 00 00 00 01 00  see table 11-3 in UM */
	sim.eport.epddr = 0x0;    /* All edge port pins as inputs */
	wIRQ_EPIER |= 0x0008;
	sim.eport.epier = wIRQ_EPIER; /* Enable IRQ3 only 0 0 0 0 1 0 0 0 */
	SetIntc(0, (long)&out_irq3_pin_isr, 3, 2, 1);
	#endif
}
/* See table 19-3 in the reference manual for bits 8-11 */
//if wIntervalReg = 0, stop PITR
void SetupPITR( int nPITR_Ch, WORD wIntervalReg, BYTE bPCSR_PreScaler)
{
	WORD tmp;
	if ( ( nPITR_Ch < 1 ) || ( nPITR_Ch > 3 ) )
	{
		iprintf( "*** ERROR - PIT channel out of range ***\r\n" );
		return;
	}
	//
	// Populate the interrupt vector in the interrupt controller. The
	// SetIntc() function is supplied by the NetBurner API to make the
	// interrupt control register configuration easier
	//

	//
	// Configure the PIT for the specified time values
	//
	sim.pit[nPITR_Ch].pmr = wIntervalReg; // Set PIT modulus value
	tmp = bPCSR_PreScaler;
	//EN(0), Reload(1), PIT interrupt flag(2), PIE(PIT interrupt enable)
	if (wIntervalReg == 0) {
		tmp = ( tmp << 8 ) | 0x06; //disable intrrupt
		sim.pit[nPITR_Ch].pcsr = tmp; // Set system clock divisor to 16 and
		iprintf("PITR STOP\r\n");
	}
	else {
		tmp = ( tmp << 8 ) | 0x0F; //prescaler bit 8 ~ 11
		sim.pit[nPITR_Ch].pcsr = tmp; // Set system clock divisor to 16 and
		iprintf("PITR START\r\n");
	}
	//
	#if MODULE_TYPE == MT_MOD5270B
	//source, priority
	SetIntc( ( long ) &my_pitr_func, PITR_CH_OFS + nPITR_Ch, 2 /* IRQ2 */, 3 );
	#elif MODULE_TYPE == MT_MOD5282
	SetIntc( 0, ( long ) &my_pitr_func, PITR_CH_OFS + nPITR_Ch, 2 /* IRQ2 */, 3 );
	#endif
	// set bits [3:0] in PCSR
}

void SetupPITR( int nPITR_Ch, float fFrequency)
{
	BYTE bPreScaler;
	WORD wPMR, wPreScaler;
	//
	//fFrequency = fFrequency / 2;
	if (fFrequency < 0 || fFrequency > 1000)
		return;
	// Let us make PIT happen at 1000 Hz. The base clock is
	// 66,355,200 Hz, so the equation is:
	//
	//                 System_Clock_Frequency
	// PMR Value = ------------------------------- - 1
	//               Prescalar * Desired_Frequency
	//
	//               66355200
	// PMR Value = ----------- - 1
	//              16 * 1000
	//
	// 1000 Hz, PMR Value = 4146, prescaler=3
	//  100 Hz, PMR Value = 5183, prescaler=6
	//   10 Hz, PMR Value = 3239, prescaler=10
	//    1 Hz, PMR Value = 4049, prescaler=13
	// prescaler 3(16), 4(32), 5(64), 6(128), 7(256), 8(512), 9(1024),
	// 10(2048), 11(4096), 12(8192), 13(16384), 14(32768), 15(65536)
	//
	// Note that the PIT Count Register is a 16-bit counter, so the
	// clock count maximum is 65,535
	//MOD5282: 66,355,200 Hz
	//10000 Hz, PMR value = 829.44, prescaler=2(8)
	// 1000 Hz, PMR Value = 4146, prescaler=3(16)
	//  100 Hz, PMR Value = 5183, prescaler=6(128)
	//   10 Hz, PMR Value = 3239, prescaler=10(2048)
	//    1 Hz, PMR Value = 4049, prescaler=13(16384)
	//MOD5270B: 73,728,000 Hz
	//10000 Hz, PMR value = , prescaler=2(8)
	// 1000 Hz, PMR Value = , prescaler=3(16)
	//  100 Hz, PMR Value = , prescaler=6(128)
	//   10 Hz, PMR Value = , prescaler=10(2048)
	//    1 Hz, PMR Value = , prescaler=13(16384)
	if (fFrequency <= 1) //
		bPreScaler = 13; //16384
	else if (fFrequency <= 20)
		bPreScaler = 10; //2048
	else if (fFrequency <= 200)
		bPreScaler = 6; //128
	else //if (fFrequency <= 1000)
		bPreScaler = 3; //16
	wPreScaler = 0x0001 << (bPreScaler + 1); //
	if (fFrequency == 0)
		wPMR = 0; //disable interrupt
	else
		wPMR = (WORD)(SYSCLK / fFrequency / wPreScaler) - 1; //
	SetupPITR(nPITR_Ch, wPMR, bPreScaler);
}

void StopPITR(void)
{
	WORD tmp = sim.pit[1].pcsr; // Get PIT1 Control & Status Register
	tmp &= 0xFF00; // Bits 4-7 cleared
	tmp |= 0x06; // Bits 0-3 set EN(0)=PITE(3)=0
	sim.pit[1].pcsr = tmp;
	iprintf("PITR STOP\n");
}

void InitFastScan(void)
{
	int nV = 8000;
	int nDataStream = 1;
	int i;
	WORD wVW, wVR;
	pwFpga[SET_DELAY_DESCENT] = (WORD)NV_Settings.nDelayDescent; //nDelayIDec
	delay_void();
	pwFpga[SET_DELAY_PIXEL] = (WORD)NV_Settings.nPixelDwellTime; //unit 50*20=1000 ns = 1 us
	delay_void();
	//
	wVR = pwFpga[GET_STATUS2];
	if (wVR & 0x0800) { //has_num = 1
		for (i = 0; i < 4; i++) {
			wVW = nScanPixelNumX + nOverscan;
			pwFpga[SET_X_NUM] = wVW;
			wVR = pwFpga[GET_X_NUM];
			if (wVR == wVW) break;
		}
		for (i = 0; i < 4; i++) {
			wVW = nScanPixelNumY;
			pwFpga[SET_Y_NUM] = wVW;
			wVR = pwFpga[GET_Y_NUM];
			if (wVR == wVW) break;
		}
		if (bDebug == 11)
			printf("X=%d,OVERSCAN=%d,Y=%d\n", nScanPixelNumX, nOverscan, nScanPixelNumY);
	}
	else {
		wVW = nScanPixelNumX + nOverscan;
		pwFpga[SET_X_NUM] = wVW;
		delay_void();
		pwFpga[SET_X_NUM] = wVW; //set twice
		delay_void();
		pwFpga[SET_Y_NUM] = (WORD)nScanPixelNumY;
		delay_void();
		pwFpga[SET_Y_NUM] = (WORD)nScanPixelNumY;
		delay_void();
	}
	//pwFpga[SET_FRAME_NUM] = 1; //get 1 image per vadc_scan_start
	//
	nDataStream = (bImageMask == 0x01) ? 1 :
		(bImageMask == 0x02) ? 1 : 2; //0x03 take longer
	//
	/*if (NV_Settings.bEnableFIFO == 1) {
		//20 MHz, delay time unit = 200 ns
		//40 MHz, delay time unit = 200 ns
	}
	else */
	if (NV_Settings.bAutoXLineDelay) { //time unit 25 ns
		//speed of memmove, each WORD take 640 ns for MOD5270B
		if (NV_Settings.nSysClk == 20) //MHz
			nV = (6000 + nScanPixelNumX * 650 / 50) * nDataStream ; //300 / 50;
		else if (NV_Settings.nSysClk == 30) //if FPGA_SYSCLK == 30 //MHz
			nV = (18000 + nScanPixelNumX * 650 / 33) * nDataStream ;
		else //if (NV_Settings.nSysClk == 40)
			nV = (10000 + nScanPixelNumX * 625 / 25) * nDataStream;
		//0.6 us/pixel, memmove()
		//800 pixels takes 476 us
		//1200 pixels takes 728 usec
		//1600 pixels takes 960 usec
		//2400 pixels takes 1440 usec
		//4000 pixels takes 2400 usec
		//one tick is 4 counts, 200 ns(20 MHz), 100 ns(40 MHz)
		nV = nV / 4; //x_line_delay[17:0] = {databus1[15:0], 2'b00};
		if (nV > 65535) nV = 65535;
		SysParam.wXLineDelay = nV;
	} //300 ns * pixel number /
	pwFpga[SET_X_LINE_DELAY] = SysParam.wXLineDelay; //16,000x50 ns= 800 us
}

int IsRowDataReady(void)
{
	int i;
	WORD wV;
	for (i = 0;  i < 3; i++) {
		wV = pwFpga[GET_STATUS];
		if ((wV & ST_ROW_DATA_READY) == 0) //bit 5
			return FALSE;
	}
	return TRUE;
}

#define CHECK_SCAN_OVER		3
//
int FastScan(int nConnType) //use interrupt signal
{
	//WORD wV;
	//int i; // nV;
	int nPY = 0, nV;
	int nTimeout = 0;
	//int nTimeoutMax = 20;
	int nRet = SUCCESS;
	//int nScanOver = 0;
	WORD wStatus[2] = {0, 0};
	char szBuffer[64];
	int nDelayTime = 0;
	int nYInc = 1;
	int nYEnd;
	int nScanTime = 0;
	int nScanTimeMax = 48;
	int nFPGATimeout = 40;
	int nTxTimeout = 0;
#if CALC_MAXMIN == 1
	WORD *pwData, wV;
	int i, nNdx;
#endif
	//int nBytesNum;
	//int nPtr, nNdx;
	//
	// OSTimeDly(1);
	//
	bDB3Busy = 1;
	bACK = 0;
	//bPktOrderNdx = 0;
	ClearCommandInQueue();
#if CALC_MAXMIN == 1
	wZMaxFW[0] = 0x0;
	wZMaxFW[1] = 0x0;
	wZMinFW[0] = 0xFFFF;
	wZMinFW[1] = 0xFFFF;
#endif
	/*
	if (SysParam.bScanning == OP_SEM_SCAN)
		dwImageAddr = 0;
	else if (SysParam.bScanning == OP_AUTO_FOCUS)
		dwImageAddr = 0;	//(DWORD)wAFYStart * (DWORD)nDataBytesPerLine;
	else if (SysParam.bScanning == OP_AUTO_FOCUS_SW)
		dwImageAddr = 0;
	*/
	//GetAllVideoADC will change this flag
	if (bImageMask != 0x01)		//send two images, 0x01, 0x03, 0x07
		pwFpga[READ_ALL_VADC] = 1;
	else
		pwFpga[READ_ALL_VADC] = 0;
	dwImageAddr = 0;
	//pwFpga[ENABLE_VIDEO_ADC] = 1;	//power up video ADC chip
	delay_us(2); //!!! important delay
	AutoScanStart();
	if (bRegionScan == 1) {
		if (NV_Settings.bUP2DN == 1) { //from top to bottom
			nPY = nRegionYEnd;
			nYEnd = nRegionYStart + 1;
		}
		else { //from bottom to top
			nPY = nRegionYStart;
			nYEnd = nRegionYEnd - 1;
		}
	}
	else {
		if (NV_Settings.bUP2DN == 1) { //from top to bottom
			nPY = nScanPixelNumY - 1;	//nPY = (nScanPixelNumY - 1) to 0
			nYEnd = 0;
		}
		else { //from bottom to top
			nPY = 0;					//nPY = 0 to (nScanPixelNumY - 1)
			nYEnd = nScanPixelNumY - 1;
		}
	}
	dwImageAddr = nPY * nDataBytesPerLine;
	nYInc = (SysParam.bScanning == OP_AUTO_FOCUS) ? nYNdxInc :
		(SysParam.bScanning == OP_AUTO_FOCUS_SW) ? nYNdxInc : 1;
	//
	//nScanTimeMax = 128000 / nScanPixelNumX;
	nScanTimeMax = 256000 / nScanPixelNumX;
	if (nScanTimeMax < 64)
		nScanTimeMax = 64;
	//nEDSDWT time unit msec
	nV = nScanPixelNumX * nEDSDWT * TICKS_PER_SECOND / 1000 * 2;
	nFPGATimeout = (nEDSHandshake == 0) ? (TICKS_PER_SECOND * 2) : nV;
	SysParam.bTxData = 1;
	nTxTimeout = 0;
	CPU_IO0 = 1;
	while (1)
	{
		nScanTime = 0;
		while ((bACK == 0) && (nBytesIn != nBytesOut)) {
			nTimeout = 0;
			if (bScanAbort == 1) break;
			if (bScanRestart == 1) break;
			if (bScanPause == 1) break; //don't transmit data anymore
			/*
			//if (nResendNum > 0) {
				while (nResendNum > 0) {
					nNdx = nResendY[nResendNum - 1]; //=nPY
					nPtr = nQueAddrBuf[nNdx]; //for RESEND
					if (bImageMask & 0x01) {
						dwImageAddr = nNdx * nDataBytesPerLine;
						TransmitData(nConnType, nDataBytesPerLine, nPtr, 0);
					}
					if (bImageMask & 0x02) {
						dwImageAddr = nNdx * nDataBytesPerLine;
						TransmitData(nConnType, nDataBytesPerLine, nPtr, 1); //ADCCH = 3, SEI
					}
					nResendNum--;
				}
			*/
			//	continue;
			//}
			if ((NV_Settings.bUP2DN == 1) && (nPY - nYInc < nYEnd)) {
				//from top to bottom
				bFinalPacket = 1;
			}
			else if ((NV_Settings.bUP2DN == 0) && (nPY + nYInc > nYEnd)) {
				//from bottom to top
				bFinalPacket = 1;
			}
			else bFinalPacket = 0;
			if (bFinalPacket == 1) {
				//SetDeflectorFineV(DAC_CH_DEFX_F, 0.0);
				//SetDeflectorFineV(DAC_CH_DEFY_F, 0.0);
				if (bDebug == 11) printf("final pkt 1\n");
				else delay_us(100);
				wZMax = pwFpga[GET_VADC_MAX];
				wZMin = pwFpga[GET_VADC_MIN];
				if (wZMax - wZMin < 200) wZMax = wZMin + 200;
			}
			//calculate max, min by FW
#if CALC_MAXMIN == 1
			if (nPY % 4 == 0) {
				for (nNdx = 0; nNdx < 2; nNdx++) {
					pwData = (WORD *)&pbTxBuffer[nNdx][nBytesOut];
					for (i = 0; i < nScanPixelNumX; i += 8) {
						wV = (pwData[i] & 0x00FF); //byte swap
						wV <<= 8;
						wV += ((pwData[i] & 0xFF00) >> 8);
						if (wV > wZMaxFW[nNdx]) wZMaxFW[nNdx] = wV;
						if (wV < wZMinFW[nNdx]) wZMinFW[nNdx] = wV;
					}
				}
			}
#endif
			//nQueAddrBuf[nPY] = nBytesOut; //for RESEND
			if (bImageMask & 0x01) {
				dwImageAddr = nPY * nDataBytesPerLine;
				TransmitData(nConnType, nDataBytesPerLine, nBytesOut, 0); //the first image
			}
			if (bImageMask & 0x02) {
				delay_us(100); //give some time for the first packet process
				dwImageAddr = nPY * nDataBytesPerLine;
				TransmitData(nConnType, nDataBytesPerLine, nBytesOut, 1); //the second image
			}
			nTxTimeout = 0;
			bFirstPacket = 0;
			//bFinalPacket = 0;
			//bPktOrderNdx++;
			if (NV_Settings.bUP2DN == 1)
				nPY = nPY - nYInc;
			else
				nPY = nPY + nYInc;
			if (nEDSHandshake == 1) {
				printf("%d,", nPY);
				if (nPY % 16 == 15) printf("\n");
			}
			nBytesOut = (nBytesOut + nDataBytesPerLine) % nQueSize + nQueStart;
			nScanTime++;
			if (nScanTime > nScanTimeMax) break;
			if (bFinalPacket == 1) break;
		}
		nTxTimeout++;
		if (nTxTimeout > 4) {
			if (bDebug == 11) printf("nTxTimeout\n");
			break;
		}
		//OSTimeDly(1); //process ACK, SCANSTOP command
		//if (bDebug == 11) printf("\r\n");
		/*nScanOver = 0;
		for (i = 0; i < CHECK_SCAN_OVER; i++) {
			if (nBytesIn != nBytesOut) break;
			//vadc_conv_start, running, sysclk, n_reset,
			//vadc_scan_stop, vadc_scan_start, set_dacx, vadc_to_ram,
			//scanning, histo_calc, row_data_ready, eds_rx0,
			//dout_io, delay_us_timeout, get_video_adc, x_descent
			//509E=0101_0000_1001_1110_ (get_video_adc=1)
			wStatus[0] = pwFpga[GET_STATUS];
			wStatus[1] = pwFpga[GET_STATUS2];
			SysParam.bFastScan = (wStatus[0] & ST_SCANNING) ? 1 : 0;
			if (SysParam.bFastScan == 0) nScanOver++;
		}
		if ((bACK == 0) && (nScanOver >= CHECK_SCAN_OVER - 1)) //not scanning
			break;
		*/
		if (bFinalPacket == 1) {
			dwGradient = GetGradient();
			if (bDebug == 11) printf("final pkt 2\n");
			break;
		}
		OSTimeDly(1); //process ACK, SCANSTOP command
		nDelayTime++;
		if (bScanAbort == 1) break;
		if (bScanRestart == 1) break; //scan restart again
		if (bScanPause == 1) break;
		if (bAutoFocusStart == 1) break; //go into auto focus mode
		nTimeout++;
		if (nTimeout > nFPGATimeout) { //video ADC chip crash
			//vadc_conv_start(0), running, sysclk, n_reset(3),
			//vadc_scan_stop(4), vadc_scan_start, set_dacx, vadc_to_ram,
			//scanning(8), histo_calc, row_data_ready, n_eneds_hw,
			//dout_io(12), delay_us_timeout, get_video_adc, x_descent
			//x5094->0101_0000_1001_0100,delay_us_timeout
			//x3294->0011_0010_1001_0100,row_data_ready
			sprintf(szBuffer, "FPGA->NB TIMEOUT,ST1=%04X,ST2=%04X", wStatus[0], wStatus[1]);
			SaveErrorMessage(szBuffer, 1);
			nRet = ERROR_FAIL;
			break;
		}
	} //while(1)
	SysParam.bTxData = 0;
	AutoScanStop(); //MUST!!
	SetDeflectorFineV(DAC_CH_DEFX_F, 0.0);
	SetDeflectorFineV(DAC_CH_DEFY_F, 0.0);
	//
	AutoScanStop(); //MUST!!
	bDB3Busy = 0;
	//if (nDelayTime <= 0) //at least once
	if (NV_Settings.bScanPowerSave == 1)
		OSTimeDly(1); //MUST!! for stable command process, lower power consumption
	CPU_IO0 = 0;
	return nRet;
}

/*-------------------------------------------------------------------
 User Main
 ------------------------------------------------------------------*/
void UserMain(void * pd)
{
	int i;
	char szBuffer[128];
	//BYTE bAddr;
	int nTick = 0;
	WORD wV = 0x8000, wStep = 0x200;
	WORD wStatus;
	//
	InitializeStack();		// Initialize the TCP/IP Stack

	if ( EthernetIP == 0 )
	{
		iprintf( "Trying DHCP\n" );
		GetDHCPAddress();
		iprintf( "DHCP assigned the IP address of :" );
	}
	else
	{
		iprintf( "Static IP address set to :" );
	}
	ShowIP( EthernetIP );
	iprintf( "\n" );
	//
	EnableAutoUpdate();		// Enable network code updates
	OSChangePrio(MAIN_PRIO);	// Set this task priority
	//
	watchdog_service_function = ResetWDT;
	//
	srand(1);
	#ifdef _DEBUG
	InitializeNetworkGDB();
	iprintf("Debug mode\n");
	#else
	iprintf("Release mode\n");
	#endif
	//
	iprintf("init io start\n");
	InitIO();
	iprintf("init io end\n");
	//
	pwFpga[SET_RELAY_ON] = 0;
	pwFpga[SET_SHUNT] = 0;
	pwFpga[SET_SENR_SEL] = 0;
	pwFpga[SET_VOP_ON] = 0; //VOP
	pwFpga[SET_OBJ_ON] = 0;	//obj off
	pwFpga[SET_SEL_COIL] = 0; //select whole coil
	SetEDSOn(0);
	//
	Reset_FPGA();
#if IO_PCB_VER >= 1
	iprintf("reset io\n");
	Reset_IO();
	//GetIO_Version();
#endif
	//GenerateSinePattern();
	//
	iprintf("check NV\n");
	CheckNVSettings();
	NV_Settings.bEnableVent = 0; //always 0 at power on stage
	fOverscan = NV_Settings.fOverscan;
	//GREEN ON in initial state
	SetLEDState(LEDG, ST_LED_SOLID);
	// initialization of TCP/IP connection
	for (i = 0; i < NFDS; i++)
		fdSet[i].fdConn = 0;
	//
	ResetWDT();
	//
	//CheckNVSettings();
	ClearErrorMessage();
	iprintf("init task\n");
	Set1USClkNum(); //hardware 1 us delay
#if USE_FIFO == 1
	EnableFIFO((int)NV_Settings.bEnableFIFO);
#endif
	//
	InitSysParam();
	InitSetParam();
	InitTurbo();
	//OSSemInit(&semUdpTransfer, 0);
	/* Create task to recieve UDP packets. We will pass the destination
	* port number in the optional second parameter field, and set the
	* priority to 1 less than the UserMain priority so packets get
	* processed as they are received.
	*/
	//#define UDP_READ_PRIO	MAIN_PRIO-1
	//#define SCAN_PRIO		MAIN_PRIO-2
	SerialClose( 0 );
	fdSerial[0] = OpenSerial( 0, 115200, 1, 8, eParityNone ); //NV_Settings.nBaudrate[0] reserved
	ReplaceStdio( 0, fdSerial[0] );   // stdin via UART 0
	ReplaceStdio( 1, fdSerial[0] );   // stdout via UART 0
	ReplaceStdio( 2, fdSerial[0] );   // stderr via UART 0
	//BYTE OSTaskCreate( void ( * task )( void * taskfunc ), void * data, void * pstacktop, void * pstackbot, BYTE priority );
//*/
	/*if (OSTaskCreate( TimerTask,
			(void  *)UDP_LISTEN_PORT,
			&TimerTaskStack[USER_TASK_STK_SIZE] ,
			TimerTaskStack,
			MAIN_PRIO - 3)!= OS_NO_ERR)
			iprintf("TimerTask creation error\n");	// higher priority than UserMain
	*/
	// TimerTask ticks every 200 msec, use interrupt instead
	iprintf("create task\n");
	if (OSTaskCreate( Serial0Task,			//serial 0 is used for debug
			0,
			&Serial0TaskStack[USER_TASK_STK_SIZE] ,
			Serial0TaskStack,
			MAIN_PRIO - 3)!= OS_NO_ERR)
			printf("Serial0Task creation error\n");	// higher priority than UserMain
	if (OSTaskCreate(Serial1Task,			//serial 1 is used to control HV
			0,
			&Serial1TaskStack[USER_TASK_STK_SIZE],
			Serial1TaskStack,
			MAIN_PRIO - 4) != OS_NO_ERR)
			printf("Serial1Task creation error\n");
#if ENABLE_UART2 == 1
	if (OSTaskCreate(Serial2Task,			//serial 2 is used to motor
			0,
			&Serial2TaskStack[USER_TASK_STK_SIZE],
			Serial2TaskStack,
			MAIN_PRIO - 2) != OS_NO_ERR)
			printf("Serial2Task creation error\n");
#endif
/*
	if (OSTaskCreate(Serial3Task,			//serial 3 is reserved
			0,
			&Serial3TaskStack[USER_TASK_STK_SIZE],
			Serial3TaskStack,
			MAIN_PRIO - 2) != OS_NO_ERR)
			printf("Serial3Task creation error\n");
*/
	if (OSTaskCreate(VacuumStateTask,
			0,
			&VacuumStateTaskStack[USER_TASK_STK_SIZE],
			VacuumStateTaskStack,
			MAIN_PRIO - 5) != OS_NO_ERR)
			printf("VacuumStateTask creation error\n");
	if (OSTaskCreate( ScanMonitorTask,
			(void  *)UDP_LISTEN_PORT,
			&ScanMonitorTaskStack[USER_TASK_STK_SIZE] ,
			ScanMonitorTaskStack,
			MAIN_PRIO - 6)!= OS_NO_ERR)
			printf("ScanMonitorTask creation error\n");	// higher priority than UserMain
	//UdpReadTask() has the highest priority
	if (OSTaskCreate( UdpReaderTask,
			( void * )UDP_LISTEN_PORT, //command port
			&UdpReaderStack[USER_TASK_STK_SIZE],
			UdpReaderStack,
			MAIN_PRIO - 7) != OS_NO_ERR)
			iprintf("UdpReaderTask creation error\n");
	if (OSTaskCreate( TcpListenTask,
			( void * )TCP_CMD_PORT,
			&TcpListenStack[USER_TASK_STK_SIZE],
			TcpListenStack,
			MAIN_PRIO - 1) != OS_NO_ERR)
			iprintf("TcpListenTask creation error\n");
	//MAIN_PRIO(50), HTTP_PRIO (45), PPP_PRIO (44), TCP_PRIO (40), IP_PRIO (39)
	//
	SerialClose( 1 ); // close UART 1
	fdSerial[1] = OpenSerial( 1, NV_Settings.nBaudrate[1], 1, 8, eParityNone );
	if (fdSerial[1] < 0)
		printf("serial1 open error\n");
	//
	ResetWDT();
#if ENABLE_UART2 == 1
	SerialClose( 2 ); // close UART 2
	fdSerial[2] = OpenSerial( 2, NV_Settings.nBaudrate[2], 1, 8, eParityNone );
	if (fdSerial[2] < 0)
		printf("serial2 open error\n");
	//must call these two pins settings again
	//MOD54415				MOD5270B			MOD5282
	//UART9_RX: J2_41		UART2_RX: J2_41		UART2_RX: J2_41
	//UART9_TX: J2_44		UART2_TX: J2_44		UART2_TX: J2_42 or 44
	J2[42].function(0);	//reserved, SCL
	J2[39].function(0);	//reserved, SDA
#if MODULE_TYPE == MT_MOD5270B
	J2[44].function(PINJ2_44_UART2_TX);
	J2[41].function(PINJ2_41_UART2_RX);
#else //MOD5282
	J2[44].function(PINJ2_44_UTXD2);
	J2[41].function(PINJ2_41_URXD2);
#endif
#endif
	//
#if ENABLE_PID == 1
	SPI_SetBaud(0, NV_Settings.nPIDBaudrate[0]);
	SPI_SetBaud(1, NV_Settings.nPIDBaudrate[1]);
#endif
	//
	//OSCritInit(&OsCritical);
	//SetupPITR(1, 10);	//start 10 Hz timer (100 ms)
	//
#if ENABLE_IRQ1 == 1	//row data ready interrupt
	SetupIRQ1(1);		//IRQ1 rising edge interrupt
#endif
#if ENABLE_IRQ3 == 1	//power line modulation interrupt
	SetupIRQ3(1);		//IRQ3 falling edge interrupt
#endif
	iprintf( "===== Starting %s Program =====\r\n", AppName );
	//no mid delay
	//sprintf(szBuffer, "SETAVGNUM:%d#DELAY:5:0:20#", nVADCAvgNum);
	//use only one channel, SHADOW:0
	//use 4 channels, SHADOW:5
	ResetWDT();
#if SCAN_PCB_VER >= 17
	if (NV_Settings.bBSEMode == 2)
		sprintf(szBuffer, "SHADOW:11#SETAVGNUM:%d#", nVADCAvgNum);
	else
		sprintf(szBuffer, "SHADOW:1#SETAVGNUM:%d#", nVADCAvgNum);
#else
	sprintf(szBuffer, "SHADOW:0#SETAVGNUM:%d#", nVADCAvgNum);
#endif
	DecipherMultipleCommand(CONN_UART0, szBuffer); //default value
	//
	//nShadowMode = 0;
	//pwFpga[SET_SHADOW_MODE] = 0;
	if (NV_Settings.bEnableWDT == 1) {
		printf("WDT Enabled\n");
		EnableWDT(1, WDT_TIMEOUT);
	}
#if ENABLE_PID_UART == 1
	printf("PID_UART Enabled\n");
#else
	printf("PID_UART Disabled\n");
#endif
	while (1)
	{
		if (NV_Settings.bEnableWDT == 1) //enable watch dog timer
			ResetWDT();
		OSTimeDly(TICKS_PER_SECOND);
		//
		lTick1S++;
		if (nTick == 4) { //strange bug of power LED green, TBD, MUST be larger than 4!!!
			SetVOPCOn(1);
			if (NV_Settings.bLEDAuto == 1)
				SetIO_LED_Auto(1);
#if IO_PCB_VER >= 6
			SetIO_VAC_Gauges(1); //turn on gauge power
#endif
			LEDTest(1);
		}
		else if (nTick == 5) { //strange bug of power LED green, TBD, MUST be larger than 4!!!
			if (NV_Settings.bLEDAuto == 1)
				SetIO_LED_Auto(1);
#if IO_PCB_VER >= 6
			SetIO_VAC_Gauges(1); //turn on gauge power
#endif
			SetPLParameter(1); //set PL parameters to FPGA
		}
		else if (nTick == 6) {
			LEDTest(0);
			InitAnalog();
			pwFpga[WAIT_EOC] = NV_Settings.wWaitEOC;
			DecipherMultipleCommand(CONN_UART0, "SCANSTOP");
		}
		else if (nTick == 7) {
			SetObjR((int)NV_Settings.bObjR);
			fObjStandbyV = CalculateObjStandbyV();
			SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fObjStandbyV);
		}
 		else if (nTick == 8) {
			DecipherMultipleCommand(CONN_UART0, "SETZOOM:0:0:60:600:450#");
			printf("INIT OK\n");
			bInitOK = 1;
		}
		if (nTick < 100) //power on timer tick for init process
			nTick++;
		//
		if (bTestMode != 0) {
			if (bTestMode == 1)
				SetTestDAC();
			else if (bTestMode == 2) //test deflector
				TestDeflector();
			else if (bTestMode == 3) //test stigmator
				TestStigmator();
			else if (bTestMode == 4)
				TestObjective();
			bTestMode = 0;
		}
		if (bLEDTest > 0) {
			bLEDTest--;
			//printf("LED test %d\r\n", (int)bLEDTest);
			if (bLEDTest == 0)
				LEDTest(0);
		}
		//SHORT --> PIN=H --> PZT_MOVE_X = 1
		SysParam.bPZTMoveX = (PZT_MOVE_X == 1) ? 1 : 0; //J2[10], PIN 1,
		SysParam.bPZTMoveY = (PZT_MOVE_Y == 1) ? 1 : 0; //J2[9], PIN 3, SHORT=0
		if (bInitOK == 0)
			continue;
		// external EDS control
		if (NV_Settings.bEDSOnByHW == 1) {
			wStatus = pwFpga[GET_STATUS];
			wStatus &= 0x0010;
			if ((wStatus == 0) && (bEnableEDS == 0)) { //N_ENEDS_HW==0, ON
				EnableEDS(1);
			}
			else if ((wStatus != 0) && (bEnableEDS == 1)) { //OFF
				EnableEDS(0);
			}
		}
		//
		if (bDebug == 2) {
			if ((lTick1S % 5) == 0)
				ShowVacuumStatus(CONN_UART0);
		}
		if (bDebug == 15) {
			DecipherCommand(CONN_UART0, "GETAII:1");
		}
		if (bDebug == 16) {
			//SetDeflectorFineV(DAC_CH_DEFX_F, 1.0);
			//SetDeflectorFineV(DAC_CH_DEFY_F, 1.0);
			if (wV > 0x9000) wStep = 0xFD00;
			if (wV < 0x7000) wStep = 0x200;
			wV += wStep;
			SetFineDACXY(wV, wV);
		}
		//------------------------------------------------
		//check IO status
		//------------------------------------------------
		SysParam.bCPU_IO[0] = CPU_IO0;
		SysParam.bCPU_IO[1] = CPU_IO1;
		SysParam.bCPU_IO[2] = CPU_IO2;
		SysParam.bCPU_IO[3] = CPU_IO3;
		//
		CheckHVTime(); //called every seconds to calculate HV_ON time
		if (bSerial2Got == 1) {
			bSerial2Got = 0;
		}
		if (((bEnableIO == 1) || (bEnableIO == 3)) && (bDebug < 22)) {
			GetAllVoltage(NULL); //update ADC value every seconds
		}
		if ((bEnableIO == 1) && (bDebug < 22))
			GetDigitalInput(1); //execute once per second
		if (bDebug == 22) {
			SetVoltage (BOARD_IO, 0, 2.0); //nCh=0, chip = 0
			SetVoltage (BOARD_IO, 8, 2.0); //nCh=8, chip = 1
		}
		else if (bDebug == 23) {
			WriteIOSerialDAC32b(0, (DWORD)0x08000001); //use internal reference
		}
	}
}
