/* IO register initialization */
/*****************************************************************
 * nCS1: used for SRAM access
 * nCS2: used for FPGA, FIFO access
 * nCS3: reserved
*****************************************************************/
#include <predef.h>
#include <stdio.h>
#include <string.h>
#include <serial.h>
#include <NetworkDebug.h>
#include <math.h>	//fabs()
#include "mod_sel.h"
#include <basictypes.h>		//BYTE, PWORD
#include <startnet.h>		//TICKS_PER_SECOND
#include <pins.h>
#include "main.h"
#include "cpu_cmd.h"
#include "io.h"
#include "data_struct.h"
#include "function.h"
#include "Turbo_Control_Protocol.h"
#include <ucos.h>
//
#define DEBUG_SIO				0
#define RS485_BYTE_DELAY		12
//
#define DAC_INIT_CMD		0x800C //GAIN=0,BUF=1,VDD=0
#define DAC_REF_RATIO		1.01		//used to avoid overflow
//
//#define BASE_ADDR			0xA0000000
#define SRAM_BASE_ADDR		0xB0000000	//A100_0000 ~ A7FF_FFFF, 16 MBytes
#define FPGA_BASE_ADDR		0xB2000000	//A800_0000 ~ A8FF_FFFF, 1 Mbytes
#define FIFO_BASE_ADDR		0xB4000000	//A800_0000 ~ A8FF_FFFF, 1 Mbytes
//
#define IO_TEST_SAMPLE_NUM	3		//minimum 3
//
//#define IO_TEST_SAMPLE_1	7
//
//#define TEST_PIN		J2[16]
/*
static volatile PWORD pwSram;	//nCS1
static volatile PWORD pwFpga;	//nCS2
static volatile PWORD pwFifo;	//nCS2, //nCS3
*/
volatile PWORD pwSram;	//nCS1
volatile PWORD pwFpga;	//nCS2
volatile PWORD pwFifo;	//nCS2, //nCS3
//
BYTE bInitIO = FALSE;
WORD wDACFAddrX;
WORD wDACFAddrY;
WORD wSCAN_DO = 0;	//SCAN board output
WORD wIO_DO = 0;	//IO board output
WORD wIO_DI = 0;	//IO board input
#if GAUGE_POWER_CTRL == 1
WORD wDIO_OUT = 0x01C0;	//IDC10 output
WORD wDIO_IN = 0x0000;	//IDC10 input
#else
WORD wDIO_OUT = 0;
#endif
WORD wDI = 0;
WORD wDO = 0xFFFF;
WORD wScanFPGAVer = 0;
WORD wIOFPGAVer = 0;
BYTE bEDSOn = 0;
//
WORD wIOADDA = 0;
//BYTE bCaseClose = DOOR_OPEN;
//BYTE bChamClose = DOOR_OPEN;
BYTE bEDSHWOn = 0;
BYTE bSetDOBit = 0;
int nOverscan = 0;
float fOverscan = 0.06;
float fOverscanP1 = 1.0;
//
//int nObjOn = 0;
//int nTurboPumpOn = 0;
//int nScrollPumpOn = 0;
//int nSenrSel = 0;
int nCoilSwitch = 0;
//int nCoilShunt = 0;
float fRatioShunt = 1.0;
float fRatioShuntMax = 1.0;
int nSenRSet = 0;
int nShuntSet = 0;
int nSelSmallDef = 0; //0:whole coil, 1: small coil
//
int nVacOn = 0;
int nVacOff = 0;
int nVacOffSent = 0;
int nGotoAir = 0;
int nVacSwap = 0;
long lVacTotalTime = 0;
//
int nShadowMode = SHADOW_4CH; //0~3:SINGLE(1CH),4:SUM(4CH),5:HDIFF,6:VDIFF,
//
//BYTE bCoilZoomCtrl = 0;
//BYTE bZoomCtrl = 0; //=0,1,2,3,4
//float fMagNow = MAG_MIN;
float fMagMinimum = 30.0;
float fMagMaximum = 10000.0;
float fImageWidthReal = 1.0; //mm
float fIMaxAbs = 1.05;
float fObjV = OBJ_ON_IMIN_V;
float fInitialX = 0;
float fInitialY = 0;
float fFinalY = 4.0;
//
//BYTE bHVPower = 0;
//BYTE bGVPower = 0;
//BYTE bVOPPower = 0;
//int nHVON = 0;
int nHVOnTime = 0;
//int nGateValveState[4] = {VALVE_CLOSE, VALVE_CLOSE, VALVE_CLOSE, VALVE_CLOSE};
BYTE bVACBypassOK[VAC_GAUGE_NUM_MAX] = {VAC_NONE, VAC_NONE, VAC_NONE, VAC_NONE};
float fVACLevel[VAC_GAUGE_NUM_MAX] = {3.1, 1.2, 3.1, 5.0};
float fVACLevelBuf[VAC_GAUGE_NUM_MAX][VAC_BUF_NUM] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}}; //volt/sec
float fVACChangeRate[VAC_GAUGE_NUM_MAX] = {0, 0, 0, 0}; //volt/sec
//
int nInterlockCode = SUCCESS;
float fDACRefRatio;
OS_CRIT OsCrit;
float fTemperature[TEMP_SENSOR_NUM_MAX] = {0, 0, 0, 0, 0};
//
int nIOPCBVersion = IO_PCB_VER;
//int nScanDirection = 1; //1:DACX ascending, -1: DACX descending
//
float fRatioCoil = 1.0; //use small or all coil
float fRatioVDAC = 1.0;
float fRatioTrigno = 1.0;
float fRatioFocus = 1.0;
//float fDefSenR = DEF_SEN_R0; //10 ohm(DEF_X10=0), 5 ohm(DEF_X10=1)
float fRatioDefSenR = 1.0;
float fRatioDefSenRMax = 1.0;
//float fRatioSineCosine = 1; //1 or 1.6
//
extern int fdNet; //file handle for TCP/IP connection
extern int nScanIntervalTick;
extern int nScanPixelNumX;
extern int nScanPixelNumY;
extern int nScanPixelNumXDV;
extern int nScanPixelNumYDV;
extern BYTE bPktNum;
extern int nDataBytesPerLine;
//extern int nBytesPerLine;
//extern int nTotBytePerPacket[PKT_NUM_MAX];
extern int nDataBytePerPacket[PKT_NUM_MAX];
extern int nBytePerPixel;
extern int nQueSize;
extern int nPixelPerPacket[PKT_NUM_MAX];
extern int nEDSHandshake;
//extern int nConnType;
//
extern BYTE bScanAbort;
extern BYTE bChkTP;
extern BYTE bDB2Busy;
//extern BYTE bScaleMode;
//extern BYTE bEnableRotation;
//extern BYTE bScanning;
extern BYTE bDebug;
extern BYTE bEnableEDS;
//
extern BYTE bFPGAMajorVersion;
extern BYTE bFPGAMinorVersion;
extern BYTE bFPGADateMonth;
extern BYTE bFPGADateDay;
//
extern BYTE bIOFPGAMajorVersion;
extern BYTE bIOFPGAMinorVersion;
extern BYTE bBlanking;
//----------------------------------------------------------------------
// Power: Green On/Off
// HV: Red On/Off
// Vacuum: Blue On (ready), Off not ready. Blinking: processing
//----------------------------------------------------------------------
extern BYTE bEnableLED;
extern BYTE bLEDState[LED_NUM];
//
extern WORD wDACX[PIXEL_NUM_X_MAX_PI];
extern WORD wDACY[PIXEL_NUM_Y_MAX];
extern WORD wDACDX[PIXEL_NUM_X_MAX_PI];
extern WORD wDACDY[PIXEL_NUM_Y_MAX];
extern float fDACXFSet;
extern int nEnableDACXFSet;
WORD wDACX_Min;
WORD wDACX_Max;
WORD wDACY_Min;
WORD wDACY_Max;
//
extern int nScanDelay[PIXEL_NUM_X_MAX_PI];
extern NV_SETTINGS NV_Settings;
extern SYS_PARAM_3 SysParam;
extern SYS_PARAM_2 SysParam2; //for HV
//
extern int fdSerial[4];

extern int nPrevConn;
extern int nIdleTimeout;
//
extern int nTurboRotationSpeed;
extern int nTurboReadySpeed;
extern int nTurboTemperature;
//
extern int nConnTest;
//extern int nIGTimeout;
//
//extern float fMaxXAtDEFX0, fMaxYAtDEFX0;
//extern float fRangeMax[2];
//extern float fRangeMin;
//
// index = 0:DEFX, 1:DEFY, 2:OBJ
//
float fVoltCoarse[COIL_CH_NUM];
float fVoltFine[COIL_CH_NUM];
float fCoarseRatio[COIL_CH_NUM];	//deflector X and deflector Y
float fFineRatio[COIL_CH_NUM];
float fRMagnify = 1;
float fXYRatio[2] = {1.0, 1.0};
float fARatio[2] = {1.0, 1.0}; // coil strength ratio
extern float fFocus;
//float fSensorR[SIG_CH_NUM];		//sense resistor, 1: 1 ohm, or 10: 10 ohms, Zoom area
//float fLoadR[SIG_CH_NUM]; 		//coil resistor
// (0-1.5)/2*8=-6,(2.5-1.5)/2*8=4
//
//-------------------------------------------------------------
//BR1(0),BR2(1),BR0(2),CO(3),						//SCAN
//VID_REF(4),BR3(5),SINE(6),COSINE(7),				//SCAN
//BR4(8),CO1(9),BR(10),CO2(11)						//SCAN
//NBSE0(12),NBSE1(13)								//SCAN
//----------------------------------------------------------
//AL0(0),AL1(1),AL2(2),AL3(3),  					//IO coil control
//STIGX(4),STIGY(5),OBJ_C(6),OBJ_F(7),				//IO
//RESV0(8),RESV1(9),RESV2(10),RESV3(11),			//IO PMT control
//HV_FILA(12),HV_BIAS(13),HV_ACC(14),HV_FEG(15)		//IO
//-------------------------------------------------------------
float fDACVMax[BOARD_NUM][16] = {
		{5.0, 5.0, 5.0, 10,
		DACF_OUT_MAX, 5.0, DAC_SINE_COSINE_MAX, DAC_SINE_COSINE_MAX,
		5.0, 10.0, 5.0, 10.0,
		5.0, 5.0, 5.0, 5.0},
		{5.125, 5.125, 5.125, 5.125, //coil current control, AL
		5, 5, 10, 10,
		5, 5, 5, 5, 				//PMT control, RESV
		10.5, 10.5, 10.5, 10.5}}; 	//HV control
float fDACVMin[BOARD_NUM][16] = {
		{-5.0, -5.0, -5.0, 0,
		0, -5.0, -DAC_SINE_COSINE_MAX, -DAC_SINE_COSINE_MAX,
		-5, 0, -5, 0,
		-5, -5, -5, -5},
		{-5.125, -5.125, -5.125, -5.125, //coil current control,AL
		-5, -5, 0, 0,
		0, 0, 0, 0,				//PMT control, RESV
		0, 0, 0, 0}};			//HV control
float fVDACVMax[VDAC_CH_NUM_MAX] = {DACF_PV, DACF_PV};
float fVDACVMin[VDAC_CH_NUM_MAX] = {DACF_MV, DACF_MV};
//------------------------------------------------------------------------
//
BYTE bADCGet[DAC_CH_NUM_MAX] = {0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0};
/*float fADCVal[ADC_CH_NUM_MAX] = {0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0};*/
#if SCAN_PCB_VER >= 10
int nEnableADC[ADC_CH_NUM_MAX] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
#elif SCAN_PCB_VER >= 9
int nEnableADC[ADC_CH_NUM_MAX] = {1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1};
#else
int nEnableADC[ADC_CH_NUM_MAX] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
#endif

float fCoarse2FineRatio = SCAN_C2FR_1; //deflector coarse to fine ratio
float fObjScale = OBJ_C2FR_1;
BYTE bInitDA = 0;
//
float GetVoltageEx(int nCh);
//
/*
	voltage range: -10 volts to +10 volts
	12 bit DAC nMax = 4095
	16 bit ADC nMax = 65535
*/
WORD VoltToWord(float fV, int nMax, float fVMax, float fVMin)
{
	WORD wV;
	if (fV > fVMax)
		fV = fVMax;
	else if (fV < fVMin)
		fV = fVMin;
	wV = ((fV - fVMin) / (fVMax - fVMin)) * (nMax - 1);
	return wV;
}
/* wV = 0 ~ (DAC_12BIT-1) */
float WordToVolt(WORD wV, int nMax, float fVMax, float fVMin)
{
	float fV;
	fV = (float)wV * (fVMax - fVMin) / (nMax - 1) + fVMin;
	return fV;
}

/*
	get the sum voltage of coarse and fine
	TotalVolt = V2;
	nCh = COIL_CH_DEFX, COIL_CH_DEFY, COIL_CH_OBJ
*/
// nCh =0(COIL_CH_DEFX),1(COIL_CH_DEFY),2(COIL_CH_OBJ)
float GetTotalVolt(int nCh)
{
	float fTotalVolt;
	fTotalVolt = fCoarseRatio[nCh] * fVoltCoarse[nCh] + fFineRatio[nCh] * fVoltFine[nCh];
	return fTotalVolt;
}
// BID = board ID
void SetBoardID(int nNewBID)
{
	static int nOldBID = -1;
	if (nNewBID == nOldBID)
		return;		//return if board ID is not changed
	J2[6] = (nNewBID & 0x01) ? 1 : 0;
	J2[7] = (nNewBID & 0x02) ? 1 : 0;
	J2[8] = (nNewBID & 0x04) ? 1 : 0;
	nOldBID = nNewBID;
}

void InitIO(void)
{	int i;
	//
	sim.cs[1].csar = ( SRAM_BASE_ADDR >> 16 );
	//bit 8: WP=0, read/write are allowed
	//bit 0: V=1, nCS1 chip select valid
	//CSMR: BAM(31,16)_XXXXXXX_WP_X_AM_CI_SC_SD_UC_UD_V
	//4100_0000 -> 44FF_0000
	sim.cs[1].csmr = 0x00FF0001;			//16 Mbytes=24 bit, 24-16=8 bits=0xFF
	//13-10: wait state = 1000
	//8: AA=1, auto acknowledge is disabled
	//7-6: 1x, 16-bit port size
	//5: BEM=0, byte enable mode disabled
	//CSCR: XX_WS_x_AA_PS1_PS0_BEM_BSTR_BSTW_XXX
	sim.cs[1].cscr = 0x2180;				//00_10(2)00_0_1(1)_1x_00(8)0000(0)
	pwSram = ( PWORD ) SRAM_BASE_ADDR;
	//
	sim.cs[2].csar = ( FPGA_BASE_ADDR >> 16 );
	//bit 8: WP=0, read/write  are allowed
	//bit 0: V=1, nCS2 chip select valid
	//4500_0000 -> 45FF_0000
	sim.cs[2].csmr = 0x000F0001;			//1 Mbytes=20 bit, 20-16=4 bits=0x0F
	//13-10: wait state = 00_1000_X_1_1X_XXX000
	//8: 1, auto acknowledge is enabled
	//7-6: 1x, 16-bit port size
	sim.cs[2].cscr = 0x2180;				// 00_10(2)00_0_1(1)_1x_00(8)0000(0)
	pwFpga = ( PWORD ) FPGA_BASE_ADDR;
	//
	// nCS3 keep no change
	sim.cs[3].csar = ( FIFO_BASE_ADDR >> 16 ); //bit[15-0], base address
	//bit31-16:base address mask
	//bit 8: WP=0, read/write  are allowed
	//bit 0: V=1, nCS3 chip select valid
	sim.cs[3].csmr = 0x000F0001;			//1 Mbytes=20 bit, 20-16=4 bits=0x0F
	//13-10: wait state = 10_00
	//8: 1, auto acknowledge is enabled
	//7-6: 1x, 16-bit port size, 01, 8-bit port size
	sim.cs[3].cscr = 0x2180;				// 00_10(2)00_0_1(1)_1x_00(8)0000(0)
	pwFifo = ( PWORD ) FIFO_BASE_ADDR;
	// d:\nburn\MOD5282\system\ioboard.cpp
	//sim.cs[3].csar = ( BASE_ADDR >> 16 );
	//Base address mask(31-16),5 bit set, 2^21 -> 2 MBytes, V(valid=1)
	//4000_0000 -> 401F_0000
	//sim.cs[3].csmr = 0x001F0001;
	//00_1000_0_1__01_0___0____00___0_0
	//R__WS___R_AA_PS_BEM_BSTR_BSTW
	sim.cs[3].cscr = 0x2140;                  // 0010 0001 0100 0000
	//
	J2[6].function(0);	//PID_TX_EMPTY
	J2[6].hiz();
	J2[7].function(0);	//NC
	J2[8].function(0);	//PID_RX_DATA
	J2[8].hiz();
	//
	J2[9].function(0);		//nCASE_CLOSE
	J2[10].function(0);		//nCHAM_CLOSE
	J2[9].hiz();
	J2[10].hiz();
	//
	//DATA_RnW = 1; //reserved, not used
	CPU_IO0.function(0);		//J2[11],CPU_IO0
	//CPU_IO0.hiz();
	CPU_IO0.set(0);
	CPU_IO1.function(0);		//J2[19],CPU_IO1
	//CPU_IO1.hiz();
	CPU_IO1.set(0);
	CPU_IO2.function(0);		//J2[17],CPU_IO2
	//CPU_IO2.hiz();
	CPU_IO2.set(0);
	CPU_IO3.function(0);		//J2[24],CPU_IO3
	//CPU_IO3.hiz();
	CPU_IO3.set(0);
	//
	//nSCAN_START = 1;
	J2[12].function(0);		//DEF_X10
	J2[12].set(0);			//DEF_X10=0, NC X1
	J2[13].function(0);		//COIL_SWITCH
	J2[13].set(0);
	J2[15].function(0);		//
	J2[15].set(0);			//EDX_CTRL=0
	//
	J2[18].function(0);		//CS_PID
	J2[18] = 0;
	J2[36].function(0);		//DOUT_PID
	J2[36].hiz();
	J2[20].function(0);		//MUX_CTRL1
	//
	J2[26].function(0);
	J2[26] = 1;				//N_RESET_FPGA
	//
	//VER 5 use J2_17 to control RS-485
	J2[17].function(0);
	//VER 7 use J2_38 to control RS-485
	//
	LED_GREEN.function(0);
	LED_GREEN_OFF;
	LED_RED.function(0);
	LED_RED_OFF;
	//
#if SCAN_PCB_VER >= 11
	RS485_TX1.function(0);
	RS485_TX1 = 0;
	RS485_TX2.function(0);
	RS485_TX2 = 0;
	RS485_TX3.function(0);
	RS485_TX3 = 0;
#else
	J2[38].function(0);
	RS485_TX_DISABLE;
	J2[31].function(0);		//RELAY_ON control
	J2[31] = 0;
#endif
	J2[32].function(0);	 //nCS_DAC0
	J2[32] = 0;
	// VER 6... use J2_38(RTS) to control RS-485
//#if SCAN_PCB_VER >= 6
//	J2[38].function(PINJ2_38_URTS1); //2:UART0 RTS, 3:UART1 RTS
//	Serial485HalfDupMode(1, 1); //enable UART1 half duplex
//#endif
	//
	LED_BLUE.function(0); //LED blue
	LED_BLUE_OFF;
	LED_YELLOW.function(0); //LED yellow
	LED_YELLOW_OFF;
	//
#if SCAN_PCB_VER >= 9
	J2[42].function(0); //1-wire
	J2[42].hiz();
	//UART2 is not available on all platform!!!???
	//However, after then calling OpenSerial for COM2, the port did not work. It turns out we need to call OpenSerial first, then call the following to make everything OK:
	//J2[42].function(PINJ2_42_SCL);
	//J2[39].function(PINJ2_39_SDA);
	//J2[44].function(PINJ2_44_UTXD2);
	//J2[41].function(PINJ2_41_URXD2);
/*
#if ENABLE_UART2 == 1
	#if MODULE_TYPE == MT_MOD5270B
		J2[44].function(PINJ2_44_UART2_TX);
		J2[41].function(PINJ2_41_UART2_RX);
	#else
		J2[44].function(PINJ2_44_UTXD2);
		J2[41].function(PINJ2_41_URXD2);
	#endif
#endif
*/
#endif
	//
	//J2[4].function(PINJ2_4_UTXD0);	//serial 0, default enabled?
	//J2[3].function(PINJ2_3_URXD0);
	//J2[21].function(PINJ2_21_URXD1);	//serial 1
	//J2[22].function(PINJ2_22_UTXD1);
	// objective Zoom scale is fixed
	//SetBoardID(BOARD_SCAN);
	//SetDigitalOutput(BOARD_SCAN, wSCAN_DO);
	//
	//pwFpga[SET_X_LINE_DELAY] = SysParam.wXLineDelay;
	//
	for (i = 0; i < COIL_CH_NUM; i++)
	{
		fVoltCoarse[i] = 0;		//coarse voltage
		fVoltFine[i] = 0;		//coarse voltage
		fCoarseRatio[i] = 1;
		fFineRatio[i] = 1;
	}
	/*for (i = 0; i < ISEN_CH_NUM; i++)
		fSensorR[i] = NV_Settings.fV2IR[i];
	for (i = 0; i < ISEN_CH_NUM; i++)
		fLoadR[i] = NV_Settings.fLoadR[i]; */
	//
	/*
	//!!!strange error, power on fails if added
	SetHVSwitch(0);
	SetDeflectorScale(1);	//set deflector scale
	SetSenrSel(0); //set larger resistor DEF_X10=0
	//SetObjScale(1);	//set objective scale
	SetObjOn(0);
	SetRotationAngle(0);
	EnableVideoADC(1);
	//
	*/
	for (i = 0; i < VAC_GAUGE_NUM_MAX; i++) {
		SysParam.sGaugeStatus[i] = VAC_NONE;
	}
	pwFpga[SET_DELAY_OVERSCAN] = NV_Settings.wDelayOverscan;
	bInitIO = TRUE;
	OSCritInit( &OsCrit );
}

int InitAnalog(void) //called in main while loop
{
	int i;
	float fV;
	//
	printf("init analog\r\n");
	//
#if IO_PCB_VER >= 1		//no ADC chip
	InitSerialDAC(); 	//1 DAC chip
	InitSerialIO();
#elif SERIAL_ADC == 1
	InitSerialADC();
	InitSerialDAC(); //3 DAC chip
#endif
	for (i = 0; i < DAC0_CH_NUM_MAX; i++) 	//SCAN board
	{
		if (i == DAC_CH_VID_REF1) continue;
		if (i == DAC_CH_BR0) continue;
		if (i == DAC_CH_BR1) continue;
		if (i == DAC_CH_BR2) continue;
		if (i == DAC_CH_BR3) continue;
		if (i == DAC_CH_BR4) continue;
		if (i == DAC_CH_BR5) continue;
		if (i == DAC_CH_CO) continue;
		SetVoltage(BOARD_SCAN, i, 0);
	}
	for (i = 0; i < DAC1_CH_NUM_MAX; i++) 	//IO board
	{
		if (i == DAC_CH_OBJ_C) continue;
		if (i == DAC_CH_OBJ_F) continue;
		SetVoltage(BOARD_IO, i, 0);
	}
	//
	SetVoltage(BOARD_SCAN, DAC_CH_VID_REF1, DACF_OUT_MAX); //default video reference voltage=5, ch8
	//
	SetDeflectorFineV(DAC_CH_DEFX_F, 0);
	SetDeflectorFineV(DAC_CH_DEFY_F, 0);
	//
	//DecipherCommand(CONN_UART0, "SETBR:35#SETCO:60#");
	pwFpga[READ_ALL_VADC] = 0;
	fV = PercentToVolt(60.0, DAC_CH_BR); //50 percent
	SetVoltage(BOARD_SCAN, DAC_CH_BR0, fV);
	SetVoltage(BOARD_SCAN, DAC_CH_BR1, fV);
	SetVoltage(BOARD_SCAN, DAC_CH_BR2, fV);
	SetVoltage(BOARD_SCAN, DAC_CH_BR3, fV);
	SetVoltage(BOARD_SCAN, DAC_CH_BR4, fV);
	SetVoltage(BOARD_SCAN, DAC_CH_BR5, fV);
	fV = PercentToVolt(50.0, DAC_CH_CO);  //30 percent
	SetVoltage(BOARD_SCAN, DAC_CH_CO, fV);
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, OBJ_ON_IMIN_V);
	SetVoltage(BOARD_IO, DAC_CH_OBJ_F, 0.0);
	//
	SetRelayOn(1); //relay power on
	OSTimeDly(4);
	SetVOPPower(1); //power for deflector and stigmator
	OSTimeDly(4);
	SetHVSwitch(0);
	SetSenrSel(0); //set larger resistor DEF_X10=0
	SetCoilShunt(0);
	//--------------------------------
	// heat up objective at power on stage
	//--------------------------------
	SetObjOn(1);
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, OBJ_ON_INIT_V);
	//
	return 1;
}

//----------------------------------------------------------------
// the function is used to get fine scan voltage
//fX, fY: position in unit mm
//fX = fXc + fXf	//coarse + fine
//fY = fYc + fYf	//coarse + fine
//fVX, fVY: corresponding voltage at (fX, fY) in unit volt. Coil current is proportional to voltage
// return current unit mA
//----------------------------------------------------------------
void GetVoltFromPos(float fX, float fY, float *pfVX, float *pfVY)
{	//fSensorR[1] = fSensorR[0]
	float fIX, fIY;
	//calculate current first
	fIX = fX * NV_Settings.fAPerMM[0][0]; //unit A
	fIY = fY * NV_Settings.fAPerMM[0][1];
	// distortion correction
	//fIX = a1*fX+a2*fY+a3*theta;
	//fIY = b1*fX+b2*fY+b3*theta;
	*pfVX = fIX * NV_Settings.fV2IR[0] * fRMagnify; //*pfVX unit volt, 10 or 100
	*pfVY = fIY * NV_Settings.fV2IR[1] * fRMagnify;
}
//----------------------------------------------------------------
// Start scanning process
//----------------------------------------------------------------
/*
void StartScan(void)
{	//send nSCAN_START negative pulse
	nSCAN_START = 0;
	delay_us(10);
	nSCAN_START = 1;
	nScanIntervalTick = 0;
}*/
//----------------------------------------------------------------
// set deflector scale, DOUT0, DOU1
//----------------------------------------------------------------
int SetDeflectorScale(int nZoom)
{
#if SHUNT_BY_ZOOM_CTRL == 1
	//bCoilZoomCtrl = 0x00;
	return SUCCESS;
#else
	int nMAG;
	switch (nZoom)
	{
		case 1: //use coarse scan, 0: logig ON, ADG441, fine v = 0
			wSCAN_DO = wSCAN_DO & (~0x000C);	//DEF_SCALE0=0(2), DEF_SCALE1=1(3)
			wSCAN_DO = wSCAN_DO | 0x0000;		//XXXX_00XX
			bCoilZoomCtrl = 0x00;
			fCoarse2FineRatio = SCAN_C2FR_1;
			nMAG = 1;
			break;
		case 2:
			wSCAN_DO = wSCAN_DO & (~0x000C);	//DEF_SCALE0=0(2), DEF_SCALE1=1(3)
			wSCAN_DO = wSCAN_DO | 0x0004;		//XXXX_01XX
			bCoilZoomCtrl = 0x01;
			fCoarse2FineRatio = SCAN_C2FR_2;
			nMAG = 1;
			break;
		case 3:
			wSCAN_DO = wSCAN_DO & (~0x000C);	//DEF_SCALE0=0(2), DEF_SCALE1=1(3)
			wSCAN_DO = wSCAN_DO | 0x0008;		//XXXX_10XX
			bCoilZoomCtrl = 0x10;
			fCoarse2FineRatio = SCAN_C2FR_3;
			nMAG = 1;
			break;
		case 4:
			wSCAN_DO = wSCAN_DO & (~0x000C);	//DEF_SCALE0=0(2), DEF_SCALE1=1(3)
			wSCAN_DO = wSCAN_DO | 0x000C;		//XXXX_11XX
			bCoilZoomCtrl = 0x11;
			fCoarse2FineRatio = SCAN_C2FR_4;
			nMAG = 1;
			break;
		case 5:
			wSCAN_DO = wSCAN_DO & (~0x000C);	//DEF_SCALE0=0(2), DEF_SCALE1=1(3)
			wSCAN_DO = wSCAN_DO | 0x000C;		//XXXX_10XX
			bCoilZoomCtrl = 0x11;
			fCoarse2FineRatio = SCAN_C2FR_4;	//DAC output range
			nMAG = 1;
			break;
		default:	//invalid number
			//wSCAN_DO = wSCAN_DO & (~0x003C);	//DEF_SCALE0=0(2), DEF_SCALE1=1(3), DEF_SCALE2=1(4), DEF_SCALE2=1(5)
			//wSCAN_DO = wSCAN_DO | 0x0038;
			//break;
			return ERROR_FAIL;
	}
	if (nMAG == 1) {
		/*NV_Settings.fV2IR[0] = 10; //5V --> 500 mA
		NV_Settings.fV2IR[1] = 10; //5V --> 500 mA
		NV_Settings.fLoadR[0] = 10; //10 ohm
		NV_Settings.fLoadR[1] = 10; */
		fRMagnify = 1;
	}
	else {
		/*NV_Settings.fV2IR[0] = 10*SCAN_R1R2; //5V --> 50 mA
		NV_Settings.fV2IR[1] = 10*SCAN_R1R2; //5V --> 50 mA
		NV_Settings.fLoadR[0] = 10*SCAN_R1R2; //100 ohm
		NV_Settings.fLoadR[1] = 10*SCAN_R1R2; */
		fRMagnify = SCAN_R1R2;
	}
#if SCAN_PCB_VER >= 9
	pwFpga[SET_DO0] = wSCAN_DO;	//set zoom scale digital output
#else
	if (bDB2Busy == 1)
		wDO = wSCAN_DO;
	else
		pwFpga[SET_DO0] = wSCAN_DO;	//set zoom scale digital output
#endif
	//
	//Vset = 2*Vc*C2FR/(1+C2FR) + 2*Vf*1/(1+C2FR)
	//Vset = Vc*fCoraseRatio + Vf*fFineRatio
	fCoarseRatio[COIL_CH_DEFX] = 2.0 * fCoarse2FineRatio / (1 + (float)fCoarse2FineRatio);
	fFineRatio[COIL_CH_DEFX] = 2.0 * 1 / (1 + (float)fCoarse2FineRatio);
	fCoarseRatio[COIL_CH_DEFY] = 2.0 * fCoarse2FineRatio / (1 + (float)fCoarse2FineRatio);
	fFineRatio[COIL_CH_DEFY] = 2.0 * 1 / (1 + (float)fCoarse2FineRatio);
	return SUCCESS;
#endif
}
//----------------------------------------------------------------
// set objective fine scale
//----------------------------------------------------------------
/*
int SetObjScale(int nZoom)
{
	switch (nZoom)
	{
		case 1:
			//wSCAN_DO = wSCAN_DO & (~0x00C3);	//OBJ_SCALE0=0(0), OBJ_SCALE1=1(1), OBJ_SCALE2=1(6), OBJ_SCALE2=1(7)
			//wSCAN_DO = wSCAN_DO | 0x00C2;		//1100_0010
			wSCAN_DO = wSCAN_DO & (~0x0003);	//OBJ_SCALE0=0(0), OBJ_SCALE1=1(1)
			wSCAN_DO = wSCAN_DO | 0x0000;		//XXXX_XX00
			nZoom = OBJ_C2FR_1;
			fObjScale = OBJ_C2FR_1;
			break;
		case 2:
			//wSCAN_DO = wSCAN_DO & (~0x00C3);	//OBJ_SCALE0=1(0), OBJ_SCALE1=0(1), OBJ_SCALE2=1(6), OBJ_SCALE2=1(7)
			//wSCAN_DO = wSCAN_DO | 0x00C1;		//1100_0001
			wSCAN_DO = wSCAN_DO & (~0x0003);	//OBJ_SCALE0=0(0), OBJ_SCALE1=1(1)
			wSCAN_DO = wSCAN_DO | 0x0001;		//XXXX_XX01
			nZoom = OBJ_C2FR_2;
			fObjScale = OBJ_C2FR_2;
			break;
		case 3:
			//wSCAN_DO = wSCAN_DO & (~0x00C3);	//OBJ_SCALE0=1(0), OBJ_SCALE1=1(1), OBJ_SCALE2=0(6), OBJ_SCALE2=1(7)
			//wSCAN_DO = wSCAN_DO | 0x0083;		//1000_0011
			wSCAN_DO = wSCAN_DO & (~0x0003);	//OBJ_SCALE0=0(0), OBJ_SCALE1=1(1)
			wSCAN_DO = wSCAN_DO | 0x0002;		//XXXX_XX10
			nZoom = OBJ_C2FR_3;
			fObjScale = OBJ_C2FR_3;
			break;
		case 4:
			//wSCAN_DO = wSCAN_DO & (~0x00C3);	//OBJ_SCALE0=1(0), OBJ_SCALE1=1(1), OBJ_SCALE2=1(6), OBJ_SCALE2=0(7)
			//wSCAN_DO = wSCAN_DO | 0x0043;		//0100_0011
			wSCAN_DO = wSCAN_DO & (~0x0003);	//OBJ_SCALE0=0(0), OBJ_SCALE1=1(1)
			wSCAN_DO = wSCAN_DO | 0x0003;		//XXXX_XX11
			nZoom = OBJ_C2FR_4;
			fObjScale = OBJ_C2FR_4;
			break;
		default:	//invalid number
			return ERROR_FAIL;
	}
#if SCAN_PCB_VER >= 9
	pwFpga[SET_DO0] = wSCAN_DO;
#else
	if (bDB2Busy == 1)
		wDO = wSCAN_DO;
	else
		pwFpga[SET_DO0] = wSCAN_DO;	//set zoom scale digital output
#endif
	//
	fCoarseRatio[COIL_CH_OBJ] =  2 * (float)nZoom / (1 + (float)nZoom);
	fFineRatio[COIL_CH_OBJ] =  2 * (float)1 / (1 + (float)nZoom);
	return SUCCESS;
}
*/
//
float fVXO = 0, fVYO = 0;	//C:coarse, left-bottom position,or orign pos
float fVXOC = 0, fVYOC = 0;	//these are the voltages set to DAC_DEFX_C and DAC_DEFY_C
//
void SetFineScanPos(int nX, int nY)
{
#if USE_FINE_SCAN_ONLY == 1
	SetFineDACX(wDACX[nX]);
	SetFineDACY(wDACY[nY]);
	//pwFpga[SET_DACX_F] = wDACX[nX]; //-5 ~ 5
	//pwFpga[SET_DACY_F] = wDACY[nY];
#else
	if (fCoarse2FineRatio != SCAN_C2FR_1) { //fine scan
		pwFpga[SET_DACX_F] = wDACX[nX]; //-5 ~ 5
		pwFpga[SET_DACY_F] = wDACY[nY];
	}
	else { //-5 ~ +5, rough scan
		//SetCoarseScan(wDACX[nX], wDACY[nY]);
		/*
		step_mode_w = databus1[0]; 			//0, 1 step by step
		r_nw_dac_gp = databus1[1];			//1
		n_cs_dac_gp[0] = databus1[2];		//1
		n_cs_dac_gp[1] = databus1[3];		//1,
		n_cs_dacf_s_buf = databus1[4];		//1, 0x1E
		dacf_ch_s_buf = databus1[5];		//0,
		n_cs_dac_gp[2] = databus1[6];
		*/
		pwFpga[SET_DAC_CH] = DAC_CH_DEFX_C;
		pwFpga[SET_DB2_DATA] = wDACX[nX]; //write dac_gp_buf
		//pwFpga[SET_DAC_PIN] = 0x005F;	//*0000_0000_0101_1111	r_nw=1,n_cs[0]=1,stepmode=1
		//pwFpga[SET_DAC_PIN] = 0x005D;	//*0000_0000_0101_1101	r_nw=0,n_cs[0]=1,stepmode=1
		pwFpga[SET_DAC_PIN] = 0x0059;	//0000_0000_0101_1001	r_nw=0,n_cs[0]=0,stepmode=1
		delay_us(COARSE_DAC_DELAY); //this delay is a MUST
		pwFpga[SET_DAC_PIN] = 0x005D;	//0000_0000_0101_1101	r_nw=0,n_cs[0]=1,stepmode=1
		pwFpga[SET_DAC_PIN] = 0x005F;	//0000_0000_0101_1111	r_nw=1,n_cs[0]=1,stepmode=1
		//pwFpga[SET_DAC_PIN] = 0x005E;	//*0000_0000_0101_1110	r_nw=1,n_cs[0]=1,stepmode=0
		//
		pwFpga[SET_DAC_CH] = DAC_CH_DEFY_C;
		pwFpga[SET_DB2_DATA] = wDACY[nY]; //write dac_gp_buf
		//pwFpga[SET_DAC_PIN] = 0x005F;	//*0000_0000_0001_1111	r_nw=1,n_cs[1]=1,stepmode=1
		//pwFpga[SET_DAC_PIN] = 0x005D;	//*0000_0000_0001_1101	r_nw=0,n_cs[1]=1,stepmode=1
		pwFpga[SET_DAC_PIN] = 0x0055;	//0000_0000_0001_0101	r_nw=0,n_cs[1]=0,stepmode=1
		delay_us(COARSE_DAC_DELAY); //this delay is a MUST
		pwFpga[SET_DAC_PIN] = 0x005D;	//0000_0000_0001_1101	r_nw=0,n_cs[1]=1,stepmode=1
		//pwFpga[SET_DAC_PIN] = 0x005F;	//*0000_0000_0001_1111	r_nw=1,n_cs[1]=1,stepmode=1
		pwFpga[SET_DAC_PIN] = 0x005E;	//0000_0000_0001_1110	r_nw=1,n_cs[1]=1,stepmode=0
	}
#endif
}

void SetScanOrigin(void)
{
	//SetDeflectorCoarseV(DAC_CH_DEFX_C, fVXOC);	//set coarse DAC X
	//SetDeflectorCoarseV(DAC_CH_DEFY_C, fVYOC);	//set coarse DAC Y
}
/*
int SetZoomScale(float fCenterX, float fCenterY, float fRangeX, float fRangeY)
{
	float fV, fV1, fV2;
	float fRange;
	float fVR[6]; //range
	int nCoilCh;
	//
	// 2048 correspond one-side range 2 mm (ZOOM_RANGE_MAX <==> DAC_12BIT_D2)
	// deflector corase X and Y = -5V ~ +5V
	// deflector fine X and Y = -5V ~ +5V
	// one-side range = fRangeX <--> nNX/2 <--> 2048
	// scan max from left-bottom (-ZOOM_RANGE_MAX, -ZOOM_RANGE_MAX) to right-top (+ZOOM_RANGE_MAX, +ZOOM_RANGE_MAX)
	// scan range =
	// fCenterX - fRangeX <--> fCenterX + fRangeX
	// fCenterY - fRangeY <--> fCenterY + fRangeY
	// RangeMax = 5 volt/V2IR/APERMM
	//choose the larger one
	//Vset = Vc*fCoraseRatio + Vf*fFineRatio
	fRangeX = fabs(fRangeX);
	fRangeY = fabs(fRangeY);
	GetScanRangeMax(fRangeX, 1); //calculate .fRangeMax[0], .fRangeMax[1], DEFX=1
	fV1 = fRangeX / fRangeMax[0]; //swing range = 4 ~ 0.8
	fV2 = fRangeY / fRangeMax[1];
	fRange = (fV1 > fV2) ? fRangeX : fRangeY; //choose the larger ratio to set VID_REF
	nCoilCh = (fV1 > fV2) ? 0 : 1; //choose the larger range ratio
	//current ratio, the larger fAPerMM, the weaker coil
	fV1 = NV_Settings.fAPerMM[nCoilSwitch][0];
	fV2 = NV_Settings.fAPerMM[nCoilSwitch][1];
	fARatio[0] = (fV1 > fV2) ? 1.0 : (fV2/fV1); //coil0 is weaker than coil1
	fARatio[1] = (fV1 > fV2) ? (fV2/fV1) : 1.0;
	//---------------------------------------------------------------
	// Range = DACF_OUT_MAX/fV2I[]/fAPerMM
	//0: 0.5 *2/(1+  1)/fAPerMM[0] = 0.5/0.15 = 3.3 mm
	//1: 0.5 *2/(1+4.7)/fAPerMM[0] = 0.5840 mm
	//2: 0.5 *2/(1+ 27)/fAPerMM[0] = 0.3030 mm
	//3: 0.5 *2/(1+100)/fAPerMM[0] = 0.0694 mm
	//4: 0.5 *2/(1+47)/SCAN_R1R2/fAPerMM[0] =  0.017 mm = 17 um
	//C2FR : coarse to fine ratio
	//---------------------------------------------------------------
	fV = (float)DACF_OUT_MAX/10/NV_Settings.fAPerMM[nCoilSwitch][nCoilCh]; //maximal range in mm
	//fVR[x} is the ratio set in COIL board
	fVR[0] = 2*fV/(1+SCAN_C2FR_1); //range in mm
	fVR[1] = 2*fV/(1+SCAN_C2FR_2);
	fVR[2] = 2*fV/(1+SCAN_C2FR_3);
	fVR[3] = 2*fV/(1+SCAN_C2FR_4);
	fVR[4] = 2*fV/(1+SCAN_C2FR_4)/2.2; //SCAN_R1R2, obsolete
	//
	if (fRange > fVR[0])	//out of range
		return ERROR_FAIL;
	else if (fRange >= fVR[1]) //between fVR[0] > x >= fVR[1]
	{	//400 um/2000 pixel = 200 nm/pixel
		SetDeflectorScale(1);	//1
		fDACRefRatio = fRange / fVR[0];
	}
	else if (fRange >= fVR[2]) //between fVR[1] > x >= fVR[2]
	{	//80 um/2000 pixel = 40 nm/pixel
		SetDeflectorScale(2);	//4.7
		fDACRefRatio = fRange / fVR[1];
	}
	else if (fRange >= fVR[3]) //between fVR[2] > x >= fVR[3]
	{	//40 um/2000 pixel = 20 nm/pixel
		SetDeflectorScale(3);	//10
		fDACRefRatio = fRange / fVR[2];
	}
	else //if (fRange >= fVR[4]) //between fVR[3] > x >= fVR[4]
	{	//40 um/2000 pixel = 20 nm/pixel
		SetDeflectorScale(4);	//47
		fDACRefRatio = fRange / fVR[3];
	}
	else // fVR[4] > x > fRangeMin, change to R2 sense
	{	//40 um/2000 pixel = 20 nm/pixel
		SetDeflectorScale(5);	//47x4
		fDACRefRatio = fRange / fVR[4];
	}
	if (fCenterX + fRangeX > fRangeMax[0])
		return ERROR_FAIL;
	if (fCenterX - fRangeX < -fRangeMax[0])
		return ERROR_FAIL;
	if (fCenterY + fRangeY > fRangeMax[1])
		return ERROR_FAIL;
	if (fCenterY - fRangeY < -fRangeMax[1])
		return ERROR_FAIL;
	//Vf*fFineRatio[] = fDACVMax*fDACRefRatio
	//voltage applied to DAC reference
	//set video DAC reference voltage
	fV = fDACRefRatio * fDACVMax[BOARD_SCAN][DAC_CH_VID_REF];
	SetVoltage(BOARD_SCAN, DAC_CH_VID_REF, fV);
	return SUCCESS;
} */
//----------------------------------------------------
// set to fixed pixel
//----------------------------------------------------
/*int SetFixedPixel(float fX, float fY)
{
	float fXC, fYC;
	float fVoltX1, fVoltX2, fVoltY1, fVoltY2;
	float fVXOC, fVYOC;
	WORD wV[2];
	//
	fXC = 0;
	fYC = 0;
	SetZoomScale(fXC, fYC, fX*1.1, fY*1.1); //set scale and reference voltage
	GetVoltFromPos(fXC, fYC, &fVXOC, &fVYOC);
	GetVoltFromPos(fX, fY, &fVoltX1, &fVoltY1);
	//fVoltX1 = fCoarseRatio * fVoltCoarse(DAC=fVXOC) + fFineRatio * fVoltFine(DAC)
	//fVoltFine(DAC) = (fVoltX1 - fCoarseRatio * fVoltCoarse) / fFineRatio;
	fVoltX2 = (fVoltX1 - fVXOC * fCoarseRatio[COIL_CH_DEFX]) / fFineRatio[COIL_CH_DEFX];	//unit volt
	//fVoltX1 = voltage applied at input of V2I block
	//fVoltX2 = voltage of DAC output when reference is +5V
	//now reference voltage is changed to fDACRefRatio*(+5V)
	//DAC_REF_RATIO*fDACRefRatio is used to avoid overflow
	fVoltX2 = fVoltX2 / fDACRefRatio / DAC_REF_RATIO; //avoid overflow
	wV[0] = VoltToWord(fVoltX2, DAC_12BIT, DACF_OUT_MAX, DACF_OUT_MIN);
	//
	fVoltY2 = (fVoltY1 - fVYOC * fCoarseRatio[COIL_CH_DEFY]) / fFineRatio[COIL_CH_DEFY];	//unit volt
	fVoltY2 = fVoltY2 / fDACRefRatio / DAC_REF_RATIO; //avoid overflow
	wV[1] = VoltToWord(fVoltY2, DAC_12BIT, DACF_OUT_MAX, DACF_OUT_MIN);
	//
	SetFineDACX(wV[0]);
	SetFineDACY(wV[1]);
	return SUCCESS;
}*/
//nXN, nYN: number of pixel
int UpdatePixelNum(int nNX, int nNY)
{
	int i, nVS, nVX;
	wDACFAddrX = 0;
	wDACFAddrY = 0;
//
	nScanPixelNumX = nNX;
	nScanPixelNumY = nNY;
	nScanPixelNumXDV = nScanPixelNumX / 8; //used to generate simulation data
	nScanPixelNumYDV = nScanPixelNumY / 8;
	//BYTE_PER_PIXEL
	nDataBytesPerLine = nScanPixelNumX * nBytePerPixel; //unit in byte
	//nBytesPerLine = nDataBytesPerLine + sizeof(DATA_BEGIN_HEADER);
	//
	//bPktNum = (nDataBytesPerLine / 1400) + 1;
	bPktNum = (nDataBytesPerLine / PKT_BYTE_NUM) + 1;
	if (bPktNum > PKT_NUM_MAX)
		return ERROR_FAIL;
	nVS = nScanPixelNumX / bPktNum; //bPktNum=1 or 2 or 3
	if ((nScanPixelNumX % bPktNum) != 0)
		nVS = nVS + 1;
	for (i = 0, nVX = 0; nVX < nScanPixelNumX; i++, nVX += nVS) {
		if (nScanPixelNumX - nVX > nVS) {
			nPixelPerPacket[i] = nVS;
		}
		else {
			nPixelPerPacket[i] = nScanPixelNumX - nVX;
		}
		nDataBytePerPacket[i] = nPixelPerPacket[i] * nBytePerPixel;
		//nTotBytePerPacket[i] = nDataBytePerPacket[i] + sizeof(DATA_BEGIN_HEADER);
	}
	//nDataBytePerPacket = nDataBytesPerLine / (int)bPktNum;
	//Queue start from QUE_DATA_START
	nQueSize = nDataBytesPerLine * QUE_NUM_MAX;
	return SUCCESS;
}

void SetDescentCurve(int nNX, int nNY)
{
	int i;
	//float fStep;
	//nDataNum = 0 ~ PI
	pwFpga[SET_ADDR_BASE_L] = 0;
	pwFpga[SET_ADDR_BASE_H] = 0;
	//
	//fStep = (float)(wDACX_Max - wDACX_Min) / nNX;
	for (i = 0; i <= nNX + nOverscan; i++) {
		//wDACDX[i] = (wDACX_Max + wDACX_Min) / 2 + (WORD)((float)(wDACX_Max - wDACX_Min) / 2 * cos((float)i * 3.1415926 / nNX));
		//wDACDX[i] = wDACX_Max - (WORD)((float)i * fStep);
		wDACDX[i] = wDACX[nNX + nOverscan - i];
		pwSram[ADDR_DACDX_START + i] = wDACDX[i];
	}
	//
	//fStep = (float)(wDACY_Max - wDACY_Min) / nNY;
	for (i = 0; i <= nNY; i++) {
		//wDACDY[i] = (wDACY_Max + wDACY_Min) / 2 + (WORD)((float)(wDACY_Max - wDACY_Min) / 2 * cos((float)i * 3.1415926 / nNY));
		//wDACDY[i] = wDACY_Max - (WORD)((float)i * fStep);
		wDACDY[i] = wDACY[nNY - i];
		pwSram[ADDR_DACDY_START + i] = wDACDY[i];
	}
}
/*
void SetCoarseScan(WORD wVX, WORD wVY)
{
	//step_mode_w = databus1[0]; 			//0, 1 step by step
	//r_nw_dac_gp = databus1[1];			//1
	//n_cs_dac_gp[0] = databus1[2];		//1
	//n_cs_dac_gp[1] = databus1[3];		//1,
	//n_cs_dacf_s_buf = databus1[4];		//1, 0x1E
	//dacf_ch_s_buf = databus1[5];		//0,
	//n_cs_dac_gp[2] = databus1[6];
	//after reset: step_mode=0, r_nw=1
	pwFpga[SET_DAC_CH] = DAC_CH_DEFX_C;
	pwFpga[SET_DB2_DATA] = wVX; //write dac_gp_buf
	//pwFpga[SET_DAC_PIN] = 0x005F;	//0000_0000_0001_1111	r_nw=1,n_cs[0]=1,stepmode=1
	pwFpga[SET_DAC_PIN] = 0x005D;	//0000_0000_0001_1101	r_nw=0,n_cs[0]=1,stepmode=1
	pwFpga[SET_DAC_PIN] = 0x0059;	//0000_0000_0001_1001	r_nw=0,n_cs[0]=0
	delay_us(COARSE_DAC_DELAY); //this delay is a MUST
	pwFpga[SET_DAC_PIN] = 0x005D;	//0000_0000_0001_1101	r_nw=0,n_cs[0]=1
	pwFpga[SET_DAC_PIN] = 0x005F;	//0000_0000_0001_1111	r_nw=1,n_cs[0]=1
	//pwFpga[SET_DAC_PIN] = 0x005E;	//0000_0000_0001_1110	r_nw=1,n_cs[0]=1,stepmode=0
	//
	pwFpga[SET_DAC_CH] = DAC_CH_DEFY_C;
	pwFpga[SET_DB2_DATA] = wVY; //write dac_gp_buf
	//pwFpga[SET_DAC_PIN] = 0x005F;	//0000_0000_0001_1111	r_nw=1,n_cs[1]=1,stepmode=1
	pwFpga[SET_DAC_PIN] = 0x005D;	//0000_0000_0001_1101	r_nw=0,n_cs[1]=1,stepmode=1
	pwFpga[SET_DAC_PIN] = 0x0055;	//0000_0000_0001_0101	r_nw=0,n_cs[1]=0
	delay_us(COARSE_DAC_DELAY); //this delay is a MUST
	pwFpga[SET_DAC_PIN] = 0x005D;	//0000_0000_0001_1101	r_nw=0,n_cs[1]=1
	//pwFpga[SET_DAC_PIN] = 0x005F;	//0000_0000_0001_1111	r_nw=1,n_cs[1]=1
	pwFpga[SET_DAC_PIN] = 0x005E;	//0000_0000_0001_1110	r_nw=1,n_cs[1]=1,stepmode=0
}
//
void SetFineScan(WORD wVX, WORD wVY)
{
	//step_mode_w = databus1[0]; 			//0, 1 step by step
	//r_nw_dac_gp = databus1[1];			//1
	//n_cs_dac_gp[0] = databus1[2];		//1
	//n_cs_dac_gp[1] = databus1[3];		//1,
	//n_cs_dacf_s_buf = databus1[4];		//1, 0x1E
	//dacf_ch_s_buf = databus1[5];		//0,
	//n_cs_dac_gp[2] = databus1[6];
	//
	pwFpga[SET_DB2_DATA] = wVX; //write dacf_x
	pwFpga[SET_DAC_PIN] = 0x5D;	//101_1101,n_cs=1,dacf_ch=0,step_mode_w=1
	pwFpga[SET_DAC_PIN] = 0x4D;	//100_1101,n_cs=0,dacf_ch=0,step_mode_w=1
	delay_us(1); //MUST
	pwFpga[SET_DAC_PIN] = 0x5D;	//101_1101,n_cs=1,dacf_ch=0,step_mode_w=1
	//
	pwFpga[SET_DB2_DATA] = wVY; //write dacf_y
	pwFpga[SET_DAC_PIN] = 0x7D;	//111_1101,n_cs=1,dacf_ch=1,step_mode_w=1
	pwFpga[SET_DAC_PIN] = 0x6D;	//110_1101,n_cs=0,dacf_ch=1,step_mode_w=1
	pwFpga[SET_DAC_PIN] = 0x6D;	//110_1101,n_cs=0,dacf_ch=1,step_mode_w=1
	//delay_us(1); //MUST
	//pwFpga[SET_DAC_PIN] = 0x7D;	//111_1101,n_cs=1,dacf_ch=1,step_mode_w=1
	pwFpga[SET_DAC_PIN] = 0x7E;	//111_1110,n_cs=1,dacf_ch=1,step_mode_w=0
}
*/
//
void SetFineDACXY(WORD wVX, WORD wVY)
{
	pwFpga[SET_DB3_DEV] = 1; //access_bus3_dev_reg=1
	delay_void(); //MUST!, important
	pwSram[SET_DACX_F] = wVX;//access_bus_dev=1
	delay_void(); //MUST!, important
	pwSram[SET_DACY_F] = wVY;//access_bus_dev=1
	delay_void(); //MUST!, important
	pwFpga[SET_DB3_DEV] = 0; //access_bus3_dev_reg=1
}

void SetFineDACX(WORD wVX)
{
	pwFpga[SET_DB3_DEV] = 1; //access_bus3_dev_reg=1
	delay_void(); //MUST!, important
	pwSram[SET_DACX_F] = wVX;//access_bus_dev=1
	delay_void(); //MUST!, important
	pwFpga[SET_DB3_DEV] = 0; //access_bus3_dev_reg=1
}
//
void SetFineDACY(WORD wVY)
{
	pwFpga[SET_DB3_DEV] = 1;
	delay_void(); //MUST!, important
	pwSram[SET_DACY_F] = wVY;
	delay_void(); //MUST!, important
	pwFpga[SET_DB3_DEV] = 0;
}
//
#define DACSET_CH_NUM	7
#define ADCGET_CH_NUM	9
int nChDAC[DACSET_CH_NUM] = {DAC_CH_OBJ_C, DAC_CH_CO, DAC_CH_BR, DAC_CH_STIGX, DAC_CH_STIGY,
  		DAC_CH_SINE, DAC_CH_COSINE};
int nChADC[ADCGET_CH_NUM] = {ADC_CH_VAC0, ADC_CH_VAC1, ADC_CH_DEFX, ADC_CH_DEFY,
		ADC_CH_STIGX, ADC_CH_STIGY, ADC_CH_OBJ, ADC_CH_TEMPE0, ADC_CH_TEMPE1};
//
void UpdateADCGet(void)
{
	int i;
	for (i = 0; i < ADCGET_CH_NUM; i++) {
		if (nEnableADC[i] == 0) continue;
		if (bADCGet[i] == 0) continue;
		GetVoltage(nChADC[i]);
		bADCGet[i] = 0;
	}
}
//
void UpdateDigitalInput(void)
{
	//no DI on scan board
	//wDI = pwFpga[GET_DI];
}

void UpdateDigitalOutput(void)
{
	if (wDO != 0xFFFF) {
		pwFpga[SET_DO0] = wDO;
		wDO = 0xFFFF;
	}
	//pwFpga[SET_DO0] = wSCAN_DO;
}
//
// general purpose DAC output
//
void SetVoltage(int nBoard, int nCh, float fV)
{
	WORD wV = 0;
	int nChip = 0;
	int nChOrg = 0;
	static int nReentry = 0;
	DWORD dwV = 0;
	int RemapAd56Ch[8] = {0, 2, 4, 6, 7, 5, 3, 1};
	//
	if (nReentry == 1) return;
		nReentry = 1;
	nChOrg = nCh;
	if (nBoard == BOARD_SCAN) {
		//video DAC
		if (nCh == DAC_CH_DEFX_F) {
			SetDeflectorFineV(DAC_CH_DEFX_F, fV);
			goto SetVEnd;
		}
		if (nCh == DAC_CH_DEFY_F) {
			SetDeflectorFineV(DAC_CH_DEFY_F, fV);
			goto SetVEnd;
		}
		fV *= NV_Settings.fDACSlope[nBoard][nChOrg];
		fV += NV_Settings.fDACOffset[nBoard][nChOrg];
		wV = VoltToWord(fV, DAC_12BIT, fDACVMax[BOARD_SCAN][nChOrg], fDACVMin[BOARD_SCAN][nChOrg]);
		//general purpose DAC
		nChip = nCh/8; 		//nChip=0,1
		nCh = nCh % 8;		//nCh=0,1,2,...7
		if (NV_Settings.bDACType[0] == DAC_AD5328) {
			nCh = nCh << 12; 	//bit12~bit14
			wV = wV + nCh;		//wV=0x0000~0x0FFF
			WriteSerialDAC(nChip, wV);
		}
		else if (NV_Settings.bDACType[0] == DAC_AD5668) {
			wV = VoltToWord(fV, DAC_16BIT, fDACVMax[BOARD_SCAN][nChOrg], fDACVMin[BOARD_SCAN][nChOrg]);
			dwV = (DWORD)wV;
			dwV = dwV << 4;		//16-bit DAC change to 20-bit value
		}
		if (NV_Settings.bDACType[0] > DAC_AD5328) {
			dwV |= 0x03000000; 	//(C3-C0)=0011
			nCh = RemapAd56Ch[nCh]; //remap channel
			nCh = nCh << 20; 	//(A3=0,A2-A0)
			dwV += nCh;
			WriteSerialDAC32b(nChip, dwV);
		}
	}
	else if (nBoard == BOARD_IO) {
		if (nCh == DAC_CH_OBJ_C) //record obj value
			fObjV = fV;
		//sine, cosine need high precision
		fV *= NV_Settings.fDACSlope[nBoard][nChOrg];
		fV += NV_Settings.fDACOffset[nBoard][nChOrg];
		//nCh = nChOrg - IO_DAC_CH_START;
		nChip = nCh / 8; 		//nChip=0,1
		nCh = nCh % 8;		//nCh=0,1,2,...7
		if (NV_Settings.bDACType[1] == DAC_AD5328) {
			wV = VoltToWord(fV, DAC_12BIT, fDACVMax[BOARD_IO][nChOrg], fDACVMin[BOARD_IO][nChOrg]);
			nCh = nCh << 12; 	//bit12~bit14
			wV = wV + nCh;		//wV=0x0000~0x0FFF
			WriteIOSerialDAC(nChip, wV); //set IO board DAC value
		}
		else if (NV_Settings.bDACType[1] == DAC_AD5648) {
			wV = VoltToWord(fV, DAC_14BIT, fDACVMax[BOARD_IO][nChOrg], fDACVMin[BOARD_IO][nChOrg]);
			dwV = (DWORD)wV;
			dwV = dwV << 6;		//14-bit DAC change to 20-bit value
		}
		else if (NV_Settings.bDACType[1] == DAC_AD5668) {
			wV = VoltToWord(fV, DAC_16BIT, fDACVMax[BOARD_IO][nChOrg], fDACVMin[BOARD_IO][nChOrg]);
			dwV = (DWORD)wV;
			dwV = dwV << 4;		//16-bit DAC change to 20-bit value
		}
		if (NV_Settings.bDACType[1] > DAC_AD5328) {
			//AD5648=x,x,x,x,C3,C2,C1,C0,A3,A2,A1,A0,D13,D12,....D0,x,x,x,x,x,x
			//AD5668=x,x,x,x,C3,C2,C1,C0,A3,A2,A1,A0,D15,D14,........D0,x,x,x,x
			//A3=1 all addresses
			//(C3,C2,C1,C0)=(0,0,0,1) update DAC register
			//(C3,C2,C1,C0)=(0,0,1,1) write and update DAC register
			//(C3,C2,C1,C0)=(1,0,0,0) setup internal reference
			//dwV = 0x08000001; 	//(C3-C0)=1000,DB0=1, use internal reference
			dwV |= 0x03000000; 	//(C3-C0)=0011
			nCh = RemapAd56Ch[nCh]; //remap channel
			nCh = nCh << 20; 	//(A3=0,A2-A0)
			dwV += nCh;
			WriteIOSerialDAC32b(nChip, dwV);
		}
	}
SetVEnd:
	nReentry = 0;
}

#if IO_PCB_VER >= 1
void InitSerialIO(void)
{
	N_CS_IO.function(0);
	SCK_IO.function(0);
	DIN_IO.function(0);		//SCAN --> IO
	//
	//CS_DISABLE;
	//SCK_0;
	//DIN_0;
	//74HC14(NOT) --> NV_Settings.bIOINV == 1
	//74HC07(OC)  --> NV_Settings.bIOINV == 0
	N_CS_IO = (NV_Settings.bIOINV == 1) ? 0 : 1;
	SCK_IO = (NV_Settings.bIOINV == 1) ? 1 : 0;
	DIN_IO = (NV_Settings.bIOINV == 1) ? 1 : 0;
#if USE_FPGA_DIO == 1
	SCK_IO.hiz();		//SCK_IO is not used
	DIN_IO.hiz();		//DIN_IO is not used
#endif
	DOUT_IO.function(0);	//IO --> SCAN (input)
	DOUT_IO.hiz();
	//
	N_RESET_IO.function(0);
	N_RESET_IO = 1;
	N_RESET_IO = 0;
	delay_us(30);
	N_RESET_IO = 1;
}
#endif

#if SCAN_PCB_VER >= 11
void InitSerialADC(void)
{
	//do nothing
}
//
WORD WriteSerialADC(WORD wV)
{	//read only
	return 0;
}
//SCAN board
void InitSerialDAC(void)
{
	DWORD dwV;
	N_CS_DAC0.function(0);
	N_CS_DAC0 = 1;
	N_CS_DAC1.function(0);
	N_CS_DAC1 = 1;
	SCLK_DAC.function(0);
	SCLK_DAC = 0;
	DIN_DAC.function(0);
	DIN_DAC = 0;
	N_LDAC.function(0);
	N_LDAC = 1;
	//
#if SCAN_PCB_VER >= 24 //2 DAC chips
	if (NV_Settings.bDACType[0] > DAC_AD5328) {
		dwV = 0x08F00001; //C=(1000),A=(1111)
		WriteSerialDAC32b(0, dwV); 	//chip 0 reference output
		WriteSerialDAC32b(1, dwV);	//chip 1 reference input
	}
	else {
		WriteSerialDAC(0, DAC_INIT_CMD); //GAIN=1,BUF=1,VDD=0, scan board
		WriteSerialDAC(1, DAC_INIT_CMD);
	}
	//two DAC chips on SCAN board for SCAN_PCB_VER >= SCAN_VA23
	//SCAN board DAC initialization
#elif SCAN_PCB_VER >= 23 //2 DAC chips
	WriteSerialDAC(0, DAC_INIT_CMD); //GAIN=1,BUF=1,VDD=0, scan board
	WriteSerialDAC(1, DAC_INIT_CMD);
#else
	WriteSerialDAC(0, DAC_INIT_CMD); //GAIN=1,BUF=1,VDD=0, scan board
#endif
	//IO board DAC initialization
	if (NV_Settings.bDACType[1] > DAC_AD5328) { //AD5648, AD5668
		//AD5648=x,x,x,x,C3,C2,C1,C0,A3,A2,A1,A0,D13,D12,....D0,x,x,x,x,x,x
		//AD5668=x,x,x,x,C3,C2,C1,C0,A3,A2,A1,A0,D15,D14,........D0,x,x,x,x
		//A3=1 all addresses
		//(C3,C2,C1,C0)=(0,0,0,1) update DAC register
		//(C3,C2,C1,C0)=(0,0,1,1) write and update DAC register
		//(C3,C2,C1,C0)=(1,0,0,0) setup internal reference
		//dwV = 0x08000001; 	//(C3-C0)=1000,DB0=1, use internal reference
		//IO_PCB_VER <= 4 use external reference
#if IO_PCB_VER >= 6
		dwV = 0x08F00001; //C=(1000),A=(1111)
		WriteIOSerialDAC32b(0, dwV); 	//chip 0 reference output
		WriteIOSerialDAC32b(1, dwV);	//chip 1 reference output
#endif
	}
}
// write DAC on SCAN board
void WriteSerialDAC(int nChip, WORD wV)
{
	WORD wMask = 0x8000;
	int i;
	OSCritEnter( &OsCrit, 1 );
	N_CS_DAC0 = (nChip == 0) ? 0 : 1;
	N_CS_DAC1 = (nChip == 1) ? 0 : 1;
	for (i = 0; i < 16; i++) {
		DIN_DAC = (wV & wMask) ? 1 : 0;
		SCLK_DAC = 1;
		wMask = wMask >> 1; //write MSB first
		SCLK_DAC = 0;
	}
	N_LDAC = 0;
	delay_void();
	N_LDAC = 1;
	//
	N_CS_DAC0 = 1;
	N_CS_DAC1 = 1;
	OSCritLeave( &OsCrit );
	return;
}

//SCAN board function
void WriteSerialDAC32b(int nChip, DWORD dwV)
{
	DWORD dwMask = 0x80000000; //write MSB first
	int i;
	OSCritEnter( &OsCrit, 1 );
	N_CS_DAC0 = (nChip == 0) ? 0 : 1;
	N_CS_DAC1 = (nChip == 1) ? 0 : 1;
	for (i = 0; i < 32; i++) {
		DIN_DAC = (dwV & dwMask) ? 1 : 0;
		SCLK_DAC = 1;
		dwMask = dwMask >> 1; //write MSB first
		SCLK_DAC = 0;
	}
	N_LDAC = 0;
	delay_void();
	N_LDAC = 1;
	//
	N_CS_DAC0 = 1;
	N_CS_DAC1 = 1;
	OSCritLeave( &OsCrit );
	return;
}
#elif SCAN_PCB_VER >= 10
void InitSerialADC(void)
{
	N_CS_ADC.function(0);	//J2[30]
	N_CS_ADC = 1;
	SCLK_ADC.function(0);	//J2[25]
	SCLK_ADC = 0;
	DIN_ADC.function(0);	//J2[27]
	DIN_ADC = 0;
	DOUT_ADC.function(0);	//ADC output, NB input
	DOUT_ADC.hiz();			//J2[28]
}
//
WORD WriteSerialADC(WORD wV)
{
	int i;
	WORD wMask = 0x8000;
	WORD wDIN = 0;
	N_CS_ADC = 0;
	for (i = 0; i < 16; i++) {
		DIN_ADC = (wV & wMask) ? 1 : 0; //CPU output, ADC input
		SCLK_ADC = 1;
		if (DOUT_ADC) wDIN |= wMask;
		wMask = wMask >> 1; //read MSB first
		SCLK_ADC = 0;
	}
	N_CS_ADC = 1;
	return wDIN;
}
//
void InitSerialDAC(void)
{
	N_CS_DAC0.function(0);
	N_CS_DAC0 = 1;
	N_CS_DAC1.function(0);
	N_CS_DAC1 = 1;
	N_CS_DAC2.function(0);
	N_CS_DAC2 = 1;
	SCLK_DAC.function(0);
	SCLK_DAC = 0;
	DIN_DAC.function(0);
	DIN_DAC = 0;
	N_LDAC.function(0);
	N_LDAC = 1;
	//
	WriteSerialDAC(0, DAC_INIT_CMD); //GAIN=1,BUF=1,VDD=0
	WriteSerialDAC(1, DAC_INIT_CMD); //GAIN=1,BUF=1,VDD=0
	WriteSerialDAC(2, DAC_INIT_CMD); //GAIN=1,BUF=1,VDD=0
}
//
void WriteSerialDAC(int nChip, WORD wV)
{
	WORD wMask = 0x8000;
	int i;
	N_CS_DAC0 = (nChip == 0) ? 0 : 1;
	N_CS_DAC1 = (nChip == 1) ? 0 : 1;
	N_CS_DAC2 = (nChip == 2) ? 0 : 1;
	for (i = 0; i < 16; i++) {
		DIN_DAC = (wV & wMask) ? 1 : 0;
		SCLK_DAC = 1;
		wMask = wMask >> 1; //write MSB first
		SCLK_DAC = 0;
	}
	N_LDAC = 0;
	delay_void();
	N_LDAC = 1;
	//
	N_CS_DAC0 = 1;
	N_CS_DAC1 = 1;
	N_CS_DAC2 = 1;
	return;
}
#endif
//
void GetAllVoltageNoEx(float *fpADC)
{
	int i;
	for (i = 0; i < ADC_CH_NUM_MAX; i++) {
		fpADC[i] = SysParam.fADC[i]; //fADCVal[i];
	}
}
//-------------------------------------------------
// for IO_PCB_VER>=1, takes about 55 msec to complete (debug mode)
// 2.8 ms (release mode)
//-------------------------------------------------
void GetAllVoltage(float *fpADC)
{
	int nV;
	float fV;
	static int nReentry = 0;
	//
	if (nReentry == 1)
		return;
	nReentry = 1;
#if SERIAL_ADC == 1 //use AD7928, 8 ch, 12-bit ADC
	WORD wDIN, wV;
	int nChR, nChW, nChX;
	for (nV = 0; nV < MUX_NUM; nV++) {
		SetADCMux(nV); //set MUX_CTRL0 and MUX_CTRL1
		//------------------------------------------------
		//WRITE(11)=1,SEQ(10)=0,DC(9),ADDR2-0(8-6),
		//PM1(5)=PM0(4)=1, normal operation
		//SHADOW(3),DC(2)
		//RANGE(1)=0, 0 to 2xVREF
		//CODING(0)=1, straight binary
		//1000_0011_0001 = 0x831
		//------------------------------------------------
#if IO_PCB_VER >= 1
		WriteIOSerialADC(0x8310);
#else
		WriteSerialADC(0x8310); //nChW=0, BOARD_SCAN
#endif
		for (nChW = 0; nChW < CH_PER_MUX; nChW++) { //0-3
			//0,4,8,12; 1,5,9,13; 2,6,10,14; 3,7,11,15
			//0,6,12,18; 1,7,13,19; 2,8,14,20; 3,9,15,21; 4,10,16,22
			//nChW = 0 ~ (CH_PER_MUX-1)
			wV = (nChW + 1) << 6; //bit 8~6, 1,2,3,4
			//0000_1000_0011_0001 --> 1000_0011_0001_0000
			wV = wV | 0x0831;
			wV = wV << 4;
#if IO_PCB_VER >= 1
			wDIN = WriteIOSerialADC(wV);
#else
			wDIN = WriteSerialADC(wV); 		//read ADC at specified channel
#endif
			wV = wDIN & 0x0FFF; 			//12-bit ADC data
			nChR = (wDIN & 0x7000) >> 12;	//ADC channel
			fV = WordToVolt(wV, ADC_12BIT, ADC_IN_MAX, ADC_IN_MIN);
			//---------------------------------------
			// nChR = 0 ~ 5
			//---------------------------------------
			nChX = nChR * 4 + nV; //nChX = 0 ~ 15
			if (bDebug == 6)
				printf("CHW=%d,CHX=%d,CHR=%d,%4X,%.3f\n", nChW, nChX, nChR, wDIN, fV);
			if (nChX >= ADC_CH_NUM_MAX) //illegal value, avoid array corrupt
				continue; //important step!!!
			fV += NV_Settings.fADCOffset[nChX];
			//fADCVal[nChX] = fV;
			bADCGet[nChX] = 0;
			if (SysParam.bSimuVAC == 0) {
				SysParam.fADC[nChX] = fV;
			}
			else if (SysParam.bSimuVAC == 1) { //use simulation value
				if ((nChX == ADC_CH_IG) || ((nChX >= ADC_CH_VAC0) && (nChX <= ADC_CH_VAC2))) {
					//keep all these vacuum values unchanged during simulation
				}
				else SysParam.fADC[nChX] = fV;
			}
			if (fpADC != NULL) {
				fpADC[nChX] = SysParam.fADC[nChX];
			}
		}
	}
#else
	int nCh;
	for (nV = 0; nV < 4; nV++) {
		for (nCh = 0; nCh < ADC_CH_NUM_MAX; nCh += 4) {
			//0,4,8,12; 1,5,9,13; 2,6,10,14; 3,7,11,15
			fV = GetVoltage(nCh + nV); //nCh = channel index, 0-based
			SysParam.fADC[nChX] = fV;
			if (fpADC != NULL) {
				fpADC[nCh + nV] = fV;
			}
		}
	}
#endif //end of #if SERIAL_ADC == 1
	nReentry = 0;
}
//
// nCh = 0 ~ (ADC_CH_NUM_MAX-1)
//
/*float GetVoltageEx(int nCh)
{
	return SysParam.fADC[nCh];
}*/

float GetVoltage(int nCh)
{
	return SysParam.fADC[nCh]; //fADCVal[nCh];
}
//nADC_Ch change from 0 to (ADC_CH_NUM_MAX-1) (0 ~ 15)
int SetADCMux(int nADC_Ch)
{
	int nChMux;
	int nMUX;
	static int nMUX_Curr[2] = {0, 0};
	static int nMUX_Prev[2] = {-1, -1};
	//
#if IO_PCB_VER >= 1
	//nADC_Ch: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,... ,20, 21, 22, 23
	//MUX:     0, 1, 2, 3, 0, 1, 2, 3, 0, 1,  2,  3, ...    , 0,  1,  2,  3
	//Chip_Ch: 0, 0, 0, 0, 1, 1, 1, 1, 2, 2,  2,  2, ...    , 5,  5,  5,  5
	nChMux = nADC_Ch; // / 4;	//0~3 = [A1, A0]
	SetIO_ADChannel(nChMux);
#endif
	//nChMux = nADC_Ch / 4;	//0~4, index for ADC chip
	nMUX = nADC_Ch % 4;	//0~3
	//
	nMUX_Curr[0] = (nMUX & 0x0001) ? 1 : 0; //bit 0
	nMUX_Curr[1] = (nMUX & 0x0002) ? 1 : 0; //bit 1
	if ((nMUX_Curr[0] != nMUX_Prev[0]) || (nMUX_Curr[1] != nMUX_Prev[1])) {
		delay_void(); //delay is a MUST
		delay_void(); //delay is a MUST
		asm("nop");
	}
	nMUX_Prev[0] = nMUX_Curr[0];
	nMUX_Prev[1] = nMUX_Curr[1];
	return nChMux;
}
#if SCAN_PCB_VER >= 10
float GetVideoFactor(int nAvgNum)
{
	float fV;
	fV = (float)nAvgNum;
	if (nAvgNum <= 2)
		return 2.0/fV;
	else if (nAvgNum <= 4)
		return 4.0/fV;
	else if (nAvgNum <= 8)
		return 8.0/fV;
	else if (nAvgNum <= 16)
		return 16.0/fV;
	else if (nAvgNum <= 32)
		return 32.0/fV;
	else if (nAvgNum <= 64)
		return 64.0/fV;
	else if (nAvgNum <= 128)
		return 64.0/(fV/2);
	else
		return 1.0;
}
#endif
//
void EnableVideoADC(int nEnable)
{
	//bEnableVADC = (nEnable == 1) ? 1 : 0;
	//pwFpga[ENABLE_VIDEO_ADC] = (nEnable == 1) ? 1 : 0;
	//pwFpga[ENABLE_VIDEO_CLK] = (nEnable == 1) ? 1 : 0;
	delay_us(2); //important delay!!! power on video ADC
}

WORD GetVideoADC(void)
{
	int i;
	WORD wV;
#if USE_AD7655 == 0
	pwFpga[ENABLE_VIDEO_ADC] = 1; //POWERDN=0, AD9057 video ADC
#endif
	if (SysParam.bScanning != OP_IDLE)
		goto GetVideoADC;
	VADCConvStart(); //vadc_conv_start
	//
	for (i = 0; i < 10; i++) {
		wV = pwFpga[GET_STATUS];
		if (wV & ST_GET_VIDEO_ADC == 0) break; //wait until get_video_adc = 0
		delay_void(); //MUST, can't be removed for time delay, about 1.5 use
	}
GetVideoADC:
	if (nShadowMode == SHADOW_HDIFF)
		wV = pwFpga[GET_VADC_HDIFF];
	else
		wV = pwFpga[GET_VADC_SUM]; //SHADOW_VSUM, SHADOW_M2, SHADOW_M3
#if USE_AD7655 == 0
	pwFpga[ENABLE_VIDEO_ADC] = 0; //POWERDN=1
#endif
	return wV;
}
//
extern BYTE bDB3Busy;
//
void GetAllVideoADC(int nForce)
{
	int i; //, j, k;
	WORD wV;
	float fV;
	static WORD wVADC[10] = {0};
	//pwFpga[START_VADC_FPGA] = ADC_CH_VIDEO0; //vadc_conv_start,start video state machine
#if USE_AD7655 == 0
	pwFpga[ENABLE_VIDEO_ADC] = 1; //en_vadc_reg=1,n_en_vadc=0, AD9057 video ADC
#endif
	if (bDB3Busy == 1)
		return;
	if (nForce == 0)
		return;
	//
	pwFpga[READ_ALL_VADC] = 1;
	//avoid databus3 conflict
	VADCConvStart(); //generate vadc_conv_start strobe, n_vadc_conv pulse
	//
	for (i = 0; i < 10; i++) { //it takes 5 usec
		delay_void(); //MUST, can't be removed for time delay, about 1.5 use
		wV = pwFpga[GET_STATUS];
		if ((wV & ST_GET_VIDEO_ADC) == 0) break; //wait until get_video_adc = 0
	}
	wVADC[0] = pwFpga[GET_VADC0];
	wVADC[1] = pwFpga[GET_VADC1];
	wVADC[2] = pwFpga[GET_VADC2];
	wVADC[3] = pwFpga[GET_VADC3];
	wVADC[4] = pwFpga[GET_VADC4];
	wVADC[5] = pwFpga[GET_VADC5];
	wVADC[6] = pwFpga[GET_VADC_SUM];		//2CH
	wVADC[7] = pwFpga[GET_VADC_SUM];		//4CH
	wVADC[8] = pwFpga[GET_VADC_HDIFF];	//signed short
	wVADC[9] = pwFpga[GET_VADC_VDIFF];	//signed short
	for (i = 0; i < VIDEO_ADC_CH_NUM; i++) {
		fV = WordToVolt(wVADC[i], ADC_16BIT, 5, 0);
		fV -= NV_Settings.fScanADCOffset[i];
		SysParam.fVADC[i] = fV;
	}
	SysParam.fVADC[7] = WordToVolt(wVADC[7], ADC_16BIT, 5, 0); //VSUM
	SysParam.fVADC[8] = WordToVolt(wVADC[8], ADC_16BIT, 5, 0); //HDIFF
	SysParam.fVADC[9] = WordToVolt(wVADC[9], ADC_16BIT, 5, 0); //VDIFF
	//pwV[8] = pwFpga[GET_VADC_M2];		//M2
	//pwV[9] = pwFpga[GET_VADC_M3];		//M3
	//printf("GetAllVideoADC %d %04X\n", i, wV);
	//pwFpga[READ_ALL_VADC] = 0;
}
// fV: input unit: volt
void SetObjectiveCoarseV(float fV)
{
	//DAC output from -10 V to +10 V
	//VOUT = VREFN + (VREFP - VREFN)*D/4096
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fV);
}
//
void SetObjectiveFineV(float fV)
{
	SetVoltage(BOARD_IO, DAC_CH_OBJ_F, fV);
}
//------------------------------------------------------
// nCh: DAC_CH_STIGX or DAC_CH_STIGY
// fV: input unit: volt
//------------------------------------------------------
void SetStigmaV(int nCh, float fV)
{
	SetVoltage(BOARD_IO, nCh, fV);
}
//------------------------------------------------------
// fV in unit volt
// nCh = DAC_CH_DEFX_F or DAC_CH_DEFY_F
// or
// nCh = 0, 1
//------------------------------------------------------
void SetDeflectorFineV(int nCh, float fV)
{
	WORD wV;
	if (nCh > 2) //change to 0,1
		nCh = nCh - DAC_CH_DEFX_F;
	//0:X, 1:Y
	fV *= NV_Settings.fScanDACSlope[nCh];
	fV += NV_Settings.fScanDACOffset[nCh];
	wV = VoltToWord(fV, DAC_16BIT, DACF_OUT_MAX, DACF_OUT_MIN);
	if (nCh == 0)
		SetFineDACX(wV);
	else if (nCh == 1)
		SetFineDACY(wV);
}
//----------------------------------------------
// Vacuum control subroutine
//----------------------------------------------
// nNdx = 0,1,2 --> GV0(11), GV1(12), GV2(13)
// nOpen = 1, open; 0: close
int SetGateValve(int nNdx, int nOpen)
{
	WORD wV;
#if IO_PCB_VER >= 3
	int wCtrlIO[VAC_GATE_NUM_MAX] = {IO_GV_CTRL0, IO_GV_CTRL1, IO_GV_CTRL2,
		IO_GV_CTRL3, IO_GV_CTRL4, IO_GV_CTRL5};
#else
	int wCtrlIO[VAC_GATE_NUM_MAX] = {IO_GV_CTRL0, IO_GV_CTRL1, IO_GV_CTRL2, 0x0400};
#endif
	//
	if (nNdx > VAC_GATE_NUM) //illegal gate valve
		return -1;
	if (nOpen == VALVE_OPEN) {
		if (nNdx < VAC_GATE_NUM_MAX)
			wV = wIO_DO | wCtrlIO[nNdx];
		else
			return -1;
#if SYS_PARAM_VER >= 3
		SysParam.bGateValve2[nNdx] = 1; //open
#else
		SysParam.bGateValve[nNdx] = 1; //open, obsolete
#endif
		if ((nNdx == TP_GATE_VALVE) && (SysParam.bSimuVAC == 0))
			SetTurboGV(VALVE_OPEN); //set turbo pump gate valve
		printf("GV%d OPEN\n", nNdx);
	}
	else { // VALVE_CLOSE
		if (nNdx < VAC_GATE_NUM_MAX)
			wV = wIO_DO & (~wCtrlIO[nNdx]);
		else
			return -1;
#if SYS_PARAM_VER >= 3
		SysParam.bGateValve2[nNdx] = VALVE_CLOSE; //open
#else
		SysParam.bGateValve[nNdx] = 0; //0:close
#endif
		if ((nNdx == TP_GATE_VALVE) && (SysParam.bSimuVAC == 0))
			SetTurboGV(VALVE_CLOSE);
		printf("GV%d CLOSE\n", nNdx);
	}
	if (SysParam.bSimuVAC == 0)
		SetDigitalOutput(BOARD_IO, wV);
#if USE_IP_CTRL_B == 1 //use IP control PCB
	if ((nNdx == MOTOR_GATE_VALVE) && (SysParam.bSimuVAC == 0))
		SetMotorGateValve(nOpen);
#endif
	OSTimeDly(TICKS_PER_SECOND/2);
	return 0;
}
//-------------------------------------
// custom HV control
// get custom HV status
//-------------------------------------
void GetHVC(void)
{
	char szBuffer[32];
	sprintf(szBuffer, "$%d,GETV\n", HV_ADDR);
	RS485_WriteString_P2(szBuffer); //port2
}

void SetMotorGateValve(int nAct)
{
	char szBuffer[32];
	sprintf(szBuffer, "$%d,MOVE:%d\n", IP_MOTOR_ADDR, nAct);
	RS485_WriteString_P2(szBuffer); //port2
}
//----------------------------------------------
// nOn = 1, on; 0: off, EM200
//----------------------------------------------
void SetIonPump(int nOn)
{
#if USE_IP_CTRL_B == 1
	char szBuffer[32];
#else
	WORD wV;
#endif
	//
	if (nOn == 1) {
		SysParam.bIonPumpOn = 1;
#if USE_IP_CTRL_B == 1
		sprintf(szBuffer, "$%d,IPON:1\n", IP_MOTOR_ADDR);
		if (SysParam.bSimuVAC == 0)
			RS485_WriteString_P2(szBuffer);
#else
		wV = wIO_DO | IO_IP_CTRL; //ion pump
#endif
	}
	else {
		SysParam.bIonPumpOn = 0;
#if USE_IP_CTRL_B == 1
		sprintf(szBuffer, "$%d,IPON:0\n", IP_MOTOR_ADDR);
		if (SysParam.bSimuVAC == 0)
			RS485_WriteString_P2(szBuffer);
#else
		wV = wIO_DO & (~IO_IP_CTRL);
#endif
	}
#if USE_IP_CTRL_B == 1
#else
	SetDigitalOutput(BOARD_IO, wV);
#endif
	OSTimeDly(1);
}
//----------------------------------------------
// nOn = 1, on; 0: off
//----------------------------------------------
void SetIonGauge(int nOn)
{
	WORD wV;
	if (nOn == 1) {
		wV = wIO_DO | IO_IG_CTRL; //ion pump
		SysParam.bIonGaugeOn = 1;
		//nIGTimeout = 15; //wait until IG is stable
	}
	else {
		wV = wIO_DO & (~IO_IG_CTRL);
		SysParam.bIonGaugeOn = 0;
		//nIGTimeout = 0;
	}
	if (SysParam.bSimuVAC == 0)
		SetDigitalOutput(BOARD_IO, wV);
	OSTimeDly(TICKS_PER_SECOND/2);
}
//----------------------------------------------
// nOn = 1, on; 0: off
//----------------------------------------------
void SetTurboPump(int nOn)
{
	WORD wV;
	if (nOn == 1) {
		wV = wIO_DO | IO_TP_CTRL;
		SysParam.bTurboPumpOn = 1;
	}
	else {
		wV = wIO_DO & (~IO_TP_CTRL);
		SysParam.bTurboPumpOn = 0;
	}
	if (SysParam.bSimuVAC == 0)
		SetDigitalOutput(BOARD_IO, wV);
	OSTimeDly(TICKS_PER_SECOND/2);
}
//
void SetScrollPump(int nOn)
{
	WORD wV;
	if (nOn == 1) {
		wV = wIO_DO | IO_SP_CTRL;
		SysParam.bScrollPumpOn = 1;
#if SHOW_VACUUM_STATE == 1
		printf("SP ON, wait until SP is stable.\n");
#endif
	}
	else {
		wV = wIO_DO & (~IO_SP_CTRL);
		SysParam.bScrollPumpOn = 0;
#if SHOW_VACUUM_STATE == 1
		printf("SP OFF\n");
#endif
	}
	if (SysParam.bSimuVAC == 0)
		SetDigitalOutput(BOARD_IO, wV);
	OSTimeDly(TICKS_PER_SECOND/2);
}
//
void SetRelayOn(int nOn)
{
	WORD wV;
	if (nOn == 1) {
		pwFpga[SET_RELAY_ON] = 1;	//INDEPENDENT_IO
		wV = wSCAN_DO | DO_RELAY_ON;
		SetDigitalOutput(BOARD_SCAN, wV);
	}
	else {
		pwFpga[SET_RELAY_ON] = 0;	//INDEPENDENT_IO
		wV = wSCAN_DO & (~DO_RELAY_ON);
		SetDigitalOutput(BOARD_SCAN, wV);
	}
	//OSTimeDly(1);
}
//---------------------------------------------
// relay control on scan board
//---------------------------------------------
void SetEDSOn(int nOn)
{
	WORD wV;
	if (nOn == 1) {
		bEDSOn = 1;
		wV = wSCAN_DO | DO_EDS_ON;
		SetDigitalOutput(BOARD_SCAN, wV);
	}
	else { //nOn = 0 or 2
		bEDSOn = 0;
		wV = wSCAN_DO & (~DO_EDS_ON);
		SetDigitalOutput(BOARD_SCAN, wV);
	}
}
//----------------------------------------------
// nOn = 1, on; 0: off, turn on/off object constant current source
//----------------------------------------------
void SetObjOn(int nOn)
{
	WORD wV;
	if ((nOn == 1) || (nOn == 2)) {
		if (SysParam.bObjOn == 0) { // spike current protection
			SetVoltage(BOARD_IO, DAC_CH_OBJ_C, OBJ_ON_IMIN_V); //set small current first
			SetVoltage(BOARD_IO, DAC_CH_OBJ_F, 0);
			OSTimeDly(4); //wait until current is stable
		}
		pwFpga[SET_OBJ_ON] = 1;	//INDEPENDENT_IO
		//wSCAN_DO |= DO_OBJ_ON;
		wV = wSCAN_DO | DO_OBJ_ON;
		SetDigitalOutput(BOARD_SCAN, wV);
		SysParam.bObjOn = nOn;
		SetVoltage(BOARD_IO, DAC_CH_OBJ_C, OBJ_ON_INIT_V);
	}
	else { //nOn==0
		SetVoltage(BOARD_IO, DAC_CH_OBJ_C, OBJ_ON_IMIN_V); //make current small first
		SetVoltage(BOARD_IO, DAC_CH_OBJ_F, 0);
		OSTimeDly(1);
		pwFpga[SET_OBJ_ON] = 0;	//INDEPENDENT_IO
		//wSCAN_DO &= (~DO_OBJ_ON);
		wV = wSCAN_DO & (~DO_OBJ_ON);
		SetDigitalOutput(BOARD_SCAN, wV);
		SysParam.bObjOn = 0;
	}
}
//
extern float fFocusVC;
//
void SetObjToMin(int nSetMinimum)
{
	static float fObjVSave = OBJ_ON_IMIN_V;
	static BYTE bEnable = 0;
	if ((nSetMinimum == 1) && (bEnable == 0)) {
		SetVoltage(BOARD_IO, DAC_CH_OBJ_C, OBJ_ON_IMIN_V); //set small current first
		//SetVoltage(DAC_CH_OBJ_F, 0);
		fObjVSave = fFocusVC; //changed in SetFocus()
		bEnable = 1;
	}
	else if ((nSetMinimum == 0) && (bEnable == 1)) { //nEnableMin==0
		SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fObjVSave); //make current small first
		//SetVoltage(DAC_CH_OBJ_F, 0);
		bEnable = 0;
	}
}
//-----------------------------------------------
// deflector current sense resistor selection
// nOn = 1, relay on, resistors parallelled, smaller resistor
// nOn = 0, relay off, resistors singled, larger resistor
// fRatioDefSenR >= 1
//-----------------------------------------------
void SetSenrSel(int nOn)
{
	WORD wV;
	if (nOn == 1) { //smaller resistor (parallel linked)
		pwFpga[SET_SENR_SEL] = 1;	//INDEPENDENT_IO
		//wSCAN_DO |= DO_SENR_SEL;
		wV = wSCAN_DO | DO_SENR_SEL;	//for scan board
		SetDigitalOutput(BOARD_SCAN, wV);
		SysParam.bSenrSel = 1;	//SENRSEL=1, smaller resistor
		SysParam.fDefSenR = NV_Settings.fSenRCalc[1]; //parallel linked resistor, smaller
		NV_Settings.fLoadR[ISEN_CH_DEFX] = SysParam.fDefSenR;
		NV_Settings.fLoadR[ISEN_CH_DEFY] = SysParam.fDefSenR;
	}
	else { //single resistor, larger resistor
		pwFpga[SET_SENR_SEL] = 0;	//INDEPENDENT_IO
		//wSCAN_DO &= (~DO_SENR_SEL);
		wV = wSCAN_DO & (~DO_SENR_SEL);
		SetDigitalOutput(BOARD_SCAN, wV);
		SysParam.bSenrSel = 0; 	//SENRSEL=0, larger resistor
		SysParam.fDefSenR = NV_Settings.fSenRCalc[0];	//larger resistor, SENRSEL=0
		NV_Settings.fLoadR[ISEN_CH_DEFX] = SysParam.fDefSenR;
		NV_Settings.fLoadR[ISEN_CH_DEFY] = SysParam.fDefSenR;
	}
	//SysParam.fDefSenR = current deflector sense resistor value
	fRatioDefSenR = SysParam.fDefSenR / NV_Settings.fSenRCalc[1]; // 1 or 6.4
	fRatioDefSenRMax = NV_Settings.fSenRCalc[0] / NV_Settings.fSenRCalc[1];
}
//
void SetIOState(int nBoard, int nCh, int nOn) //DOUT4, DOUT5
{
	WORD wV[2];
	WORD wMask = 0x0001;

	wMask = wMask << nCh;
	if (nOn == 1) {
		wV[0] = wSCAN_DO | wMask;
		wV[1] = wIO_DO | wMask;
	}
	else if (nOn == 0) {
		wV[0] = wSCAN_DO & (~wMask);
		wV[1] = wIO_DO & (~wMask);
	}
	else
		return;
	if (nBoard == BOARD_IO)
		SetDigitalOutput(BOARD_IO, wV[1]);
	else { //scan board
		SetDigitalOutput(BOARD_SCAN, wV[0]);
	}
}
//
// IO_PCB_VER<=2, get status of gate valve 0 ~ 2 status
// IO_PCB_VER>=3, get status of gate valve 0 ~ 5 status
int GetValveStatus(int nNdx, char *szValue)
{
	WORD wMask = 0;
	if (nNdx == 0)
		wMask = wIO_DO & IO_GV_CTRL0;
	else if (nNdx == 1)
		wMask = wIO_DO & IO_GV_CTRL1;
	else if (nNdx == 2)
		wMask = wIO_DO & IO_GV_CTRL2;
#if IO_PCB_VER >= 3
	else if (nNdx == 3)
		wMask = wIO_DO & IO_GV_CTRL3;
	else if (nNdx == 4)
		wMask = wIO_DO & IO_GV_CTRL4;
	else if (nNdx == 5)
		wMask = wIO_DO & IO_GV_CTRL5;
#endif
	//
	if (wMask != 0) {
		sprintf(szValue, "GV%d:OPEN,", nNdx);
		return VALVE_OPEN;
	}
	else {
		sprintf(szValue, "GV%d:CLOSE,", nNdx);
		return VALVE_CLOSE;
	}
}
//
//execute once per second
void GetVacuumChangeRate(void)
{
	int i, j;
	for (i = 0; i < VAC_GAUGE_NUM; i++) {
		for (j = 0; j < VAC_BUF_NUM - 1; j++) {
			if (fVACLevelBuf[i][j] == 0) //push new data into buffer
				fVACLevelBuf[i][j] = fVACLevel[i];
			else
				fVACLevelBuf[i][j] = fVACLevelBuf[i][j + 1]; //2->1, 3->2, 4->3
			if (bDebug == 10) printf("%.4f,", fVACLevelBuf[i][j]);
		}
		//remove abnormal data
		if ((fVACLevelBuf[i][1] > fVACLevelBuf[i][0]) && (fVACLevelBuf[i][1] > fVACLevelBuf[i][2]))
			fVACLevelBuf[i][1] = (fVACLevelBuf[i][0] + fVACLevelBuf[i][2]) / 2;
		else if ((fVACLevelBuf[i][1] < fVACLevelBuf[i][0]) && (fVACLevelBuf[i][1] < fVACLevelBuf[i][2]))
			fVACLevelBuf[i][1] = (fVACLevelBuf[i][0] + fVACLevelBuf[i][2]) / 2;
		fVACLevelBuf[i][VAC_BUF_NUM - 1] = fVACLevel[i];
		if (bDebug == 10) printf("%.4f,", fVACLevelBuf[i][VAC_BUF_NUM - 1]);
		//rate = new - old
		fVACChangeRate[i] = (fVACLevelBuf[i][VAC_BUF_NUM - 1] - fVACLevelBuf[i][0]);
		if (bDebug == 10) printf("\n");
	}
}
//
int GetVacuumStateName(BYTE bState, char *szValue)
{
	szValue[0] = 0;
	switch (bState)
	{
		case VAC_ST_INIT:
			strcpy(szValue, "INIT");
			break;
		case VAC_ST_AIR:
			strcpy(szValue, "AIR");
			break;
#if MODEL_TYPE == EM_100
		case VAC_ST_STANDBY:
			strcpy(szValue, "STANDBY");
			break;
#else
		case VAC_ST_GUN_TO_AIR:
			strcpy(szValue, "GUN_TO_AIR");
			break;
		case VAC_ST_STANDBY0:
			strcpy(szValue, "STANDBY0");
			break;
		case VAC_ST_STANDBY1:
			strcpy(szValue, "STANDBY1");
			break;
#endif
		case VAC_ST_STANDBY_WAIT:
			strcpy(szValue, "STANDBY_WAIT");
			break;
		case VAC_ST_AIR_WAIT:
			strcpy(szValue, "AIR_WAIT");
			break;
		case VAC_ST_LO:
			strcpy(szValue, "LO");
			break;
		case VAC_ST_LO_WAIT:
			strcpy(szValue, "LO_WAIT");
			break;
		case VAC_ST_HI:
			strcpy(szValue, "HI");
			break;
		case VAC_ST_HI_WAIT:
			strcpy(szValue, "HI_WAIT");
			break;
		case VAC_ST_UH:
			strcpy(szValue, "UH");
			break;
		case VAC_ST_UH_WAIT:
			strcpy(szValue, "UH_WAIT");
			break;
#if GAUGE_OP_MODE == 2
		case VAC_ST_INT_LO_WAIT:
			strcpy(szValue, "INT_LO_WAIT");
			break;
		case VAC_ST_INT_HI_WAIT:
			strcpy(szValue, "INT_HI_WAIT");
			break;
		case VAC_ST_INT2_LO_WAIT:
			strcpy(szValue, "INT2_LO_WAIT");
			break;
		case VAC_ST_INT2_HI_WAIT:
			strcpy(szValue, "INT2_HI_WAIT");
			break;
		case VAC_ST_INT_HI:
			strcpy(szValue, "INT_HI");
			break;
		case VAC_ST_INT_UH_WAIT:
			strcpy(szValue, "INT_UH_WAIT");
			break;
		case VAC_ST_VENT_WAIT:
			strcpy(szValue, "VENT_WAIT");
			break;
		case VAC_ST_NOVENT_WAIT:
			strcpy(szValue, "NOVENT_WAIT");
			break;
		case VAC_ST_ERROR1:
			strcpy(szValue, "ERROR1");
			break;
		case VAC_ST_ERROR2:
			strcpy(szValue, "ERROR2");
			break;
#else
		case VAC_ST_ERROR:
			strcpy(szValue, "ERROR");
			break;
#endif
		default:
			strcpy(szValue, "UNKNOWN");
			return -1;
			break;
	}
	return 0;
}

//return mA
float GetIonPumpCurrent(void)
{
	float fV;
	fV = GetVoltage(ADC_CH_1V_1MA);
	return fV;
}

//input torr
float TorrToVolt(float fTorr)
{
	float fVolt;
	//1 mbar = 0.75 torr
	//PEG-100
	//3MB0-00Q-000P:(1.398~8.6 V)
	//output voltage = [0.6·log (p / mbar) + 6.798] Volt
	//8.6 V --> 1E+4 mbar
	//  2 V --> 1E-8 mbar
	fTorr = fTorr * 1.333; //change to mbar,
	//fVolt = 1.33 * log10(fTorr) + 12.66; //PEG-100
	fVolt = 0.6 * log10(fTorr) + 6.798;
	if (fVolt > 10.0) fVolt = 10.0;
	if (fVolt < 0.0) fVolt = 0.0;
	return fVolt;
}

float VoltToTorr(float fVolt)
{
	float fTorr;
	if (fVolt > 10.0) fVolt = 10.0;
	if (fVolt < 0.0) fVolt = 0.0;
	//fVolt = (fVolt - 12.66) / 1.33;
	fVolt = (fVolt - 6.798) / 0.6;
	fTorr = pow(10.0, fVolt); //unit mbar, change to torr
	fTorr = fTorr * 0.75;
	return fTorr;
}

float GetPiraniGauge(int nNdx)
{
	float fV;
	fV = GetVoltage(ADC_CH_VAC0 + nNdx);
	return fV;
}
//return volt
//high vacuum, voltage low
float GetIonGauge(void)
{
	float fV;
	fV = GetVoltage(ADC_CH_IG);
	//3MAx-xxx-0x0Q:(0.667~10 V)
	//output voltage = [1.33·log (p / mbar) + 12.66] Volt
	//1 mbar = 0.75 torr
	//10 V --> 1E-2 mbar
	//2.0 V --> 1E-8 mbar
	//3MB0-00Q-000P
	//3MBx-xxx-0x0P:(1.398~8.6 V)
	//output voltage = [0.6·log (p / mbar) + 6.798] Volt
	//8.6 V --> 1E+4 mbar
	//2.0 V --> 1E-8 mbar
	return fV;
}
//
// return 1: HV On, 0: HV Off
/*
int GetHVStatus(char *szValue)
{
	WORD wV;
	int nRet = HV_ST_ON;
	//
	wV = GetDigitalInput();
	nRet = (wV & DI_HV_ST) ? HV_ST_ON : HV_ST_OFF;
	if (nRet == HV_ST_ON)
	{
		strcpy(szValue, "HV_ST=1\n");
		return HV_ST_ON;
	}
	else //HV_ST_OFF
	{
		strcpy(szValue, "HV_ST=0\n");
		return HV_ST_OFF;
	}
}
*/
//------------------------------------------------
//return 1:close, 0:open
//CON19.1
//------------------------------------------------
int IsChamberClose(char *szResponse)
{
	//
	if (SysParam.bChamClose == DOOR_OPEN)
	{
		if (szResponse != NULL)
			strcpy(szResponse, "CHAM:OPEN\n");
		return DOOR_OPEN;
	}
	else //if (bRet == TRUE)
	{
		if (szResponse != NULL)
			strcpy(szResponse, "CHAM:CLOSE\n"); //chamber
		return DOOR_CLOSE;
	}
}
//------------------------------------------------
//return 1:close, 0:open
//CON19.3
//------------------------------------------------
int IsCaseClose(char *szResponse)
{
	//
	if (SysParam.bCaseClose == DOOR_OPEN)
	{
		if (szResponse != NULL)
			strcpy(szResponse, "CASE:OPEN\n");
		return DOOR_OPEN;
	}
	else //if (bRet == TRUE)
	{
		if (szResponse != NULL)
			strcpy(szResponse, "CASE:CLOSE\n");
		return DOOR_CLOSE;
	}
}
//--------------------------------------------
//turn high voltage switch on/off, NC/NO
//--------------------------------------------
int SetHVSwitch(BYTE bEnable)
{	//
	WORD wV;
	if (bEnable == 0) {
#if IO_PCB_VER >= 1
		wV = wIO_DO & (~IO_HV_ON);
#else
		wV = wSCAN_DO & (~DO_HV_ON);
#endif
		SysParam.bHVON = 0;
		SetHVFilament(0);
		SetHVBias(0);
		SetHVAccelerator(0);
		//
		//bLEDState[LEDR] = ST_LED_DARK; //blue
		SetLEDState(LEDR, ST_LED_DARK);
	}
	else if (IsVacuumReady() == TRUE) { //VACUUM must be ready
		//SetHVFilament(0);
		//SetHVBias(0);
		//SetHVAccelerator(0);
		//
#if IO_PCB_VER >= 1
		wV = wIO_DO | IO_HV_ON;
#else
		wV = wSCAN_DO | DO_HV_ON;
#endif
		SysParam.bHVON = 1;
		//bLEDState[LEDR] = ST_LED_SOLID;
		SetLEDState(LEDR, ST_LED_SOLID);
	}
	else
		return ERROR_FAIL;
	SetDigitalOutput(BOARD_IO, wV);
	return SUCCESS;
}
//--------------------------------------------
//turn high voltage power +24V on/off
//--------------------------------------------
void SetHVPower(BYTE bEnable)
{
	WORD wV;
	if (bEnable == 0) {
#if IO_PCB_VER >= 1
		wV = wIO_DO & (~IO_HV_PWR);
#else
		wV = wSCAN_DO & (~DO_HV_PWR);
#endif
		//bLEDState[LEDY] = ST_LED_DARK;
		//SetLEDState(LEDY, ST_LED_DARK);
	}
	else {
#if IO_PCB_VER >= 1
		wV = wIO_DO | IO_HV_PWR;
#else
		wV = wSCAN_DO | DO_HV_PWR;
#endif
		//bLEDState[LEDY] = ST_LED_SOLID;
		//SetLEDState(LEDY, ST_LED_SOLID);
	}
	SetDigitalOutput(BOARD_IO, wV);
	SysParam.bHVPower = bEnable;
}
//--------------------------------------------
//set gate valve power on/off
//--------------------------------------------
void SetGVPower(BYTE bEnable)
{
	/* obsolete
	if (bEnable == 0) wSCAN_DO &= (~DO_GV_PWR);
	else wSCAN_DO |= DO_GV_PWR;
	pwFpga[SET_DO0] = wSCAN_DO;
	*/
	SysParam.bGVPower = bEnable;
}
//--------------------------------------------
//set VOPA, VOPB power on/off
//--------------------------------------------
void SetVOPPower(BYTE bEnable)
{
	WORD wV;
	if (bEnable == 1) {
		pwFpga[SET_VOP_ON] = 1;
		wV = wSCAN_DO | DO_VOP_ON;
		SetDigitalOutput(BOARD_SCAN, wV);
	}
	else {
		pwFpga[SET_VOP_ON] = 0;
		wV = wSCAN_DO & (~DO_VOP_ON);
		SetDigitalOutput(BOARD_SCAN, wV);
	}
	nIdleTimeout = IDLE_TIMEOUT;
	/*if (bEnable == 1) { //wait until VOP is stable
		OSTimeDly(TICKS_PER_SECOND);
	}*/
	SysParam.bVOPPower = bEnable;
}
/*
	function reserved, HV will be controlled by RS-232
*/
float GetHVBiasCurrentV(void)
{
	float fV;
	fV = GetVoltage(ADC_CH_BIAS_I);
	//fV = GetVoltage(ADC_CH_MON_2);
	return fV;
}
//-----------------------------------------------
//Accelerator V*4096=actual voltage
//Bias V*600= actual voltage
//Filament V*1.5=actual power (W)
//Current V*33.33=current (uA)
//fV unit: voltage
//-----------------------------------------------
//#define FILA_V_MAX 	5.0
//#define ACC_V_MAX		3.8
//#define BIAS_V_MAX	4.0
//
// fV unit volt
void SetHVFilament(float fV)
{
	if (fV > NV_Settings.fFilaMaxV) //avoid illegal operation
		return;
	SetVoltage(BOARD_IO, DAC_CH_HV_FILA, fV);
}
/*
	function reserved, HV will be controlled by RS-232
*/
void SetHVBias(float fV)
{
	if (fV > NV_Settings.fBiasMaxV) //avoid illegal operation
		return;
	SetVoltage(BOARD_IO, DAC_CH_HV_BIAS, fV);
}
/*
	function reserved, HV will be controlled by RS-232
*/
void SetHVAccelerator(float fV)
{
	if (fV > NV_Settings.fAccMaxV) //avoid illegal operation
		return;
	SetVoltage(BOARD_IO, DAC_CH_HV_ACC, fV);
}
//
void SetHVFEG(float fV)
{
	SetVoltage(BOARD_IO, DAC_CH_HV_FEG, fV);
}
//----------------------------------------------
// input fI in unit mA
// nSen_Ch = ISEN_CH_DEFX or ISEN_CH_DEFY
//----------------------------------------------
void SetDeflectorCurrent(int nSen_Ch, float fI)
{
	float fV;
	int nCoil_Ch;
	//fI=2*fV/R1/R3=2*fV*2/3.3/10=fV/8.25
	//I to V conversion, TBD
	nCoil_Ch = (nSen_Ch == ISEN_CH_DEFX) ? COIL_CH_DEFX : COIL_CH_DEFY;
	fV = fI * NV_Settings.fV2IR[nSen_Ch];
	fV = fV * 0.9;
	fVoltCoarse[nCoil_Ch] = fV;
	//
	fVoltFine[nCoil_Ch] = (fV - fVoltCoarse[nCoil_Ch] * fCoarseRatio[nCoil_Ch]) / fFineRatio[nCoil_Ch];
	if (nSen_Ch == ISEN_CH_DEFX)
		SetDeflectorFineV(DAC_CH_DEFX_F, fVoltFine[nCoil_Ch]);
	else
		SetDeflectorFineV(DAC_CH_DEFY_F, fVoltFine[nCoil_Ch]);
}
//----------------------------------------------
// input fI in unit A
// nCh = DAC_CH_OBJ
//----------------------------------------------
/*void SetObjectiveCurrent(float fI)
{
	//I = 1.1764 - 0.11764 * OBJ_SET
	//OBJ_SET = fDACC * fCoarseRatio[COIL_CH_OBJ] + fDACF * fFineRatio[COIL_CH_OBJ]
	float fObjSet;
	float fObjSetRatio, fObjSetC, fObjSetF;
	float fMax;
	//
	fMax = fDACVMax[BOARD_IO][DAC_CH_OBJ_C];
	fObjSet = (1.1764 - fI) / 0.11764; //0 ~ 10
	if ((fObjSet > fMax) || (fObjSet < 0))
		return;
	//fObjSetRatio = fObjSet/fMax;
	fObjSetC = fObjSet;
	//fObjSetC = fMax * (float)VoltToWord(fObjSetC, DAC_12BIT, fMax, 0) / DAC_12BIT;
	//fObjSetF = (fObjSet - fObjSetC * fCoarseRatio[COIL_CH_OBJ]) / fFineRatio[COIL_CH_OBJ];
	SetVoltage(BAORD_IO, DAC_CH_OBJ_C, fObjSetC);
	//SetVoltage(BAORD_IO, DAC_CH_OBJ_F, fObjSetF);
	if (fI == 0)
		SetObjOn(0);
	else
		SetObjOn(1);
}
*/
#define OBJ_I_TABLE_NUM		11
float fFocusT[OBJ_I_TABLE_NUM]={0, 10.0, 20.0, 30.0, 40.0, 50.0,
		60.0,70.0,80.0,90.0,100.1};
//field of view (FOV) table
//float fFOVT[OBJ_I_TABLE_NUM]={0.52, 0.58, 0.65, 0.72, 0.79, 0.86,
//		0.93,1.00,1.08,1.14,1.22};
//FOV = (0.0038*ATurn + 0.6956) by KW-20171017-02 2017/11/29
float fFOVT[OBJ_I_TABLE_NUM]={0.6956, 0.7336, 0.7716, 0.8096, 0.8476,
		0.8856, 0.9236, 0.9616, 0.9996, 1.0376, 1.0756};
//rotation due to focus change
//float fRotT[OBJ_I_TABLE_NUM]={3.0, 6.0, 8.451, 10.912, 12.472, 13.5,
//		14.85, 16.3, 18.233, 19.978, 22.8};
//ROT = (-0.1157*ATurn + 9.8349) by KW-20171017-02 2017/11/29
float fRotT[OBJ_I_TABLE_NUM]={-9.83, -10.99, -12.15, -13.31, -14.46,
		-15.62, -16.78, -17.93, -19.09, -20.25, -21.40};
//ObjV/17+I/2+(-10)/17 = 0
//I = (10-ObjV)/8.5
//control voltage, object ADC offset must be accurate
float fVT[OBJ_I_TABLE_NUM]={0, 1, 2, 3, 4, 5,
		6, 7, 8, 9, 10};
//current (ampere)
float fIT[OBJ_I_TABLE_NUM]={1.065, 1.040, 0.928, 0.811, 0.693, 0.581,
	0.469, 0.356, 0.234, 0.122, 0.0};
//
//ampere-turn table for 15kV   turn=634  Imax=1.4
//float fINT[OBJ_I_TABLE_NUM]={887.60, 824.20, 768.80, 697.40, 634.00,
//		570.60, 507.20, 380.40, 253.60, 126.80, 0.00};
float fINT[OBJ_I_TABLE_NUM]={1000.0, 900.0, 800.0, 700.0, 600.0,
		500.0, 400.0, 300.0, 200.0, 100.0, 0};
//field of view (FOV) table for 15kV  FOV = (0.0057*ATurn - 1.4412)/3
/*
by KW-20171017-02 turn=616 2017/11/29
15kV  FOV = (0.0019*ATurn - 0.5149)
12kV  FOV = (0.0024*ATurn - 0.6351)
10kV  FOV = (0.0028*ATurn - 0.6882)
 8kV  FOV = (0.0032*ATurn - 0.6617)
*/
float fINFOVT[OBJ_ENERGY_NUM][OBJ_I_TABLE_NUM]={
			{1.3851, 1.1951, 1.0051, 0.8151, 0.6251, 0.4351, 0.2451, 0.0551, -0.1349, -0.3249, -0.5149},
			{1.7649, 1.5249, 1.2849, 1.0449, 0.8049, 0.5649, 0.3249, 0.0849, -0.1551, -0.3951, -0.6351},
			{2.1118, 1.8318, 1.5518, 1.2718, 0.9918, 0.7118, 0.4318, 0.1518, -0.1282, -0.4082, -0.6882},
			{2.5383, 2.2183, 1.8983, 1.5783, 1.2583, 0.9383, 0.6183, 0.2983, -0.0217, -0.3417, -0.6617},
			{2.5383, 2.2183, 1.8983, 1.5783, 1.2583, 0.9383, 0.6183, 0.2983, -0.0217, -0.3417, -0.6617}
		};
//rotation due to focus change for 15kV   ROT = -0.0632*ATurn + 33.737
/*
by KW-20171017-02 turn=616 2017/11/29
15kV  ROT = (-0.0598*ATurn + 28.843)
12kV  ROT = (-0.0780*ATurn + 34.329)
10kV  ROT = (-0.0813*ATurn + 30.951)
 8kV  ROT = (-0.0931*ATurn + 31.814)
*/
float fINRotT[OBJ_ENERGY_NUM][OBJ_I_TABLE_NUM]={
			{-30.96, -24.98, -19.00, -13.02, -7.04, -1.06, 4.92, 10.90, 16.88, 22.86, 28.84},
			{-43.67, -35.87, -28.07, -20.27, -12.47, -4.67, 3.13, 10.93, 18.73, 26.53, 34.33},
			{-50.35, -42.22, -34.09, -25.96, -17.83, -9.70, -1.57, 6.56, 14.69, 22.82, 30.95},
			{-61.29, -51.98, -42.67, -33.36, -24.05, -14.74, -5.43, 3.88, 13.19, 22.50, 31.81},
			{-61.29, -51.98, -42.67, -33.36, -24.05, -14.74, -5.43, 3.88, 13.19, 22.50, 31.81}
		};
//rotation angle
float fTT[OBJ_I_TABLE_NUM]={ -25.5, -19.3, -13.04, -6.518, 0, 6.394,
		12.97, 19.24, 20, 20, 20}; //theta, degree
//ratio table for FOV
float fRT[OBJ_I_TABLE_NUM]={1.25, 1.15, 1.12, 0.920, 0.71, 0.666,
	0.666, 0.666, 0.666, 0.666, 0.666}; //current
//
void SetObjR(int nR)
{	int i;
	float fTT1[OBJ_I_TABLE_NUM]={ -25.5, -19.3, -13.04, -6.518, 0, 6.394,
		12.97, 19.24, 20, 20, 20}; //theta, degree
	float fTT2[OBJ_I_TABLE_NUM]={ -25.5, -19.3, -13.04, -6.518, 0, 6.394,
		12.97, 19.24, 20, 20, 20}; //theta, degree
	float fTT3[OBJ_I_TABLE_NUM]={ -25.5, -19.3, -13.04, -6.518, 0, 6.394,
		12.97, 19.24, 20, 20, 20}; //theta, degree
	if (nR == 1) { //R=17 ohm
		for (i = 0; i < OBJ_I_TABLE_NUM; i++) {
			fIT[i] = (10.0 - fVT[i]) / 8.5;
			fTT[i] = fTT1[i];
		}
	}
	else if (nR == 2) { //R=14 ohm
		for (i = 0; i < OBJ_I_TABLE_NUM; i++) {
			fIT[i] = (10.0 - fVT[i]) / 7;
			fTT[i] = fTT2[i];
		}
	}
	else if (nR == 3) { //R=12 ohm
		for (i = 0; i < OBJ_I_TABLE_NUM; i++) {
			fIT[i] = (10.0 - fVT[i]) / 6;
			fTT[i] = fTT3[i];
		}
	}
	/*for (i = 0; i < OBJ_I_TABLE_NUM; i++) { //ampere turn table
		fINT[i] = fIT[i] * NV_Settings.nObjTurn;
	}*/
}

float ObjFocus2FOVRatio(float fF)
{
	int i;
	float fRatio = 1.0;
	//
	for (i = 0; i < OBJ_I_TABLE_NUM - 1; i++) {
		if ((fF >= fFocusT[i]) && (fF < fFocusT[i + 1])) {
			fRatio = fFOVT[i] + (fFocusT[i] - fF)/(fFocusT[i] - fFocusT[i+1]) * (fFOVT[i+1] - fFOVT[i]);
			return fRatio;
		}
	}
	return fRatio;
}

//return angle in degree
float ObjFocus2Rotate(float fF)
{
	int i;
	float fAngle = 0.0;
	//
	for (i = 0; i < OBJ_I_TABLE_NUM - 1; i++) {
		if ((fF >= fFocusT[i]) && (fF < fFocusT[i + 1])) {
			fAngle = fRotT[i] + (fFocusT[i] - fF)/(fFocusT[i] - fFocusT[i+1]) * (fRotT[i+1] - fRotT[i]);
			return fAngle;
		}
	}
	return fAngle;
}
//-----------------------------------------
// 目前focus變小時，螢幕上東西要變大。
//-----------------------------------------
/*float ObjI2Ratio(float fI)
{
	int i;
	float fRatio = 1.0;
	//
	for (i = 0; i < OBJ_I_TABLE_NUM - 1; i++) {
		if ((fI <= fIT[i]) && (fI > fIT[i + 1])) {
			fRatio = fRT[i] + (fIT[i] - fI)/(fIT[i] - fIT[i+1]) * (fRT[i+1] - fRT[i]);
			return fRatio;
		}
	}
	return fRatio; //fRatioFocus
}*/
//
//get voltage and rotation angle when obj current change
//fI: objective current
//*pfVolt = voltage for obj control
//*pfTheta = rotation angle due to focus change
//
int ObjI2V(float fI, float *pfVolt, float *pfTheta)
{
	//obji=0.692A, 46.97 deg; 0.822A, 55.04 deg--> 62.1 deg/A
	int i;
	//
	for (i = 0; i < OBJ_I_TABLE_NUM - 1; i++) {
		if ((fI <= fIT[i]) && (fI > fIT[i + 1])) {
			*pfVolt = fVT[i] + (fIT[i] - fI)/(fIT[i] - fIT[i+1]) * (fVT[i+1] - fVT[i]);
			*pfTheta = fTT[i] + (fIT[i] - fI)/(fIT[i] - fIT[i+1]) * (fTT[i+1] - fTT[i]);
			return SUCCESS;
		}
	}
	*pfTheta = 0; //can't find the current range
	return ERROR_FAIL; //error
}
//
extern int nObjTableNdx;
//------------------------------------------
// obj ampere turn to voltage
// *pfVolt is control voltage
//------------------------------------------
//IN = ampere turn
int ObjIN2V(float fIN, float *pfVolt, float *pfTheta)
{
	//obji=0.692A, 46.97 deg; 0.822A, 55.04 deg--> 62.1 deg/A
	int i;
	//fINT from large to small
	for (i = 0; i < OBJ_I_TABLE_NUM - 1; i++) {
		if ((fIN <= fINT[i]) && (fIN > fINT[i + 1])) {
			*pfVolt = fVT[i] + (fINT[i] - fIN)/(fINT[i] - fINT[i+1]) * (fVT[i+1] - fVT[i]);
			*pfTheta = fINRotT[nObjTableNdx][i] + (fINT[i] - fIN)/(fINT[i] - fINT[i+1]) * (fINRotT[nObjTableNdx][i+1] - fINRotT[nObjTableNdx][i]);
			return SUCCESS;
		}
	}
	*pfTheta = 0; //can't find the current range
	return ERROR_FAIL; //error
}

float ObjIN2Rotate(float fIN)
{
	int i;
	float fAngle = 0.0;
	//
	for (i = 0; i < OBJ_I_TABLE_NUM - 1; i++) {
		if ((fIN <= fINT[i]) && (fIN > fINT[i + 1])) {
			fAngle = fINRotT[nObjTableNdx][i] + (fINT[i] - fIN)/(fINT[i] - fINT[i+1]) * (fINRotT[nObjTableNdx][i+1] - fINRotT[nObjTableNdx][i]);
			return fAngle;
		}
	}
	return fAngle;
}
//
//目前focus變小時，螢幕上東西要變大。
//IN = ampere turn
float ObjIN2FOVRatio(float fIN)
{
	int i;
	float fRatio = 1.0;
	//
	for (i = 0; i < OBJ_I_TABLE_NUM - 1; i++) {
		if ((fIN <= fINT[i]) && (fIN > fINT[i + 1])) {
			fRatio = fINFOVT[nObjTableNdx][i] + (fINT[i] - fIN)/(fINT[i] - fINT[i+1]) * (fINFOVT[nObjTableNdx][i+1] - fINFOVT[nObjTableNdx][i]);
			return fRatio;
		}
	}
	return fRatio; //fRatioFocus
}
//----------------------------------------------
// input fI in unit mA
// nSen Ch = ISEN_CH_STIGX or ISEN_CH_STIGY, ALIGNER
//----------------------------------------------
void SetCoilCurrent(int nSen_Ch, float fI)
{
	float fV;
	int nADC_Ch;
	//
	if ((nSen_Ch >= ISEN_CH_STIGX) && (nSen_Ch <= ISEN_CH_STIGY))
		nADC_Ch = nSen_Ch - ISEN_CH_STIGX + ADC_CH_STIGX;
	else if ((nSen_Ch >= ISEN_CH_AL0) && (nSen_Ch <= ISEN_CH_AL3))
		nADC_Ch = nSen_Ch - ISEN_CH_AL0 + ADC_CH_AL0;
	else
		return;
	//fI=2*fV/R1/R3=2*fV*2/6.8/20=fV/34
	//I to V conversion, TBD
	fV = fI * NV_Settings.fV2IR[nSen_Ch];
	SetVoltage(BOARD_IO, nADC_Ch, fV);
}
//--------------------------------------------------------------
//nCh = ISEN_CH_DEFX, ISEN_CH_DEFY, ISEN_CH_STIGX, ISEN_CH_OBJ,...
// return current in unit mA
//--------------------------------------------------------------
float GetCoilCurrent(int nSen_Ch)
{
	float fV, fI;
	int nADC_Ch;
	//
	if (nSen_Ch == ISEN_CH_DEFX)
		nADC_Ch = ADC_CH_DEFX;
	else if (nSen_Ch == ISEN_CH_DEFY)
		nADC_Ch = ADC_CH_DEFY;
	else if (nSen_Ch == ISEN_CH_STIGX)
		nADC_Ch = ADC_CH_STIGX;
	else if (nSen_Ch == ISEN_CH_STIGY)
		nADC_Ch = ADC_CH_STIGY;
	else if	 (nSen_Ch == ISEN_CH_OBJ)
	 	nADC_Ch = ADC_CH_OBJ;
	else if ((nSen_Ch >= ISEN_CH_AL0) && (nSen_Ch <= ISEN_CH_AL3))
	 	nADC_Ch = nSen_Ch - ISEN_CH_AL0 + ADC_CH_AL0;
	else
		return 0;
	//----------------------------------------------------
	fV = GetVoltage(nADC_Ch);
	if ((nSen_Ch >= ISEN_CH_DEFX) && (nSen_Ch <= ISEN_CH_DEFY)) {
		//fI = fV / NV_Settings.fLoadR[nSen_Ch];
		fI = fV / SysParam.fDefSenR / fRatioShunt;
	}
	else
		fI = fV / NV_Settings.fLoadR[nSen_Ch];
	return fI;
}
//0 ~ 4
//TEMP0: ADC14
//TEMP1: ADC15
//TEMP2: ADC18
//TEMP3: ADC17
float GetTemperature(int nNdx)
{
	float fV, fTemp;
	static float fPrevTemp[TEMP_SENSOR_NUM_MAX+1] = {20, 20, 20, 20, 20, 20};
	//
	if (nNdx == TEMP_IN_CASE) { //0
		fV = GetVoltage(ADC_CH_TEMPE0);		//ADC14
		if (VoltToTemperature(fV, &fTemp) < 0)		//convert volt to temperature
			fTemp = fPrevTemp[0]; //avoid illegal value
		else
			fPrevTemp[0] = fTemp;
	}
	else if (nNdx == TEMP_COIL_HS) { //1
		fV = GetVoltage(ADC_CH_TEMPE1); 	//ADC15
		if (VoltToTemperature(fV, &fTemp) < 0)
			fTemp = fPrevTemp[1]; //avoid illegal value
		else
			fPrevTemp[1] = fTemp;
	}
	else if (nNdx == TEMP_TURBO_START) //2
		fTemp = (float)nTurboTemperature;
	else if (nNdx == TEMP_POWER_HS) { //3
		fV = GetVoltage(ADC_CH_TEMPE2); //ADC18
		if (VoltToTemperature(fV, &fTemp) < 0)
			fTemp = fPrevTemp[3];
		else
			fPrevTemp[3] = fTemp;
	}
	else if (nNdx == TEMP_TURBO_RUN) { //4
		fTemp = (float)nTurboTemperature;
	}
	else if (nNdx == TEMP_TEMPE3) { //5
		fV = GetVoltage(ADC_CH_TEMPE3); //ADC17
		if (VoltToTemperature(fV, &fTemp) < 0)
			fTemp = fPrevTemp[4];
		else
			fPrevTemp[4] = fTemp;
	}
	fTemperature[nNdx] = fTemp;
	return fTemp;
}
//--------------------------------------------
//nNdx=0, room temperature
//nNdx=1, heat sink1 temperature, coil heatsink
//nNdx=2, turbo pump start temperature
//nNdx=3, heat sink2 temperature, power heatsink
//nNdx=4, turbo pump run temperature
//--------------------------------------------
int CheckTemperature(int nNdx)
{
	float fV;
	static int nTempAlarm[TEMP_SENSOR_NUM_MAX] = {0, 0, 0, 0, 0};
	if (nNdx == 2) { //TP_START
		//don't check when speed is high enough
		if (nTurboRotationSpeed > nTurboReadySpeed * 2 / 3)
			return SUCCESS;
	}
	else if (nNdx == 4) { //TP_RUN
		//don't check when speed is low enough
		if (nTurboRotationSpeed < nTurboReadySpeed - 10)
			return SUCCESS;
	}
	fV = GetTemperature(nNdx);
	if (fV >= NV_Settings.fTempHiLimit[nNdx])
		nTempAlarm[nNdx]++;
	else if (fV <= NV_Settings.fTempLoLimit[nNdx])
		nTempAlarm[nNdx]++;
	else
		nTempAlarm[nNdx] = 0;
	if (nTempAlarm[nNdx] >= 4)
		return ERROR_FAIL;
	else
		return SUCCESS;
}
#if IO_PCB_VER >= 1
WORD GetDigitalInput(int nForce)
{
	int i;
	WORD wMask = 0x0001;
	WORD wDI = 0;
	//
	if (nForce == 1) {
		wDI = GetIO_In();
		//
		if (wDI == 0xFFFF) //invalid value
			return wIO_DI;
		for (i = 0; i < 16; i++) {
			//#define DI_SW0				0x0040	//DIN6
			//#define DI_SW1				0x0080	//DIN7
			SysParam.bIODIVAL[i] = (wDI & wMask) ? 1 : 0;
			wMask = wMask << 1;
		}
	}
	else
		wDI = wIO_DI;
	return wDI;
}
#endif
//
WORD GetDigitalInput(void)
{
	wDI = GetDigitalInput(0); //nForce = 0
	return wDI;
}

//nBoard is not used
WORD GetDOSetting(char *szRet, int nBoard)
{
	char szValue[32];
	char szBuffer[64];
	WORD wV = 0;
	//
	//wV = (nBoard == 0) ? wSCAN_DO : wIO_DO;
	if (szRet == NULL) {
		return wV;
	}
	*szRet = 0;
#if IO_PCB_VER >= 1
	wV = pwFpga[GET_DO];
	WordToBinaryString(wV, szValue);
	sprintf(szBuffer, "DO_0_FPGA=%s\n", szValue);
	strcat(szRet, szBuffer);
	WordToBinaryString(wSCAN_DO, szValue); //bbbb_bbbb_bbbb_bbbb_
	sprintf(szBuffer, "DO_0=%s\n", szValue);
	strcat(szRet, szBuffer);
	WordToBinaryString(wIO_DO, szValue); //bbbb_bbbb_bbbb_bbbb_
	sprintf(szBuffer, "DO_1=%s\n", szValue);
	strcat(szRet, szBuffer);
#else
	WordToBinaryString(wSCAN_DO, szValue); //bbbb_bbbb_bbbb_bbbb_
	sprintf(szBuffer, "DO_0=%s\n", szValue);
	strcat(szRet, szBuffer);
#endif
	return wV;
}
//
void SetDigitalOutput(int nBoard, WORD wV)
{
	int i;
	WORD wChangeBit;
	WORD wMask = 0x0001;
	static BYTE bInit = 2;
	if (nBoard == BOARD_IO) {
		SetIO_Out(wV);
		if (bInit != 0) {
			SetIO_Out(wV); //!!!must set twice, strange behavior
			//SetIO_Out(wV); //!!!must set twice, strange behavior
			bInit--;
		}
#if IO_PCB_VER == 1 //send twice to overcome schmitt trigger problem
		SetIO_Out(wV);
#endif
	}
	else if (nBoard == BOARD_SCAN) {
		wChangeBit = wV ^ wSCAN_DO;
		pwFpga[SET_DO0] = wV;
		wSCAN_DO = wV;
		for (i = 0; i < 16; i++) {
			SysParam.bDOVAL[i] = (wV & wMask) ? 1 : 0;
			wMask = wMask << 1;
		}
		//nV = (wChangeBit & DO_RELAY_ON) ? 1 : 0; //always on
		//pwFpga[SET_RELAY_ON] = (wV & DO_RELAY_ON) ? 1 : 0;	 //PWRON:3:1
		if (wChangeBit & DO_SHUNT0)
			pwFpga[SET_SHUNT] = (wV & DO_SHUNT0) ? 1 : 0; //bit 2
		if (wChangeBit & DO_SENR_SEL)
			pwFpga[SET_SENR_SEL] = (wV & DO_SENR_SEL) ? 1 : 0; //bit 6
		if (wChangeBit & DO_VOP_ON)
			pwFpga[SET_VOP_ON] = (wV & DO_VOP_ON) ? 1 : 0; //bit 8
		if (wChangeBit & DO_OBJ_ON)
			pwFpga[SET_OBJ_ON] = (wV & DO_OBJ_ON) ? 1 : 0; //bit 9
		if (wChangeBit & DO_SEL_COIL)
			pwFpga[SET_SEL_COIL] = (wV & DO_SEL_COIL) ? 1 : 0; //bit 10
		//bit 11: VOPC ON
	}
}
//------------------------------------------------------------
//nBoard=0(SCAN), 1(IO), 2(IO board ext IDC10)
//nBit = 0 ~ 15
//nV = 0 ~ 1
//------------------------------------------------------------
void SetDigitalOutputBit(int nBoard, int nBit, int nV)
{
	WORD wV;
	WORD wMask = 0x0001;
	wMask = wMask << nBit;
	if (nBoard == 2) { //IO extension (IDC10) on IO board
		if (nV == 1) wV = wDIO_OUT | wMask; //5-bit IO
		else wV = wDIO_OUT & (~wMask);
		SetIO_DIO_OUT(wV);
		return;
	}
	if (nBoard == BOARD_SCAN) { //SCAN board
		if (nV == 1) wV = wSCAN_DO | wMask;
		else wV = wSCAN_DO & (~wMask);
		//bSetDOBit = 0;
		SetDigitalOutput(BOARD_SCAN, wV); //???
		//bSetDOBit = 0;
		/*if (wMask == DO_RELAY_ON) //relay always on
			pwFpga[SET_RELAY_ON] = nV;
		else if (wMask == DO_SHUNT0)
			pwFpga[SET_SHUNT] = nV;
		else if (wMask == DO_SENR_SEL)
			pwFpga[SET_SENR_SEL] = nV;
		else if (wMask == DO_VOP_ON)
			pwFpga[SET_VOP_ON] = nV;
		else if (wMask == DO_OBJ_ON)
			pwFpga[SET_OBJ_ON] = nV;
		else if (wMask == DO_SEL_COIL)
			pwFpga[SET_SEL_COIL] = nV;*/
		return;
	}
	if (nBoard == BOARD_IO) { //IO board
		if (nV == 1) wV = wIO_DO | wMask;
		else wV = wIO_DO & (~wMask);
		SetDigitalOutput(BOARD_IO, wV);
		return;
	}
}
/*
// for 16-bit data bus, A0 is always 0, only A[15..1] will changes.
// for example, pwFpga[16], A[15..0] will be 0x0020, not 0x0010
 * wAddr = 0 ~ 0x7FFF
 * DATA_R_NW = 0x80, mapped to A8(0x100)
*/
WORD Addr_RW(int nCS, WORD wAddr, int nRnW, WORD wV)
{
	WORD *pwV;
	//
	if (nCS == 1) //write memory
	{
		//DIRECT_SRAM = 1;
		pwV = pwSram;
	}
	else //if (nCS == 2), write register, DIO, ADC, DAC
	{
		//DIRECT_ADDR = (wAddr & DIRECT_FLAG) ? 1 : 0;
		pwV = pwFpga;
	}
	if (nRnW == 1) //read
	{
		wV = pwV[wAddr];
	}
	else
		pwV[wAddr] = wV;
	//if (nCS == 1)
	//	DIRECT_SRAM = 0;
	return wV;
}

void LEDTest(int nAction)
{
	static BYTE bBuffer[LED_NUM];
	static BYTE bEnable;
	if (nAction == 1) {
		bEnable = bEnableLED;
		memmove(bBuffer, bLEDState, 4);
		bEnableLED = 1;
		SetLEDState(LEDG, ST_LED_BLINK_1HZ);
		SetLEDState(LEDR, ST_LED_BLINK_1HZ);
		SetLEDState(LEDB, ST_LED_BLINK_1HZ);
		SetLEDState(LEDY, ST_LED_BLINK_1HZ);
	}
	else if (nAction >= 2) {
		SetLEDState(LEDG, (BYTE)nAction);
		SetLEDState(LEDR, (BYTE)nAction);
		SetLEDState(LEDB, (BYTE)nAction);
		SetLEDState(LEDY, (BYTE)nAction);
	}
	else { //restore LED state
		bEnableLED = bEnable;
		//bLEDState[0] = ST_LED_DARK;
		//bLEDState[1] = ST_LED_DARK;
		//bLEDState[2] = ST_LED_DARK;
		//bLEDState[3] = ST_LED_DARK;
		memmove(bLEDState, bBuffer, 4);
		SetLEDState(LEDG, bBuffer[0]);
		SetLEDState(LEDR, bBuffer[1]);
		SetLEDState(LEDB, bBuffer[2]);
		SetLEDState(LEDY, bBuffer[3]);
	}
}

void TestSystem(void)
{
	float fV1, fV2, fV;
	float fVInc;
	int nDelay = 6;
	//
	//printf("Check LED\n");
	LEDTest(1);
	//
	SetVoltage(BOARD_SCAN, DAC_CH_VID_REF1, 5);
	SetDeflectorFineV(DAC_CH_DEFX_F, 0);
	//
	SetDeflectorScale(1);
	printf("Check deflector X coil coarse control:");
	SetDeflectorFineV(DAC_CH_DEFX_F, 0);
	OSTimeDly(nDelay);
	fV1 = GetCoilCurrent(ISEN_CH_DEFX);
	if (fV1 > 0.11 || fV1 < 0.09)
		printf("FAIL, %.4f A\n", fV1);
	else
		printf("PASS, %.4f A\n", fV1);
	//
	printf("Check deflector X coil fine control:");
	SetDeflectorScale(2);
	SetDeflectorFineV(DAC_CH_DEFY_F, 0);
	OSTimeDly(1);
	fV1 = GetCoilCurrent(ISEN_CH_DEFY);
	SetDeflectorFineV(DAC_CH_DEFX_F, 1);  //1V --> 0.1 A
	fVInc = 0.1 * fFineRatio[COIL_CH_DEFX];
	OSTimeDly(nDelay);
	fV2 = GetCoilCurrent(ISEN_CH_DEFX);
	if ((fV2 < fV1 + fVInc * 1.2) && (fV2 > fV1 + fVInc * 0.8))
		printf("PASS, I(0)=%.4f,I(1)=%.4f A,INC=%.4f\n", fV1, fV2, fVInc);
	else
		printf("FAIL, I(0)=%.4f,I(1)=%.4f A,INC=%.4f\n", fV1, fV2, fVInc);
	SetDeflectorFineV(DAC_CH_DEFX_F, 0);
	//
	SetDeflectorScale(1);
	printf("Check deflector Y coil coarse control:");
	SetDeflectorFineV(DAC_CH_DEFY_F, 0);
	//SetVoltage(BOARD_IO, DAC_CH_DEFY_C, 1);
	OSTimeDly(nDelay);
	fV1 = GetCoilCurrent(ISEN_CH_DEFY);
	if (fV1 > 0.11 || fV1 < 0.09)
		printf("FAIL, %.4f A\n", fV1);
	else
		printf("PASS, %.4f A\n", fV1);
	//
	printf("Check deflector Y coil fine control:");
	SetDeflectorScale(2);
	SetDeflectorFineV(DAC_CH_DEFY_F, 0);
	OSTimeDly(1);
	fV1 = GetCoilCurrent(ISEN_CH_DEFY);
	SetDeflectorFineV(DAC_CH_DEFY_F, 1);
	fVInc = 0.1 * fFineRatio[COIL_CH_DEFY];
	OSTimeDly(nDelay);
	fV2 = GetCoilCurrent(ISEN_CH_DEFY);
	if ((fV2 < fV1 + fVInc * 1.2) && (fV2 > fV1 + fVInc * 0.8))
		printf("PASS, I(0)=%.4f,I(1)=%.4f A,INC=%.4f\n", fV1, fV2, fVInc);
	else
		printf("FAIL, I(0)=%.4f,I(1)=%.4f A,INC=%.4f\n", fV1, fV2, fVInc);
	SetDeflectorFineV(DAC_CH_DEFY_F, 0);
	//
	printf("Check stigmator X coil:");
	SetVoltage(BOARD_IO, DAC_CH_STIGX, 1);
	OSTimeDly(nDelay);
	fV = GetCoilCurrent(ISEN_CH_STIGX);
	if (fV > 0.11 || fV < 0.09)
		printf("FAIL, %.4f A\n", fV);
	else
		printf("PASS, %.4f A\n", fV);
	SetVoltage(BOARD_IO, DAC_CH_STIGX, 0);
	//
	printf("Check stigmator Y coil:");
	SetVoltage(BOARD_IO, DAC_CH_STIGY, 1);
	OSTimeDly(nDelay);
	fV = GetCoilCurrent(ISEN_CH_STIGY);
	if (fV > 0.11 || fV < 0.09)
		printf("FAIL, %.4f A\n", fV);
	else
		printf("PASS, %.4f A\n", fV);
	SetVoltage(BOARD_IO, DAC_CH_STIGY, 0);
	//
	printf("Check objective coil coarse control:");
	SetObjOn(1);
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, OBJ_ON_IMIN_V); //1V --> 0.17A, 0.33A
	SetVoltage(BOARD_IO, DAC_CH_OBJ_F, 0);
	OSTimeDly(nDelay);
	fV1 = GetCoilCurrent(ISEN_CH_OBJ);
	if (fV1 > 0.36 || fV1 < 0.3)
		printf("FAIL, %.4f A\n", fV1);
	else
		printf("PASS, %.4f A\n", fV1);
	printf("Check objective coil fine control:");
	SetVoltage(BOARD_IO, DAC_CH_OBJ_F, 2); //1V --> 0.012A
	OSTimeDly(nDelay);
	fVInc = 0.117/10*2;
	fV2 = GetCoilCurrent(ISEN_CH_OBJ);
	if ((fV2 > fV1 - fVInc * 1.2) && (fV2 < fV1 - fVInc * 0.8) && (fV2 > 0.29))
		printf("PASS, I(0)=%.4f,I(1)=%.4f A,INC=%.4f\n", fV1, fV2, fVInc);
	else
		printf("FAIL, I(0)=%.4f,I(1)=%.4f A,INC=%.4f\n", fV1, fV2, fVInc);
	SetObjOn(0);
	SetDeflectorFineV(DAC_CH_DEFX_F, 0);
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, OBJ_ON_IMIN_V); //1V --> 0.1A
	SetVoltage(BOARD_IO, DAC_CH_OBJ_F, 0);
	printf("coil integrity check over\n");
	LEDTest(0);
}

void TestBoard(void)
{
	//
}

int IsInterlockSafe(int nAction)
{
	char szResponse[64];
	int nV1, nV2;
	int nTempOK;

	nInterlockCode = SUCCESS;
	if (IsChamberClose(szResponse) == DOOR_OPEN) {
		//printf("CHAMBER OPEN\r\n");
		nInterlockCode = ERROR_CHAM_OPEN;
		return nInterlockCode;
	}
	if (IsCaseClose(szResponse) == DOOR_OPEN) {
		//printf("CASE OPEN\r\n");
		nInterlockCode = ERROR_CASE_OPEN;
		return nInterlockCode;
	}
	//
	nTempOK = CheckTemperature(0); //temperature must be OK
	//
	//check vacuum status
	nV1 = GetGaugeStatus(0, szResponse);
	nV2 = GetGaugeStatus(1, szResponse); //PG1
	//
	switch (nAction)
	{
		case ACT_SET_HV:	//set HVA, HVB, HVF
			break;
		case ACT_HV_ON:
#if GAUGE_OP_MODE == 1 //use 1 PG
#if USE_G1_VOLTAGE == 1 //don't check G0
			if (nV2 != VAC_HI) {//G1 status,
				nInterlockCode = ERROR_VAC1_NOK;
				return nInterlockCode;
			}
#else
			if (nV1 != VAC_HI) {//G0 status
				nInterlockCode = ERROR_VAC0_NOK;
				return nInterlockCode;
			}
			if (nV2 != VAC_HI) {//G1 status
				nInterlockCode = ERROR_VAC1_NOK;
				return nInterlockCode;
			}
#endif
			if (IsTurboSpeedHighEnough() == 0) {
				nInterlockCode = ERROR_TP_NOK;
				return nInterlockCode;
			}
#else //GAUGE_OP_MODE == 2, //use 1 PG + 1 IG
			if (nV2 != VAC_HI) {//G1 status,
				nInterlockCode = ERROR_VAC1_NOK;
				return nInterlockCode;
			}
#endif
			if (nTempOK == ERROR_FAIL) { //board temperature abnormal
				nInterlockCode = ERROR_BTEMP_HI;
				return nInterlockCode;
			}
			break;
		case ACT_SCROLL_ON: //turn scroll pump on
			if (nTempOK == ERROR_FAIL) { //board temperature abnormal
				nInterlockCode = ERROR_BTEMP_HI;
				return nInterlockCode;
			}
			break;
		case ACT_TURBO_ON: //check temperature of turbo pump
			if (nTempOK == ERROR_FAIL) { //board temperature abnormal
				nInterlockCode = ERROR_BTEMP_HI;
				return nInterlockCode;
			}
			if ((nV2 & VAC_L0H) == 0) { //must be low or high
				nInterlockCode = ERROR_VAC1_NOK;
				return nInterlockCode;
			}
			break;
	}
	return SUCCESS;
}
//-------------------------------------------------------------
//  9600: 104
// 38400:  26
//115200:   8
//RS485 address, port 2
//4: motor X
//5: motor Y
//6: motor Z (reserved)
//1: HV module
//-------------------------------------------------------------
int RS485_WriteString(int nPort, char *pcCmd)
{
	int i, n;
	int nDelay;
	int fdS;
	static int nReentry = 1;
	char szBuffer[256];
	//
	fdS = (nPort == 0) ? fdSerial[0] :
		(nPort == 1) ? fdSerial[1] :
		(nPort == 2) ? fdSerial[2] : fdSerial[3];
	if (fdS < 0)
		return -1;
	strcpy(szBuffer, pcCmd);
	n = strlen(szBuffer);
	nReentry = 1;
	if (nPort == 1)
		RS485_TX_ENABLE;
	else if (nPort == 2)
		RS485_TX2_ENABLE;
	else if (nPort == 3)
		RS485_TX3_ENABLE;
	//
	if (NV_Settings.nBaudrate[nPort] < 4800) //avoid abnormal value
		nDelay = 104 * 11; //must be larger than or equal to 11!!!
	else
		nDelay = (1000000/NV_Settings.nBaudrate[nPort]) * RS485_BYTE_DELAY; //11, change to 12, 2016/12/9
	delay_us(nDelay / 2);
	for (i = 0; i < n; i++) {
		writeall(fdS, &szBuffer[i], 1);
		//write() is a non-blocking function
		//writeall() is a blocking function
		// the following delay is a MUST, 2012/11/22
		delay_us(nDelay); //1/9600=100 us
	}
	if (nPort == 1)
		RS485_TX_DISABLE;
	else if (nPort == 2)
		RS485_TX2_DISABLE;
	else if (nPort == 3)
		RS485_TX3_DISABLE;
	nReentry = 0;
	return 0;
}

int RS485_WriteString_P1(char *pcCmd)
{
	int i, n;
	int nDelay;
	int fdS;
	int nPort = 1;
	char szBuffer[256];
	//
	fdS = fdSerial[nPort];
	if (fdS < 0) return -1;
	strcpy(szBuffer, pcCmd);
	n = strlen(szBuffer);
	RS485_TX1_ENABLE;
	if (NV_Settings.nBaudrate[nPort] < 4800) //avoid abnormal value
		nDelay = 104 * 11; //must be larger than or equal to 11!!!
	else
		nDelay = (1000000/NV_Settings.nBaudrate[nPort]) * RS485_BYTE_DELAY; //11, change to 12, 2016/12/9
	delay_us(nDelay / 2);
	OSCritEnter( &OsCrit, 1 );
	for (i = 0; i < n; i++) {
		writeall(fdS, &szBuffer[i], 1);
		//write() is a non-blocking function
		//writeall() is a blocking function
		// the following delay is a MUST, 2012/11/22
		delay_us(nDelay); //1/9600=100 us
	}
	OSCritLeave( &OsCrit );
	RS485_TX1_DISABLE;
	return 0;
}

int RS485_WriteString_P2(char *pcCmd)
{
	int i, n;
	int nDelay;
	int fdS;
	int nPort = 2;
	char szBuffer[256];
	//
	fdS = fdSerial[nPort];
	if (fdS < 0) return -1;
	strcpy(szBuffer, pcCmd);
	n = strlen(szBuffer);
	RS485_TX2_ENABLE;
	if (NV_Settings.nBaudrate[nPort] < 4800) //avoid abnormal value
		nDelay = 104 * 11; //must be larger than or equal to 11!!!
	else
		nDelay = (1000000/NV_Settings.nBaudrate[nPort]) * RS485_BYTE_DELAY; //11, change to 12, 2016/12/9
	delay_us(nDelay / 2);
	OSCritEnter( &OsCrit, 1 );
	for (i = 0; i < n; i++) {
		writeall(fdS, &szBuffer[i], 1);
		//write() is a non-blocking function
		//writeall() is a blocking function
		// the following delay is a MUST, 2012/11/22
		delay_us(nDelay); //1/9600=100 us
	}
	OSCritLeave( &OsCrit );
	RS485_TX2_DISABLE;
	return 0;
}
//---------------------------------------------------
//nPort = 0,1,2
//szData="02030405060A"
void RS232_WriteHex(int nPort, char *szData)
{
	int i, j;
	char bData[64];
	int nByteNum;
	int nSize;
	int fdS;
	//
	nSize = strlen(szData);
	nByteNum = nSize / 2;
	for(i = 0; i < nByteNum; i++)
	{
		sscanf(szData + 2 * i, "%2X", &j);
		bData[i] = j;
		//sprintf(szHexBuffer+i,"%c%c",iHexBuffer[i],10);
		//sprintf(bData + i, "%c", bData[i]);
	}
	bData[nByteNum] = 0;
	fdS = (nPort == 0) ? fdSerial[0] :
		(nPort == 1) ? fdSerial[1] :
		(nPort == 2) ? fdSerial[2] :fdSerial[3];
	if (fdS < 0) return;
	//writeall(fdS, &bData[i], nByteNum);
	for (i = 0; i < nByteNum; i++) {
		writeall(fdS, &bData[i], 1);
		//write() is a non-blocking function
		//writeall() is a blocking function
		//for (j = 0; j < nRS485_Delay; j++) delay_void(); //RELEASE=1500,DEBUG=850
	}
}

void RS232_WriteString(int nPort, char *szStr)
{
	int fdS;
	//
	fdS = (nPort == 0) ? fdSerial[0] :
		(nPort == 1) ? fdSerial[1] : fdSerial[2];
	if (fdS < 0) return;
	writeall(fdS, szStr, strlen(szStr));
	//write() is a non-blocking function
	//writeall() is a blocking function
}

void EnableWDT(int nEnable, float fInterval)
{
	WORD wWMR;
	float fTick;
	//
	fTick = 1.0/(SYSCLK/8192); //=1/9000=0.111 ms
	//9000=1 sec, 1800 = 2 sec
	wWMR = fInterval/fTick;
	if (nEnable == 1) {
		//sim.wtm.wcr = 0x0001; 	//HALTED=0,EN=1
		sim.wtm.wcr = 0x0003; 	//HALTED=1,EN=1
		sim.wtm.wmr = wWMR;
		sim.wtm.wcr = 0x0001;	//HALTED=0,EN=1
	}
	else if (nEnable == 0) {
		sim.wtm.wcr = 0x0002; 	//HALTED=1,EN=0
		sim.wtm.wmr = wWMR;
		sim.wtm.wcr = 0x0000; 	//HALTED=0,EN=0
	}
}

void ResetWDT(void)
{
	sim.wtm.wsr = 0x5555;
	sim.wtm.wsr = 0xAAAA;
	/*
	volatile unsigned short * wsr = (unsigned short *) 0x40140006;
	 *wsr = 0x5555;
	 *wsr = 0xAAAA;
	*/
}

WORD GetWDT(int nV)
{
	if (nV == 0)
		return sim.wtm.wcntr;
	else
		return sim.wtm.wcr;
}

/*DWORD GetAddrParameter(int nType)
{
	DWORD dwAddr = 0;
	return dwAddr;
}*/

void SetBaseAddr(DWORD dwAddr)
{
	WORD wVL, wVH;
	//
	wVL = (WORD)(dwAddr & 0xFFFF);
	wVH = (WORD)((DWORD)(dwAddr & 0xFFFF0000) >> 16);
	pwFpga[SET_ADDR_BASE_L] = wVL;
	pwFpga[SET_ADDR_BASE_H] = wVH;
	return;
}

void AutoScanStart(void)
{	WORD wV;
	int i;
	//
	//pwFpga[SET_SCAN_START] = 0x0001;	//set scanning=1,vadc_scan_start=1
	//delay_void();
	//pwFpga[SET_SCAN_START] = 0x0000;
	//
	for (i = 0; i < 10; i++) {
		pwFpga[SET_SCAN_START] = 0x0001;	//set scanning=1,vadc_scan_start=1
		delay_void();
		pwFpga[SET_SCAN_START] = 0x0000;	//set scanning=1,vadc_scan_start=0
		delay_void();
		pwFpga[SET_SCAN_START] = 0x0000;
		delay_void();
		pwFpga[SET_SCAN_START] = 0x0000;
		delay_void();
		wV = pwFpga[GET_STATUS];
		//ST_RUNNING(0x4000), ST_SCANNING(0x0080)
		//break if running(0x0400)=scanning(0x0080)=1
		if ((wV & 0x4080) == 0x4080)
			break;
	}
	delay_void();
}

void AutoScanStop(void)
{	WORD wV;
	int i;
	for (i = 0; i < 10; i++) {
		pwFpga[SET_SCAN_START] = 0x0002;	//set scanning=0,vadc_scan_stop=1
		delay_void();
		pwFpga[SET_SCAN_START] = 0x0000;
		delay_void();
		pwFpga[SET_SCAN_START] = 0x0000;
		delay_void();
		wV = pwFpga[GET_STATUS];
		//break if running(0x0400)=0 or scanning(0x0080)=0
		if ((wV & 0x4080) != 0x4080)
			break;
	}
}

void AutoScanContinue(void)
{
	pwFpga[SET_SCAN_START] = 0x0004;
	delay_void();
	pwFpga[SET_SCAN_START] = 0x0000;
}

void SetImageInit(void)
{
	WORD wImageStart = 0;
	wImageStart = (nEDSHandshake == 1) ? 0x0002 : 0x0000; //image start pulse
	pwFpga[SET_IMAGE_INIT] = (wImageStart | 1); 	//reset wZMax, wZMin values
	delay_void(); //must be wider than 1 usec
	delay_void();
	delay_void();
	pwFpga[SET_IMAGE_INIT] = 0x0000;
	delay_void();
#if ENABLE_RESET_VADC == 1
	ResetVADC();
#endif
}
//
// this process has some bugs, must try more than one times
void VADCConvStart(void)
{
	WORD wV;
	int i, nTry = 2;
	//
	for (i = 0; i < nTry; i++) {
		pwFpga[VADC_CONV_START] = 0x0001; 	//set cpu_vadc_conv_start flag
		delay_void();
		wV = pwFpga[GET_STATUS];
		if ((wV & ST_VADC_CONV_START) != 0)
			break;
		}
	for (i = 0; i < nTry; i++) {
		pwFpga[VADC_CONV_START] = 0x0000; //clear cpu_vadc_conv_start flag
		delay_void();
		wV = pwFpga[GET_STATUS];
		if ((wV & ST_VADC_CONV_START) == 0)
			break;
	}
}
//---------------------------------------------------
// fRatio <= 1, DEFX10=0
//---------------------------------------------------
int SetCoilRatio(float fRatio)
{
	if (fRatio > SCAN_RATIO_1) {
		fRatioVDAC = 1.0;
		return ERROR_FAIL;
	}
	SetDeflectorScale(1);	//1, obsolete function
	fRatioVDAC = fRatio / SCAN_RATIO_1;	//=fRatio
#ifdef DEBUG_ROTATE
	printf("R_COIL=%f,R_VDAC=%f\n", fRatioCoil, fRatioVDAC);
#endif
	return SUCCESS;
}
// EDS: SCALE_NO_ROTATION, use rotation board to set ratio
// SEM: SCALE_BY_DACVID, no rotation board
void SetDACRefRatio(float fRatio2)
{
	float fV = 0;
	fRatioVDAC = 1.0;
	fRatioTrigno = fRatio2 * fRatioCoil;
	if (bEnableEDS == 1) {
		fV = DACF_REF; //fDACVMax[DAC_CH_VID_REF1]; //generate EDS simulation data
	}
	else if (NV_Settings.bScaleMode == SCALE_BY_TRIG_W_ROT)		//SEM rotation function
		fV = DACF_REF; //fDACVMax[DAC_CH_VID_REF1]; //=VDAC reference
	else if (NV_Settings.bScaleMode == SCALE_BY_DACVID) {			//SEM operation
		//use DAC_VID_REF to set ratio
		fRatioVDAC = fRatio2; //fRatioCoil>=1.0
		fV = DACF_REF * fRatioVDAC * fRatioCoil; //fDACVMax[DAC_CH_VID_REF1] * fRatioVDAC;
		fRatioTrigno = 1.0;
	}
	else if (NV_Settings.bScaleMode == SCALE_BY_TRIG_WO_ROT) {	//EDS operation
		//use SINE and COSINE to set ratio
		fV = DACF_REF; //VDACfDACVMax[DAC_CH_VID_REF1];
	}
	SetVoltage(BOARD_SCAN, DAC_CH_VID_REF1, fV);
}
//-------------------------------------------------------------------
// theta unit: degree
//-------------------------------------------------------------------
void SetRotationAngle(float fTheta)
{
	float fV1 = 5, fV2 = 5;
	float fRatio;
	//
	if (NV_Settings.bObjRotateAdj == 1)
		fTheta += SysParam.fObjAdjAngle;
	if (fTheta > 360)
		fTheta -= 360;
	else if (fTheta < 0)
		fTheta += 360;
	if (bDebug == 17)
		printf("ROTALL=%.2f", fTheta);
	fTheta = fTheta / 180 * 3.141592; //change from degree to radian
	//-------------------------------------------------------------------
	// TRIG: trignometric calculation
	//-------------------------------------------------------------------
	fRatio = (bEDSOn == 1) ? 1.0 : fOverscanP1;
	//if (SysParam.bScanning == OP_BLANKING) {	//rotation function, no EDS
	if (bBlanking != 0) {
		//blanking operation
		//DACVID=5V, DACX_F and DACY_F amplitude = -5V ~ +5V
		fV1 = SINE_COSINE_MAX * sin(fTheta);
		fV2 = SINE_COSINE_MAX * cos(fTheta);
	}
	else if (NV_Settings.bScaleMode == SCALE_BY_TRIG_W_ROT) {	//rotation function, no EDS
		//with rotation
		//DACVID=5V, DACX_F and DACY_F amplitude = -5V ~ +5V
		fV1 = SINE_COSINE_MAX * sin(fTheta) * fRatioTrigno;
		fV1 *= (fRatioFocus * fRatio);
		fV2 = SINE_COSINE_MAX * cos(fTheta) * fRatioTrigno;
		fV2 *= (fRatioFocus * fRatio);
	}
	else if (NV_Settings.bScaleMode == SCALE_BY_DACVID) {	//SEM mode, no rotation board
		//these two DACs (sine, cosine) are useless in this mode
		fV1 = SINE_COSINE_MAX * sin(fTheta);
		fV1 *= (fRatioFocus * fRatio);
		fV2 = SINE_COSINE_MAX * cos(fTheta);
		fV2 *= (fRatioFocus * fRatio);
	}
	else  if (NV_Settings.bScaleMode == SCALE_BY_TRIG_WO_ROT) { //use ROTATION board to set ratio, EDS mode
		//X=XIN*SINE, Y=YIN*COSINE
		//scale by trignometric DAC
		fV1 = SINE_COSINE_MAX *  fRatioTrigno * fARatio[0];
		fV1 *= (fRatioFocus * fRatio);
		fV2 = SINE_COSINE_MAX *  fRatioTrigno * fARatio[1];
		fV2 *= (fRatioFocus * fRatio);
	}
	SetVoltage(BOARD_SCAN, DAC_CH_SINE, fV1);
	SetVoltage(BOARD_SCAN, DAC_CH_COSINE, fV2);
}
//
// return minimal magnification ratio
//
/*float GetMagnificationMin(void)
{
	int nCoilCh;
	//
	//assume full range = X30
	GetScanRangeMax(0, 1); //DEFX=1, calculate .fRangeMax[0], .fRangeMax[1]
	//choose the larger range as base
	nCoilCh = (fRangeMax[0] > fRangeMax[1]) ? 0 : 1; //swing range = 4 ~ 0.8
	//the larger fRangeMax, the smaller fMagMinimum
	fMagMinimum = NV_Settings.fImageWidthOnScreen / 2 / fRangeMax[nCoilCh];
	return fMagMinimum;
}*/
float fMagMin10X = 50;
float fCurrent10X = 0.94;
void CalculateMagMin10X(void)
{
	if (NV_Settings.fImageWidthOnScreen > 200) {
		fMagMin10X = 50;
	}
	else if (NV_Settings.fImageWidthOnScreen > 100) {
		fMagMin10X = 30;
	}
	fMagMin10X = (float)((int)(fMagMinimum/10) + 1) * 10.0;
	fCurrent10X = NV_Settings.fImageWidthOnScreen / fMagMin10X / 2 * NV_Settings.fAPerMM[0][0];
}
//
// set sense resistor and shunt path
//
int SetSenRAndShunt(float fMag)
{
	float fICoil; //, fITot;
	float fIMax = 1.0;
	float fSenR[SENR_NUM] = {DEF_SEN_R1, DEF_SEN_R0};  //R1=1.5, R0=8.2
	float fShunt[SHUNT_NUM] = {DEF_SHUNT_RATIO0, DEF_SHUNT_RATIO1}; //1,21
	int nSenRNdx = 0;
	int nShuntNdx = 0;
	float fIThreshold[5];
	//
	fSenR[0] = NV_Settings.fSenRCalc[0]; //DEF_X10=0, R larger=8.2 ohm
	fSenR[1] = NV_Settings.fSenRCalc[1]; //DEF_X10=1, R smaller=1.2 ohm
	fShunt[0] = 1.0;
	fShunt[1] = NV_Settings.fCoilShuntRatio[0]; //135/4.7=29
	//
	fICoil = fIMaxAbs * (fMagMinimum / fMag);
	//fIMaxAbs = DACF_PV * DEF_V_RATIO / NV_Settings.fSenRCalc[1];
	//----------------------------------------------
	//check from small to large current
	//fIMaxFactor=0.99 is used to avoid overflow
	fIMax = fIMaxAbs * NV_Settings.fIMaxFactor; //max current drivable
	//----------------------------------------------
	// bZoomCtrl  bSenrSel   bCoilShunt  SmallCoil    Magnification
	// 0	      0,small    0		     0            20X
	// 1	      1,large    0		     0            130X  (R ratio = 6.5
	// 2	      0,small    1		     0            560X  (shunt ratio = 28)
	// 3	      1,large    1		     0            3600X (28x6.5x20 = 3600)
	// 4	      1,large    1		     1            10800X (28x6.5x20x3)
	//----------------------------------------------
	fIThreshold[0] = DACF_PV * DEF_V_RATIO / fSenR[1];
	fIThreshold[1] = DACF_PV * DEF_V_RATIO / fSenR[0] * NV_Settings.fIThRatio;
	fIThreshold[1] *= IMAX_LOWER_TH;
	fIThreshold[2] = DACF_PV * DEF_V_RATIO / fSenR[1] / fShunt[1] * NV_Settings.fIThRatio;
	fIThreshold[2] *= IMAX_LOWER_TH;
	fIThreshold[3] = DACF_PV * DEF_V_RATIO / fSenR[0] / fShunt[1] * NV_Settings.fIThRatio;
	fIThreshold[3] *= IMAX_LOWER_TH;
	fIThreshold[4] = DACF_PV * DEF_V_RATIO / fSenR[0] / fShunt[1] * NV_Settings.fIThRatio * NV_Settings.fCoilRatio;
	fIThreshold[4] *= IMAX_LOWER_TH;
	if (fICoil > fIThreshold[0]) //coil current too large
		return ERROR_FAIL;
	else if ((fICoil <= fIThreshold[0]) && (fICoil > fIThreshold[1])) {
		nSenRNdx = 1;		//1: use smaller sense R
		nShuntNdx = 0;		//0: shunt off
		nSelSmallDef = 0;	//0: use whole coil
		SysParam.bZoomCtrl = 0;
	}
	else if ((fICoil <= fIThreshold[1]) && (fICoil > fIThreshold[2])) {
		nSenRNdx = 0; 	//larger sense R, parallel
		nShuntNdx = 0;	//shunt off
		nSelSmallDef = 0;	//use whole coil
		SysParam.bZoomCtrl = 1;
	}
	else if ((fICoil <= fIThreshold[2]) && (fICoil > fIThreshold[3])) {
		nSenRNdx = 1;	//smaller R
		nShuntNdx = 1;	//shunt on
		nSelSmallDef = 0;	//use whole coil
		SysParam.bZoomCtrl = 2;
	}
	else if (NV_Settings.bEnCoilSW == 0) { //if (fICoil <= fIThreshold[3])
		//coil switch function is disabled
		nSenRNdx = 0;	//sense R relay off, larger R
		nShuntNdx = 1;	//shunt on
		nSelSmallDef = 0;	//use whole coil
		SysParam.bZoomCtrl = 3;
	}
	else if ((fICoil <= fIThreshold[3]) && (fICoil > fIThreshold[4])) {
		nSenRNdx = 0;		//0: larger sense R
		nShuntNdx = 1;		//1: shunt on
		nSelSmallDef = 0;	//0: use whole coil
		SysParam.bZoomCtrl = 3;
	}
	else { //use second small coil
		nSenRNdx = 0;
		nShuntNdx = 1;
		nSelSmallDef = 1;	//1: use smaller coil
		SysParam.bZoomCtrl = 4;
	}
	if (bDebug == 3)
		printf("%d(SENR=%.3f),%d(SHUNT_RATIO=%.3f),COIL=%d\n",
			nSenRNdx, fSenR[nSenRNdx], nShuntNdx, fShunt[nShuntNdx], nSelSmallDef);
	//------------------------------------------------------------
	//nSenRSet=1, turn on SenR relay, resistor parallel linked
	//nSelSmallDef=1, turn on coil switch relay, use small coil
	//------------------------------------------------------------
	//nSenRNdx=0, smaller resistor, relay on
	//nSenRNdx=1, larger resistor, relay off
	nSenRSet = nSenRNdx; //inverted
	nShuntSet = nShuntNdx;
	SetSenrSel(nSenRSet); 		//set senr relay, 0:larger ohm, 1: smaller resistor
	OSTimeDly(1); //wait until current stable
	SetCoilShunt(nShuntSet);	//set shunt relay
	OSTimeDly(1); //wait until current stable
	SelectDeflectorCoil(nSelSmallDef); //select small coil
	return SUCCESS;
}
//
void CalculateMagMin(void)
{
	fMagMinimum = NV_Settings.fImageWidthOnScreen / 2 / fIMaxAbs * NV_Settings.fAPerMM[0][0];
	fMagMinimum = fMagMinimum / NV_Settings.fIMaxFactor; //avoid maximal current
}

int SetMagnification(float fMag)
{
	float fRatio1, fRatio2;
	int nRet;
	//
	//fIMaxAbs = maximal current for coil driver
	fIMaxAbs = DACF_PV * DEF_V_RATIO / NV_Settings.fSenRCalc[1]; //smaller one
	//e.g. 286/2/1*0.33 = 47.9	//50
	//e.g. 150/2/1*0.33 = 24.57 //30
	//e.g. 286/2/1.17*0.33 = 40.33
	//fMagMinimum is equal magnification at fIMaxAbs ampere
	fOverscanP1 = 1.0 + fOverscan;
	CalculateMagMin();
	if (fMag < fMagMinimum) {
		if (bDebug == 3)
			printf("MAG TOO SMALL=%.4f, MIN=%.4f\n", fMag, fMagMinimum);
		return ERROR_FAIL;
	}
	fRatio1 = fMagMinimum / fMag; //Ix/Imaxabs
	//
	CalculateMagMin10X(); //50 or 30, calculate fMagMinX10
	//
	nRet = SetSenRAndShunt(fMag); //set sense R, shunt and coil switch
	if (bDebug == 3) {
		printf("MAGMIN=%.4f,R1=%.4f\n", fMagMinimum, fRatio1);
		printf("DEFSEN_RATIO=%.3f,SHUNT_RATIO=%.3f\n", fRatioDefSenR, fRatioShunt);
	}
	if (nRet != SUCCESS)
		return ERROR_FAIL;
	//----------------------------------------------------------------------
	// fRatioDefSenR = 1 or 6.46 (=8.2/1.268)
	// fRatioShunt = 1 or 28.7 (=135/4.7)
	//----------------------------------------------------------------------
	//Ix=Imaxabs*fMagMin/fMag=Imaxabs*fRatio1;
	//Ix/Imaxabs=fRatio1
	//DACF_PV*fRatioVDAC*DEF_V_RATIO = Ix * fRatioDefSenR * fSenRCalc[1] * fRatioShunt ---- (1)
	//[1] is parallel-linked resistor, smaller
	//DACF_PV*DEF_V_RATIO = Imaxabs * fSenRCalc[1] * 1.0 ----------------------------------- (2)
	//(1)/(2)=fRatio2=fRatioVDAC=(Ix/Imaxabs)*fRatioDefSenR*fRatioShunt
	//----------------------------------------------------------------------
	SysParam.fMagNow = fMag;
	fImageWidthReal = NV_Settings.fImageWidthOnScreen / SysParam.fMagNow; //unit mm
	//fRatio1 = Ix/Imaxabs = fMagMinimum/fMag
	//make fRatio2 large enough to set
	fRatio2 = fRatio1 * fRatioDefSenR * fRatioShunt;
	//fRatio1 = fRatioVDAC * fRatioCoil / fRatioDefSenR / fRatioShunt;
	//set VDAC voltage
	SetDACRefRatio(fRatio2); //set video reference voltage by fRatioVDAC
	//set fRatioVDAC and fRotateAngle
	SetRotationAngle(SysParam.fRotateAngle); //set DAC_SINE and DAC_COSINE
	return SUCCESS;
}
//-----------------------------------------
// set fXYRatio[0..1]
//-----------------------------------------
int SetXYDACRatio(float fRangeX, float fRangeY)
{
	float fV1, fV2;
	// image pixel ratio, consider pixel only
	fXYRatio[0] = (fRangeX >= fRangeY) ? 1 : (fRangeX/fRangeY);
	fXYRatio[1] = (fRangeX >= fRangeY) ? (fRangeY/fRangeX) : 1;
	//
	//coil strength ratio
	fV1 = NV_Settings.fAPerMM[0][0];
	fV2 = NV_Settings.fAPerMM[0][1];
	//fARatio[]: coil current ratio
	fARatio[0] = 1.0; //coil0 is always weaker than coil1
	fARatio[1] = (fV2/fV1); //if fV2 larger, need more current
	//coil strength correction, current correction is put at sine cosine
	//fXYRatio[] include pixel number and APERMM correction factor
	if (bEnableEDS == 1) {
		//EDS control signal is scaled by SINE and COSINE, not by VIDEO DAC
		fXYRatio[0] = 1.0; //-5V ~ +5V
		fXYRatio[1] = 1.0;
	}
	else if (NV_Settings.bScaleMode == SCALE_BY_DACVID) {
		fXYRatio[0] = fXYRatio[0] * fARatio[0];
		fXYRatio[1] = fXYRatio[1] * fARatio[1];
	}
	return SUCCESS;
}
//--------------------------------------------------------------------------
// Y deflector coil should has a larger magnetic force (current smaller)
// max X = sqrt(X^2 + Y^2) cos(arctan(Y/X) - theta)
// max Y = sqrt(X^2 + Y^2) sin(arctan(Y/X) + theta)
//--------------------------------------------------------------------------
#define EDGE_STEP_NUM	4
//nX = 0 ~ (nNX-1)
//nY = 0 ~ (nNY-1)
void SetDACPos(int nX, int nY)
{
	if ((nX >= nScanPixelNumX) || (nX < 0))
		return;
	if ((nY >= nScanPixelNumY) || (nY < 0))
		return;
	SetFineDACXY(wDACX[nX], wDACY[nY]);
}

int SetPixelNum(int nNX, int nNY)
{
	int nPixelX, nPixelY;
	int nXNN;
	int nXND2, nYND2;
	//float fTop = 0, fBottom = 0, fEdgeStep = 0;
	float fRangeX, fRangeY;
	float fStepX, fStepY;
	float fVoltX1, fVoltY1;
	//
	fRangeX = NV_Settings.fImageWidthOnScreen / 2 / SysParam.fMagNow; //unit mm
	fRangeY = fRangeX / nNX * (float)nNY;
	if (SetXYDACRatio(fRangeX, fRangeY) != SUCCESS) //set fXYRatio[], fARatio[]
		return ERROR_FAIL;
	//
	if (SysParam.bScanning == OP_ENABLE_CL) {
		fOverscanP1 = 1.0; //for POS command
		nOverscan = 0;
	}
	else if (nEDSHandshake == 1) { //no overscan when external handshake is enabled
		fOverscanP1 = 1.0;
		nOverscan = 0;
	}
	else {
		fOverscanP1 = 1.0 + fOverscan;
		nOverscan = (int)(fOverscan * nNX) / 2;
	}
	pwFpga[SET_X_OVERSCAN] = (WORD)nOverscan;
	pwFpga[SET_X_NUM] = nNX + nOverscan;
	delay_void();
	pwFpga[SET_Y_NUM] = nNY;
	delay_void();
	//
	SetRotationAngle(SysParam.fRotateAngle); //set DAC_SINE and DAC_COSINE DAC
	//
	UpdatePixelNum(nNX, nNY);
	//---------------------------------------------
	// write pattern data into SRAM
	//---------------------------------------------
	wDACFAddrX = 0;
	SetBaseAddr(0);
	//-5*fXYRatio[0] to +5V*fXYRatio[0], 0 ~ (4095/4096)*VREF
	nXNN = nNX;
	nXND2 = nXNN / 2;
	fStepX = (float)(DACF_PV - DACF_MV) * fXYRatio[0] / nXNN / fOverscanP1;
	//
	for (nPixelX = 0; nPixelX <= nXNN + nOverscan; nPixelX++) //X is even, one more point is necessary (<=)
	{	//total nNX+X_SKIP_NUM points
		//fVoltX1 = voltage of DAC output when reference is +5V
		if (nEnableDACXFSet == 1)
			fVoltX1 = fDACXFSet;
		else
			fVoltX1 = fStepX * (float)(nPixelX - nXND2 - nOverscan); //avoid overflow
			//fVoltX1 = DACF_M5V * fXYRatio[0] + fStepX * (float)nPixelX; //avoid overflow
		if (nPixelX == 0) fInitialX = fVoltX1;
		fVoltX1 *= NV_Settings.fScanDACSlope[0];
		fVoltX1 += NV_Settings.fScanDACOffset[0];
		//
#if VDAC_BITN == 16
		wDACX[wDACFAddrX] = VoltToWord(fVoltX1, DAC_16BIT, DACF_OUT_MAX, DACF_OUT_MIN);
#else
		wDACX[wDACFAddrX] = VoltToWord(fVoltX1, DAC_12BIT, DACF_OUT_MAX, DACF_OUT_MIN);
#endif
		pwSram[ADDR_DACX_START + nPixelX] = wDACX[wDACFAddrX];
		wDACFAddrX++;
	}
	wDACX[wDACFAddrX] = wDACX[wDACFAddrX - 1]; //put one more data
	wDACX_Min = wDACX[0];
	wDACX_Max = wDACX[wDACFAddrX - 1];
	//
	wDACFAddrY = 0;
	fStepY = (float)(DACF_PV - DACF_MV) * fXYRatio[1] / nNY / fOverscanP1;
	fStepY = fStepY * fARatio[1];
	nYND2 = nNY/2;
	if (bDebug == 4) fStepY *= 2.0;
	for (nPixelY = 0; nPixelY <= nNY; nPixelY++) //Y is even, one more point is necessary (<=)
	{
		if (NV_Settings.bUP2DN == 1) //positive to negative
			fVoltY1 = fStepY * (float)(nYND2 - nPixelY); //avoid overflow
		else //negative to positive
			fVoltY1 = fStepY * (float)(nPixelY - nYND2); //avoid overflow
		//fVoltY1 = DACF_M5V*fXYRatio[1] + fStepY * (float)nNdx; //avoid overflow
		if (nPixelY == 0) fInitialY = fVoltY1;
		if (nPixelY == nNY - 1) fFinalY = fVoltY1;
		fVoltY1 *= NV_Settings.fScanDACSlope[1];
		fVoltY1 += NV_Settings.fScanDACOffset[1];
		//DACY pattern
#if VDAC_BITN == 16
		wDACY[wDACFAddrY] = VoltToWord(fVoltY1, DAC_16BIT, DACF_OUT_MAX, DACF_OUT_MIN);
#else
		wDACY[wDACFAddrY] = VoltToWord(fVoltY1, DAC_12BIT, DACF_OUT_MAX, DACF_OUT_MIN);
#endif
		pwSram[ADDR_DACY_START + nPixelY] = wDACY[wDACFAddrY];
		wDACFAddrY++;
	}
	wDACY[wDACFAddrY] = wDACY[wDACFAddrY - 1]; //put one more data
	if (NV_Settings.bUP2DN == 1) {
		wDACY_Max = wDACY[0];
		wDACY_Min = wDACY[wDACFAddrY - 1];
	}
	else {
		wDACY_Min = wDACY[0];
		wDACY_Max = wDACY[wDACFAddrY - 1];
	}
	return SUCCESS;
}

int SetViewArea(float fMag, int nNX, int nNY)
{
	int nPixelX, nPixelY;
	int nXNN;
	int nXND2, nYND2;
	//float fTop = 0, fBottom = 0, fEdgeStep = 0;
	float fRangeX, fRangeY;
	float fStepX, fStepY;
	float fVoltX1, fVoltY1;
	//
	if (SetMagnification(fMag) != SUCCESS)
		return ERROR_FAIL;
	fRangeX = NV_Settings.fImageWidthOnScreen / 2 / fMag; //unit mm
	fRangeY = fRangeX / nNX * (float)nNY;
	if (SetXYDACRatio(fRangeX, fRangeY) != SUCCESS) //set fXYRatio[], fARatio[]
		return ERROR_FAIL;
	//
	fOverscanP1 = 1.0 + fOverscan;
	nOverscan = (int)(fOverscan * nNX) / 2;
	pwFpga[SET_X_OVERSCAN] = (WORD)nOverscan;
	//
	SetRotationAngle(SysParam.fRotateAngle); //set DAC_SINE and DAC_COSINE DAC
	//
	UpdatePixelNum(nNX, nNY);		//initialize deflector pattern
	//
	//---------------------------------------------
	// write pattern data into SRAM
	//---------------------------------------------
	wDACFAddrX = 0;
	//pwFpga[SET_ADDR_BASE_L] = 0;
	//pwFpga[SET_ADDR_BASE_H] = 0;
	SetBaseAddr(0);
	//-5*fXYRatio[0] to +5V*fXYRatio[0], 0 ~ (4095/4096)*VREF
	nXNN = nNX;
	nXND2 = nXNN / 2;
	fStepX = (float)(DACF_PV - DACF_MV) * fXYRatio[0] / nXNN / fOverscanP1;
	//
	for (nPixelX = 0; nPixelX < nXNN + nOverscan; nPixelX++) //Y is even
	{	//total nNX+X_SKIP_NUM points
		//fVoltX1 = voltage of DAC output when reference is +5V
		if (nEnableDACXFSet == 1)
			fVoltX1 = fDACXFSet;
		else
			fVoltX1 = fStepX * (float)(nPixelX - nXND2 - nOverscan); //avoid overflow
			//fVoltX1 = DACF_M5V * fXYRatio[0] + fStepX * (float)nPixelX; //avoid overflow
		if (nPixelX == 0) fInitialX = fVoltX1;
		fVoltX1 *= NV_Settings.fScanDACSlope[0];
		fVoltX1 += NV_Settings.fScanDACOffset[0];
		//
#if VDAC_BITN == 16
		wDACX[wDACFAddrX] = VoltToWord(fVoltX1, DAC_16BIT, DACF_OUT_MAX, DACF_OUT_MIN);
#else
		wDACX[wDACFAddrX] = VoltToWord(fVoltX1, DAC_12BIT, DACF_OUT_MAX, DACF_OUT_MIN);
#endif
		pwSram[ADDR_DACX_START + nPixelX] = wDACX[wDACFAddrX];
		wDACFAddrX++;
	}
	wDACX[wDACFAddrX] = wDACX[wDACFAddrX - 1]; //put one more data
	wDACX_Min = wDACX[0];
	wDACX_Max = wDACX[wDACFAddrX - 1];
	//
	wDACFAddrY = 0;
	fStepY = (float)(DACF_PV - DACF_MV) * fXYRatio[1] / nNY / fOverscanP1;
	nYND2 = nNY/2;
	if (bDebug == 4) fStepY *= 2.0;
	for (nPixelY = 0; nPixelY < nNY; nPixelY++) //Y is even
	{
		if (NV_Settings.bUP2DN == 1) //positive to negative
			fVoltY1 = fStepY * (float)(nYND2 - nPixelY); //avoid overflow
		else //negative to positive
			fVoltY1 = fStepY * (float)(nPixelY - nYND2); //avoid overflow
		//fVoltY1 = DACF_M5V*fXYRatio[1] + fStepY * (float)nNdx; //avoid overflow
		if (nPixelY == 0) fInitialY = fVoltY1;
		if (nPixelY == nNY - 1) fFinalY = fVoltY1;
		fVoltY1 *= NV_Settings.fScanDACSlope[1];
		fVoltY1 += NV_Settings.fScanDACOffset[1];
		//DACY pattern
#if VDAC_BITN == 16
		wDACY[wDACFAddrY] = VoltToWord(fVoltY1, DAC_16BIT, DACF_OUT_MAX, DACF_OUT_MIN);
#else
		wDACY[wDACFAddrY] = VoltToWord(fVoltY1, DAC_12BIT, DACF_OUT_MAX, DACF_OUT_MIN);
#endif
		pwSram[ADDR_DACY_START + nPixelY] = wDACY[wDACFAddrY];
		wDACFAddrY++;
	}
	wDACY[wDACFAddrY] = wDACY[wDACFAddrY - 1]; //put one more data
	if (NV_Settings.bUP2DN == 1) {
		wDACY_Max = wDACY[0];
		wDACY_Min = wDACY[wDACFAddrY - 1];
	}
	else {
		wDACY_Min = wDACY[0];
		wDACY_Max = wDACY[wDACFAddrY - 1];
	}
	return SUCCESS;
}
//
int SetTestDAC(void)
{
	int nPX, nPY, nPixelX, nPixelY;
	int nNX, nXNN, nNY;
	//
	float fVoltX1, fVoltY1;
	//
	nNX = 256;
	nNY = 256;
	SetDACRefRatio(1.0); //set DAC video ratio
	SetRotationAngle(SysParam.fRotateAngle);
	//
	UpdatePixelNum(nNX, nNY);		//initialize deflector pattern
	//---------------------------------------------
	// write pattern data into SRAM
	//---------------------------------------------
	wDACFAddrX = 0;
	pwFpga[SET_ADDR_BASE_L] = 0;
	pwFpga[SET_ADDR_BASE_H] = 0;
	//-5 to +5V, 0 ~ (4095/4096)*VREF
	nXNN = nNX;
	for (nPixelX = 0; nPixelX < nXNN; nPixelX++) //Y is even
	{	//total nNX+X_SKIP_NUM points
		nPX = nPixelX/32;
		if (nPX % 4 == 0)
			fVoltX1 = -0.7;
		else if (nPX % 4 == 1)
			fVoltX1 = -0.2;
		else if (nPX % 4 == 2)
			fVoltX1 = 0.3;
		else if (nPX % 4 == 3)
			fVoltX1 = 0.8;
		fVoltX1 += NV_Settings.fScanDACOffset[0];
		//
		wDACX[wDACFAddrX] = VoltToWord(fVoltX1, DAC_12BIT, DACF_OUT_MAX, DACF_OUT_MIN);
		pwSram[ADDR_DACX_START + nPixelX] = wDACX[wDACFAddrX];
		wDACFAddrX++;
	}
	wDACX[wDACFAddrX] = wDACX[wDACFAddrX - 1]; //put one more data
	wDACX_Min = wDACX[0];
	wDACX_Max = wDACX[wDACFAddrX - 1];
	//
	wDACFAddrY = 0;
	for (nPixelY = 0; nPixelY < nNY; nPixelY++) //Y is even
	{
		nPY = nPixelY/32;
		if (nPY % 4 == 0)
			fVoltY1 = -0.7;
		else if (nPY % 4 == 1)
			fVoltY1 = -0.2;
		else if (nPY % 4 == 2)
			fVoltY1 = 0.3;
		else if (nPY % 4 == 3)
			fVoltY1 = 0.8;
		fVoltY1 += NV_Settings.fScanDACOffset[1];
		wDACY[wDACFAddrY] = VoltToWord(fVoltY1, DAC_12BIT, DACF_OUT_MAX, DACF_OUT_MIN);
		pwSram[ADDR_DACY_START + nPixelY] = wDACY[wDACFAddrY];
		wDACFAddrY++;
	}
	wDACY[wDACFAddrY] = wDACY[wDACFAddrY - 1]; //put one more data
	wDACY_Min = wDACY[0];
	wDACY_Max = wDACY[wDACFAddrY - 1];
	return SUCCESS;
}

#define TEST_V_MAX	4.0
//return 0: OK
//return -1: FAIL
int TestDeflector(void)
{
	float fVV[2] = {0};
	float fIV[2] = {0};
	char szBuffer[64];
	int nResult = 0;
	float fRatio;
	//
	TxString(nConnTest, "$DEF_TEST_START\n");
	SetMagnification(40); //SENR=1,SHUNT=0
	fRatioTrigno = 1.0; //full range
	//fSenR[0] = NV_Settings.fSenRCalc[0]; //DEF_X10=0, R larger=8.2 ohm
	//fSenR[1] = NV_Settings.fSenRCalc[1]; //DEF_X10=1, R smaller=1.2 ohm
	//fShunt[0] = 1.0;
	SysParam.fRotateAngle = 0.0;
	SetRotationAngle(SysParam.fRotateAngle);
	fVV[0] = fVV[1] = 0;
	for (fVV[0] = -TEST_V_MAX; fVV[0] <= +TEST_V_MAX; fVV[0] += TEST_V_MAX) {
		if (fVV[0] == 0) continue;
		SetDeflectorFineV(DAC_CH_DEFX_F, fVV[0]);
		SetDeflectorFineV(DAC_CH_DEFY_F, fVV[1]);
		OSTimeDly(TICKS_PER_SECOND / 2);
		GetAllVoltage(NULL);
		fIV[0] = GetVoltage(ADC_CH_DEFX);
		fIV[1] = GetVoltage(ADC_CH_DEFY);
		fRatio = fabs((fVV[0] - fIV[0] / DEF_V_RATIO) / fVV[0]);
		sprintf(szBuffer, "ROT=%.1f,VX=%.3f,VY=%.3f,IX=%.3f,IY=%.3f,R=%.2f\n",
			SysParam.fRotateAngle, fVV[0], fVV[1], fIV[0], fIV[1], fRatio);
		TxString(nConnTest, szBuffer);
		if (fRatio > 0.12) {
			nResult = -1;
			goto TestEnd;
		}
	}
	fVV[0] = fVV[1] = 0;
	for (fVV[1] = -TEST_V_MAX; fVV[1] <= +TEST_V_MAX; fVV[1] += TEST_V_MAX) {
		if (fVV[1] == 0) continue;
		SetDeflectorFineV(DAC_CH_DEFX_F, fVV[0]);
		SetDeflectorFineV(DAC_CH_DEFY_F, fVV[1]);
		OSTimeDly(TICKS_PER_SECOND / 2);
		GetAllVoltage(NULL);
		fIV[0] = GetVoltage(ADC_CH_DEFX);
		fIV[1] = GetVoltage(ADC_CH_DEFY);
		fRatio = fabs((fVV[1] - fIV[1] / DEF_V_RATIO) / fVV[1]);
		sprintf(szBuffer, "ROT=%.1f,VX=%.3f,VY=%.3f,IX=%.3f,IY=%.3f,R=%.2f\n",
			SysParam.fRotateAngle, fVV[0], fVV[1], fIV[0], fIV[1], fRatio);
		TxString(nConnTest, szBuffer);
		if (fRatio > 0.12) {
			nResult = -1;
			goto TestEnd;
		}
	}
	//
	SysParam.fRotateAngle = 90.0;
	SetRotationAngle(SysParam.fRotateAngle);
	fVV[0] = fVV[1] = 0;
	for (fVV[0] = -TEST_V_MAX; fVV[0] <= +TEST_V_MAX; fVV[0] += TEST_V_MAX) {
		if (fVV[0] == 0) continue;
		SetDeflectorFineV(DAC_CH_DEFX_F, fVV[0]);
		SetDeflectorFineV(DAC_CH_DEFY_F, fVV[1]);
		OSTimeDly(TICKS_PER_SECOND / 2);
		GetAllVoltage(NULL); //VX+ -> IY+, VX- ->IY-
		fIV[0] = GetVoltage(ADC_CH_DEFX);
		fIV[1] = GetVoltage(ADC_CH_DEFY);
		fRatio = fabs((fVV[0] - fIV[1] / DEF_V_RATIO) / fVV[0]);
		sprintf(szBuffer, "ROT=%.1f,VX=%.3f,VY=%.3f,IX=%.3f,IY=%.3f,R=%.2f\n",
			SysParam.fRotateAngle, fVV[0], fVV[1], fIV[0], fIV[1], fRatio);
		TxString(nConnTest, szBuffer);
		if (fRatio > 0.12) {
			nResult = -1;
			goto TestEnd;
		}
	}
	fVV[0] = fVV[1] = 0;
	for (fVV[1] = -TEST_V_MAX; fVV[1] <= +TEST_V_MAX; fVV[1] += TEST_V_MAX) {
		if (fVV[1] == 0) continue;
		SetDeflectorFineV(DAC_CH_DEFX_F, fVV[0]);
		SetDeflectorFineV(DAC_CH_DEFY_F, fVV[1]);
		OSTimeDly(TICKS_PER_SECOND / 2);
		GetAllVoltage(NULL); //VY+ -> IX-, VY- ->IX+
		fIV[0] = GetVoltage(ADC_CH_DEFX);
		fIV[1] = GetVoltage(ADC_CH_DEFY);
		fRatio = fabs((fVV[1] + fIV[0] / DEF_V_RATIO) / fVV[1]); //use + !!
		sprintf(szBuffer, "ROT=%.1f,VX=%.3f,VY=%.3f,IX=%.3f,IY=%.3f,R=%.2f\n",
			SysParam.fRotateAngle, fVV[0], fVV[1], fIV[0], fIV[1], fRatio);
		TxString(nConnTest, szBuffer);
		if (fRatio > 0.12) {
			nResult = -1;
			goto TestEnd;
		}
	}
TestEnd:
	SetDeflectorFineV(DAC_CH_DEFX_F, 0.0);
	SetDeflectorFineV(DAC_CH_DEFY_F, 0.0);
	if (nResult == 0)
		TxString(nConnTest, "$TEST=OK\n");
	else
		TxString(nConnTest, "$TEST=FAIL\n");
	return nResult;
}

int TestStigmator(void)
{
	float fVV[2] = {0};
	float fIV[2] = {0};
	char szBuffer[64];
	int nResult = 0;
	float fRatio;
	//
	TxString(nConnTest, "$STIG_TEST_START\n");
	fVV[0] = fVV[1] = 0;
	for (fVV[0] = -TEST_V_MAX; fVV[0] <= +TEST_V_MAX; fVV[0] += TEST_V_MAX) {
		if (fVV[0] == 0) continue;
		SetVoltage(BOARD_IO, DAC_CH_STIGX, fVV[0]);
		SetVoltage(BOARD_IO, DAC_CH_STIGY, fVV[1]);
		OSTimeDly(TICKS_PER_SECOND / 2);
		GetAllVoltage(NULL);
		fIV[0] = GetVoltage(ADC_CH_STIGX);
		fIV[1] = GetVoltage(ADC_CH_STIGY);
		fRatio = fabs((fVV[0] - fIV[0]) / fVV[0]);
		sprintf(szBuffer, "VX=%.3f,VY=%.3f,IX=%.3f,IY=%.3f,R=%.2f\n",
			fVV[0], fVV[1], fIV[0], fIV[1], fRatio);
		TxString(nConnTest, szBuffer);
		if (fRatio > 0.12) {
			nResult = -1;
			goto TestEnd;
		}
	}
	//
	fVV[0] = fVV[1] = 0;
	for (fVV[1] = -TEST_V_MAX; fVV[1] <= +TEST_V_MAX; fVV[1] += TEST_V_MAX) {
		if (fVV[1] == 0) continue;
		SetVoltage(BOARD_IO, DAC_CH_STIGX, fVV[0]);
		SetVoltage(BOARD_IO, DAC_CH_STIGY, fVV[1]);
		OSTimeDly(TICKS_PER_SECOND / 2);
		GetAllVoltage(NULL);
		fIV[0] = GetVoltage(ADC_CH_STIGX);
		fIV[1] = GetVoltage(ADC_CH_STIGY);
		fRatio = fabs((fVV[1] - fIV[1]) / fVV[1]);
		sprintf(szBuffer, "VX=%.3f,VY=%.3f,IX=%.3f,IY=%.3f,R=%.2f\n",
			fVV[0], fVV[1], fIV[0], fIV[1], fRatio);
		TxString(nConnTest, szBuffer);
		if (fRatio > 0.12) {
			nResult = -1;
			goto TestEnd;
		}
	}
TestEnd:
	SetVoltage(BOARD_IO, DAC_CH_STIGX, 0.0);
	SetVoltage(BOARD_IO, DAC_CH_STIGY, 0.0);
	if (nResult == 0)
		TxString(nConnTest, "$TEST=OK\n");
	else
		TxString(nConnTest, "$TEST=FAIL\n");
	return nResult;
}

int TestObjective(void)
{
	int nResult = 0;
	char szBuffer[64];
	float fVV, fIV, fVC;
	float fRatio;
	TxString(nConnTest, "$OBJ_TEST_START\n");
	SetObjOn(1);
	//
	fVV = 5.0;
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fVV); //set small current first
	OSTimeDly(TICKS_PER_SECOND);
	GetAllVoltage(NULL);
	fIV = GetVoltage(ADC_CH_OBJ);
	ObjI2V(fIV, &fVC, &SysParam.fObjAdjAngle);
	fRatio = fabs((fVV - fVC) / fVV);
	sprintf(szBuffer, "VV=%.3f,IV=%.3f,VC=%.3f,R=%.2f\n",
		fVV, fIV, fVC, fRatio);
	TxString(nConnTest, szBuffer);
	if (fRatio > 0.12) {
		nResult = -1;
		goto TestEnd;
	}
	fVV = 3.0;
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fVV); //set small current first
	OSTimeDly(TICKS_PER_SECOND);
	GetAllVoltage(NULL);
	fIV = GetVoltage(ADC_CH_OBJ);
	ObjI2V(fIV, &fVC, &SysParam.fObjAdjAngle);
	fRatio = fabs((fVV - fVC) / fVV);
	sprintf(szBuffer, "VV=%.3f,IV=%.3f,VC=%.3f,R=%.2f\n",
		fVV, fIV, fVC, fRatio);
	TxString(nConnTest, szBuffer);
	if (fRatio > 0.12) {
		nResult = -1;
		goto TestEnd;
	}
TestEnd:
	SetObjOn(0);
	if (nResult == 0)
		TxString(nConnTest, "$TEST=OK\n");
	else
		TxString(nConnTest, "$TEST=FAIL\n");
	return nResult;
}
//---------------------------------------------------------
// set shunt coil, fAPerMM[] is NOT changed
//---------------------------------------------------------
/*void SetCoilShunt(float fMag)
{
	if (fMag >= 1000) {
		COIL_SWITCH_ON;
		SysParam.bCoilShunt = 1;
		fRatioShunt = NV_Settings.fCoilShuntRatio;
		OSTimeDly(2);
	}
	else {
		COIL_SWITCH_OFF;
		SysParam.bCoilShunt = 0;
		fRatioShunt = 1.0;
	}
}*/

//nNdx = 0, BOARD_SCAN
void SetZoomIO(int nNdx, int nOn)
{	//senr_sel = bit 6, 0x0040
	//shunt    = bit 2, 0x0004
	//obj_on   = bit 9, 0x0200
	WORD wV;
	if ((nNdx == 0) && (nOn == 0)) {
		wV = wSCAN_DO & (~DO_SHUNT0);	//DEF_SCALE0=0(bit 2, 0x0004), DEF_SCALE1=1(bit 3, 0x0008)
		SetDigitalOutput(BOARD_SCAN, wV);
	}
	else if ((nNdx == 0) && (nOn == 1)) {
		wV = wSCAN_DO | DO_SHUNT0;
		SetDigitalOutput(BOARD_SCAN, wV);
	}
	/*else if ((nNdx == 1) && (nOn == 0)) {
		wV = wSCAN_DO & (~DO_SHUNT1);	//DEF_SCALE0=0(2), DEF_SCALE1=1(3)
		SetDigitalOutput(BOARD_SCAN, wV);
	}
	else if ((nNdx == 1) && (nOn == 1)) {
		wV = wSCAN_DO | DO_SHUNT1;
		SetDigitalOutput(BOARD_SCAN, wV);
	}*/
}
//--------------------------------------------
// set shunt current ratio
// fRatioShunt >= 1.0
// nOn = 1, relay on, go to shunt path
// nOn = 0, relay off, go to direct path
//--------------------------------------------
void SetCoilShunt(int nOn)
{
	WORD wV;
	if (nOn == 1) {
		pwFpga[SET_SHUNT] = 1;	//INDEPENDENT_IO
		//
		wV = wSCAN_DO | DO_SHUNT0;
		SetDigitalOutput(BOARD_SCAN, wV);
		SysParam.bCoilShunt = 1; //pass large resistor
		fRatioShunt = NV_Settings.fCoilShuntRatio[0];
	}
	else if (nOn == 0) {
		pwFpga[SET_SHUNT] = 0;	//INDEPENDENT_IO
		//
		wV = wSCAN_DO & (~DO_SHUNT0);
		SetDigitalOutput(BOARD_SCAN, wV);
		SysParam.bCoilShunt = 0; //pass small resistor
		fRatioShunt = 1.0;
	}
	fRatioShuntMax = NV_Settings.fCoilShuntRatio[0];
}
//--------------------------------------------------
//1: use small coil
//0: use whole coil
//--------------------------------------------------
void SelectDeflectorCoil(int nSet)
{
	WORD wV;
	if (nSet == 1) {
		//SetDigitalOutputBit(2, 0, 1); //IO1 board CON33.1, use small coil
		fRatioCoil = 1.0 / NV_Settings.fCoilRatio; //>=1
		pwFpga[SET_SEL_COIL] = 1; //INDEPENDENT_IO
		//wSCAN_DO |= DO_SEL_COIL;
		wV = wSCAN_DO | DO_SEL_COIL;
		SetDigitalOutput(BOARD_SCAN, wV);
		//SetDigitalOutputBit(0, 10, 1);	//SCAN CON30.4
	}
	else {
		//SetDigitalOutputBit(2, 0, 0); //IO1 board CON33.PIN1, use whole coil
		fRatioCoil = 1.0;
		pwFpga[SET_SEL_COIL] = 0;	//INDEPENDENT_IO
		//wSCAN_DO &= (~DO_SEL_COIL);
		wV = wSCAN_DO & (~DO_SEL_COIL);
		SetDigitalOutput(BOARD_SCAN, wV);
		//SetDigitalOutputBit(0, 10, 0);	//SCAN CON30.4
	}
}

void PrintZoomCtrl(void)
{
	float fIx, fVx;
	printf("IMAX=%.4f,MAGMIN=%.1f\n", fIMaxAbs, fMagMinimum);
	printf("IMAXFACTOR=%.4f\n", NV_Settings.fIMaxFactor);
	printf("SENRSEL=%d,CSHUNT=%d\n", (int)SysParam.bSenrSel, (int)SysParam.bCoilShunt);
	printf("DEFSENR=%.3f,SHUNTR=%.3f\n", SysParam.fDefSenR, fRatioShunt);
	printf("ZOOMCTRL=%d\n", (int)SysParam.bZoomCtrl); //heat sink? second temperature sensor
	printf("R_COIL=%.3f,R_VDAC=%.3f\n",	fRatioCoil, fRatioVDAC);
	printf("R_FOCUS=%.3f\n", fRatioFocus);
	printf("MAGNOW=%.1fX\n", SysParam.fMagNow);
	fVx = DACF_PV*fRatioCoil*fRatioVDAC; //DAC output
	fIx = fVx*DEF_V_RATIO/SysParam.fDefSenR/fRatioShunt;
	printf("Ix=%.4f mA,Vx=%.3f mV\n", fIx * 1000, fVx * 1000);
}

//used in BOARD_SCAN BR and CO
float PercentToVolt(float fPercent, int nCh)
{
	float fV;
	if (fPercent > 100.0)
		fPercent = 100.0;
	if (fPercent < 0.0)
		fPercent = 0.0;
	fV = (fPercent*fDACVMax[BOARD_SCAN][nCh] + (100.0-fPercent)*fDACVMin[BOARD_SCAN][nCh]) / 100.0;
	return fV;
}

void SetFreqGenerator(int nEnable, float fFreq)
{
	WORD wV;
	float fV;
	wV = (nEnable) ? 0x8000 : 0x0000;
	//--------------------------------------------
	// 1,000,000/25,000/2=20
	// 20,000,000/1000/256=78.0625
	// 20,000,000/100/256=781
	// 20,000,000/50/256=1562
	// 20,000,000/20/256=3906
	fV = (float)NV_Settings.nSysClk * 1e6 / fFreq / 256;
	if (fV > 32768) return; //overflow
	wV = wV + (WORD)fV;
	//pwFpga[SET_FREQ_GEN_NUM] = wV;
}

void GenerateSinePattern(void)
{
	/*int i;
	float fV, fVolt, fStep;
	WORD wV;
	//
	pwFpga[SET_ADDR_BASE_L] = 0;
	pwFpga[SET_ADDR_BASE_H] = 0;
	fStep = (DACF_PV - DACF_MV) / SINE_DATA_NUM;
	for (i = 0; i < SINE_DATA_NUM; i++) {
		//fVolt = DACF_M5V + fStep * (float)i; //avoid overflow
		fV = (float)i/SINE_DATA_NUM*6.2831852;
		fVolt = DACF_PV * sin(fV);
		wV = VoltToWord(fVolt, DAC_16BIT, DACF_OUT_MAX, DACF_OUT_MIN);
		//printf("%d:%.3f,%.3f,0X%04X ", i, fV, fVolt, wV);
		//if (i % 4 == 3) printf("\n");
		pwSram[ADDR_FREQG_START + i] = wV;
	}*/
}
//------------------------------------------------
//this function is called every second
//calculate HV on time
//------------------------------------------------
int CheckHVTime(void)
{
	float fV;
	if (SysParam.bHVON == 0)
		return nHVOnTime;
	fV = GetHVBiasCurrentV();
	//read acc mon I from Spellmen or Matsusada HV
	SysParam.fHVBiasV = fV;
	SysParam.fHVBiasI = fV * NV_Settings.fAccMonI;
	if (fV < 0.3)
		return nHVOnTime;
	nHVOnTime++;
	return nHVOnTime;
}
//
//#define IO_OUT_BITN		10
//#define IO_ADDA_BITN	4
//#define IO_LED_BITN		4
//
void Reset_IO(void)
{
	N_RESET_IO = 1;
	delay_us(10);
	N_RESET_IO = 0;
	OSTimeDly(1);
	N_RESET_IO = 1;
}

void Reset_FPGA(void)
{
	N_RESET_FPGA = 1;
	delay_us(10);
	N_RESET_FPGA = 0;
	delay_us(10);
	N_RESET_FPGA = 1;
}
/*------------------------------------------------
ig_ctrl = spi_in_reg[15];		0x8000
ip_ctrl = spi_in_reg[14];		0x4000
//
tp_ctrl = spi_in_reg[9];		0x0200
sp_ctrl = spi_in_reg[8];		0x0100
hv_pwr_on = spi_in_reg[7];		0x0080
hv_on = spi_in_reg[6];			0x0040
gv_ctrl[2] = spi_in_reg[5];		0x0020
gv_ctrl[1] = spi_in_reg[4];		0x0010
gv_ctrl[0] = spi_in_reg[3];		0x0008
gv_ctrl[5] = spi_in_reg[2];
gv_ctrl[4] = spi_in_reg[1];
gv_ctrl[3] = spi_in_reg[0];
//
------------------------------------------------*/
void SetIO_Out(WORD wV)
{
	int i;
	WORD wMask = 0x0001;
	int nRet = 0;
	if (bIOFPGAMinorVersion == 0) {
		//no IO board, do nothing
		nRet = WriteIODataCRC(IO_SET_IO_OUT0, wV, 16); //must be 16 bits
	}
	else if (wIOFPGAVer >= 0x0403) { //calculate CRC
		for (i = 0; i < 2; i++) {
			nRet = WriteIODataCRC(IO_SET_IO_OUT0, wV, 16);
			if (nRet == 0) break; //OK
			if (i == 0) SaveErrorMessage("SetIO CRC Error\r\n", 0);
		}
	}
	else {
		WriteIOCmdData(IO_SET_IO_OUT1, wV, 16); //write twice to ensure the bits are the same
		WriteIOCmdData(IO_SET_IO_OUT0, wV, 16);
	}
	wIO_DO = wV;
	for (i = 0; i < 16; i++) {
		SysParam.bIODOVAL[i] = (wV & wMask) ? 1 : 0;
		wMask = wMask << 1;
	}
	return;
}
/********************************************************
#define DI_VAC_GAUGE0		0x0001	//DIN0
#define DI_VAC_GAUGE1		0x0002	//DIN1
#define DI_CHAM_CLOSE		0x0004	//DIN2
#define DI_VAC_GAUGE2		0x0008	//DIN3
#define DI_VAC_GAUGE3		0x0010	//DIN4, reserved, IP_HV_ON
#define DI_CASE_CLOSE		0x0020	//DIN5
#define DI_SW0				0x0040	//DIN6
#define DI_SW1				0x0080	//DIN7
#define DI_IP_HV_ON			0x0100	//DIN8
#define DI_EDS_ON			0x0200	//DIN9
#define DI_RESERVE1			0x0400	//DIN10
********************************************************/
WORD GetIO_In(void)
{
	WORD wV = 0x0;
	WORD wVMask;
	static int nReentry = 0;
	//
	if (nReentry == 1)
		return wIO_DI;
	nReentry = 1;
	if (bIOFPGAMinorVersion == 0) { //I/O board is not connected
		SysParam.bCaseClose = DOOR_CLOSE;
		SysParam.bChamClose = DOOR_CLOSE;
		wIO_DI = (DI_VAC_GAUGE0 | DI_VAC_GAUGE1 | DI_VAC_GAUGE2);
		goto GetIO_OK;
	}
	if (bIOFPGAMajorVersion >= 3) {
		wV = WriteIOCmdData(IO_GET_IO_IN, wV, 16);
		//default xFF10
		wVMask = wV & 0x0610; //xxxx_x11x_xxx1_xxxx
		if (wVMask != 0x0610)
			goto GetIO_OK;
		wIO_DI = wV;
		goto GetIO_IN2;
	}
	else if (bIOFPGAMajorVersion >= 2) {
		if (bIOFPGAMinorVersion >= 4) {
			wV = WriteIOCmdData(IO_GET_IO_IN, wV, 16);
			wVMask = wV & 0xA810; //1010_1xxx_xxx0_xxxx
			if (wVMask != 0xA800)
				goto GetIO_OK;
		}
		else
			wV = WriteIOCmdData(IO_GET_IO_IN, wV, 12); //read 12 bits (bit11 to bit0)
	}
	else
		wV = WriteIOCmdData(IO_GET_IO_IN, wV, 8); //read 8 bits (bit7 to bit0)
	wVMask = wV & 0x0010; //DIN4 is always 0
	if ((wV == 0xFF) || (wV == 0xFFFF) || (wVMask != 0)) //invalid value
		wV = wIO_DI;	//restore previous value
	else
		wIO_DI = wV;	//save current value
	//
GetIO_IN2:
	if (bDebug == 8) printf("IO=X%04X,", wV); //0xFF10
	SysParam.bCaseClose = (wV & DI_CASE_CLOSE) ? DOOR_OPEN : DOOR_CLOSE;
	SysParam.bChamClose = (wV & DI_CHAM_CLOSE) ? DOOR_OPEN : DOOR_CLOSE;
	//bVAC_OK[] are obsolete
	//SysParam.bVAC_OK[0] = ((wV & DI_VAC_GAUGE0) == 0) ? VAC_OK : VAC_NOT_OK;
	//SysParam.bVAC_OK[1] = ((wV & DI_VAC_GAUGE1) == 0) ? VAC_OK : VAC_NOT_OK;
	//SysParam.bVAC_OK[2] = ((wV & DI_VAC_GAUGE2) == 0) ? VAC_OK : VAC_NOT_OK;
	//
	//bEDSHWOn = ((wV & DI_EDS_ON) != 0) ? 1 : 0;
	nReentry = 0;
	return wV;
GetIO_OK:
	nReentry = 0;
	return wIO_DI;
}

WORD GetIO_Version(void)
{
	WORD wV = 0x0;
	//read IO board FPGA version
	wV = WriteIOCmdData(IO_GET_VERSION, wV, 8);
	if ((wV == 0xFF) || (wV == 0x00)) { //if IO board is not connected
		bIOFPGAMajorVersion = (wV & 0x00F0) >> 4;
		bIOFPGAMajorVersion = IO_PCB_VER;
		bIOFPGAMinorVersion = 0;
	}
	else {
		bIOFPGAMajorVersion = (wV & 0x00F0) >> 4;
		bIOFPGAMinorVersion = (wV & 0x000F);
	}
	return wV;
}

void SetIO_DIO_OUT(WORD wV)
{
	WORD wMask;
	int i;
	wV &= 0x001F; //dio_out[4:0]
	wDIO_OUT = wDIO_OUT & 0xFFE0; //5 bits IO
	wDIO_OUT |= wV;
	WriteIOCmdData(IO_SET_PARAM, wDIO_OUT, 16);
	wMask = 0x0001;
	for (i = 0; i < 5; i++) {
		SysParam.bDIO_OUT[i] = (wDIO_OUT & wMask) ? 1 : 0;
		wMask = wMask << 1;
	}
}

void SetIO_ADChannel(int nCh)
{
#if DEBUG_SIO == 1
	return;
#endif
	nCh = nCh & 0x0003; //0 ~ 3
	/*nCh = nCh << 14;
	wIOADDA = wIOADDA & 0x3FFF;
	wIOADDA = wIOADDA | nCh; */
	wIOADDA = wIOADDA & 0xFFFC;
	wIOADDA = wIOADDA | nCh;
	//set ADC chip address A0(bit 14), A1(bit 15)
	//???
	WriteIOCmdData(IO_SET_ADC_A, wIOADDA, 2);
}

/*void SetIO_nLDAC(int nSet)
{
	return;
}*/

/*void SetIO_LED(WORD wV)
{
	WriteIOData(IO_SET_LED, wV, 4);
	return;
}*/

void SetIO_LED(int nG, int nR, int nY, int nB)
{
	static WORD wLED = 0;
	WORD wV = 0x0000;
	//led_reg[3:0] = spi_in_reg[15:12];
	if (nG == 0) wV |= 0x0001;	//nG=1, LED ON, bit=0
	if (nR == 0) wV |= 0x0002;	//nR
	if (nY == 0) wV |= 0x0004;	//nY, led_reg[2]
	if (nB == 0) wV |= 0x0008;	//nB, led_reg[3],
	if (wLED == wV) //if LED state no change
		return;
	wLED = wV;
	WriteIOCmdData(IO_SET_LED, wLED, 4);
	return;
}

void SetIO_LED_Auto(int nAuto)
{
	if (nAuto == 1)
		wDIO_OUT = wDIO_OUT | 0x0020; //bit 5 = 1  (led_auto = 1)
	else
		wDIO_OUT = wDIO_OUT & (~0x0020); //bit 5 = 0 (led_auto = 0)
	WriteIOCmdData(IO_SET_PARAM, wDIO_OUT, 16);
}

#if IO_PCB_VER >= 6
//nNdx = 0,1,2
void SetIO_VAC_Gauge(int nNdx, int nOn)
{
	WORD wMask;
	wMask = (nNdx == 0) ? 0x0040 :
		(nNdx == 1) ? 0x0080 : 0x0100;
	if (nOn == 1)
		wDIO_OUT = wDIO_OUT | wMask;
	else
		wDIO_OUT = wDIO_OUT & (~wMask);
	WriteIOCmdData(IO_SET_PARAM, wDIO_OUT, 16);
}

void SetIO_VAC_Gauges(int nOn)
{
	WORD wMask;
	wMask = 0x01C0; //0000_0001_1100_0000
	if (nOn == 1)
		wDIO_OUT = wDIO_OUT | wMask;
	else
		wDIO_OUT = wDIO_OUT & (~wMask);
	WriteIOCmdData(IO_SET_PARAM, wDIO_OUT, 16);
}

void ResetVacPower(void)
{
	SetIO_VAC_Gauges(0);
	OSTimeDly (TICKS_PER_SECOND / 2);
	SetIO_VAC_Gauges(1);
}
#endif

WORD PackLEDPattern(void)
{
	WORD wV, wLED = 0;
	//DARK(0),SOLID(1),1HZ(2),2HZ(3),1P4(4),2P4(5),3AP4(6),3P4(7)
	//1111   ,0000    ,0011  ,0101  ,1110  ,1100  ,0100   ,1000
	WORD wPattern[ST_LED_NUM_MAX] = {0x0F, 0x00, 0x03, 0x05, 0x0E, 0x0C, 0x04, 0x08, 0x0F};
	wV = wPattern[(int)bLEDState[3]]; //y, ready
	wV = wV << 12;
	wLED += wV;
	wV = wPattern[(int)bLEDState[2]]; //b, vacuum
	wV = wV << 8;
	wLED += wV;
	wV = wPattern[(int)bLEDState[1]]; //r, HV
	wV = wV << 4;
	wLED += wV;
	wV = wPattern[(int)bLEDState[0]]; //g, power
	wLED += wV;
	return wLED;
}
//---------------------------------------------------
//0, green, power on/off
//1, red, HV module on/off
//2, blue, vacuum state
//3, yellow, ready
//---------------------------------------------------
void SetLEDState(int nLED, BYTE bState)
{
	WORD wV;
	if (nLED >= LED_NUM) return;
	bLEDState[nLED] = bState;
	if (NV_Settings.bLEDAuto == 0) return;
	wV = PackLEDPattern();
	SetIO_LED_Pattern(wV);
}

void SetIO_LED_Pattern(WORD wV)
{
	static WORD wLED = 0;
	if (wLED == wV) //if LED state no change
		return;
	wLED = wV;
	WriteIOCmdData(IO_SET_LED_RG, wLED, 16);
	return;
}
/*
#define HV_FREQ			3000000		//1 MHz
#define HV_FREQ_MIN		(HV_FREQ/32767/2)+1
//nFreq in unit kHz
int SetIO_HV_Freq(int nType, float fFreq)
{
	WORD wV;
	WORD wSet;
	//avoid abnormal settings
	if ((nType < 0) || (nType > 2))
		return -1;
	//30 MHz oscillator
	if (fFreq > HV_FREQ_MIN) //enable frequency generator
		wV = 0x8000 + (WORD)(HV_FREQ/fFreq/2);
	else //disable frequency generator
		wV = 0;
	wSet = (nType == 0) ? IO_SET_FREQ_ACC :
		(nType == 1) ? IO_SET_FREQ_FILA : IO_SET_FREQ_BIAS;
	//
#if DEBUG_SIO == 1
	return 0;
#endif
	WriteIOCmdData(wSet, wV, 16);
	return 0;
}*/

void GetVersion(void)
{
	WORD wV;
	wV = pwFpga[GET_VERSION];
	wScanFPGAVer = wV;
	bFPGAMajorVersion = (wV&0xFF00)>>8;
	bFPGAMinorVersion = (wV&0x00FF);
	wV = pwFpga[GET_DATE];
	bFPGADateMonth = (wV&0xFF00)>>8;
	bFPGADateDay = (wV&0x00FF);
#if IO_PCB_VER >= 1
	GetIO_Version();
#endif
	wIOFPGAVer = (WORD)bIOFPGAMajorVersion;
	wIOFPGAVer = wIOFPGAVer << 8;
	wIOFPGAVer += (WORD)bIOFPGAMinorVersion;
}

int EDS_Start(void)
{
	if (SysParam.bVacuumState != VAC_ST_READY)
		return ERR_INV_VAC_ST;
	if ((SysParam.bErrorHWVersion & ERR_HW_PCBID) != 0)
		return ERR_INV_PID;
	if (SysParam.bErrorHWVersion != 0)
		return ERR_INV_VERSION; //FPGA and PCB version mismatch
	if (SysParam.bScanning != OP_SEM_SCAN)
		return ERR_INV_MODE;
	bEnableEDS = 1;
	SetEDSOn(bEnableEDS);
	SysParam.bPrevScanning = SysParam.bScanning;
	SysParam.bScanning = OP_EDS;
	return SUCCESS;
}

int EDS_Stop(void)
{
	bEnableEDS = 0;
	SetEDSOn(bEnableEDS);
	SysParam.bScanning = SysParam.bPrevScanning;
	return SUCCESS;
}
//-----------------------------------
// check EDS HW switch
//-----------------------------------
void CheckEDSHWSW(void)
{
	if (bEDSHWOn == 0)
		return;
	if (bEnableEDS == 1) //already on
		return;
	EDS_Start();
}

void PreampOutputCalibration(float fThreshold)
{
	int i;
	float fADC[ADC_CH_NUM_MAX];
	float fMax[VADC_CH_NUM] = {5, 5, 5, 5};
	float fMid[VADC_CH_NUM] = {0, 0, 0, 0};
	float fMin[VADC_CH_NUM] = {-5, -5, -5, -5};
	int nADCCh[VADC_CH_NUM] = {ADC_CH_RESV8, ADC_CH_RESV9, ADC_CH_RESV10, ADC_CH_RESV11};
	int nDACCh[VADC_CH_NUM] = {DAC_CH_RESV0, DAC_CH_RESV1, DAC_CH_RESV2, DAC_CH_RESV3};
	int nDone = 0x0000;
	int nDoneFlag[VADC_CH_NUM] = {0x0001, 0x0002, 0x0004, 0x0008};
	//
	while (nDone != 0x000F) {
		for (i = 0; i < VADC_CH_NUM; i++)
			SetVoltage(BOARD_IO, nDACCh[i], fMid[i]);
		OSTimeDly(1);
		GetAllVoltage(fADC);
		for (i = 0; i < VADC_CH_NUM; i++) {
			//inverted output
			if (fADC[nADCCh[i]] > fThreshold) { //input increase, output decrease
				fMin[i] = fMid[i];
				fMid[i] = (fMax[i] + fMin[i]) / 2;
				printf("CALVID%d=%.3f", i, fMid[i]);
			}
			else if (fADC[nADCCh[i]] < -fThreshold) { //input decrease, output increase
				fMax[i] = fMid[i];
				fMid[i] = (fMax[i] + fMin[i]) / 2;
				printf("CALVID%d=%.3f", i, fMid[i]);
			}
			else nDone |= nDoneFlag[i];
		}
	}
	for (i = 0; i < VADC_CH_NUM; i++)
		NV_Settings.fCalVideo[i] = fMid[i];
}

void SetPreampCalibration(void)
{
	int i;
	int nDACCh[VADC_CH_NUM] = {DAC_CH_RESV0, DAC_CH_RESV1, DAC_CH_RESV2, DAC_CH_RESV3};
	for (i = 0; i < VADC_CH_NUM; i++)
		SetVoltage(BOARD_IO, nDACCh[i], NV_Settings.fCalVideo[i]);
}

void Set1USClkNum(void)
{
	if (NV_Settings.nSysClk == 20) { //20 MHz
		pwFpga[SET_1US_CLK] = 9;
	}
	else if (NV_Settings.nSysClk == 30) {
		pwFpga[SET_1US_CLK] = 14;
	}
	else if (NV_Settings.nSysClk == 40) {
		pwFpga[SET_1US_CLK] = 19;
	}
}

#if ENABLE_RESET_VADC == 1
//--------------------------------------
// reset video ADC AD7655
//--------------------------------------
void ResetVADC(void)
{
	pwFpga[RESET_VADC] = 0x0001;
	delay_us(100);
	pwFpga[RESET_VADC] = 0x0000;
}
#endif

void SetVOPCOn(BYTE bOn)
{
	SetDigitalOutputBit(BOARD_SCAN, 11, (int)bOn);
}
