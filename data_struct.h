/*
 * data_struct.h
 *
 *  Created on: 2012/10/12
 *      Author: USER
 */

#ifndef DATA_STRUCT_H_
#define DATA_STRUCT_H_
//
#define UUID_NUM	16
#define BOARD_SCAN			0		//SCAN BOARD
#define BOARD_IO			1		//IO BOARD
#define BOARD_NUM			2
#define BR_NUM				6 //brightness number
#define CO_NUM				3 //contrast number
#define SYS_VADC_NUM		10
#define OBJ_ENERGY_NUM		5	//15,12,10,8,5 kV
//
#pragma pack (1)
//
typedef struct DCFD_SET	//NFDS
{
	int fdConn;
	IPADDR iaClient;
	WORD wClientPort;
} DCFD_SET;
//
typedef struct ONE_PKT_HEADER
{
	BYTE bHeader1;
	BYTE bHeader2;
	BYTE bPixelX_LSB;	//bZMax_LSB
	BYTE bPixelX_MSB;	//bZMax_MSB
	BYTE bPixelY_LSB;	//bZMin_LSB
	BYTE bPixelY_MSB;	//bZMax_MSB
	BYTE bLen_LSB;
	BYTE bLen_MSB;
	BYTE bAddr[4];		//12,LSB first
	BYTE bReserved[4];	//16
	BYTE bTimeAsc_LSB;
	BYTE bTimeAsc_MSB;
	BYTE bFirstPacket;
	BYTE bFinalPacket;	//20
	BYTE bZMax_LSB;
	BYTE bZMax_MSB;
	BYTE bZMin_LSB;
	BYTE bZMin_MSB;
	BYTE bReserved2[6]; //30
} ONE_PKT_HEADER; //30 bytes
//data packet header and footer
typedef struct DATA_BEGIN_HEADER
{
	BYTE bHeader1;
	BYTE bHeader2;
	BYTE bPixelX_LSB;
	BYTE bPixelX_MSB;
	BYTE bPixelY_LSB;
	BYTE bPixelY_MSB;
	BYTE bLen_LSB;
	BYTE bLen_MSB;
	BYTE bAddr[4];			//LSB first
	BYTE bFooterHeader;		//reserved
	BYTE bPktNdx;
	BYTE bPktNum;
	BYTE bACK;
	BYTE bReserved[2];
	BYTE bNdx;
	BYTE bADCCh;
} DATA_BEGIN_HEADER;

typedef struct DATA_END_HEADER
{
	BYTE bHeader1;
	BYTE bHeader2;
	BYTE bZMax_LSB;
	BYTE bZMax_MSB;
	BYTE bZMin_LSB;
	BYTE bZMin_MSB;
	BYTE bLen_LSB;
	BYTE bLen_MSB;
	BYTE bAddr[4];	//LSB first
	BYTE bResv12;
	BYTE bPktNdx;
	BYTE bPktNum;
	BYTE bACK;
	BYTE bTimeAsc_LSB;
	BYTE bTimeAsc_MSB;
	BYTE bNdx;
	BYTE bADCCh;
} DATA_END_HEADER;

typedef struct DATA_HEADER
{
	BYTE bHeader1;
	BYTE bHeader2;		//2
	BYTE bResv2;
	BYTE bResv3;		//4
	BYTE bResv4;
	BYTE bResv5;		//6
	BYTE bLen_LSB;
	BYTE bLen_MSB;
	BYTE bAddr[4];	//LSB first
	BYTE bResv12;
	BYTE bPktNdx;
	BYTE bPktNum;
	BYTE bACK;	//image refresh
	BYTE breserved[2];
	BYTE bNdx;
	BYTE bADCCh;
} DATA_HEADER;

typedef struct NV_SETTINGS
{
	DWORD dwVerifyKey;
	float fDACSlope[BOARD_NUM][DAC_CH_NUM_MAX]; 	//general purpose DAC
	float fDACOffset[BOARD_NUM][DAC_CH_NUM_MAX]; 	//general purpose DAC
	float fScanDACSlope[VDAC_CH_NUM_MAX];
	float fScanDACOffset[VDAC_CH_NUM_MAX]; 	//video x, y
	float fADCOffset[ADC_CH_NUM_MAX]; //general purpose ADC for BOARD_IO1
	float fScanADCOffset[VADC_CH_NUM_MAX]; 		//video ADC input,0,1,2,3,HDIFF,VDIFF,SUM
	float fV2IR[ISEN_CH_NUM];	//used to calculate voltage on coil, I=Ein/Reff
	float fLoadR[ISEN_CH_NUM];	//used to calulate coil current
	//float fRatioBR; //brightness ratio
	//float fRatioCO; //contrast ratio
	WORD wDelayOverscan;
	BYTE bAutoVAC;
	BYTE bUDPPort;
	float fObjStandbyRatio;
	float fReserved3[4]; //float fScanScale[6];
	float fIGINT2Th;
	int nIdleTimeout;
	float fReserved4[2];
	//COIL_SWITCH
	float fAPerMM[2][2];	//ampere per mm, deflector X, Iout=Ein/10, e.g. 500 mA <-> +5V <-> 3 mm => 0.166
	//[0]: major coil, 1: minor coil
	//float fReserve2[2];
	//int nXSkipNum;
	short int sTurboBrakeSpeed; //
	BYTE bScanPowerSave;
	BYTE bEnableFIFO;
	//float fXYRatio;
	//float fRangeMax[2];	//DACF_OUT_MAX, DACF_OUT_MIN*fV2IR[0]*fAPerMM[0]
	//float fRangeMin;	//8 um
	WORD wWaitEOC;
	WORD wReserve1;
	BYTE bEnableVent;	//1:enable venting
	BYTE bEDSOnByHW;
	BYTE bAcckVAdjust;
	BYTE bReserve3;
	float fReserved[2];
	//0:INCASE, 1:HEAT_SINK, 2:TP_START, 3:HEAT_SINK, 4:TP_RUN
	float fTempHiLimit[TEMP_SENSOR_NUM_MAX];
	float fTempLoLimit[TEMP_SENSOR_NUM_MAX];
	float fImageWidthOnScreen;	//unit mm
	int nTurboType;
	float fCoilShuntRatio[2];	//
	float fSenRCalc[2];		//
	float fVACLevel[2]; 	//VAC0 and VAC1
	float fShuntR[2];		//
	float fSenR[2];			//real resistor value
	float fPulsePerMM;		//motor control spatial resolution
	//BYTE bADCEnable[ADC_CH_NUM_MAX];
	BYTE bEnableWDT;
	BYTE bIOINV;
	BYTE bOnePkt;
	short int sVACNOKTh[2];	//short: 2 bytes x2 = 4bytes
	short int sXNdxInc;
	BYTE bBSEMode;	//1:2x2seg, 2:1x4seg
	short int sHighWaitTimeout;
	float fAutoFocusThreshold; //auto focus threshold
	BYTE bScaleMode;
	BYTE bDACType[2]; //0:SCAN, 1:IO, DACT
	//BYTE bReserved2;
	long lHighWaitTimeout;
	BYTE bObjRotateAdj; 	//1
	int nBaudrate[4]; 		//16, int = 4 bytes
	short int sLowWaitTimeout;		//2, LO_WAIT timeout in seconds
	//short int sHighWaitTimeout;		//2, HI_WAIT timeout in seconds
	BYTE bUseIN; //use ampere-turn
	BYTE bReserved4;
	long lUHighWaitTimeout; 	//2, MH to UH, UH_WAIT timeout in seconds
	short int sAirWaitTimeout;		//2, AIR_WAIT timeout in seconds
	float fLowChangeRate;	//4, LOW_WAIT change rate threshold
	float fHighChangeRate;	//4, READY_WAIT change rate threshold
	float fIMaxFactor;		//4
	float fCalVideo[VADC_CH_NUM];	//video preamp output calibration
	int nSysClk;
	float fVAccRatio; //real voltage = DAC voltage * fVAccratio
	float fVBiasRatio;
	float fFilaRatio;	//watt = fV * fFilaRatio
	float fFilaResistance;
	//int nPLFreq;		//=60 or 50 Hz
	int nDelayLB;		//delay at left bottom point, time unit 50 msec
	BYTE bEnablePLModX;
	BYTE bEnablePLModY;
	//float fPLAmp;
	//float fPLPhase;
	float fReserved6[2];
	BYTE bObjR;
	BYTE bReserved7;
	int nObjTurn;
	float fReserved2;
	WORD wPLAddrNum;		//addr number
	BYTE bPLFreq;
	BYTE bPLSyncAuto;
	BYTE bBinPkt; 	//use binary packet to transmit/receive setting and system status
	BYTE bUP2DN;	//from up to down
	BYTE bAutoXLineDelay; 	//= 1;
	int nDelayDescent; 		//= 4F;
	int nPixelDwellTime; 	//= 20;
	BYTE bHVID[HV_ID_NUM];	//HV module ID
	float fAccMaxV;
	float fFilaMaxV;
	float fBiasMaxV;
	int nTurboReadySpeed[TURBO_TYPE_NUM_MAX];
	float fIGHITh;		//IG: HVAC threshold voltage
	float fIGUHTh;		//IG: UHVAC threshold voltage
	float fPGLOTh;		//pirani low vacuum threshold, 2.8
	float fPGHITh;		//pirani high vacuum threshold, 1.0
	float fIGLOTh;		//IG: LOW threshold voltage
	float fIGMHTh;		//IG: medium high threshold voltage
	short int sMHighWaitTimeout;	//7200, HI to MH
	short int sIGWaitTimeout;
	short int sVentWaitTimeout;
	BYTE bLEDAuto; //IO board LED auto display
	BYTE bEnCoilSW; //use small and large coil
	long lAFPixelNum;
	float fAccMonI;
	int nEdgeDiff;
	float fCoilRatio;	//switch between small and large coil
	float fIThRatio;	//I aspect ratio
	float fFocusMin;	//auto focus
	float fFocusMax;
	int nAFLimitNum;
	BYTE bFocusFOV;		//FOV adjustment with focus change
	char szSN[13];		//serial number
	BYTE bUUID[UUID_NUM];		//UUID_NUM=16
	BYTE bObjMode;	//0:normal, 1:on/off
	//WORD wXInit;
	BYTE bReserved0[2];
	int nPIDBaudrate[2];
	short int sStandbyWaitTimeout;
	float fOverscan;
	int nTurboLowSpeed[TURBO_TYPE_NUM_MAX];
	short int sTPGVTimeout;
	//40
	float fBaseBR[BR_NUM]; //1:BEI, 3:SEI
	float fRangeBR[BR_NUM]; //16
	float fReserved5[2];
	short int sIPLeakNum;			//2,132
	short int sIPLeakOnTime[CHECK_VOLT_NUM];	//8
	short int sIPLeakOffTime[CHECK_VOLT_NUM];  //8,148
	float fIPLeakCheckV[CHECK_VOLT_NUM];	//16,164
	BYTE bHVType;
	BYTE bIPType;
	float fObjIMax[OBJ_ENERGY_NUM];
	float fObjIMin[OBJ_ENERGY_NUM];
	float fBaseCO[CO_NUM];
	float fRangeCO[CO_NUM]; 		//100%
	BYTE bReserved3[40];	//960
} NV_SETTINGS;
//
#define NV_HEADER_1		0x51
#define NV_HEADER_2		0x60
#define SET_HEADER_1	0x7B
#define SET_HEADER_2	0xA4
#define SYS_HEADER_1	0x7C
#define SYS_HEADER_2	0xB4
#define SYS2_HEADER_1	0x7D //SYS_PARAM_2
#define SYS2_HEADER_2	0xB5
#define SYS3_HEADER_1	0x7E //SYS_PARAM_3
#define SYS3_HEADER_2	0xB6
//
typedef struct SET_PARAM
{
	BYTE bHeader1;
	BYTE bHeader2;
	BYTE bTurboType;	//
	BYTE bScaleMode;
	// byte alignment
	float fObjIMax[OBJ_ENERGY_NUM];		//8
	float fObjIMin[OBJ_ENERGY_NUM];		//12
	float fAPerMM[2];	//20
	float fMagMinimum;  //24
	BYTE bHWUSDelay;	//25
	//BYTE bEnablePLModX;		//26, enable PL modulation X
	//BYTE bEnablePLModY;		//27
	BYTE bReserved1[3];	//28
	// byte alignment
	float fPLAmp;		//32
	float fPLPhase;		//36
	float fReserved1;	//40
	BYTE bPLFreq;		//50,60
	BYTE bPLSyncAuto;	//0,1
	WORD wTurboReadySpeed;
	BYTE bBinPkt;
	BYTE bReserved[83];	//pragma pack (4)
} SET_PARAM;
//
//must be aligned with 4 bytes margin
typedef struct SYS_PARAM_3
{
	BYTE bHeader1;
	BYTE bHeader2;
	BYTE bVersion;
	BYTE bCaseClose; //1:open, 0:close
	BYTE bChamClose; //1:open, 0:close
	BYTE bCoilShunt;
	BYTE bSenrSel;
	BYTE bSimuMode;
	BYTE bGVPower;		//8
	BYTE bHVPower;
	BYTE bVOPPower;
	BYTE bHVON;
	BYTE bObjOn;
	BYTE bPLSync; 		//0,1,2,3
	BYTE bScanning;
	BYTE bPrevScanning;	//16
	BYTE bScrollPumpOn;
	BYTE bTcpConnected;
	BYTE bTurboPumpOn;
	BYTE bVacuumState;		//current vacuum state
	BYTE bNextVacuumState;	//next vacuum state
	BYTE bPrevVacuumState;
	BYTE bTxData;
	BYTE bHVONSW;	//24, 4-byte alignment is important!!
	BYTE bReserved1[4];
	BYTE bIonPumpOn;
	BYTE bIonGaugeOn;
	BYTE bFastScan;
	// HV module parameters
	BYTE bFilaOK;		//40
	BYTE bFilaWDT;		//1
	BYTE bReserved2[3];
	float fFilaI; 		//44
	float fFilaV;
	float fHVBiasV;
	float fHVBiasI;		//56
	float fBiasV;		//4, custom HV bias V
	//
	BYTE bDOVAL[16]; 	//72,scan board digital output
	BYTE bIODIVAL[16];	//88,io board digital input
	BYTE bIODOVAL[16]; 	//104,io board digital output
	//
	short int sGaugeStatus[VAC_GAUGE_NUM_MAX]; //8
	short int sTPSpeed; //2
	WORD wXLineDelay;	//2
	WORD wDI;			//2
	WORD wDO[2]; 		//4, 0:SCAN, 1:IO
	BYTE bReserved3[2];
	long lHVOnTime;		//
	float fMagNow;
	float fDefSenR;
	float fTempTP;			//130,turbo pump temperature
	float fTempIN;			//134,TEMP0, current temperature
	float fTempHS[3];		//146,heatsink temperature
	float fTimeXAsc;		//150,X scan ascending time
	float fTimeX;			//154,X scan time
	float fADC[ADC_CH_NUM_MAX]; //250,24*4
	float fVADC[SYS_VADC_NUM];		//282, video ADC
	float fRotateAngle;
	float fObjAdjAngle;		//290
	BYTE bCPU_IO[4];		//294,VAC_GATE_NUM_MAX
	BYTE bDIO_OUT[5]; 		//299
	BYTE bSimuVAC;    		//300
	BYTE bGateValve2[6]; 	//308,VAC_GATE_NUM_MAX=6
	short int sVacTimeout; //302 (unit min for EM_200, sec for EM_100)
	BYTE bZoomCtrl;
	BYTE bPZTMoveX;
	BYTE bPZTMoveY;
	BYTE bStandby;	//1:VENT, 0:VACUUM
	BYTE bCusHVON;		//1
	BYTE bReserved4;
	//float fAccVoltage; 	//4		//custom HV
	//float fAccCurrent; 	//4		//custom HV
	float fReserved6[2];
	//IP_CTRL parameters
	float fIpHV;		//4
	float fIpAmp;		//4
	BYTE bMotorMove;	//1
	BYTE bIPON;			//1
	short int sMotorTime;		//2
	short int sTPGVTimeout; 	//2
	short int sIGTimeout;		//342, (unit min for EM-200)
	BYTE bErrorHWVersion;
	BYTE bReserved5;
	short int sOverscan;
	BYTE bReserved7[10];
} SYS_PARAM_3;
//
typedef struct SYS_PARAM_2 //HV parameters
{
	BYTE bHeader1;
	BYTE bHeader2;
	BYTE bVersion;
	BYTE bReserved1;
	//
	BYTE bExtCtrl;
	BYTE bFilaOK;		//40
	BYTE bFilaWDT;
	BYTE bCusHVON;
	BYTE bHVONSW;
	BYTE bReserved2[3];
	//
	float fFilaI; 		//A
	float fFilaV;		//
	//read acc mon I from Spellmen or Matsusada HV
	float fHVBiasV;		//unit volt
	float fHVBiasI;		//unit uA
	float fBiasV;		//real bias voltage (custom HV)
	//GETV, ADC voltage
	//0: ACC primary sense
	//1: filament primary sense
	//2: ACC feedback
	//3: ACC monitor I
	//4: external filament control
	//5: external ACC control
	//6: VCC power primary sense //reserved
	//7: VCC primary external control (VCC power supply)
	float fADCV[8];
	float fTempe[2];
	float fAccVoltage; 	//4		//custom HV, ACC_FB, unit kV
	float fAccCurrent; 	//4		//custom HV, ACC_MON_I, unit uA
	BYTE bReserved3[68];
} SYS_PARAM_2;

typedef struct AUTO_FOCUS
{
	int nFocusSet[AF_LIMIT_NUM];
	DWORD dwFocusGradient[AF_LIMIT_NUM];
	float fFocusLimitSet[AF_LIMIT_NUM];
	float fGradient[AF_LIMIT_NUM];
} AUTO_FOCUS;
//
#define VAC_ST_LOG_NUM 		50
#define VAC_ST_LOG_NUM_M1	49
#define TEMP_LOG_NUM		4
//
typedef struct VAC_ST_LOG
{
	long lTick;
	BYTE bState;
	float fPG[2];
	float fIG;
	float fTempe[TEMP_LOG_NUM]; //0:case,1:coil,2:turbo,3:power
} VAC_ST_LOG;
#endif /* DATA_STRUCT_H_ */
