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
extern float fFocus;
extern float fRatioFocus;
extern NV_SETTINGS NV_Settings;
extern SYS_PARAM_3 SysParam;
extern BYTE bDebug;
//
extern volatile PWORD pwFpga;
extern int nSenRSet;
extern int nShuntSet;
extern int nSelSmallDef;
extern int nObjTableNdx;
extern float fDACVMax[BOARD_NUM][DAC_CH_NUM_MAX];
extern float fDACVMin[BOARD_NUM][DAC_CH_NUM_MAX];
extern OS_CRIT OsCrit;
//
extern BYTE bPIDMajorVersion;
extern BYTE bPIDMinorVersion;
extern BYTE bErrorHWVersion;
//
//NB read byte from PID MCU
BYTE SPI_ReadByte(void)
{
	BYTE bMask = 0x01;
	BYTE bDIN = 0x0;
	int i;
	//inv(0x8000),din(0x0004),sck(0x0002),n_cs(0x0001)
	for (i = 0; i < 8; i++) {
		pwFpga[SET_DIO] = 0x0002; //SCK=1, DIN=0
		delay_us(5);
		if (DOUT_PID) bDIN |= bMask;
		pwFpga[SET_DIO] = 0x0000; //SCK=0, DIN=0
		delay_us(4);
		bMask = bMask << 1;
	}
	delay_us(10);
	return bDIN;
}

//NB write byte to MCU
//read LSB first
void SPI_WriteByte(BYTE bDOUT)
{
	BYTE bMask = 0x01;
	BYTE bSet = 0;
	int i;
	//
	for (i = 0; i < 8; i++) {
		bSet = (bDOUT & bMask) ? 0x0004 : 0x0000;
		pwFpga[SET_DIO] = bSet; //SCK=0, DIN=SET
		bSet |= 0x0002;
		delay_us(1);
		pwFpga[SET_DIO] = bSet; //SCK=1, DIN=SET
		delay_us(5);
		bSet &= (~0x0002);
		pwFpga[SET_DIO] = bSet; //SCK=0, DIN=SET
		bMask = bMask << 1;
		delay_us(4);
	}
	delay_us(6);
	return;
}

BYTE bSPIRxNum[2] = {0, 0};
int SPI_GetRxQueNum(int nPort)
{
	int i;
	//
	CS_PID = 1;
	for (i = 0; i < 100; i++) //wait DOUT_PID
		if (DOUT_PID) break;
	delay_us(10);
	SPI_WriteByte(0x02);
	bSPIRxNum[0] = SPI_ReadByte();
	bSPIRxNum[1] = SPI_ReadByte();
	CS_PID = 0;
	//
	return (int)bSPIRxNum[nPort];
}

int SPI_RxHasData(int nPort)
{
	if (PID_RX_DATA == 0)
		return 0;
	return SPI_GetRxQueNum(nPort);
}

#define WR_SPI_WAIT		20
void SPI_ReadUUID(BYTE *bUUID)
{
	int i;
	//
	OSCritEnter( &OsCrit, 1 );
	CS_PID = 1;
	for (i = 0; i < 100; i++) //wait DOUT_PID
		if (DOUT_PID) break;
	delay_us(WR_SPI_WAIT);
	SPI_WriteByte(0x01);
	for (i = 0; i < UUID_NUM; i++) {
		bUUID[i] = SPI_ReadByte();
	}
	CS_PID = 0;
	OSCritLeave( &OsCrit );
}
//
//nPort = 0, 1
void SPI_WriteUART(int nPort, char *szData)
{
	int nLen;
	int i;
	//
	if ((nPort > 1) || (nPort < 0)) return;
	nLen = strlen(szData);
	CS_PID = 1;
	for (i = 0; i < 100; i++) //wait DOUT_PID
		if (DOUT_PID) break;
	delay_us(WR_SPI_WAIT);
	SPI_WriteByte(0x80 + nPort); //0x80, 0x81
	SPI_WriteByte((BYTE)nLen); //byte number
	for (i = 0; i < nLen; i++) {
		SPI_WriteByte(szData[i]);
	}
	CS_PID = 0;
}

//nPort = 0, 1
int SPI_ReadUART(int nPort, char *szData)
{
#if ENABLE_PID_UART == 0
	return 0;
#else
	int nLen;
	int i;
	//
	if ((nPort > 1) || (nPort < 0))
		return 0;
	/*CS_PID = 1;
	for (i = 0; i < 100; i++) //wait DOUT_PID
		if (DOUT_PID) break;
	delay_us(WR_SPI_WAIT);
	SPI_WriteByte(0x02);
	bSPIRxNum[0] = SPI_ReadByte();
	bSPIRxNum[1] = SPI_ReadByte();
	CS_PID = 0; */
	//
	nLen = (int)bSPIRxNum[nPort];
	if (nLen <= 0)
		return 0;
	if (nLen >= PID_UART_SIZE)
		nLen = PID_UART_SIZE - 1;
	delay_us(100);
	//
	CS_PID = 1;
	for (i = 0; i < 100; i++) //wait DOUT_PID
		if (DOUT_PID) break;
	delay_us(WR_SPI_WAIT);
	SPI_WriteByte(0x03 + nPort);
	for (i = 0; i < nLen; i++) {
		szData[i] = SPI_ReadByte();
	}
	szData[i] = 0;
	CS_PID = 0;
	return nLen;
#endif
}

void SPI_GetVersion(void)
{
	int i;
	//
	OSCritEnter( &OsCrit, 1 );
	CS_PID = 1;
	for (i = 0; i < 100; i++) { //wait until DOUT_PID=1
		if (DOUT_PID) break;
	}
	delay_us(WR_SPI_WAIT); //wait longer is necessary
	SPI_WriteByte(0x05);
	bPIDMajorVersion = SPI_ReadByte();
	bPIDMinorVersion = SPI_ReadByte();
	CS_PID = 0;
	OSCritLeave( &OsCrit );
	//
	return;
}

void SPI_SetBaud(int nPort, int nBaud)
{
	int i;
	nBaud = nBaud / 4800;
	if ((nPort > 1) || (nPort < 0)) return;
	//
	CS_PID = 1;
	for (i = 0; i < 100; i++) //wait DOUT_PID
		if (DOUT_PID) break;
	delay_us(WR_SPI_WAIT);
	SPI_WriteByte(0x82);
	SPI_WriteByte((BYTE)nPort);
	SPI_WriteByte((BYTE)nBaud);
	CS_PID = 0;
}

BYTE bUUID_PID[UUID_NUM];
int UUID_Init(char *szID)
{
	char szValue[16];
	int i;
	//
	SPI_ReadUUID(bUUID_PID);
	szID[0] = 0;
	for (i = 0 ; i < UUID_NUM; i++) {
		sprintf(szValue, "%02X-", bUUID_PID[i]);
		strcat(szID, szValue);
	}
	strcat(szID, "\r\n");
	//printf("%s", szBuffer);
	//
	if (CheckPID() == 0) { //OK
		//bErrorHWVersion &= (~ERR_HW_PCBID);
		return 2;
	}
	else {
		//bErrorHWVersion |= ERR_HW_PCBID;
		return -1;
	}
}

int CheckPID(void)
{
	if (memcmp(NV_Settings.bUUID, bUUID_PID, UUID_NUM) == 0) { //the process takes 8 msec
		SysParam.bErrorHWVersion &= (~ERR_HW_PCBID);
		return 0;
	}
	else {
		SysParam.bErrorHWVersion |= ERR_HW_PCBID;
		return -1;
	}
}

void EnableBlanking(int nEnable)
{
	if (nEnable == 2) {
		SetSenrSel(1); 		//set sense resistor relay, 0:large ohm, 1: small resistor (large current)
		SetCoilShunt(0);	//set shunt relay, 0:no shunt(large current), 1: shunt
		SelectDeflectorCoil(0);	//use whole coil
		pwFpga[SET_BLANKING] = 1;
	}
	else if (nEnable == 1) {
		SetSenrSel(1); 		//set sense resistor relay,0:large ohm,1:small resistor (large current)
		SetCoilShunt(0);	//set shunt relay, 0:no shunt(large current), 1: shunt
		SelectDeflectorCoil(0);	//use whole coil
		//pwFpga[SET_BLANKING] = 1;
	}
	else {
		SetSenrSel(nSenRSet); 		//reset sense r relay, 0:larger ohm, 1: smaller resistor
		SetCoilShunt(nShuntSet);	//reset shunt relay
		SelectDeflectorCoil(nSelSmallDef); //1:smaller deflector, 0:whole deflector
		pwFpga[SET_BLANKING] = 0;
	}
}
#define BLANKING_STEP_NUM	32
void BlankingProcess(int nOperation)
{
	static float fRadian = 0;
	static float fVX[BLANKING_STEP_NUM];
	static float fVY[BLANKING_STEP_NUM];
	static int nStepNdx = 0;
	float fFactor = 0.92;
	//0.96x1.4142=1.35 ampere
	//0.92x1.4142=1.30 ampere
	int i;
	//
	if (nOperation == 0) { //calculate sine/cosine table
		for (i = 0; i < BLANKING_STEP_NUM; i++) {
			fVX[i] = fFactor * DACF_PV * sin(fRadian);
			fVY[i] = fFactor * DACF_PV * cos(fRadian);
			fRadian += (6.283184/BLANKING_STEP_NUM);
		}
		return;
	}
	else if (nOperation == 1) {
		SetDeflectorFineV(DAC_CH_DEFX_F, fVX[nStepNdx]);
		SetDeflectorFineV(DAC_CH_DEFY_F, fVY[nStepNdx]);
		nStepNdx = (nStepNdx + 1) % BLANKING_STEP_NUM;
		return;
	}
	else if (nOperation == 3) {
		SetVoltage(BOARD_SCAN, DAC_CH_DEFX_F, fFactor * DACF_PV / 1.4142);
		SetVoltage(BOARD_SCAN, DAC_CH_DEFY_F, fFactor * DACF_PV / 1.4142);
	}
}

//
float fFocusVC = 3.0;
float fFocusVF = 0.0;

extern float fAcckV;
//
//fPercent = 0 ~ 100
int SetFocus(float fPercent)
{
	float fI, fIN, fVUnit;
	//float fVC, fVF;
	float fV1;
	float fDivide = DAC_12BIT;
	int nRet;
	static int nReentry = 0;
	//
	if (nReentry == 1)
		return SUCCESS;
	nReentry = 1;
	fFocus = fPercent;
	fI = NV_Settings.fObjIMin[nObjTableNdx] + (NV_Settings.fObjIMax[nObjTableNdx] - NV_Settings.fObjIMin[nObjTableNdx]) * fFocus / 100;
	if (NV_Settings.bAcckVAdjust == 1) {
		fI = fI * sqrt(fAcckV / ACC_KV_BASE);
	}
	fIN = fI * (float)NV_Settings.nObjTurn;
	//if (NV_Settings.bUseIN == 1)
	//	nRet = ObjIN2V(fIN, &fV1, &SysParam.fObjAdjAngle);
	//else
		nRet = ObjI2V(fI, &fV1, &SysParam.fObjAdjAngle); //voltage, rotation angle(degree)
	if (nRet != SUCCESS) {
		goto SetFocusEnd;
	}
	if (NV_Settings.bUseIN == 1)
		SysParam.fObjAdjAngle = ObjIN2Rotate(fIN);
	else
		SysParam.fObjAdjAngle = ObjFocus2Rotate(fFocus);
	//
	if (NV_Settings.bDACType[1] == DAC_AD5328)
		fDivide = DAC_12BIT;
	else if (NV_Settings.bDACType[1] == DAC_AD5648)
		fDivide = DAC_14BIT;
	else if (NV_Settings.bDACType[1] == DAC_AD5668)
		fDivide = DAC_16BIT;
	fVUnit = (fDACVMax[BOARD_IO][DAC_CH_OBJ_C] - fDACVMin[BOARD_IO][DAC_CH_OBJ_C]) / fDivide;
	//10/4096=0.00244 volt
	//0.00244x100=0.244 volt
	//10/65536=0.0001526 volt
	fFocusVC = (float)((int)(fV1/fVUnit)) * fVUnit;
	fFocusVF = (fV1 - fFocusVC) * 100.0; //1K/100K ratio
	SetVoltage(BOARD_IO, DAC_CH_OBJ_C, fFocusVC);
	SetVoltage(BOARD_IO, DAC_CH_OBJ_F, fFocusVF);
	//
	if (SysParam.bScanning == OP_AUTO_FOCUS)
		fRatioFocus = 1.0;
	else if (NV_Settings.bFocusFOV == 1) {
		if (NV_Settings.bUseIN == 1)
			fRatioFocus = ObjIN2FOVRatio(fIN);
		else
			fRatioFocus = ObjFocus2FOVRatio(fFocus); //ObjI2Ratio(fI);
	}
	else
		fRatioFocus = 1.0;
	SetRotationAngle(SysParam.fRotateAngle);
SetFocusEnd:
	if (bDebug == 17) {
		printf("FOCUS=%.2f,ROT=%.3f,OBJROT=%.3f,OBJFOV=%.3f\n", fFocus,
				SysParam.fRotateAngle, SysParam.fObjAdjAngle, fRatioFocus);
		printf("OBJI SET=%.3f mA,OBJ_IN=%.2f,V=%.3f,RET=%d\n", fI * 1000.0, fIN, fV1, nRet);
		fI = GetCoilCurrent(4);
		printf("OBJI READ=%.3f mA\n", fI * 1000.0);
	}
	nReentry = 0;
	return SUCCESS;
}

void SetSCK_IO(void)
{
	//SCK_1;
	//delay_us(1);
	//SCK_0;
#if USE_FPGA_DIO == 1
	WORD wSet;
	wSet = (NV_Settings.bIOINV == 1) ? 0x8000: 0x0000;
	wSet |= 0x0002;			//SCK_1;
	pwFpga[SET_DIO] = wSet;
#if IO_74HC14 == 1
	asm("nop");
	asm("nop");
#else
	asm("nop");
	asm("nop");
	asm("nop");
#endif
	wSet &= (~0x0002);		//SCK_0;
	pwFpga[SET_DIO] = wSet;
#else
	SCK_IO = (NV_Settings.bIOINV == 1) ? 0 : 1;	//SCK_1;
	delay_us(1);
	SCK_IO = (NV_Settings.bIOINV == 1) ? 1 : 0;	//SCK_0;
#endif
}
//write MSB first
void WriteIOData(WORD wWrite, int nBitNum)
{
	WORD wMaskW = 0x0001;
	BYTE bINV;
	wMaskW = wMaskW << (nBitNum - 1);
	//clock speed is about 100 kHz = 10 us (too slow!)
	WORD wSet;
	bINV = NV_Settings.bIOINV;
	wSet = (bINV == 1) ? 0x8000: 0x0000;
	while (nBitNum > 0) {
		if (wWrite & wMaskW)
			wSet |= 0x0004; //DIN_1; n_cs(0),sck(1),din(2)
		else
			wSet &= (~0x0004);
		pwFpga[SET_DIO] = wSet; //set din_io
		asm("nop");
		asm("nop");
		asm("nop");
		wSet |= 0x0002;
		pwFpga[SET_DIO] = wSet; //SCK_1; //180 ns
		nBitNum--;
		asm("nop");
		asm("nop");
		asm("nop");
		wMaskW = wMaskW >> 1; //write MSB bit first, 3 usec
		wSet &= (~0x0002);
		pwFpga[SET_DIO] = wSet; //SCK_0;
	}
	return;
}

void WriteIOData32b(DWORD dwWrite, int nBitNum)
{
	DWORD dwMaskW = 0x0001;
	BYTE bINV;
	dwMaskW = dwMaskW << (nBitNum - 1);
	//clock speed is about 100 kHz = 10 us (too slow!)
	WORD wSet;
	bINV = NV_Settings.bIOINV;
	wSet = (bINV == 1) ? 0x8000: 0x0000;
	while (nBitNum > 0) {
		if (dwWrite & dwMaskW)
			wSet |= 0x0004; //DIN_1; n_cs(0),sck(1),din(2)
		else
			wSet &= (~0x0004);
		pwFpga[SET_DIO] = wSet; //set din_io
		asm("nop");
		asm("nop");
		wSet |= 0x0002;
		pwFpga[SET_DIO] = wSet; //SCK_1; //180 ns
		nBitNum--;
		asm("nop");
		asm("nop");
		dwMaskW = dwMaskW >> 1; //write MSB bit first, 3 usec
		wSet &= (~0x0002);
		pwFpga[SET_DIO] = wSet; //SCK_0;
	}
	return;
}

WORD ReadWriteIOData(WORD wWrite, int nBitNum)
{
	WORD wMaskW = 0x0001;
	WORD wMaskR = 0x8000;
	WORD wRead = 0;
	int i;
	BYTE bINV;
	wMaskW = wMaskW << (nBitNum - 1);
	WORD wSet = 0;
	bINV = NV_Settings.bIOINV; //always 1
	wSet = (bINV) ? 0x8000: 0x0000;
	for (i = 0; i < nBitNum; i++) {
		if (wWrite & wMaskW)
			wSet |= 0x0004; //DIN=1; n_cs(0),SCK(1,x0002),DIN(2,0x0004)
		else
			wSet &= (~0x0004); //DIN=0
		pwFpga[SET_DIO] = wSet;	//set din_io
		asm("nop");
		asm("nop");
		asm("nop");
		wSet |= 0x0002;
		pwFpga[SET_DIO] = wSet;	//SCK=1;
		if (pwFpga[GET_STATUS] & ST_DOUT_IO) //0x0008, read DOUT_IO
			wRead = wRead | wMaskR;
		wMaskR = wMaskR >> 1; 	//read MSB first
		wMaskW = wMaskW >> 1; //write MSB bit first, 3 usec
		wSet &= (~0x0002);
		pwFpga[SET_DIO] = wSet; //SCK_0;
	}
	return wRead;
}
// wCmd: command
// wSet: setting value
WORD WriteIOCmdData(WORD wCmd, WORD wSet, int nBitNum)
{
	WORD wV = 0;
	BYTE bINV;
	bINV = NV_Settings.bIOINV; //CS, SCK, DIN
	OSCritEnter( &OsCrit, 1 );
	N_CS_IO = (bINV == 1) ? 1 : 0;		//CS_ENABLE;
	//delay_us(1);
	WriteIOData(wCmd, IO_CMD_BITN);
	wV = ReadWriteIOData(wSet, nBitNum);
	N_CS_IO = (bINV == 1) ? 0 : 1;		//CS_DISABLE;
	//
	SetSCK_IO(); //one more clock after N_CS_IO=1 is needed
	OSCritLeave( &OsCrit );
	if (nBitNum < 16) //shift LSB to bit 0
		wV = wV >> (16 - nBitNum);
	return wV;
}

int GetBitNum(WORD wV)
{
	WORD wMask = 0x0001;
	int nBit1Num = 0;
	while (wMask != 0) {
		if (wV & wMask)
			nBit1Num++;
		wMask = wMask << 1;
	}
	return nBit1Num;
}
//---------------------------------------------------------
//return 0: CRC OK
//return -1: CRC error
//nBitNum must be 16 !!!
//---------------------------------------------------------
int WriteIODataCRC(WORD wCmd, WORD wSet, int nBitNum)
{
	BYTE bINV;
	WORD wCRC;
	int nRet = -1;
	//
	bINV = NV_Settings.bIOINV; //CS, SCK, DIN
	wCRC = (WORD)GetBitNum(wCmd);
	wCRC += (WORD)GetBitNum(wSet);
	wCRC &= 0x0F; //select three bits
	//
	OSCritEnter( &OsCrit, 1 );
	N_CS_IO = (bINV == 1) ? 1 : 0;		//CS_ENABLE;
	//delay_us(1);
	WriteIOData(wCmd, IO_CMD_BITN); //write msb first
	WriteIOData(wSet, nBitNum);
	WriteIOData(wCRC, 4); 				//4-bit CRC
	N_CS_IO = (bINV == 1) ? 0 : 1;		//CS_DISABLE;
	//
	if (pwFpga[GET_STATUS] & ST_DOUT_IO) //0x0008, read DOUT_IO
		nRet = 0; //SUCCESS
	SetSCK_IO(); //one more sck is needed, change DOUT_IO to HiZ
	OSCritLeave( &OsCrit );
	return nRet;
}
//-------------------------------------
//IO board, write 16-bit data
//-------------------------------------
void WriteIOSerialDAC(int nChip, WORD wSet)
{
	WORD wHeader;
	wHeader = (nChip == 0) ? IO_SET_DAC_0 : IO_SET_DAC_1;
#if DEBUG_SIO == 1
	return;
#endif
	BYTE bINV;
	bINV = NV_Settings.bIOINV;
	OSCritEnter( &OsCrit, 1 );
	N_CS_IO = (bINV == 1) ? 1 : 0;		//CS_ENABLE;
	WriteIOData(wHeader, IO_CMD_BITN); //write MSB first
	WriteIOData(wSet, 16); //write MSB first
	N_CS_IO = (bINV == 1) ? 0 : 1;		//CS_DISABLE;
	SetSCK_IO();
	OSCritLeave( &OsCrit );
	return;
}
//--------------------------------------------------
//IO board, write 32-bit data to AD5648 or AD5668
//--------------------------------------------------
void WriteIOSerialDAC32b(int nChip, DWORD dwSet)
{
	WORD wHeader;
	wHeader = (nChip == 0) ? IO_SET_DAC_0 : IO_SET_DAC_1;
#if DEBUG_SIO == 1
	return;
#endif
	BYTE bINV;
	bINV = NV_Settings.bIOINV; //always 1
	OSCritEnter( &OsCrit, 1 );
	N_CS_IO = (bINV == 1) ? 1 : 0;		//CS_ENABLE;
	WriteIOData(wHeader, IO_CMD_BITN); //write MSB first
	WriteIOData32b(dwSet, 32); //write MSB first
	N_CS_IO = (bINV == 1) ? 0 : 1;		//CS_DISABLE;
	SetSCK_IO();
	OSCritLeave( &OsCrit );
	return;
}
//
WORD WriteIOSerialADC(WORD wSet)
{
	WORD wDIN = 0;
	//
#if DEBUG_SIO == 1
	return wDIN;
#endif
	BYTE bINV;
	bINV = NV_Settings.bIOINV;
	OSCritEnter( &OsCrit, 1 );
	N_CS_IO = (bINV == 1) ? 1 : 0;		//CS_ENABLE;
	WriteIOData(IO_GET_ADC, IO_CMD_BITN);
	wDIN = ReadWriteIOData(wSet, 16);
	N_CS_IO = (bINV == 1) ? 0 : 1;		//CS_DISABLE;
	SetSCK_IO();
	OSCritLeave( &OsCrit );
	return wDIN;
}

WORD wParameterCL = 0;
void EnableCL(int nEnable)
{
	if (nEnable)
		wParameterCL |= 0x8000;
	else
		wParameterCL &= (~0x8000);
	pwFpga[ENABLE_CL] = wParameterCL;
}

void SetCCD_SH(int nEnable)
{
	if (nEnable)
		wParameterCL |= 0x0001;
	else
		wParameterCL &= (~0x0001);
	pwFpga[ENABLE_CL] = wParameterCL;
}

void SetCCD_Gain(int nEnable)
{
	if (nEnable)
		wParameterCL |= 0x0002;
	else
		wParameterCL &= (~0x0002);
	pwFpga[ENABLE_CL] = wParameterCL;
}

void SetCCD_ROG(int nEnable)
{
	if (nEnable)
		wParameterCL |= 0x0004;
	else
		wParameterCL &= (~0x0004);
	pwFpga[ENABLE_CL] = wParameterCL;
}

void SetCCD_CLK(int nEnable)
{
	if (nEnable)
		wParameterCL |= 0x0008;
	else
		wParameterCL &= (~0x0008);
	pwFpga[ENABLE_CL] = wParameterCL;
}

void InitCCD(int nEnable)
{
	EnableCL(nEnable);
	SetCCD_Gain(0);
	SetCCD_CLK(1);
}

void SetCCD_XY(WORD wX, WORD wY)
{
	int i;
	SetFineDACX(wX);
	SetFineDACY(wX);
	for (i = 0; i < 3038; i++) {
		SetCCD_CLK(0);
		SetCCD_CLK(1);
		//GetVideoADC(); //CDS 1
		SetCCD_CLK(0);
		SetCCD_CLK(1);
		//GetVideoADC(); //CDS 2
		if (i < 32) continue;
		if (i >= 3032) continue;
		//i=32 to 3031, S1 ~ S3000
		//save spectrum to array
	}
}
