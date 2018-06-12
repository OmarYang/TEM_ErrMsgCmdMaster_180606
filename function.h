/*
 * function.h
 *
 *  Created on: 2012/5/28
 *      Author: USER
 */

#ifndef FUNCTION_H_
#define FUNCTION_H_
//--------------------------------------------
// io_1.cpp
//--------------------------------------------
WORD VoltToWord(float fV, int nMax, float fVMax, float fVMin);
float WordToVolt(WORD wV, int nMax, float fVMax, float fVMin);
void SetBoardID(int nBoardNdx);
void SetCoarseScan(WORD wVX, WORD wVY);
void SetFineScan(WORD wVX, WORD wVY);
void SetFineDACXY(WORD wVX, WORD wVY);
void SetFineDACX(WORD wVX);
void SetFineDACY(WORD wVY);
//
void UpdateDACSet(void);
void UpdateADCGet(void);
void UpdateDigitalInput(void);
void UpdateDigitalOutput(void);
void SetVoltage(int nBoard, int nCh, float fV);
void GetAllVoltageNoEx(float *fpADC);
void GetAllVoltage(float *fpADC);
float GetVoltage(int nCh);
int SetADCMux(int nADC_Ch);
int SetFocus(float fPercent);
//
#if SCAN_PCB_VER >= 10
float GetVideoFactor(int nAvgNum);
#endif
//
void EnableVideoADC(int nEnable);
WORD GetVideoADC(void);
//WORD GetVideoADC(int nCh);
void GetAllVideoADC(int nForce);
int UpdatePixelNum(int nXN, int nYN);
void SetDescentCurve(int nNX, int nNY);
//
int SetGateValve(int nNdx, int nOpen);
void GetHVC(void);
//#if ENABLE_IP == 1
void SetMotorGateValve(int nOpen);
void SetIonPump(int nOn);
//#endif
void SetIonGauge(int nOn);
void SetTurboPump(int nOn);
void SetScrollPump(int nOn);
void SetRelayOn(int nOn);
void SetEDSOn(int nOn);
void SetObjOn(int nOn);
void SetObjToMin(int nEnableMin);
void SetSenrSel(int nOn);
void SetIOState(int nBoard, int nCh, int nOn); //DOUT4, DOUT5
void GetVacuumChangeRate(void);
int GetVacuumStateName(BYTE bState, char *szValue);
//return mA
float GetIonPumpCurrent(void);
float TorrToVolt(float fTorr);
float VoltToTorr(float fVolt);
float GetPiraniGauge(int nNdx);
float GetIonGauge(void);
//int GetHVStatus(char *szValue);
int GetValveStatus(int nNdx, char *szValue);
int IsChamberClose(char *szResponse); //DOOR_OPEN, DOOR_CLOSE
int IsCaseClose(char *szResponse);
void LEDTest(int nSec);
void TestSystem(void);
void TestBoard(void);
int IsInterlockSafe(int nAction);
//
void InitIO(void);
int InitAnalog(void);
int FifoHasData(void);
WORD GetFifoData(void);

float DiffVoltToCurrent(int nSigCh, float fV); //(V2-Vc)
//void SetOperationMode(BYTE bEnable);
//
void GetVoltFromPos(float fX, float fY, float *pfVX, float *pfVY);
void StartScan(void);
//void SetZoomArea(int nArea); //x1, or x10
int SetDeflectorScale(int nZoom);	//nZoom=x1, x10, x100, x1000, x10000
//int SetObjScale(int nZoom);	//nZoom=    x10, x100, x1000, x10000
void SetFineScanPos(int nX, int nY);
void SetScanOrigin(void);
int SetZoomArea(float fCenterX, float fCenterY, float fRangeX, float fRangeY, int nNX, int nNY);
int SetFixedPixel(float fX, float fY);
//
void SetStigmaV(int nCh, float fV);
void SetObjectiveCoarseV(float fV);
void SetDeflectorFineV(int nCh, float fV);
void SetObjectiveFineV(float fV);
//
//void SetCalibrationMode(BYTE bEnable);
float GetDeflectorDiffVolt(int nCh);	//0:X, 1:Y
float GetObjectiveDiffVolt(void);
float GetStigmaDiffVolt(int nCh);
//
int SetHVSwitch(BYTE bEnable);
void SetHVPower(BYTE bEnable);
void SetGVPower(BYTE bEnable);
void SetVOPPower(BYTE bEnable);
//
float GetHVBiasCurrentV(void);
void SetHVFilament(float fV);
void SetHVBias(float fV);
void SetHVAccelerator(float fV);
void SetHVFEG(float fV);
//
void SetDeflectorCurrent(int nCh, float fV);
void SetObjectiveCurrent(float fV);
void SetObjR(int nR);
//float ObjI2Ratio(float fF);
//
int ObjI2V(float fI, float *pfVolt, float *pfTheta);
float ObjFocus2Rotate(float fF);
float ObjFocus2FOVRatio(float fF);
//
int ObjIN2V(float fIN, float *pfVolt, float *pfTheta);
float ObjIN2Rotate(float fIN);
float ObjIN2FOVRatio(float fIN);
//
void SetCoilCurrent(int nCh, float fV); //STIG, AL
//
float GetCoilCurrent(int nSen_Ch);
//
float GetTemperature(int nNdx);
int CheckTemperature(int nNdx);
#if IO_PCB_VER >= 1
WORD GetDigitalInput(int nForce);
#endif
WORD GetDigitalInput(void);
WORD GetDOSetting(char *szValue, int nBoard);
void SetDigitalOutput(int nBoard, WORD wV);
void SetDigitalOutputBit(int nBoard, int nBit, int nV);
WORD Addr_RW(int nCS, WORD wAddr, int nRW, WORD wV);
int RS485_WriteString(int nPort, char *pcCmd);
int RS485_WriteString_P1(char *pcCmd); //port=1
int RS485_WriteString_P2(char *pcCmd); //port=2
//
void RS232_WriteHex(int nPort, char *szData);
void RS232_WriteString(int nPort, char *szStr);
void EnableWDT(int nEnable, float fInterval);
void ResetWDT(void);
WORD GetWDT(int nV);
//DWORD GetAddrParameter(int nType);
void SetBaseAddr(DWORD dwAddr);
void SetSingleCH0(int nEnable);
//
void AutoScanStart(void);
void AutoScanStop(void);
void AutoScanContinue(void);
void SetImageInit(void);
void VADCConvStart(void);
//
void SetRotationAngle(float fTheta);
float GetMagnificationMin(void);
void CalculateMagMin(void);
int SetMagnification(float fMag);
void SetDACPos(int nX, int nY);
int SetPixelNum(int nNX, int nNY);
int SetViewArea(float fMag, int nNX, int nNY);
int SetTestDAC(void);
int TestDeflector(void);
int TestStigmator(void);
int TestObjective(void);
void SetZoomIO(int nNdx, int nOn);
void SetCoilShunt(int nEnable);
void SelectDeflectorCoil(int nSet);
void PrintZoomCtrl(void);
float PercentToVolt(float fPercent, int nCh);
//
#if SERIAL_ADC == 1
void InitSerialADC(void);
WORD WriteSerialADC(WORD wV);
#endif
#if SERIAL_DAC == 1
void InitSerialDAC(void);
void WriteSerialDAC(int nChip, WORD wV);	//AD5328
void WriteSerialDAC32b(int nChip, DWORD dwV); //AD5649, AD5668
#endif
//
#if IO_PCB_VER >= 1
void InitSerialIO();
void WriteIOData(WORD wWrite, int nBitNum);
WORD WriteIOCmdData(WORD wCmd, WORD wSet, int nBitNum);
int WriteIODataCRC(WORD wCmd, WORD wSet, int nBitNum);
WORD WriteIOSerialADC(WORD wV);
void WriteIOSerialDAC(int nChip, WORD wV);
void WriteIOSerialDAC32b(int nChip, DWORD dwSet);
void Reset_IO(void);
void Reset_FPGA(void);
void SetIO_Out(WORD wV);
WORD GetIO_In(void);
WORD GetIO_Version(void);
void SetIO_DIO_OUT(WORD wV);
void SetIO_ADChannel(int nCh);
//void SetIO_nLDAC(int nSet);
//void SetIO_LED(WORD wV);
void SetIO_LED(int nG, int nR, int nY, int nB);
void SetIO_LED_Auto(int nAuto);
#if IO_PCB_VER >= 6
//nNdx = 0,1,2
void SetIO_VAC_Gauge(int nNdx, int nOn);
void SetIO_VAC_Gauges(int nOn);
void ResetVacPower(void);
#endif
WORD PackLEDPattern(void);
void SetLEDState(int nLED, BYTE bState);
void SetIO_LED_Pattern(WORD wV);
//int SetIO_HV_Freq(int nType, float fFreq);
#endif

void SetFreqGenerator(int nEnable, float fFreq);
void GenerateSinePattern(void);
int CheckHVTime(void);
void GetVersion(void);
//
int EDS_Start(void);
int EDS_Stop(void);
void CheckEDSHWSW(void); //check EDS HW switch
void PreampOutputCalibration(float fThreshold);
void SetPreampCalibration(void);
void EnableBlanking(int nEnable);
void BlankingProcess(int nOperation);
//
int SPI_GetRxQueNum(int nPort);
int SPI_RxHasData(int nPort);
void SPI_ReadUUID(BYTE *bUUID); //read 128-bit UUID
void SPI_WriteUART(int nPort, char *szText);
int SPI_ReadUART(int nPort, char *szData);
void SPI_GetVersion(void);
void SPI_SetBaud(int nPort, int nBaud);
int UUID_Init(char *szID);
int CheckPID(void);
#if SCAN_PCB_VER >= 20
void ResetVADC(void);
#endif
void SetVOPCOn(BYTE bOn);
//--------------------------------------------
// userparam.cpp
//--------------------------------------------
void InitSetParam(void);
void InitSysParam(void);
void ClearErrorMessage(void);
void SaveErrorMessage(char *szStr, int nPrint);
int DecipherDataPacket(int nType, BYTE *pData, int nBytes);
int DecipherMultipleCommand(int nConnType, char *szCommand);
int DecipherCommand(int fdConn, char *szCommand);
void DumpRAM(DWORD dwAddr, int nLength, int nDataType);
void SelectVideoChip(int nSelect);
void Set2ndImageType(int nType);
void SetHistoChannel(int nCh);
void EnableSimuMode(int nEnable);
void EnablePLSync(int nEnable);
void EnableIRQ3(int nEnable);
void ResetFIFO(void);
void EnableFIFO(int nEnable);
void SampleOncePerXLine(int nMode);
//
void SetPLParameter(int nSet);
void EnableAutoFocus(int nEnable);
void Set1USClkNum(void);
void ShowVacuumStatus(int nType);
//--------------------------------------------
// io2.cpp
//--------------------------------------------
void EnableCL(int nEnable);
void SetCCD_SH(int nEnable);
void SetCCD_Gain(int nEnable);
void SetCCD_ROG(int nEnable);
void SetCCD_CLK(int nEnable);
//--------------------------------------------
// main.cpp
//--------------------------------------------
void ClearSerialBuffer(void);
void ClearSerial1Buffer(void);
BOOL IsNextVacuumStateOK(int nV);
BOOL IsVacuumReady(void);
void RedundantProcess(void);
BOOL VentingProcess(void);
//
void ScanOverProcess(char *szMsg);
void TransmitData(int nConnType, int nDataByteNum, int nDataNdx, int nNdx);
//
void TxNVParam(int nType);
void TxSetParam(int nType);
void TxSysParam3(int nType);
void TxSysParam2(int nType);
void TxData(int nConnType, BYTE *pData, int nLen);
void TxString(int nConnType, char *pStr);
void ShowTitle(int nConnType);
void TxSpeedTest(void*);
void SetupPITR( int nPITR_Ch, float fFrequency);
void StopPITR(void);
void InitFastScan(void);
int FastScan(int nConnType);
//void SlowScan(int nConnType);
void TxIdle(int nWait);
void SetupIRQ3(int nEnable);
//--------------------------------------------
// utility.cpp
//--------------------------------------------
void delay_void(void);
void delay_us(int nus);
DWORD delay_100us(int nTimes);
void ReplaceChar(char *szCommand, char cOld, char cNew);
void WordToBinaryString(WORD wV, char *szBuffer);
void GetSystemTime(void);
void SetSystemTime(int nYear, int nMon, int nDay, int nHour, int nMin, int nSec);
WORD AutoFocusSimuData(int nX, int nY);
WORD GenerateCircleGridPattern(int nX, int nY);
WORD GetGridPattern(int nShift, int nRandom, int nX, int nY);
void ResetBRCOToDefault(void);
void ClearNVSettings(void);
void GetScanRangeMax(float fRange, int nDEFX);
BYTE CheckTempSetLimit(int nNdx, float fSet);
void CheckNVSettings(void);
void ClearHistogram(void);
void SetHistogram(WORD wV);
int GetHistogramPacket(BYTE *pData);
int GetHistogramPacketFromFPGA(BYTE *pData);
void ClearGradientX(void);
void SetGradientX(WORD *pwVal, int nPixelNum);
int VoltToTemperature(float fV, float *pfTemp);
float ByteSwapFloat(float fVIn);
DWORD ByteSwapDWord(DWORD dwVIn);
WORD ByteSwapWord(WORD wVIn);
//
#define CMD_IN_QUEUE_NUM	12
#define CMD_SIZE			128
void ClearCommandInQueue(void);
int GetCommandNumInQueue(void);
void PushCommandToQueue(char *szCommand, int nConnType);
int PopCommandFromQueue(char *szCommand, int nNdx);
//
void SetPLPeriod(void);
void GetXScanTime(int nX, int nAvgNum);
DWORD GetGradient(void);
int OpenSerialPort(int nPort);
void ParseHVCMessage(char *szCommand);
void ParseIPCMessage(char *szCommand);
void EnableEDS(BYTE bEnable);
void AppendLFCR(char *szText);
void RemoveLFCR(char *szText);
float CalculateObjStandbyV(void);
//--------------------------------------------
// Turbo_Control_Protocol.cpp
// turbo pump control
//--------------------------------------------
void StartTurbo (void);
void StopTurbo (void);
void EnableTurboVent(int nEnable);
int GetTurboRotationSpeed (void);
int GetTurboMotorTemp (void);
//--------------------------------------------
// one_wire.cpp
//--------------------------------------------
int ds2401_init(char *szResult);
// vacuum.cpp
int IsTurboSpeedHighEnough(void);
int IsTurboStartBrake(void);
int IsTurboSpeedLowEnough(void);
int GetGaugeStatus(int nNdx, char *szValue); //get gate valve status
void VacuumStateTask(void * pd);
void PrintVacStLog(void);
void PrintVacTimeout(void);
void ResetVACCounter(void);
void IncVACCounter(void);
//
#endif /* FUNCTION_H_ */
