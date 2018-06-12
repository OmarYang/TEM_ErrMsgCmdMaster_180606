#ifndef MAIN_H_
#define MAIN_H_
//
//2015.2.17
//add HOBOCHEN:1 command
//2015.5.27
//use PID MCU to get UUID
// add WAITEOC command
//2015.7.7
// add IP_PWR_CTRL test command IPON and MOTOGV
// add custom HV test command GETHVC
//2015.8.20
// modify "SO:" command to output to UR0, UR1, UR2
// add bPZTMoveX, bPZTMoveY into SYS_PARAM
// fix bug for tubo pump speed init in StartTurbo()
// add bObjMode in NV, OBJMODE command, for power saving purpose
// set deflector initial value at the last DelayLB 50 msec
//2015.9.3
// SCAN_VA20, add reset video ADC (AD7655) command RESET:2
// add SET_Y_START, SET_Y_END, SET_FAST_SCAN command, REGION command
//2015.10.6
// add SET_VOP_ON, GET_DO command
//2015.10.20
// enhance VACOK command for EM200
// add XINIT command, USE_CPU_IO, RELAY_ON, VOP_ON
// movd GatAllVideoADC() to ScanMonitorTask to avoid db3 conflict
//2015.12.4
// PID UART output
// PID_RX_DATA detection, read PID version
//2015.12.9
// GV3 turn off 3 times at STANDBY
// turn on IG from ERROR2 to STANDBY
// GV2 close at UH,
// UH, VACON:0,ENEVENT:1, wait 30 sec, then close scroll pump
//2015.12.19
// add InitIPLeak() and CheckIPLeak() processes.
// add STANDBY0, STANDBY1 and STANDBY_WAIT state
// add OVERSCAN command (2015.12.26)
//2016.1.15
// STANDBY to AIR fail, elongate timeout for GV0 open/close
// add codes for serial2 IP_PWR and HV communication
// ParseHVCMessage(), ParseIPMessage() (IPINFO)
// add VentToAir();
//2016.2.2
// larger temp hi limit
// add bTxData byte into SYS_PARAM
// timeout for getting turbo speed change from 200 ms to 250 ms
// execute GETSYS command when scanning (bDB3Busy == 1)
//2016.2.3
// add nTurboLowSpeed[], TURBOSP:3~5:SP setting, TURBOSP:?
// EM100 no VentToAir()
//2016.2.4
// EM200 modification
// UH->standby0, no vent
// UH->standby1, vent delayed
// standby0->standby1, if VAC_AIR vent_direct else vent_delay
//2016.2.13
// add SysParam.fBiasV
// add SETPIXN:<NX>:<NY> command
// IMAGE_INIT send image_start output
//2016.2.26
// add FILACNST command
//2016.3.9
// IMGNUM:<NV>, 1:BEI, 2:SEI, 3:BEI+SEI
// DATA_BEGIN_HEADER.ADCCh,DATA_END_HEADER.ADCCh,DATA_HEADER.ADCCh
// bADCCh=0~3(BEI),bADCCh=10(SEI),
// HISTOCH:<NV>, NV=0(BEI) or 1(SEI)
// VAC1 broken keep at ERROR2 if UH or MH
//2016.3.28
// check turbo speed more than once for lower or slow down
//2016.4.6
// SETBRX, SETCOX command
// SYS_PARAM add bCPU_IO[4] fields
//3.5.3 add SYS_PARAM.bErrorHWVersion
//3.5.4 use nSerial1Ndx, clear serial buf 1 will reset index
//3.5.5 2016/4/19
// (1) EM200 TP gate valve改用030 control, 改很多， ERROR1->AIR 和STANDBY0 --> STANDBY1流程需多測試。
// (2) add "SO:2:1:0011001206111111017" to write command to TP directly
// (3) SYS_PARAM add short int sTPGVTimeout, sIGTimeout, turbo pump gate valve open count down
// add NV.sTPGVTimeout
//3.5.6, 2016/4/28, display 3.5.5 (error)
// 改善開機LED不亮的問題，延後下SetIO_LED_Auto(1) command
//3.5.7 2016/4/29
// EM100 nVentTime=3 from AIR_WAIT to AIR
//3.5.8 2016/4/29
// EM100 use VAC_ST_AIR, LO, HI (EMV22_Controller), discard VAC_OK, VAC_NOT_OK
// WriteIODataCRC() send clock after N_CS=1
// IO board FPGA 4.4, make DOUT_IO HiZ when inactive
//3.5.9 2016/5/20
// fix bug for vacuum LO_WAIT_TIMEOUT timeout
//3.5.10 2016/6/2
// SETR:0:fRatioBR
// SETR:1:fRatioCDO
// SETR:2:fBaseBR
// SETR:3:fBaseCO
// SETCO:fV  CO = fBaseCO + (0~100%)/fRatioCO
// SETBR:fV  BR = fBaseBR + (0~100%)/fRatioBR
//3.5.11 2016/6/10
// !!! FW燒錄後，一定要執行 SETR:? 檢查參數
// SETR:0~3:fBaseBR, default 50
// SETR:4~7:fRangeBR, default 25
// SETR:8:fBaseCO, default 0
// SETR:9:fRangeCO, default 100
// SETCOX:[i]:fV  CO = fBaseCO + fV*fRangeCO/100.0, i=0~3
// SETBRX:[i]:fV  BR = fBaseBR + fV*fRangeBR/100.0, i=0~3
// MAG_MAX = 80000，高倍率時必須切換為小線圈
// SysParam.bVAC_OK[] is not used anymore
// #define VAC_OK and VAC_NOT_OK are not used anymore
// debug bHVON flag 不正確的問題, HVON command未正確執行。
//3.5.12 2016/7/1
// set gate valve 1 (GV1, EM100; GV0, EM200) close then start scroll pump
// EM200 STANDBY0 -> STANDBY1 wait, nVentTPGV algorithm changes
// remove RELAY_ON control for SetDigitalOutputBit() and SetDigitalOutput()
//----------------------------------------------------------------------
//3.5.13 2016/7/5
// ENABLE_PID_UART will cause system crash(?), set 0
// add RESEND command, RESEND:<NY>
// RESET FPGA for SCAN_VA22, J2[26]
//3.5.14, 3.5.15 2016/7/14
// push SETPIXN code to queue during scanning
// add SHADOW_2CH_P, SHADOW_2CH_M
// add NV_Settings.bBSEMode parameter
//3.5.16 2016/8/9
// show turbo speed in TURBOSP:? command
// change default STANDBY_WAIT_TIMEOUT = 220, TPGV_TIMEOUT = 200
// swap NV_Settings parameters, remove fPLAmp, fPLPhase in NV
//3.5.17 2016/8/29
// set fARatio[1] effective, APERMM for Y axis
// DAC RESV (CH16-CH19) range from 0 ~ 5V for PMT control
// remove sel_coil_small control on IDC10 pin
//3.5.18 2016/9/20
// no vacuum process until bInitOK = 1
// execute SET_X_NUM, SET_Y_NUM in SetPixelNum()
// SETPIXN:? to read pixel number in FPGA
// has_num, GET_X_NUM, GET_Y_NUM check
// change MAG_MAX to 100,000
//3.5.19 2016/10/18
// add SHADOW_2CH_M1, SHADOW_2CH_M2
//3.5.20 2016/10/30
// add SHADOW_2CH_P2, SHADOW_2CH_M3, SHADOW_2CH_M4, extend IMGNUM command
//3.5.21 2016/11/14
// bIPLeakAllDone=1, go to INT_HI only after CheckIPLeak() process is complete
// vac gauge power control (IO_PCB_VER >= 6)
// RESET:4, PWRON:7:1/0 to control vac gauge power
// PIXEL_NUM_X_MAX=3600, PIXEL_NUM_Y_MAX=2700
//3.5.22 2016/11/16
// if GAUGE_POWER_CTRL==1, WORD wDIO_OUT = 0x01C0;
//3.5.23 2016/11/18
// INT_HI_WAIT VACON=1的過程，若turbo降速(<1485)四秒以上或是IG變為AIR會跳到ERROR1
// 可設定IP leak的次數和on/off time
// 改正simulation 過程GV顯示不正確的問題 (TBD)
// 需執行IPLEAK和VACPARA:16~19把參數設對，用IPLEAK:?檢查參數。
//3.5.24 2016/11/21
// IP leak 要完成才能abort INT_HI_WAIT
//3.5.25 2016/11/21
// ADD bIPLeakAllDone=1, nErrorCount=0
//3.5.26 2016/11/21
// GETSET:8 return NV_SETTINGS structure
// SysParam.sVacTimeout unit change to min for EM200
// SysParam.sIGTimeout unit change to min
//3.5.27 2016/11/28
// VAC_ST_INIT goes to VAC_ST_AIR or VAC_ST_STANDBY
// BSEMODE=2, SHAODW:11(s0+s1); BSEMODE=1, SHADOW:1(s0+s1)
//3.5.28 2016/11/30
// VAC_ST_VENT_WAIT, VAC_ST_STANDBY_WAIT --> GV0 close
//3.6.1 2016/12/5
// add error message for TP speed and temperature communication
// TP SP timeout change from 5 to 6
//3.6.2 2016/12/6
// add IsTurboStartBrake()
// nTPGVTimeout start count down當turbo speed低於brake speed (TURBOSP:6:<NV>)
// nTPGVTimeout =10時檢查VAC，若為AIR or LOW，變成VAC_ST_STANDBY1
// nTPGVTimeout =0時, ERROE2, "STANDBY_WAIT_TIMEOUT:TPGV"
// TURBOSP:6 set turbo brake speed, TURBOSP:? 可看參數
//3.6.3 2016/12/7
// STANDBY_WAIT 下 VentToAir(VENT_DELAY)
//3.6.4 2016/12/7
// STANDBY_WAIT 下 VentToAir(VENT_DELAY)+EnableTurboVent(), GV0 OPEN
//3.6.5 2016/12/8
// EM200 use STANDBY0 and STANDBY1, EM100 use STANDBY
// StopTurbo下兩次命令
//3.6.6 2016/12/8
// RS485_WriteString() reentry protection
// GetTurboRotationSpeed(), GetTurboMotorTemp() reentry protection
// decrease StopTurbo OSTimeDly wait time from 3 to 1
//3.6.7 2016/12/12
// RS485_WriteString_P1(),RS485_WriteString_P2()獨立副程式
// SetTurboPump(1) always on
// add INT2_LO_WAIT, INT2_HI_WAIT (not complete yet)
// LED yellow shows READY state
// PWRON:8:0/1 --> turbo pump power on/off (use with care, for debug)
// PWRON:9:0/1 --> scroll pump power on/off (use with care, for debug)
// fix bug for StartTurbo, StopTurbo control, use buffer
//3.6.8 2016/12/13
// INT_HI_WAIT, HI_WAIT timeout 時間分別設定
// VACPARA:21 設定 HI_WAIT, VACPARA:1 設定  INT_HI_WAIT
// 縮短 RS485 bit delay to 11, 起始等待 = bit delay / 2
// HI_WAIT start turbo 等待時間由三秒改為一秒
// add INT2_LO_WAIT, INT2_HI_WAIT function, 由MovePZTX, MovePZTY
// VACPARA:22 設定 fIGINT2Th
// make ScanStop() more robust, AutoScanStop()
//3.6.9 2016/12/17
// remove resend function. try to solve crash problem
// set sHighWaitTimeout (VACPARA:21) and fIGINT2Th (VACPARA:22)
// check PZTMoveX=0 and PZTMoveY=1 switch status for INT2 determination (CON19.3-4 short)
//3.6.10 2016/12/18
// ENABLE_PID_UART=0 (1 might cause crash, TBD)
//3.6.11 2016/12/19
// fix bug for HI_WAIT_TIMEOUT and INT_HI_WAIT_TIMEOUT
//3.6.12 2016/12/26
// EM-100, update SysParam.sVacTimeout, unit sec
//3.6.13 2017/1/11
// transmit data continuously until nByteIn = nByteOut
//3.6.14 2017/1/19
// disable PITR at power on stage
//3.6.15 2017/2/8
// 掃瞄時真空異常會立即停止掃瞄
// ion gauge會維持恆開（原來程式AIR, ERROR時會關閉IG)
//3.6.16 2017/2/11
// add deflector,stigmator,objective test procedure (TEST:2,TEST:3,TEST:4)
// objective ADC offset must be accurate
//3.6.17 2017/2/14
// EDSONHW default change to 0 (EDS switch controlled by software)
// disable idle timeout (EN_IDLE_TIME=0)
// default idle timeout change to 7200 sec
//3.6.18 2017/2/22
// 減少GET PF TPSP NO RESP error message
//1. ERROR2時下 VACON:0+ENVENT:1 go to STANDBY_WAIT -> SATNDBY1, 僅chamber破大氣
//2. ERROR2時下 VACON:3 go to VAC_ST_INT_LO_WAIT, 全系統破真空至大氣
//3. ERROR2時下 VACON:1 go to 重抽真空(回到電子槍高真空狀態)
// EM100進入AIR_WAIT後，STANDBY可變為破大氣，也可以由破大氣變為STANDBY
//3.6.19 2017/3/6
// no overscan when handshake = 1, handshake process bug fix
// bug fix for VADC:0:1 command, shadow:14,15,16
//3.6.20 2017/4/3
// IMGNUM command
// send two images, use DataPkt.bNdx parameter to indicate image order (0,1)
// calculate ZMax, ZMin by firmware for two images
//3.6.21 2017/4/11
// add ENCL,POS command for CL control
// bScanning = OP_ENABLE_CL state
//3.6.22 2017/4/19
// speed up scan speed, improve REGION command performance
//3.6.23 2017/4/24
// HV function not verified yet!!!
// HVTYPE:0(Spellmen), 1(Matsusada), 2(custom), NV_SETTINGS add .bHVType
// define SYS_PARAM_2, header 0x7D,0xB5 (SYS2_HEADER_1
// GETSYS:2 (BINARY=1) to get HV parameters
//3.6.24 2017/4/26
// change to GETSYS:4 (BINARY=1) to get HV parameters
//3.6.25
// no overscan for ENCL, POS command
//----------------------------------------------------------------------------------
//4.1.1
// for SCAN_VA23 (FW ver 4.1.1), SETAO:nB:nCh:fV
// SYS_PARAM_VER = change from 3 to 4
// define BR_NUM=6, CO_NUM=4, BOARD_NUM=3
// video 4 (SEI) is controlled by SETBRX:4:<FV>, SETCOX:1:<FV>
// video 5 (CL) is controlled by SETBRX:5:<FV>, SETCOX:2:<FV>
// add SETSLPDA command, change SETOFSDA command
// OFS_DAC=0,xxx (scan board);OFS_DAC=1,xxx; (IO board)
// GET_VADC4, GET_VADC5
// SYS_PARAM_3 structure change, header change
//4.1.2 2017/6/6
// 新增SYS_PARAM_2.fTempe參數 (deg C)
// fAccVoltage, fAccCurrent 由SYS_PARAM移到 SYS_PARAM_2(自製高壓參數）
// RS485 time delay change from 11 to 12
//4.1.3 2017/6/28
// SCAN <-> HV GETV return "V=" change to "VADC="
// OBJ_ON_INIT_V, OBJ_STANDBY_V, heat up objective at power on stage
// SETHVB command, if (NV_Settings.bHVType == HV_CUSTOM) { //BRSEL
//4.1.4 2017/7/21
// add UDPPORT:0/1 for CL command (CL control program)
//4.1.5 2017/7/30
// add AUTOVAC for auto vacuum process
//4.1.6 2017/8/3
// ST_ERROR2依 IP error(bErrorIP)決定要不要關閉IP
// EM200 in ST_UH check more time (nErrorCount) to shutdown
// EM100 in ST_HI check turbo speed to ERROR
//4.1.7 2017/8/13
// add BLANKING:3 command (fix point)
//4.1.8 2017/8/16
// modify delay_us hardware loop to avoid dead lock, bHWUSDelay default=0
//4.1.9 2017/8/29
// OBJR:1(17 ohms) or OBJR:2(12 ohms)
//4.1.10 2017/9/4
// RESET VAC power for each VACON:XX action
//4.1.11 2017/10/2
// WriteIOCmdData(), WriteIODataCRC() add OSCritEnter(), OSCritLeave()
// add VACUUM:? read VAC_ST_LOG
//4.1.12 2017/10/11
// OBJR:1 --> 1:17ohm,2:14 ohm,3:12 ohm
// add temperature into log (VAC_ST_LOG)
// OBJSTBYR:<FV>, FV=0(OBJIMIN) ~ 1.0 (OBJIMAX)
//4.1.13 2017/10/17
// ACCKV:<NV>:<FV> adjust obj current
// PIXEL_NUM_X_MAX=3800, ERROR_COUNT_MAX=5
//4.1.14 2017/10/23
// FOCUS:14:OBJTURN, add OBJI in GETSYS:1
//4.1.15 2017/11/6
// default value update
// SetVOPCOn() control VOPC
// modify GETCI current value
//4.1.16 2017/11/16
// improve and add debug TPSP error message
// use AD5668 or AD5648, IODAC command
//4.1.17 2017/11/20
// USE_IN, ACCKV:2:(0-3), 0:15,1:12,2:10,3:8 kV
// add IODAC command (check in GETSET:4)
// FOCUS current change according to different energy
// add OBJ Amp-turn table for different energies, OBJ_ENERGY_NUM=4
// need to set 4 OBJIMAX, OBJIMIN, NV structure change
// add VAC_ST_GUN_TO_AIR for STANDBY1 status
// SATNDBY1 -> LO_WAIT -> HI -> GUN_TO_AIR -> AIR_WAIT
// add IPTYPE command
// GV2 close for AIR process, GV1 open
// NV_SETTING, SET_PARAM structure change, fObjIMAx[], fObjIMin[]
//4.1.18 2017/11/27
// add DACT command (check in GETSET:4), DACT:0:<NV> for scan board, DACT:1:<NV> for IO board
// 0:AD5328, 1:AD5648, 2:AD5668(16-bit)
// SETCOX:0:<CO> for channel 0-3, SETCOX:1:<CO> for channel 4, SETCOX:2:<CO> for channel 5
// CHECK command
//4.1.19 2017/12/6
// SYS_PARAM_2 has two fTempe[2]
// FOCUS:15:<NV> 0:use I, 1: use IN (check by FOCUS:?)
// update OBJIMAX, OBJIMIN table for different energy
// OBJ_ENERGY_NUM change to 5 (15,12,10,8,5 kV)
// SYSCLK:40 or SYSCLK:20, speed up process
//4.1.20 2017/12/8
// fix bug for DAC chip initialization for AD5668
// EnableFIFO(), ENFIFO command, not complete yet
// check #define FPGA_SYSCLK
// fix bug for obj current settings
//4.1.21 2017/12/15
// fix bug for PID version display
// EN_IDLE_TIMEOUT=1 (IDLET command)
// fix bug for obj current change ObjIN2V() to ObjI2V() settings
//4.1.22 2017/12/19
// add delay to make SCAN-IO commnuication more stable
//4.1.X
// add IPTYPE command
//----------------------------------------------------------------------------------
// 更新後需檢查的指令，參數
// GETSET:1 -> HVTYPE(0:S,1:M,2:Custom),
// GETSET:1 -> DELAY:4:XX:0 (default 20=1usec), overscan
// GETSET:1 -> SYSCLK
// GETSET:1 -> BAUD, IP_PWR_CTRL, HV Serial2 RS485 default baudrate change to 115200
// VACPARA:?
// SETR:?, fBaseCO[0-2], fRangeCO[0-2]
// FOCUS:? -> FOCUS:14:OBJTURN, FOCUS:15:USE_IN
// GETSET:4 -> UDPPORT (0:CMD_PORT(default), 1:nSrcPort), for CL control
// GETSET:4 -> AUTOVAC, OBJR(1:17 ohms, 2:14 kohms, 3:12 kohms)
// GETSET:4 -> OBJSTBYR (standby obj ratio)
// GETSET:4 -> ACCKV
// GETSET:4 -> DACT or IODAC(obsolete) (0:AD5328, 1:AD5468, 2:AD5668)
// GETSET:1 -> BASE_CO[0-2], RANGE_CO[0-2]
// check #define FPGA_SYSCLK
//----------------------------------------------------------------------------------
//
#define EM_100			1		//Tungsten (W)
#define EM_110			2		//
#define EM_200			3		//LaB6
//
#define MODEL_TYPE		EM_200
//
#define MAJOR_VER			4		//major version
#define MINOR_VER			1		//minor version
#define BUILD_VER			22		//build version
//
#define FW_DATE_Y			2017	//***
#define FW_DATE_M			12
#define FW_DATE_D			19
//------------------------------------
//EM_FW_V2.X is for VA23 and after
//two DAC chips on SCAN board
//------------------------------------
//#define SCAN_PCB_VER		23
#define SCAN_PCB_VER		21 //Omar
#define IO_PCB_VER			6
#define COIL_PCB_VER		6
#define FPGA_SYSCLK			20	//MHz
//
#define HV_SPELLMEN			0
#define HV_MATSUSADA		1
#define HV_CUSTOM			2
//
#define HV_TYPE 			HV_MATSUSADA
//
#define OBJ_CTRL_R			2	//1:17 ohm, 2:14 ohm, 3:12 ohm
#define USE_G1_VOLTAGE		1	//gauge 1 use voltage as status criteria
#define USE_GLOBAL_IG		1
//
#define EN_REDUNDANT		0
#define USE_CPU_IO			0	//RELAY_ON(CON19.1), VOP_ON(CON19.3)
#define EN_IDLE_TIMEOUT		1	//turn off HV and OBJ for idle time
//
#define CALC_MAXMIN			1	//calculate histogram max min by FW
#define USE_FIFO			0
//
//RS485 device
//$<ADDR>,<COMMAND>\n
#define IP_MOTOR_ADDR		1	//IP and motor gate valve controller
#define HV_ADDR				2	//custom made HV
#define PZT_ADDR			3
#define IO_ADDR				4	//IO_PCB_VER>=7
//
#if MODEL_TYPE == EM_100
#define MODEL_NAME		"EM100"
#define APP_NAME		"EM100 Controller"
#define SCALE_MODE		SCALE_BY_TRIG_W_ROT
#define ENABLE_IP		0	//enable ion pump
#define ENABLE_IG		0	//disable ion gauge
#define GLOBAL_IG		0	//global range IG
//gate valve 0:turbo, 1:chamber
//GV0:TP-AIR tube, GV1:SCROLL
#define TP_GATE_VALVE		0	//turbo leak by gate valve 1
#define MOTOR_GATE_VALVE	2	//gate valve 2, not used
#define GAUGE_OP_MODE		1	//use one pirani gauge only
#define USE_IP_CTRL_B		0	//use IP_PWR_CTRL board
//
#elif MODEL_TYPE == EM_110
#define MODEL_NAME		"EM110"
#define APP_NAME		"EM110 Controller"
#define SCALE_MODE		SCALE_BY_TRIG_W_ROT
#define ENABLE_IP		0
#define ENABLE_IG		0	//goto HVAC
#define GLOBAL_IG		0
#define TP_GATE_VALVE		1
#define MOTOR_GATE_VALVE	2	//gate valve 2
#define GAUGE_OP_MODE	1	//use one gun gauge only
#define USE_IP_CTRL_B		0
#elif MODEL_TYPE == EM_200		//LaB6
#define MODEL_NAME		"EM200"
#define APP_NAME		"EM200 Controller"
#define SCALE_MODE		SCALE_BY_TRIG_W_ROT
#define ENABLE_IP		1	//enable ion pump
#define ENABLE_IG		1	//goto UHVAC
#define GLOBAL_IG		1	//global range IG
//gate valve 0:chamber, 1:IP, 2:motor, 3:turbo pump
#define TP_GATE_VALVE		3	//turbo leak by gate valve 3
#define MOTOR_GATE_VALVE	2	//gate valve 2
#define GAUGE_OP_MODE		2	//use gauge(PG,G1)+ion gauge(IG,G3)
#define USE_IP_CTRL_B		1	//use IP_PWR_CTRL board
#endif
//
#if IO_PCB_VER >= 3
#define IO_74HC14			1	//***
#define SYS_PARAM_VER		4	//6 gate valve, use bGateValve2[], Slope[][], Offset[][]
#define SYS2_PARAM_VER		1	//SYS_PARAM_2 version
#else
#define IO_74HC14		0	//***
#define SYS_PARAM_VER	2	//4 gate valve, use bGateValve[]
#endif
//
#if SCAN_PCB_VER == 21
#define USE_TWO_CO		1	//CO2=BR2
#else
#define USE_TWO_CO		0
#endif
//
#if SCAN_PCB_VER >= 20
#define ENABLE_PID_UART		0 //PID_RX_DATA is connected to J2[8]
#define PID_UART_SIZE		128
#define ENABLE_PID			1 //PID product ID MCU
#define USE_AD7655			1
#elif SCAN_PCB_VER >= 17
#define ENABLE_PID_UART		0 //PID_RX_DATA is not connected to J2[8]
#define ENABLE_PID			1
#define USE_AD7655			1
#else
#define ENABLE_PID_UART		0
#define ENABLE_PID			0
#define USE_AD7655			0
#endif
//
#if SCAN_PCB_VER >= 14	//communication with IO board
#define USE_FPGA_DIO	1		//***
#else //VER12 use CPU IO
#define USE_FPGA_DIO	0		//***
#endif
//-----------------------------------------------------------
// EDS: SCALE_NO_ROTATION, use rotation board to set ratio
// SEM: SCALE_BY_DACVID, no rotation board, use DACVID to set ratio
//-----------------------------------------------------------
#if SCAN_PCB_VER >= 10
#define VDAC_BITN				16	//video DAC bit number
#define VADC_BITN				8
#define ENABLE_COIL_SHUNT		1
#define SERIAL_ADC				1
#define SERIAL_DAC				1
#elif SCAN_PCB_VER >= 9
#define VDAC_BITN				16	//video DAC bit number
#define VADC_BITN				8
#define ENABLE_COIL_SHUNT		1
#define SERIAL_ADC				0
#define SERIAL_DAC				0
#else
#define VDAC_BITN				12
#define VADC_BITN				16
#define SERIAL_ADC				0
#define SERIAL_DAC				0
#endif
//
#if SCAN_PCB_VER >= 11
#define HAS_IO_BAORD		1
#define HW_US_DELAY			1	//use hardware 1 us delay
//
#elif SCAN_PCB_VER >= 10
#define HAS_IO_BAORD		0
#define HW_US_DELAY			1	//use hardware 1 us delay

#elif SCAN_PCB_VER >= 9
#define HAS_IO_BAORD		0
#define HW_US_DELAY			1	//hardware delay_us()
#else
#define HAS_IO_BAORD		0
#define HW_US_DELAY			0
//#define FPGA_SYSCLK			20
#endif
#define COIL_RATIO_ALWAYS_1		1 //fRatioCoil is always 1
//
#if COIL_PCB_VER >= 8
#define SENR_NUM	2		//10 and 1 ohms
#define SHUNT_NUM	2
#define SENR_SHUNT_NUM  (SENR_NUM * SHUNT_NUM)
#define SHUNT_BY_ZOOM_CTRL			1
#else
#define SENR_NUM	2
#define SHUNT_NUM	2
#define SENR_SHUNT_NUM  (SENR_NUM * SHUNT_NUM)
#define SHUNT_BY_ZOOM_CTRL			1	//use DEF_SCALE0 and DEF_SCALE1 to control shunt
#endif
#define RLI		8
//
#if RLI == 10
#define DEF_SEN_R0		10.0
// 10//1=0.909, 10//1.5=1.304, 8.2//1.5=1.268
#define DEF_SEN_R1		(10*1)/(10+1) //=0.909		//parallel
// 3.3/13.3=0.24812(1.2406V), 2.7/12.7=0.21259(1.06295 V), 2.2/12.2=0.1803(0.9015V)
#define DEF_V_RATIO		(2.7/(10+2.7))		//deflector voltage set ratio
#elif RLI == 8
#define DEF_SEN_R0		8.2
#define DEF_SEN_R1		1.5
// 8.2//1.5=1.268, 8.2//1=0.891
#define DEF_SEN_R0R1	(DEF_SEN_R0*DEF_SEN_R1)/(DEF_SEN_R0+DEF_SEN_R1)		//parallel
#define DEF_R			1
#define DEF_V_RATIO		(DEF_R/(6.8+DEF_R))		//deflector voltage set ratio
//10/7.8=1.28
#endif
//
#define ENABLE_UART2			1
// bScaleMode
#define SCALE_BY_TRIG_W_ROT		1			//with rotation,ratio scaled by sine, cosine(-5~+5)
#define SCALE_BY_DACVID			2			//with rotation,ratio scaled by video DAC
#define SCALE_BY_TRIG_WO_ROT	3			//without rotation, scaled by trignometric DAC(0~10)
#define SCALE_BY_DAC16B			4			//SCAN_PCB_VER>=9, use 16-bit video DAC
//
#define DEF_SHUNT_R0			5
#define DEF_SHUNT_R1			135
#define DEF_SHUNT_RATIO0		1
#define DEF_SHUNT_RATIO1		28	//100 ohm(21), 135/5+1=28
#define DEF_SHUNT_RATIO2		55	//270 ohm
//
#define ENABLE_ROTATION			0
//
#define ENABLE_ORTHO_SIGNAL		1
//
#define SHOW_VACUUM_STATE	1
#define DEBUG_VAC_STATE		1
//
#define TURBO_CUSTOM			0
#define TURBO_PFEIFFER			1
#define TURBO_AGILENT			2
#define TURBO_TYPE_MIN			0
#define TURBO_TYPE_MAX			2
#define TURBO_TYPE_NUM_MAX		3
//
#define CONN_UART0		1
#define CONN_UDP		2
#define CONN_TCP		3
#define CONN_UART0_BIN	4
//
#define ADDR_DACX_START		0x1000	//2048 points
#define ADDR_DACY_START		0x2000	//2048 points
#define ADDR_DACDX_START	0x3000	//2048 points
#define ADDR_DACDY_START	0x4000	//2048 points
#define ADDR_DATA_START	 	0x5000	//about 4 M points
#define ADDR_BEI_START		0x5000	//
#define ADDR_HISTO_START 	0x6000	//get 512 histogram data
#define ADDR_SEI_START	 	0x7000	//2nd image is saved here
//
#define MAX_STRING_SIZE 	256
#define MAX_STRING_SIZE_P1 257
//
//#define TCP_LISTEN_PORT		10023
#define NFDS				2
#define UDP_LISTEN_PORT		10024	//binary data
#define UDP_TALK_PORT		10024
//
#define UDP_CMD_PORT		10024	//ASCII command
#define UDP_DATA_PORT		10025	//binary data
#define TCP_CMD_PORT		10024
#define TCP_DATA_PORT		10025
//
#define RX_BUFSIZE 			(4096)
//
#define TX_TEST_SIZE	1024
#define TX_TEST_NUM		1024
//
#define USE_FINE_SCAN_ONLY		1
//#define X_SKIP_NUM				0
//#define OBJ_CC_AMP				0.83	//obj constant current source, unit ampere
#define PIXEL_NUM_X_MAX		3800
#define PIXEL_NUM_Y_MAX		(PIXEL_NUM_X_MAX*3/4) //2700
#define PIXEL_NUM_X_MAX_PI	(PIXEL_NUM_X_MAX+16)
#define ROW_BYTE_NUM_MAX	(2*PIXEL_NUM_X_MAX + sizeof(DATA_BEGIN_HEADER))	//=PIXEL_NUM_X_MAX*2+10,1 pixel need two bytes
#define PIXEL_NUM_X_MIN		240
#define PIXEL_NUM_Y_MIN		180
//
#define RESEND_NUM_MAX		5
//
#define PKT_BYTE_NUM		1400
#define PKT_NUM_MAX			(PIXEL_NUM_X_MAX*2/PKT_BYTE_NUM+1)		//(4000*2/1400 + 1)
// 1,000,000 * 6 us = 10 s
#define QUE_NUM_MAX				(1000000/PIXEL_NUM_X_MAX)
#define QUE_DATA_START			sizeof(DATA_HEADER)
#define UDP_QUEUE_SIZE			(PIXEL_NUM_X_MAX*2*QUE_NUM_MAX)	//2 bytes per pixel
//
//#define RELAY_ON		J2[10]=1	//nCHAM_CLOSE
//#define RELAY_OFF		J2[10]=0
//#define VOP_ON		J2[9]=1
//#define VOP_OFF		J2[9]=0
//
#define PID_TX_EMPTY	J2[6]
#define PID_RX_DATA		J2[8]
#define	 PZT_MOVE_X		J2[10]	//CON19.1
#define	 PZT_MOVE_Y		J2[9]	//CON19.3
#define CPU_IO0			J2[11]
#define CPU_IO1			J2[19]
#define CPU_IO2			J2[17]
#define CPU_IO3			J2[24]
#define N_RESET_FPGA	J2[26] //SCAN_VA22
//
#define CS_PID				J2[18]	//MG82FG5B32 input, NB output
#define DOUT_PID			J2[36]	//MG82FG5B32 output, NB input
//
#if SCAN_PCB_VER <= 9
#define DEF_X10				J2[12]
#endif
//
#if SCAN_PCB_VER <= 10
#define COIL_SWITCH0		J2[13]		//low inductance for large magnification
#define COIL_SWITCH0_ON		J2[13]=1	//coil shunt
#define COIL_SWITCH0_OFF	J2[13]=0
#define EDS_CTRL		J2[15]
#define MUX_CTRL0		J2[18]		//AD multiplexer control
#define MUX_CTRL1		J2[20]
#define RELAY			J2[31]
#define RELAY_ON		J2[31]=1
#define RELAY_OFF		J2[31]=0
#endif
//
#define LED_GREEN		J2[16]
#define LED_GREEN_ON	J2[16]=0
#define LED_GREEN_OFF	J2[16]=1
#define LED_RED			J2[23]
#define LED_RED_ON		J2[23]=0
#define LED_RED_OFF		J2[23]=1
//
#if SCAN_PCB_VER >= 11
#define LED_BLUE		J2[34]
#define LED_BLUE_ON		J2[34]=0
#define LED_BLUE_OFF	J2[34]=1
#else
#define LED_BLUE		J2[39]
#define LED_BLUE_ON		J2[39]=0
#define LED_BLUE_OFF	J2[39]=1
#endif
//
#if SCAN_PCB_VER >= 9
#define LED_YELLOW		J2[12]
#define LED_YELLOW_ON	J2[12]=0
#define LED_YELLOW_OFF	J2[12]=1
#else
#define LED_YELLOW		J2[41]
#define LED_YELLOW_ON	J2[41]=0
#define LED_YELLOW_OFF	J2[41]=1
#endif
//
#if IO_PCB_VER == 1
#define N_CS_IO			J2[30]	//nCS0_SPI
#define SCK_IO			J2[25]
#define DIN_IO			J2[28]	//CPU output, ADC input
#define CS_ENABLE		N_CS_IO=0
#define CS_DISABLE		N_CS_IO=1
#define DIN_1			DIN_IO=1
#define DIN_0			DIN_IO=0
#define SCK_1			SCK_IO=1
#define SCK_0			SCK_IO=0
#elif IO_PCB_VER >= 2
#define N_CS_IO		J2[30]	//nCS0_SPI
#define SCK_IO		J2[25]
#define DIN_IO		J2[28]	//CPU output, ADC input
#define CS_ENABLE		N_CS_IO=0 //use 74HC07, OC output
#define CS_DISABLE		N_CS_IO=1
#define DIN_1			DIN_IO=1
#define DIN_0			DIN_IO=0
#define SCK_1			SCK_IO=1
#define SCK_0			SCK_IO=0
/*
#define CS_ENABLE		N_CS_IO=1 //use 74HC04, push-pull output
#define CS_DISABLE		N_CS_IO=0
#define DIN_1			DIN_IO=0
#define DIN_0			DIN_IO=1
#define SCK_1			SCK_IO=0
#define SCK_0			SCK_IO=1
*/
#endif
//
#define DOUT_IO			J2[27]	//CPU input, ADC output
#define N_RESET_IO		J2[20]

#if SCAN_PCB_VER <= 10
#define N_CS_ADC		J2[30]	//nCS0_SPI
#define SCLK_ADC		J2[25]
#define DIN_ADC			J2[28]	//CPU output, ADC input
#define DOUT_ADC		J2[27]	//CPU input, ADC output
#endif
//
#if SCAN_PCB_VER >= 10 //no ADC
#define N_CS_DAC0		J2[32]
#define N_CS_DAC1		J2[29]
#if SCAN_PCB_VER <= 10
#define N_CS_DAC1		J2[34]
#define N_CS_DAC2		J2[36]
#endif
#define SCLK_DAC		J2[33]
#define DIN_DAC			J2[35]	//CPU output, DAC input
#define N_LDAC			J2[40]
#endif
//
#define HVF_DEFAULT		1.80
#define HVA_DEFAULT		3.65
#define HVB_DEFAULT		2.50
//
#define OBJI_DEFAULT		0.7	//unit: ampere
#define OBJIMIN_LO			0.15
#if OBJ_CTRL_R == 1 //1.1A (17 kohm)
#define OBJIMAX_DEFAULT		1.00	//unit: ampere
#define OBJIMIN_DEFAULT		0.65	//unit: ampere
#define OBJ_ON_IMIN_V		9.0
#define OBJ_ON_INIT_V		3.5		//initial value
#define OBJ_STANDBY_V		4.5		//standby value, keep heat up
#define OBJIMAX_HI			1.15
#elif OBJ_CTRL_R == 2 //1.4 A (14 kohm)
#define OBJIMAX_DEFAULT		1.30	//unit: ampere
#define OBJIMIN_DEFAULT		0.65	//unit: ampere
#define OBJ_ON_IMIN_V		9.0
#define OBJ_ON_INIT_V		3.5		//initial value
#define OBJ_STANDBY_V		6.5		//standby value, keep heat up
#define OBJIMAX_HI			1.42
#else //1.6A (12 kohm)
#define OBJIMAX_DEFAULT		1.50	//unit: ampere
#define OBJIMIN_DEFAULT		0.8	//unit: ampere
#define OBJ_ON_IMIN_V		9.0
#define OBJ_ON_INIT_V		4.5		//initial value, 5.5/6=0.90A
#define OBJ_STANDBY_V		5.5		//standby value, 4.5/6=0.75A
#define OBJIMAX_HI			1.62
#endif
//
#define MAG_MIN			20
#define MAG_MAX			100000
//#define MAG_MAX_RANGE	20		//magnification at maximal range
//
#if SCAN_PCB_VER >= 11
#define RS485_TX_ENABLE			J2[38]=1	//UART1
#define RS485_TX_DISABLE		J2[38]=0
#define RS485_TX1				J2[38]
#define RS485_TX1_ENABLE		J2[38]=1	//UART1
#define RS485_TX1_DISABLE		J2[38]=0
#define RS485_TX2				J2[31]
#define RS485_TX2_ENABLE		J2[31]=1	//UART2
#define RS485_TX2_DISABLE		J2[31]=0
#define RS485_TX3				J2[15]
#define RS485_TX3_ENABLE		J2[15]=1	//UART3
#define RS485_TX3_DISABLE		J2[15]=0
#elif SCAN_PCB_VER >= 6
#define RS485_TX_ENABLE		J2[38]=1
#define RS485_TX_DISABLE	J2[38]=0
#else
#define RS485_TX_ENABLE		J2[17]=1
#define RS485_TX_DISABLE	J2[17]=0
#endif
#define ENABLE_EDX			J2[15]=1
#define DISABLE_EDX			J2[15]=0
//
// interlock check
#define ACT_SET_HV		1
#define ACT_HV_ON		2
#define ACT_SCROLL_ON	3	//scroll pump ON
#define ACT_TURBO_ON	4	//turbo pump ON
//
#define SHADOW_1CH_0		0	//ch0
#define SHADOW_1CH_1		1	//ch1
#define SHADOW_1CH_2		2	//ch2
#define SHADOW_1CH_3		3	//ch3
#define SHADOW_2CH			4	//ch0+ch2
#define SHADOW_4CH			5	//sum(ch0~3)
#define SHADOW_HDIFF		6	//
#define SHADOW_VDIFF		7	//
#define SHADOW_M2			8	//ch0+ch1+ch3-ch2
#define SHADOW_M3			9	//ch0+ch1+ch2-ch3
#define SHADOW_SEI			10	//used for two packet transmission
#define SHADOW_2CH_P		11	//ch0+ch1
#define SHADOW_2CH_P1		11	//ch0+ch1
#define SHADOW_2CH_M1		12	//ch0-ch1+0x7FFF
#define SHADOW_2CH_M2		13	//ch1-ch0+0x7FFF
#define SHADOW_2CH_P2		14	//ch2+ch3
#define SHADOW_2CH_M3		15	//ch2-ch3+0x7FFF
#define SHADOW_2CH_M4		16	//ch3-ch2+0x7FFF
#define SHADOW_1CH_4		17	//ch4
#define SHADOW_1CH_5		18  //ch5
//
//0: inside case, 1: coil heat sink, 2: turbo pump start, 3: power heat sink, 4: turbo pump run
//5: TEMPE3
#define TEMP_SENSOR_NUM_MAX		5
#if IO_PCB_VER >= 2
#define TEMP_SENSOR_NUM			5 //TEMP0(0),turbo_START(1),HS1(2),HS2(3),turbo_RUN(4)
#elif IO_PCB_VER >= 1
#define TEMP_SENSOR_NUM			4 //TEMP0(0),turbo(1), HS1(2), HS2(3)
#else
#define TEMP_SENSOR_NUM			3
#endif
#define TEMP_IN_CASE		0	//=TEMPE0, PCB board temperature
#define TEMP_COIL_HS		1	//=TEMPE1, coil heat sink
#define TEMP_TURBO_START	2	//turbo pump start temperature
#define TEMP_POWER_HS		3	//=TEMPE2, power heat sink
#define TEMP_TURBO_RUN		4	//TURBO run temperature
#define TEMP_TEMPE3			5	//=TEMPE3
//
#define TEMP0_HI_LIMIT		55		//case, TEMP0
#define	TEMP0_LO_LIMIT		-10
#define TEMP1_HI_LIMIT		60		//coil heat sink, TEMP1
#define	TEMP1_LO_LIMIT		-10
#define TEMP2_HI_LIMIT		90		//turbo pump start stage
#define	TEMP2_LO_LIMIT		-10
#define TEMP3_HI_LIMIT		60		//power heat sink, TEMP2
#define	TEMP3_LO_LIMIT		-10
#define TEMP4_HI_LIMIT		90		//turbo pump run stage
#define	TEMP4_LO_LIMIT		-10
//
//#define DEBUG_ROTATE	1
#if SCAN_PCB_VER >= 11
#define DAC_SINE_COSINE_MAX		10.6		//6.8 kohm, no rotation function, plus and minus
#define SINE_COSINE_MAX			10.0			//-5 ~ +5
#define	COIL_RATIO_MAX			1			//ratio amplified by SINE and COSINE
#define RANGE_MAX_RATIO			0.9
#elif SCAN_PCB_VER >= 9
#define DAC_SINE_COSINE_MAX		10.5		//no rotation function
#define SINE_COSINE_MAX			10.0			//-5 ~ +5
#define	 COIL_RATIO_MAX			1			//ratio amplified by SINE and COSINE
#define RANGE_MAX_RATIO			0.9
#elif SCAN_PCB_VER <= 7
#define DAC_SINE_COSINE_MAX		5.625		//no rotation function
#define SINE_COSINE_MAX			5.0			//-5 ~ +5
#define	 COIL_RATIO_MAX			1			//ratio amplified by SINE and COSINE
#define RANGE_MAX_RATIO		0.9
#else
//#define DAC_SINE_COSINE_MAX		5.625		//10k,15k+20k,1.25*4.5=5.625
//10k,30k+10k, 1.25*5=6.25
#define DAC_SINE_COSINE_MAX		10.5		//10k,30k+30k,1.25*7=8.75
#define SINE_COSINE_MAX			10			//-5 ~ +5
#define	 COIL_RATIO_MAX			2			//use 5 ohm deflector sense resistor
#define  RANGE_MAX_RATIO		0.9
#endif
#define DEF_I_MAX				1	//deflector maximal current
//
#if SCAN_PCB_VER >= 9
#define DI_MASK		0x0F00
#else
#define DI_MASK		0x0E80
#endif
//
#if USE_AD7655 == 1
#define MULTIPLY_NUM	128	//MULTIPLY_BITN=8
#define AVGNUM_MAX		16
#define AVGNUM_MIN		1
#elif SCAN_PCB_VER >= 10 //avg_num max = 1024
#define MULTIPLY_NUM	16384	//MULTIPLY_BITN=14
#define AVGNUM_MAX		1024
#define AVGNUM_MIN		32
#else //avg_num max = 256
#define MULTIPLY_NUM	4096	//MULTIPLY_BITN=12
#define AVGNUM_MAX		1024
#define AVGNUM_MIN		32
#endif
//
#define SINE_DATA_NUM	256	//must be power 2
#define CHECK_PER_SEC	2
#define CHECK_PER_2SEC	(CHECK_PER_SEC*2)
#define CHECK_PER_4SEC	(CHECK_PER_SEC*4)
//
#define READY_COUNT_OK	(10*CHECK_PER_SEC) //sustained 10 seconds
//
#define PORT_NUM_MAX	4	//serial port number maximum
// operation mode
#define OP_IDLE				0
#define OP_SEM_SCAN			1	//SEM operation
#define OP_EDS				2	//EDS control scanning process
#define OP_AUTO_FOCUS		3	//auto focus operation
#define OP_POINT			4	//use deflector to go to set point
#define OP_BLANKING			5
#define OP_AUTO_FOCUS_SW	6
#define OP_ENABLE_CL		7
// error message
#define ERR_UNKNOWN			-1000
#define ERR_INV_PARAM		-1
#define ERR_REENTRY			-2
#define ERR_OVER_RANGE		-3
#define ERR_INV_VAC_ST		-4
#define ERR_INTLK_FAIL		-5
#define ERR_UNDER_RANGE		-6
#define ERR_INV_VERSION		-7	//invalid hardware version, SCAN, IO
#define ERR_EDS_ONLY		-8
#define ERR_MAG_TOO_SMALL	-9
#define ERR_MAG_TOO_LARGE	-10
#define ERR_INV_MODE		-11	//invalid scan mode
#define ERR_INV_CMD			-12
#define ERR_MAG				-13
#define ERR_INV_PID			-14	//invalid PCB ID
//
#define ERR_HW_SCAN_VER		0x01	//invalid scan version
#define ERR_HW_IO_VER		0x02	//invalid I/O version
#define ERR_HW_PCBID		0x04	//invalid PCB ID
#define ERR_HW_HVID			0x08	//invalid HV ID
//
#define PCB_ID_NUM			6		//6 bytes
#define HV_ID_NUM			6
//
#define AIR_WAIT_TIMEOUT		360		//AIR_WAIT to AIR
#define LOW_WAIT_TIMEOUT		180		//LOW_WAIT to LOW
#define INT_HIGH_WAIT_TIMEOUT	259200	//INT_HI_WAIT to INT_HI, 3 days
#define HIGH_WAIT_TIMEOUT		180		//HI_WAIT to HI
//
#define MHIGH_WAIT_TIMEOUT	7200	//HI to MH
#define UHIGH_WAIT_TIMEOUT	86400	//MH to UH
#define READY_WAIT_TIMEOUT	180		//READY_WAIT to READY
//
#define TPGV_TIMEOUT			180		//s, turbo pump gate valve open time
#define STANDBY_WAIT_TIEMOUT	220	//must larger than TPGV_TIMEOUT
//
//#define PL_SAMP_NUM_PER_PERIOD	32
//
#define ECHO_TIMEOUT	10
#define IDLE_TIMEOUT	3600
//
#define HV_RS485_ADDR		1	//HV module RS485 address
#define MOTOR_RS485_ADDR	4	//4(X),5(Y),6(Z)
//
#define WDT_TIMEOUT		4.0
#define AF_LIMIT_NUM	40	//auto focus number
#define REGN_RATIO		4	//1/4 to 3/4
//
#define CHECK_VOLT_NUM	4	//IP leak process in INT_HI_WAIT
//
#if SCAN_PCB_VER >= 20
#define ENABLE_RESET_VADC	1
#else
#define ENABLE_RESET_VADC	0
#endif
//----------------------------------------------------------
#if IO_PCB_VER >= 6
#define GAUGE_POWER_CTRL	1	//gauge power control
#else
#define GAUGE_POWER_CTRL	0
#endif
//
#define ACC_KV_BASE		15.0
#define USE_IN			1		//use ampere turn
#define IO_CMD_NUM		6 		//push, pop
#endif /*MAIN_H_*/
