#ifndef CONTROL_TURBO
#define CONTROL_TURBO

#define TP_TEMP_HI		70	//centigrade
#define TP_SPEED_HI		1500	//Hz --> READY
#define TP_SPEED_LO		100		//Hz --> AIR or STANDBY
//#define	 TURBO_ROT_SPEED_LIMIT	1500 //Hz
//
int InitTurbo(void);
void StartTurbo(void);
void StopTurbo(int nEnableVenting);
int GetTurboRotationSpeed (int nUpdate);
int GetTurboMotorTemp (int nUpdate);
void SetTurboGV(int nOn); 	//035
int CloseTurbo(void);
void VentToAir(int nType);
void TurboVent(int nVent);
//
void ClearTurboCommand(void);
int GetTurboCommandNum(void);
void PushTurboCommand(int nCommand);
int PopTurboCommand(void);
void ExecuteTurboCommand(int nCommand);
void GetTurboErrorMsg(char *szError);
//
#define VENT_DELAY		1
#define VENT_NONE		2
#define VENT_DIRECT		3
//
#define TURBO_CMD_NUM		40
#define TP_CMD_STOP_VENT	1
#define TP_CMD_STOP_NO_VENT	2
#define TP_CMD_START		3
#define TP_CMD_VENT_DIRECT	4
#define TP_CMD_VENT_DELAY	5
#define TP_CMD_VENT_NONE	6
//
#endif
