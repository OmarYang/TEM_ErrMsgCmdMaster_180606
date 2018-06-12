/*
 * io.h
 *
 *  Created on: 2012/8/18
 *      Author: Robert Lee
 */

#ifndef IO_H_
#define IO_H_
//
#include "main.h"
//
#define VADC_CH_NUM			4
//----------------------------------------------
// DAC channel definition
//----------------------------------------------
#if IO_PCB_VER >= 3
#define VAC_GATE_NUM_MAX	6
#define VAC_GATE_NUM		4	//two more gate valves
#else
#define VAC_GATE_NUM_MAX	4	//gate valve number max
#define VAC_GATE_NUM		2	//gate valves used in effect
#endif
//
#if IO_PCB_VER >= 1
#define DAC_CH_NUM_MAX		16
#define DAC0_CH_NUM_MAX		14 //scan board
#define DAC1_CH_NUM_MAX		16 //IO board
//
#define IO_DAC_CH_START		0
//----------------------------------------------
// CH0~7 are on scan board
//----------------------------------------------
//scan board DAC
#define DAC_CH_BR1			0	//deflector X coarse
#define DAC_CH_BR2			1	//deflector Y coarse
#define DAC_CH_BR			2	//brightness
#define DAC_CH_BR0			2
#define DAC_CH_CO			3	//contrast
#define DAC_CH_CO0			3
#define DAC_CH_VID_REF1		4	//video reference voltage
#define DAC_CH_BR3			5
#define DAC_CH_COSINE		6	//reserved, -10V ~ +10V
#define DAC_CH_SINE			7	//reserved, -10V ~ +10V
#define DAC_CH_BR4			8
#define DAC_CH_CO1			9
#define DAC_CH_BR5			10
#define DAC_CH_CO2			11
#define DAC_CH_BSE0			12
#define DAC_CH_BSE1			13
//IO board DAC
#define DAC_CH_AL0		(IO_DAC_CH_START+0)
#define DAC_CH_AL1		(IO_DAC_CH_START+1)
#define DAC_CH_AL2		(IO_DAC_CH_START+2)
#define DAC_CH_AL3		(IO_DAC_CH_START+3)
#define DAC_CH_STIGX	(IO_DAC_CH_START+4)	//stigmator X
#define DAC_CH_STIGY	(IO_DAC_CH_START+5)	//stigmator Y
#define DAC_CH_OBJ_C	(IO_DAC_CH_START+6)	//objective coarse
#define DAC_CH_OBJ_F	(IO_DAC_CH_START+7)	//objective fine
#define DAC_CH_RESV0	(IO_DAC_CH_START+8)
#define DAC_CH_RESV1	(IO_DAC_CH_START+9)
#define DAC_CH_RESV2	(IO_DAC_CH_START+10)
#define DAC_CH_RESV3	(IO_DAC_CH_START+11)
#define DAC_CH_HV_FILA	(IO_DAC_CH_START+12)	//filament
#define DAC_CH_HV_BIAS	(IO_DAC_CH_START+13)	//bias
#define DAC_CH_HV_ACC	(IO_DAC_CH_START+14)	//accelerator
#define DAC_CH_HV_FEG	(IO_DAC_CH_START+15)	//FEG control, reserved
//IO ctrl board DAC, 16-bit DAC
//
#define DAC_CH_DEFX_F	(DAC_CH_NUM_MAX+1)	//deflector X fine
#define DAC_CH_DEFY_F	(DAC_CH_NUM_MAX+2)	//deflector Y fine
//
#endif
//
//----------------------------------------------
// fine tuned coil definition
//----------------------------------------------
#define COIL_CH_NUM			3	//fine+coarse control
#define COIL_CH_DEFX		0	//
#define COIL_CH_DEFY		1	//
#define COIL_CH_OBJ			2
//
#define VADC_CH_NUM_MAX		7
#define VDAC_CH_NUM_MAX		2
#define VDAC_CH_DEFX_F		0	//deflector X coarse
#define VDAC_CH_DEFY_F		1
//----------------------------------------------
// coil current sense definition
// DEFX,DEFY,STIGX,STIGY,OBJ,AL0,AL1,AL2,AL3
//----------------------------------------------
#define ISEN_CH_NUM			9
#define ISEN_CH_DEFX		0	//deflector
#define ISEN_CH_DEFY		1
#define ISEN_CH_STIGX		2	//stigmator
#define ISEN_CH_STIGY		3
#define ISEN_CH_OBJ			4
#define ISEN_CH_AL0			5
#define ISEN_CH_AL1			6
#define ISEN_CH_AL2			7
#define ISEN_CH_AL3			8
//
#if IO_PCB_VER >= 1
#define ADC_CH_NUM_MAX		24
#define CH_PER_MUX			6
#define MUX_NUM				4		//6
#else
#define ADC_CH_NUM_MAX		16
#define CH_PER_MUX			4
#define MUX_NUM				4		//6
#endif
// GP_AD0 (nCS_ADC0)
#define ADC_CH_VAC0			0	//MUX00,ch0,vacuum gauge 0 AO
#define ADC_CH_VAC1			1	//MUX01,ch0,vacuum gauge 1 AO
#define ADC_CH_VAC2			2	//MUX10,ch0,vacuum gauge 2 AO
#define ADC_CH_VAC3			3	//MUX11,ch0,vacuum gauge 3 AO, 10V_1MA_X
#define ADC_CH_10V_1MA		3
#define ADC_CH_IG			3	//ion gauge
// GP_AD1 (nCS_ADC0)
#define ADC_CH_AL0			4	//MUX00,ch1,aligner coil 0 sense
#define ADC_CH_AL1			5	//MUX01,ch1,aligner coil 1 sense
#define ADC_CH_AL2			6	//MUX10,ch1,aligner coil 2 sense
#define ADC_CH_AL3			7	//MUX11,ch1,aligner coil 3 sense
// GP_AD2 (nCS_ADC0)
//
#if IO_PCB_VER >= 1
#define ADC_CH_DEFX			8
#define ADC_CH_DEFY			9
#define ADC_CH_STIGX		10
#define ADC_CH_STIGY		11
#else
#define ADC_CH_STIGY		8	//MUX00,ch2,
#define ADC_CH_STIGX		9	//MUX01,ch2,
#define ADC_CH_DEFY			10	//MUX10,ch2,
#define ADC_CH_DEFX			11	//MUX11,ch2,
#endif
// GP_AD3 (nCS_ADC0)
#define ADC_CH_BIAS_I		12	//MUX00,ch3
#define ADC_CH_MON_1		12	//MUX00,ch3, HV monitor 1
#define ADC_CH_OBJ			13	//MUX01,ch3,
#define ADC_CH_TEMPE0		14	//MUX10,ch3, temperature #0
#define ADC_CH_TEMPE1		15	//MUX11,ch3, temperature #1
//
#define ADC_CH_MON_2		16	//MUX00,ch4, HV monitor 2
#define ADC_CH_TEMPE3		17	//MUX01,ch4, RESV_SEN5
#define ADC_CH_TEMPE2		18	//MUX10,ch4, RESV_SEN6
#define ADC_CH_RESV7		19	//MUX11,ch4, RESV_SEN7, ,1V_1MA_X
#define ADC_CH_1V_1MA		19
//
#define ADC_CH_RESV8		20	//MUX00,ch5, RESV_SEN4
#define ADC_CH_RESV9		21	//MUX01,ch5, RESV_SEN5
#define ADC_CH_RESV10		22	//MUX10,ch5, RESV_SEN6
#define ADC_CH_RESV11		23	//MUX11,ch5, RESV_SEN7
//
#define ADC_CH_VIDEO0		4	//4,chip2,ch0, nCS_ADC1
#define ADC_CH_VIDEO1		5	//5,chip2,ch1
#define ADC_CH_VIDEO2		6	//6,chip2,ch2
#define ADC_CH_VIDEO3		7	//7,chip2,ch3
//
// (nCS_ADC1)
#define VIDEO_ADC_CH_NUM		6	//V0,V1,V2,V3,V4,V5
//#define VIDEO_ADC_ALL_CH_NUM	9	//V0,V1,V2,V3,2CH,4CH,HDIFF,VDIFF,M2,M3
//
// IO board command
#define IO_CMD_BITN			5
/*
`define GET_ADC			5'b0_001_1
`define SET_DAC_0		5'b0_010_1
`define SET_DAC_1		5'b0_011_1
`define SET_IO_OUT		5'b0_100_1
`define GET_IO_IN		5'b0_101_1
`define SET_LED			5'b0_110_1
`define SET_ADC_A		5'b0_111_1
`define SET_FREQ_ACC	5'b1_000_1	//set acc frequency
`define SET_FREQ_FILA	5'b1_001_1	//set fila frequency
`define SET_FREQ_BIAS	5'b1_010_1	//set bias frequency
`define GET_VERSION		5'b1_011_1
*/
#if IO_PCB_VER == 1
#define IO_GET_ADC			0x03
#define IO_SET_DAC_0		0x05
#define IO_SET_DAC_1		0x07
#define IO_SET_IO_OUT		0x09
#define IO_GET_IO_IN		0x0B
#define IO_SET_LED			0x0D
#define IO_SET_ADC_A		0x0F
//
#define IO_SET_FREQ_ACC		0x11
#define IO_SET_FREQ_FILA	0x13
#define IO_SET_FREQ_BIAS	0x15
#define IO_GET_VERSION		0x17
#else
#define IO_GET_ADC			0x02
#define IO_GET_IO_IN		0x03
#define IO_GET_VERSION		0x04
//
#define IO_SET_DAC_0		0x10
#define IO_SET_DAC_1		0x11
#define IO_SET_IO_OUT		0x12
#define IO_SET_IO_OUT0		0x12

#define IO_SET_LED			0x13
#define IO_SET_ADC_A		0x14
//
#define IO_SET_FREQ_ACC		0x15
#define IO_SET_FREQ_FILA	0x16
#define IO_SET_FREQ_BIAS	0x17
#define IO_SET_LED_RG		0x18
#define IO_SET_LED_BY		0x19
#define IO_SET_PARAM		0x1A
#define IO_SET_IO_OUT1		0x1B
#endif
//
//write MSB first
/* SET_IO_OUT
senr_sel = spi_in_reg[0];
obj_on = spi_in_reg[1];
shunt = spi_in_reg[2];
gv_ctrl[0] = spi_in_reg[3];
gv_ctrl[1] = spi_in_reg[4];
gv_ctrl[2] = spi_in_reg[5];
hv_on = spi_in_reg[6];
hv_pwr_on = spi_in_reg[7];
sp_ctrl = spi_in_reg[8];
tp_ctrl = spi_in_reg[9];
*/
#if IO_PCB_VER >= 3
#define IO_GV_CTRL3			0x0001
#define IO_GV_CTRL4			0x0002
#define IO_GV_CTRL5			0x0004
#else
#define IO_SENR_SEL			0x0001
#define IO_OBJ_ON			0x0002
#define IO_SHUNT			0x0004
#endif
//
#define IO_GV_CTRL0			0x0008		//3
#define IO_GV_CTRL1			0x0010		//4
#define IO_GV_CTRL2			0x0020		//5
#define IO_HV_ON			0x0040		//6,HV module on/off
#define IO_HV_PWR			0x0080		//7,HV module power
#define IO_SP_CTRL			0x0100		//scroll pump power control
#define IO_TP_CTRL			0x0200		//turbo pump power control
//#define IO_LED_G2			0x0400		//bit10, reserved
//#define IO_LED_R2			0x0800		//bit11, reserved
//#define IO_LED_Y2			0x1000		//bit12, reserved
//#define IO_LED_B2			0x2000		//bit13, reserved
#define IO_IP_CTRL			0x4000		//bit14, ion pump control
#define IO_IG_CTRL			0x8000		//bit15, ion gauge control
//
#define IO_LED_GREEN		0x0001
#define IO_LED_RED			0x0002
#define IO_LED_YELLOW		0x0004
#define IO_LED_BLUE			0x0008
//
#define IO_ADC_A0			0x0001
#define IO_ADC_A1			0x0002
#define IO_DAC_N_LDAC		0x0004
//
// digital output
// SCAN board IO
#define DO_RELAY_ON		0x0001	//bit 0, SCAN board VER >= 11
#define DO_EDS_ON		0x0002	//bit 1, SCAN board VER >= 11
#define DO_SHUNT0		0x0004	//bit 2
#define DO_SENR_SEL		0x0040	//bit 6
#define DO_VOP_ON		0x0100	//bit 8, VOP power
#define DO_OBJ_ON		0x0200	//bit 9, OBJ_ON
#define DO_SEL_COIL		0x0400	//bit 10, SEL_COIL
#define DO_VOPC_ON		0x0800	//bit 11, VOPC_ON
//
#if IO_PCB_VER >= 1
//read MSB first
#define DI_VAC_GAUGE0		0x0001	//DIN0
#define DI_VAC_GAUGE1		0x0002	//DIN1
#define DI_CHAM_CLOSE		0x0004	//DIN2
#define DI_VAC_GAUGE2		0x0008	//DIN3
#define DI_VAC_GAUGE3		0x0010	//DIN4
#define DI_CASE_CLOSE		0x0020	//DIN5
#define DI_SW0				0x0040	//DIN6
#define DI_SW1				0x0080	//DIN7
#define DI_IP_HV_ON			0x0100	//DIN8
#if IO_PCB_VER >= 3
#define DI_IN0				0x0800	//DIN11, CON33.1
#define DI_IN1				0x1000	//DIN12, CON33.3
#define DI_IN2				0x2000	//DIN13, CON33.5
#define DI_IN3				0x4000	//DIN14, CON33.7
#define DI_IN4				0x8000	//DIN15, CON33.9
#else
#define DI_EDS_ON			0x0200	//DIN9	//dio_in0
#define DI_RESERVE1			0x0400	//DIN10	//dio_in1
#endif
#else
#define DI_VAC_GAUGE0		0x0001	//DIN0
#define DI_VAC_GAUGE1		0x0002	//DIN1
#define DI_CHAM_CLOSE		0x0004	//DIN2
#define DI_VAC_GAUGE2		0x0008	//DIN3
#define DI_VAC_GAUGE3		0x0010	//DIN4
#define DI_CASE_CLOSE		0x0020	//DIN5
//#define DI_SW0				0x0040	//DIN6
//#define DI_SW1				0x0080	//DIN7
#endif
//
#if SCAN_PCB_VER >= 9
#define DI_HV_ST			0x0080	//DIN7
#else
#define DI_HV_ST			0x0100	//DIN8
#endif
//
// vacuum state parameter
#define VAC_ST_NONE			0
#define VAC_ST_AIR			1	//atmospheric state
#define VAC_ST_STANDBY		2
#define VAC_ST_STANDBY0		2
#define VAC_ST_AIR_WAIT		3	//wait until AIR
#define VAC_ST_LO			4	//
#define VAC_ST_LO_WAIT		5	//wait until LOW
#define VAC_ST_HI			6	//=READY
// LO(low),HI(high),MH(medium high),UH(ultra high)
#define VAC_ST_HI_WAIT		7	//=READY_WAIT
#define VAC_ST_MH			8	//
#define VAC_ST_MH_WAIT		9	//wait until UHVAC
#define VAC_ST_UH			10	//
#define VAC_ST_UH_WAIT		11	//wait until UHVAC
#define VAC_ST_STANDBY1		12
#define VAC_ST_STANDBY_WAIT		13
#define VAC_ST_GUN_TO_AIR		14
//
#define VAC_ST_INT_LO_WAIT		20
#define VAC_ST_INT_HI_WAIT		22
#define VAC_ST_INT_MH_WAIT		23
#define VAC_ST_INT_UH_WAIT		24
#define VAC_ST_VENT_WAIT		27
#define VAC_ST_NOVENT_WAIT		28
#define VAC_ST_INIT				30
#define VAC_ST_CHECK_UH_WAIT	31
#define VAC_ST_INT_HI			32
#define VAC_ST_INT2_LO_WAIT		34	//-->INT2_HI_WAIT
#define VAC_ST_INT2_HI_WAIT		35	//-->INT_HI_WAIT
//
#if ENABLE_IP == 1 //enable ion pump
#define VAC_ST_ERROR1		101	//
#define VAC_ST_ERROR2		102	//
#define VAC_ST_READY		VAC_ST_UH	//
#else
#define VAC_ST_ERROR		100	//
#define VAC_ST_READY		VAC_ST_HI	//
#endif
//
/*#define STR_VAC_ST_AIR			"AIR"
#define STR_VAC_ST_STANDBY		"STANDBY"
#define STR_VAC_ST_AIR_WAIT		"AIR_WAIT"
#define STR_VAC_ST_LOW			"LOW"
#define STR_VAC_ST_LOW_WAIT		"LOW_WAIT"
#define STR_VAC_ST_READY		"HIGH"
#define STR_VAC_ST_READY_WAIT	"HIGH_WAIT"
#define STR_VAC_ST_UHVAC		"UH"
#define STR_VAC_ST_UHVAC_WAIT	"UH_WAIT"
#define STR_VAC_ST_ERROR		"ERROR"
*/
//
#define VAC_GAUGE_NUM_MAX	4	//0:chamber,1:gun,2:reserve,3:gauge
#define VAC_GAUGE_NUM		2
#define VAC_PG_NUM			3
#define VAC_BUF_NUM			5
//
#define VAC_NONE			2
#define VAC_BROKEN			3
#define VAC_AIR				0x04
#define VAC_LO				0x08	//8,IG low vacuum
#define VAC_HI				0x10	//16,IG high vacuum
#define VAC_A0L				(VAC_AIR|VAC_LO)
#define VAC_L0H				(VAC_LO|VAC_HI)
#define VAC_MH				0x20	//32,IG medium high vacuum
#define VAC_UH				0x40	//64,IG ultra high vacuum
#define VAC_M0U				(VAC_MH|VAC_UH)
#define VAC_H0M0U			(VAC_HI|VAC_MH|VAC_UH)
#define VAC_L0H0M0U			(VAC_LO|VAC_HI|VAC_MH|VAC_UH)
// GetValveStatus()
#define VALVE_OPEN			1
#define VALVE_CLOSE			0
//
#define VAC_TORR_L		5		//torr
#define VAC_TORR_H		1e-4	//torr
#define VAC_TORR_MH		2e-6
#define VAC_TORR_UH		8e-7
//
#define DOOR_OPEN			1	//
#define DOOR_CLOSE			0	//SHORT
// GetHVStatus()
#define HV_ST_ON			1
#define HV_ST_OFF			0
// GetGaugeStatus()
//#define VAC_ST_MIN			1
//#define VAC_ST_MAX			6
//
#if USE_AD7655 == 1
#define VADC_MAX	5.0
#define VADC_MIN	0.0
#else
#define VADC_MAX	1.666+0.512
#define VADC_MIN	1.666-0.512
#endif

#define HVA_MAX		7.25	//accelerator max
#define HVF_MAX		7.25	//filament max
//
#define PL_FREQ			60		//Hz
#define PL_SAMP_NO		20		//sample number per power line cycle
//
#if PCB_VER == 12
#define	 PL_DAC_WMAX	65535
#define	 PL_DAC_WMID	32768	//middle value
#define	 PL_DAC_WMIN	0
#define PL_DAC_VMAX		5.0
#define PL_DAC_VMIN		-5.0
#elif PLDAC_5V == 1
#define	 PL_DAC_WMAX	4095
#define	 PL_DAC_WMID	2048	//middle value
#define	 PL_DAC_WMIN	0
#define PL_DAC_VMAX		5.0
#define PL_DAC_VMID		2.5
#define PL_DAC_VMIN		0.0
#else
#define	 PL_DAC_WMAX	4095
#define	 PL_DAC_WMID	2048	//middle value
#define	 PL_DAC_WMIN	0
#define PL_DAC_VMAX		1.25
#define PL_DAC_VMID		0.0
#define PL_DAC_VMIN		-1.25
#endif
//
#define SET_LED_4BIT	1
//
#define UUID_NUM			16
#define OVERSCAN_MAX		3.0
//#define OVERSCAN_RATIO		(1.0+OVERSCAN_MAX)
#define IMAX_LOWER_TH	0.8
//
#define DAC_AD5328	0
#define DAC_AD5648	1
#define DAC_AD5668	2
#define DAC_CHIP	DAC_AD5328
#endif /* IO_H_ */
