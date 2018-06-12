#ifndef CPU_CMD_H
#define CPU_CMD_H
//
#include "io.h"
#include "main.h"
//
#define	 COARSE_DAC_DELAY	1
#define	 VIDEO_ADC_DELAY	1
//
#define DAC_PATTERN_BITN	16
//#define VIDEO_ADC_BITN		16
//
#define USE_INTERNAL_RAM	1
#define	 USE_UDP_TX_TASK	0	//1:enable, 0: disable
//
//#define SCAN_SAWTOOTH		1	//faster slew rate
//#define SCAN_TRIANGLE		2	//smaller slew rate
//#define SCAN_MODE			SCAN_SAWTOOTH //SCAN_TRIANGLE
//
//---------------------------------------------------------------
#define HEADER_1		0x57
#define HEADER_2		0x7A
#define FOOTER_1		0xA7
#define FOOTER_2		0x75
#define DATA_1			0x5B
#define DATA_2			0x7B
#define ONE_PKT_1		0x5C
#define ONE_PKT_2		0x7C
//#define DATA2_HEADER_1		0x58
//#define DATA2_HEADER_2		0x7B
//
#define DATA_HEADER_SIZE 	12
#define DATA_FOOTER_SIZE	8
#define DATA2_HEADER_SIZE	24
//
#define UDP_DATA_SIZE_MAX		1440
//#define UDP_TRANSMIT_SIZE		(UDP_DATA_SIZE_MAX*2)
//#define UDP_FOOTER_SIZE			6
//#define UDP_HEADER_SIZE			6
//
#define OK				0
#define SUCCESS				0
#define ERROR_FAIL			-1
#define ERROR_CHAM_OPEN		-2
#define ERROR_CASE_OPEN		-3
#define ERROR_VAC0_NOK		-4
#define ERROR_VAC1_NOK		-5
#define ERROR_TP_NOK		-6	//TP speed is too slow
#define ERROR_BTEMP_HI		-7	//board temperature too high or too low
#define ERROR_RESET			-8
// common command minimum
//-----------------------------------------------------
// SET_PARAMETER
#define PARA_SIMU_MODE		0x0001
#define PARA_AUTO_FOCUS		0x0004
#define PARA_PHASE0			0x0010	//PL phase check register
#define PARA_PHASE1			0x0020
#define PARA_HISTO_CH		0x0040
#define PARA_TWO_IMAGE		0x0080
#define PARA_2IMG_MASK		0x0380	//3-bit, two_image
#define PARA_CHIP_SEL		0x1C00
#define PARA_CHIP0_ONLY		0x0400	//only chip 0
#define PARA_CHIP1_ONLY		0x0800	//only chip 1
#define PARA_CHIP2_ONLY		0x1000	//only chip 2
#define PARA_RESET_FIFO		0x2000	//bit 13
#define PARA_ENABLE_FIFO	0x4000	//bit 14
//#define PARA_GRAD_N2M		0x0040 //gradient n2m
//
// bit0=A1, bit1=A2, ..., A0 is not used for 16-bit databus
// DIRECT_FLAG=1, databus1 will connect with databus2
//
#define DIRECT_FLAG			0x80	//A8
#define DATA_RNW_R			0x40	//A7=DATA_RNW_R,A6~A1, A0 is not used
#define DATA_RNW_W			0x00	//A7=DATA_RNW_R,A6~A1, A0 is not used
//
#define SET_SHADOW_MODE	(0+DATA_RNW_W)	//direct_addr=0,applied to all boards
//
#define SET_BLANKING	(1+DATA_RNW_W)
#define GET_FIFO_DATA	(1+DATA_RNW_R)	//pwFifo[GET_FIFO_DATA] or pwFpga[GET_FIFO_DATA]
#define SET_SIMU_MODE	(2+DATA_RNW_W)
#define SET_X_OVERSCAN	(3+DATA_RNW_W)
#define DELAY_MS_NUM	(4+DATA_RNW_W)	//for EDS operation
#define SET_PARAMETER	(5+DATA_RNW_W)
//
#define GET_STATUS2		(5+DATA_RNW_R)
//vadc_conv_start(15), running, sysclk, n_reset,
//vadc_scan_stop(11), vadc_scan_start, set_dacx, vadc_to_ram,
//scanning(7), histo_calc, row_data_ready, 1'b1,
//dout_io(3), delay_us_timeout(2), get_video_adc(1), x_descent(0)
#define GET_STATUS		(6+DATA_RNW_R)	//adc_eoc(4), adc1_eoc(3), empty(2), half full(1), full(0)
#define SET_DO0			(7+DATA_RNW_W)
#define GET_DEBUG1		(8+DATA_RNW_R)
//
//#define SET_FAST_CLOCK	(9+DATA_RNW_W)
#define SET_REGION_SCAN		(9+DATA_RNW_W)
#define SET_MULTIPLY		(10+DATA_RNW_W)
#define GET_TIME_ASC		(10+DATA_RNW_R)				//get ascending time
#define SET_SWAP			(11+DATA_RNW_W)	//SCAN VA23
#define GET_SWAP			(11+DATA_RNW_R)
//
//#define GET_DI			(11+DATA_RNW_R+DIRECT_FLAG)	//direct_addr=1,16-bit digital input
#define SET_1US_CLK		(12+DATA_RNW_W)		//9(20 MHz), 14(30 MHz), 19(40 MHz)
#define GET_TIME_X		(12+DATA_RNW_R)
//----------------------------------------------------------
// scan signal commands begin
//----------------------------------------------------------
#define SET_DACX_F		(12+DATA_RNW_W+DIRECT_FLAG)	//direct_addr=1,set DACX fine directly
#define SET_DACY_F		(13+DATA_RNW_W+DIRECT_FLAG)	//direct_addr=1,set DACY fine directly
//
#define SET_ADDR_BASE_L		(14+DATA_RNW_W)				//high address
#define SET_ADDR_BASE_H		(15+DATA_RNW_W)				//high address
#define ENABLE_CL			(16+DATA_RNW_W)		//enable CL control
#define GET_DATE			(17+DATA_RNW_R)
#define DELAY_START			(18+DATA_RNW_W)
#define DELAY_NUM			(19+DATA_RNW_W)
//
#define SET_X_NUM			(20+DATA_RNW_W)	//set x pixel number
#define SET_Y_NUM			(21+DATA_RNW_W)	//set y pixel number
#define GET_X_NUM			(20+DATA_RNW_R)	//get x pixel number
#define GET_Y_NUM			(21+DATA_RNW_R)	//get y pixel number
#define SET_SCAN_START		(22+DATA_RNW_W)
#define SET_DELAY_DESCENT	(23+DATA_RNW_W)
#define SET_DELAY_PIXEL		(24+DATA_RNW_W)
#define READ_ALL_VADC		(25+DATA_RNW_W)

#define SET_PL_PERIOD		(27+DATA_RNW_W) //60 Hz(16667) or 50 Hz(20000)
#define SET_DIO				(29+DATA_RNW_W)
#define WAIT_EOC			(30+DATA_RNW_W)	//video ADC wait EOC
#define SET_Y_START			(28+DATA_RNW_W)
#define SET_Y_END			(31+DATA_RNW_W)
//
#define SET_X_NDX_INC		(32+DATA_RNW_W)	//auto focus x increment
#define SET_Y_NDX_INC		(33+DATA_RNW_W)	//auto focus y increment
//
#define GET_VERSION			(34+DATA_RNW_R)	//get FPGA version
#define GET_VADC0			(36+DATA_RNW_R)
#define GET_VADC1			(37+DATA_RNW_R)
#define GET_VADC2			(38+DATA_RNW_R)
#define GET_VADC3			(39+DATA_RNW_R)
//
#define SET_RELAY_ON		(34+DATA_RNW_W)
#define SET_SENR_SEL		(35+DATA_RNW_W)
#define SET_SHUNT			(36+DATA_RNW_W)
#define SET_SEL_COIL		(37+DATA_RNW_W)	//select small coil
#define SET_OBJ_ON			(38+DATA_RNW_W)
#define SET_VOP_ON			(39+DATA_RNW_W)
//
#define EN_EDS_HANDSHAKE 	(40+DATA_RNW_W)
#define GET_DO			 	(40+DATA_RNW_R)
//
#define SET_X_INIT			(41+DATA_RNW_W)
#define RESET_VADC			(42+DATA_RNW_W)	//reset video ADC
#define GET_PL_NDX			(43+DATA_RNW_R)
//
#define GET_VADC_HDIFF		(44+DATA_RNW_R)	//get vertical difference
#define GET_VADC_VDIFF		(45+DATA_RNW_R)	//get horizontal difference
#define GET_VADC_SUM		(46+DATA_RNW_R)	//get total sum
//
#define VADC_CONV_START		(47+DATA_RNW_W)
#define SET_AVG_NUM			(48+DATA_RNW_W)
#define GET_VADC_MAX		(49+DATA_RNW_R)
#define GET_VADC_MIN		(50+DATA_RNW_R)
#define SET_IMAGE_INIT		(51+DATA_RNW_W)
//
#define GET_GRADIENT_L		(52+DATA_RNW_R)
#define GET_GRADIENT_H		(53+DATA_RNW_R)
#define GET_VADC4			(54+DATA_RNW_R)
#define GET_VADC5			(55+DATA_RNW_R)
//
#define SET_DELAY_OVERSCAN	(54+DATA_RNW_W)
#define SET_DB3_DEV			(55+DATA_RNW_W)	//set databus3 device
#define SET_X_LINE_DELAY	(56+DATA_RNW_W)	//set databus3 device
//
#define SET_GRADIENT_DIFF	(60+DATA_RNW_W)
#define SET_IRQ_ACK			(61+DATA_RNW_W)		//handshake for irq process
//#define ENABLE_VIDEO_ADC	(62+DATA_RNW_W)
//#define ENABLE_VIDEO_CLK	(63+DATA_RNW_W)
//
//#define		MA_PER_MM		150		//300 mA -> 2 mm deflector
#define		A_PER_MM_X0		0.22	//COIL_SHIFT=0,0.3 A -> 2 mm deflector
#define		A_PER_MM_Y0		0.22
#define		A_PER_MM_X1		1.2		//COIL_SHIFT=1
#define		A_PER_MM_Y1		1.2
#define		ZOOM_RANGE_MAX		3		//one-side range, 2 mm, from -ZOOM_RANGE_MAX to +ZOOM_RANGE_MAX
#define		ZOOM_RANGE_MIN		0.008	//8 um
//
#define 	DAC_REFN		-10		//negative reference voltage for DAC
#define 	DAC_REFP		10		//positive reference voltage for DAC
#define 	ADC_IN_MAX		10		//general purpose ADC range
#define 	ADC_IN_MIN		-10
//#define DAC_OUT_MAX		10		//
//#define DAC_OUT_MIN		-10
/*
#define DACF_OUT_MAX	5.165		//deflector fine scan range
#define DACF_OUT_MIN	-5.165		//deflector fine scan range
#define DACF_PV			5.0		//plus +5V, can't use 5, must use 5.0
#define DACF_MV			-5.0		//minus -5V
*/
#define DACF_OUT_MAX	10.165		//deflector fine scan range
#define DACF_OUT_MIN	-10.165		//deflector fine scan range
#define DACF_REF		10.0
#define DACF_PV			10.0		//plus +5V, can't use 5, must use 5.0
#define DACF_MV			-10.0		//minus -5V
//------------------------------------------------------
//for VDAC 10V operation,
//scan board
//R193 = 10 kohm
//D25,D26,D27,D28,D2,D3,D4,D5 change to BZT52C10V-SOD123
//coil board
//R156,R170 = 6.8 kohm
//#define DEF_V_RATIO		(DEF_R/(6.8+DEF_R))		//deflector voltage set ratio
//power board +-VOPC change to 13.5 V
//R166,R172=5.1K
//------------------------------------------------------
//for VDAC 5V operation,
//R193 = 0 ohm
//D25,D26,D27,D28,D2,D3,D4,D5 change to BZT52C5V6-SOD123
//coil board
//R156,R170 = 3 kohm
//#define DEF_V_RATIO		(DEF_R/(3+DEF_R))		//deflector voltage set ratio
//power board +-VOPC change to 12 V
//R166,R172=3.6K
//------------------------------------------------------
//
//DEF_V_RATIO need to be changed
//
//#define FINE_CH_NUM		3		//DEFX, DEFY, OBJ
//#define FINE_CH_DEFX		0
//#define FINE_CH_DEFY		1
//#define FINE_CH_OBJ		2
//
#define TIMER_TASK_TICK		4
#define DAC_12BIT			4096	//image DAC, general purpose DAC, 0 ~ 4095
#define DAC_12BIT_D2		2048
#define DAC_14BIT			16384
#define DAC_16BIT			65536
#define ADC_16BIT			65536	//general purpose ADC, 0 ~ 65535
#define ADC_12BIT			4096	//image ADC
//
#if VIDEO_ADC_BITN == 16 //generate simulation data
#define VIDEO_ADC_MAX		0xFFFF	//12-bit video ADC
#define VIDEO_ADC_TOP		0xC000	//3076
#define VIDEO_ADC_MID		0x8000	//2048
#define VIDEO_ADC_BOTTOM	0x4000	//1024
#else
#define VIDEO_ADC_MAX		0x1000	//12-bit video ADC
#define VIDEO_ADC_TOP		0x0C00	//3076
#define VIDEO_ADC_MID		0x0800	//2048
#define VIDEO_ADC_BOTTOM	0x0400	//1024
#endif
//
#define X_AXIS		1
#define Y_AXIS		2
//
#define DIR_POS		1	//forward, x increment
#define DIR_NEG		2	//backward, x decrement
//
#define PITR_FREQ		10
#define PITR_INTERVAL	100 //=(1000/PITR_FREQ)
//
#define ADC_SKIP_NUM	0	//
//
#define 	AT_ADDR_BASE		1
#define 	AT_RAM_ADDR_IN		2
#define		AT_RAM_ADDR_OUT		3
#define		AT_RAM_ADDR_IO_MAX	4
#define		RAM_ADDR_BITN		22 //A0~A21
//
//#define ADC_PIPELINE_CLKNUM		3
//
#define SCAN_C2FR_1		(4.7/4.7)			//4.7/4.7, 1:1, resistor Coarse To Fine Ratio (C2FR)
#define SCAN_C2FR_2		(27/4.7)		//27/4.7
#define SCAN_C2FR_3		(100/4.7)		//100/4.7
#define SCAN_C2FR_4		(470/4.7)		//470/4.7
#define SCAN_R1R2			2			//10 ohms, 5 ohms
#define DEF_R1_R2_RATIO		2			//resistor value ratio
//------------------------------------------------------
// coarse and fine voltage ratio
//------------------------------------------------------
// 0.5A -> fAPerMM
#define SCAN_RATIO_1	((float)2/(1+SCAN_C2FR_1))		//1,x80
#define SCAN_RATIO_2	((float)2/(1+SCAN_C2FR_2))		//0.35088 --> 1 mm,x228
#define SCAN_RATIO_3	((float)2/(1+SCAN_C2FR_3))		//0.18182 --> 0.54 mm,x440
#define SCAN_RATIO_4	((float)2/(1+SCAN_C2FR_4))		//0.04166 --> 0.1 mm,x1,920
#define SCAN_RATIO_5	((float)2/(1+SCAN_C2FR_5))		//0.018182 --> x4,400
#define SCAN_RATIO_6	((float)2/(1+SCAN_C2FR_6))		//0.004246 --> x19,200
#define SCAN_RATIO_0   (SCAN_RATIO_1*DEF_R1_R2_RATIO)					//use 5 ohm
//#define SCAN_MAG_RATIO	10
//
#define OBJ_C2FR_1		10	//objective
#define OBJ_C2FR_2		20
#define OBJ_C2FR_3		33
#define OBJ_C2FR_4		47
#define OBJ_RATIO_1		((float)2/(1+OBJ_C2FR_1))		//2/11
#define OBJ_RATIO_2		((float)2/(1+OBJ_C2FR_2))		//2/21
#define OBJ_RATIO_3		((float)2/(1+OBJ_C2FR_3))		//2/33
#define OBJ_RATIO_4		((float)2/(1+OBJ_C2FR_4))		//2/48
//
#define LED_NUM		4
//bLEDState[] index
#define LEDG		0	//green, power on/off
#define LEDR		1	//red, HV module on/off
#define LEDB		2	//blue, vacuum state
#define	 LEDY		3	//yellow, ready
//
#define ST_LED_DARK			0	//0000
#define ST_LED_SOLID		1	//1111
#define ST_LED_BLINK_1HZ	2	//1100, 1Hz, 1100 = 0011
#define ST_LED_BLINK_2HZ	3	//0101 = 1010, 2 Hz
#define ST_LED_MAX			3
//
#define ST_LED_BLINK_1P4	4	//LO,0001 (DC=1/4)
#define ST_LED_BLINK_2P4	5	//HI,0011 (DC=2/4)
#define ST_LED_BLINK_3AP4	6	//MH,1011 (DC=3/4)
#define ST_LED_BLINK_3P4	7	//UH,0111 (DC=/4)
#define ST_LED_ALL			8	//ALL,1111
#define ST_LED_NUM_MAX		9
// GET_STATUS flag
//#define ST_EN_VADC 				0x0010
#define ST_GET_VIDEO_ADC		0x0002
#define ST_EDS_RX0				0x0010	//P41
#define ST_DOUT_IO				0x0008	//IO1, IO2, HiZ if nCS=H
#define ST_SEM_SCANNING			0x0010
#define ST_ROW_DATA_READY		0x0020
#define ST_SCANNING				0x0080
//#define ST_ROW_DATA_GOT			0x0200
#define ST_RUNNING				0x4000	//(ST_GET_VIDEO_ADC|ST_SCANNING)
#define ST_VADC_CONV_START		0x8000
/*
//DAC control
step_mode_w = databus1[0]; 				//0, 1 step by step
r_nw_dac_gp = databus1[1];				//1
n_cs_dac_gp[0] = databus1[2];			//1
n_cs_dac_gp[1] = databus1[3];			//1, default 0x000E
n_cs_dacf_s_buf = databus1[4];			//1, video DAC
dacf_ch_s_buf = databus1[5];			//0, video DAC
//ADC control
step_mode_r = databus1[0]; 				//0, 1 step by step
n_cs_adc_gp[0] = databus1[1];			//1
n_cs_adc_gp[1] = databus1[2];			//1
n_oe_adc_s_buf = databus1[3];			//1, video ADC
n_oe_adc_video_s_buf = databus1[4];		//1, video ADC
//SET_DIO_PIN
step_mode_w = databus1[0]; 				//0, 1 step by step
do_le_s_buf = databus1[1];				//0
step_mode_r = databus1[2]; 				//0, 1 step by step
n_oe_di_s_buf = databus1[3]; 			//1, make data ready at databus2
*/
#define STEP_MODE_DISABLE	0x0000
//
typedef struct UDP_CLIENT
{
	IPADDR iaSource;
	int nSrcPort;		//source port
	int nDestPort;		//destination port
} UDP_CLIENT;
//one line, max 1024*2 = 2048 bytes
//
#endif /*CPU_CMD_*/
