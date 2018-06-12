#ifndef _ERROR_MSG_H_
#define _ERROR_MSG_H_

// =======================================================================
// COMMAND
// =======================================================================
#define ERR_CMD_UNDEF 						0x00010000
#define ERR_ACCKV_INV_PARAM_NUM 			0x00010001
#define ERR_ACCKV_INV_PARAM1_VAL 			0x00010002
#define ERR_ACCKV_INV_PARAM2_VAL 			0x00010003
#define ERR_ACK_INV_PARAM_VAL 				0x00010004
#define ERR_SCANPAUSE_INV_PARAM_NUM 		0x00010005
#define ERR_SCANPAUSE_INV_PARAM_VAL 		0x00010006
#define ERR_SCANSTOP_INV_PARAM_NUM 			0x00010007
#define ERR_SCANSTOP_INV_PARAM_VAL 			0x00010008
#define ERR_DACT_INV_PARAM_NUM 				0x00010009
#define ERR_DACT_INV_PARAM1_VAL 			0x0001000A
#define ERR_FILACNST_INV_PARAM_NUM 			0x0001000B
#define ERR_FILACNST_INV_PARAM_VAL 			0x0001000C
#define ERR_ADDR_RW_INV_PARAM_NUM 			0x0001000D
#define ERR_APERMM_INV_PARAM_NUM 			0x0001000E
#define ERR_APERMM_INV_PARAM1_VAL 			0x0001000F
#define ERR_APERMM_INV_PARAM2_VAL 			0x00010010
#define ERR_APERMM_INV_PARAM3_VAL 			0x00010011
#define ERR_AUTOVAC_INV_PARAM_NUM 			0x00010012
#define ERR_AUTOVAC_INV_PARAM_VAL 			0x00010013
#define ERR_BLANKING_INV_PARAM_NUM 			0x00010014
#define ERR_BLANKING_INV_PARAM_VAL 			0x00010015
#define ERR_BAUD_INV_PARAM_NUM 				0x00010016
#define ERR_BAUD_INV_PARAM1_VAL 			0x00010017
#define ERR_BAUD_INV_PARAM2_VAL 			0x00010018
#define ERR_BINPKT_INV_PARAM_NUM 			0x00010019
#define ERR_BINPKT_INV_PARAM_VAL 			0x0001001A
#define ERR_BRSEL_INV_PARAM_NUM 			0x0001001B
#define ERR_BSEMODE_INV_PARAM_NUM 			0x0001001C
#define ERR_BSEMODE_INV_PARAM_VAL 			0x0001001D
#define ERR_CALVID_INV_PARAM_NUM 			0x0001001E
#define ERR_CHKTP_INV_PARAM_NUM 			0x0001001F
#define ERR_CHKTP_INV_PARAM_VAL 			0x00010020
#define ERR_CSHUNT_INV_PARAM_NUM 			0x00010021
#define ERR_CSHUNT_INV_PARAM1_VAL 			0x00010022
#define ERR_DEBUG_INV_PARAM_NUM 			0x00010023
#define ERR_DEBUG_INV_PARAM_VAL 			0x00010024
#define ERR_DELAY_INV_PARAM_NUM 			0x00010025
#define ERR_DELAY_INV_PARAM1_VAL 			0x00010026
#define ERR_DELAY_INV_PARAM2_VAL 			0x00010027
#define ERR_DELAY_INV_PARAM3_VAL 			0x00010028
#define ERR_ENEDS_INV_PARAM_NUM 			0x00010029
#define ERR_ENEDS_INV_PARAM_VAL 			0x0001002A
#define ERR_ENFIFO_INV_PARAM_NUM 			0x0001002B
#define ERR_ENFIFO_INV_PARAM_VAL 			0x0001002C
#define ERR_EDSDWT_INV_PARAM_NUM 			0x0001002D
#define ERR_EDSDWT_INV_PARAM_VAL 			0x0001002E
#define ERR_EDSHS_INV_PARAM_NUM 			0x0001002F
#define ERR_EDSHS_INV_PARAM_VAL 			0x00010030
#define ERR_EDSONHW_INV_PARAM_NUM 			0x00010031
#define ERR_EDSONHW_INV_PARAM_VAL 			0x00010032
#define ERR_ENCL_INV_PARAM_NUM 				0x00010033
#define ERR_ENCL_INV_PARAM_VAL 				0x00010034
#define ERR_ENCOILSW_INV_PARAM_NUM 			0x00010035
#define ERR_ENCOILSW_INV_PARAM_VAL 			0x00010036
#define ERR_ENVENT_INV_PARAM_NUM 			0x00010037
#define ERR_ENVENT_INV_PARAM_VAL 			0x00010038
#define ERR_ENVADC_INV_PARAM_NUM 			0x00010039
#define ERR_ENVADC_INV_PARAM_VAL 			0x0001003A
#define ERR_ENWDT_INV_PARAM_VAL 			0x0001003B
#define ERR_ERROR_NOT_QUESTIONMARK_OR_0 	0x0001003C
#define ERR_FASTCLOCK_INV_PARAM_NUM 		0x0001003D
#define ERR_FASTCLOCK_INV_PARAM_VAL 		0x0001003E
#define ERR_FOCUS_INV_PARAM_NUM 			0x0001003F
#define ERR_FOCUS_INV_PARAM1_VAL 			0x00010040
#define ERR_FOCUS_INV_PARAM2_VAL 			0x00010041
#define ERR_FOCUS_INV_PARAM3_VAL 			0x00010042
#define ERR_FREQGEN_INV_PARAM_NUM 			0x00010043
#define ERR_FREQGEN_INV_PARAM1_VAL 			0x00010044
#define ERR_GETAI_INV_PARAM_NUM 			0x00010045
#define ERR_GETAI_INV_PARAM2_VAL 			0x00010046
#define ERR_GETAII_INV_PARAM_NUM 			0x00010047
#define ERR_GETAII_INV_PARAM_VAL 			0x00010048
#define ERR_GETSET_INV_PARAM_NUM 			0x00010049
#define ERR_GETSET_INV_PARAM_VAL 			0x0001004A
#define ERR_GV_INV_PARAM_NUM 				0x0001004B
#define ERR_GV_INV_PARAM1_VAL 				0x0001004C
#define ERR_GV_INV_PARAM2_VAL 				0x0001004D
#define ERR_HISTO_INV_PARAM_NUM 			0x0001004E
#define ERR_HISTO_INV_PARAM_VAL 			0x0001004F
#define ERR_HISTOCH_INV_PARAM_NUM 			0x00010050
#define ERR_HISTOCH_INV_PARAM_VAL 			0x00010051
#define ERR_HVACC_INV_PARAM_NUM 			0x00010052
#define ERR_HVBIAS_INV_PARAM_NUM 			0x00010053
#define ERR_HVON_INV_PARAM_NUM 				0x00010054
#define ERR_HVON_INV_PARAM_VAL 				0x00010055
#define ERR_HVMAX_INV_PARAM_NUM 			0x00010056
#define ERR_HVMAX_INV_PARAM1_VAL 			0x00010057
#define ERR_HVPARAM_INV_PARAM_NUM 			0x00010058
#define ERR_HVPARAM2_INV_PARAM_NUM 			0x00010059
#define ERR_HVPARAM2_INV_PARAM1_VAL 		0x0001005A
#define ERR_HVTYPE_INV_PARAM_NUM 			0x0001005B
#define ERR_HVTYPE_INV_PARAM_VAL 			0x0001005C
#define ERR_IDLET_INV_PARAM_NUM 			0x0001005D
#define ERR_IMGNUM_INV_PARAM_NUM 			0x0001005E
#define ERR_IMGNUM_INV_PARAM_VAL 			0x0001005F
#define ERR_IO_INV_PARAM_NUM 				0x00010060
#define ERR_IOINV_INV_PARAM_NUM 			0x00010061
#define ERR_IOINV_INV_PARAM_VAL 			0x00010062
#define ERR_IPLEAK_INV_PARAM_NUM 			0x00010063
#define ERR_IPLEAK_INV_PARAM1_VAL 			0x00010064
#define ERR_IPLEAK_INV_PARAM2_VAL 			0x00010065
#define ERR_IPON_INV_PARAM_NUM 				0x00010066
#define ERR_IPON_INV_PARAM_VAL 				0x00010067
#define ERR_MOTOGV_INV_PARAM_NUM 			0x00010068
#define ERR_IPTYPE_INV_PARAM_NUM 			0x00010069
#define ERR_IPTYPE_INV_PARAM_VAL 			0x0001006A
#define ERR_IRQACK_INV_PARAM_NUM 			0x0001006B
#define ERR_IRQACK_INV_PARAM_VAL 			0x0001006C
#define ERR_LEDAUTO_INV_PARAM_NUM 			0x0001006D
#define ERR_LEDAUTO_INV_PARAM_VAL 			0x0001006E
#define ERR_LED_INV_PARAM_NUM 				0x0001006F
#define ERR_LED_INV_PARAM1_VAL 				0x00010070
#define ERR_LED_INV_PARAM2_VAL 				0x00010071
#define ERR_LEDON_INV_PARAM_NUM 			0x00010072
#define ERR_LEDON_INV_PARAM_VAL 			0x00010073
#define ERR_LEDTEST_INV_PARAM_NUM 			0x00010074
#define ERR_LEDTEST_INV_PARAM_VAL 			0x00010075
#define ERR_MAG_INV_PARAM_NUM 				0x00010076
#define ERR_MAG_INV_PARAM_VAL 				0x00010077
#define ERR_MAG_SETMAGNIFICATION_FAIL 		0x00010078
#define ERR_MRESET_INV_PARAM_NUM 			0x00010079
#define ERR_MRESET_INV_PARAM_VAL 			0x0001007A
#define ERR_NOKTH_INV_PARAM_NUM 			0x0001007B
#define ERR_NOKTH_INV_PARAM1_VAL 			0x0001007C
#define ERR_OBJMODE_INV_PARAM_NUM 			0x0001007D
#define ERR_OBJMODE_INV_PARAM_VAL 			0x0001007E
#define ERR_OBJON_INV_PARAM_NUM 			0x0001007F
#define ERR_OBJON_INV_PARAM_VAL 			0x00010080
#define ERR_OBJIMAX_INV_PARAM_NUM 			0x00010081
#define ERR_OBJIMAX_INV_PARAM1_VAL 			0x00010082
#define ERR_OBJIMAX_INV_PARAM2_VAL 			0x00010083
#define ERR_OBJIMIN_INV_PARAM_NUM 			0x00010084
#define ERR_OBJIMIN_INV_PARAM1_VAL 			0x00010085
#define ERR_OBJIMIN_INV_PARAM2_VAL 			0x00010086
#define ERR_OBJR_INV_PARAM_NUM 				0x00010087
#define ERR_OBJR_INV_PARAM_VAL 				0x00010088
#define ERR_OBJSTBYR_INV_PARAM_NUM 			0x00010089
#define ERR_OBJSTBYR_INV_PARAM_VAL 			0x0001008A
#define ERR_ONEPKT_INV_PARAM_NUM 			0x0001008B
#define ERR_OVERSCAN_INV_PARAM_NUM 			0x0001008C
#define ERR_OVERSCAN_INV_PARAM1_VAL 		0x0001008D
#define ERR_OVERSCAN_INV_PARAM2_VAL 		0x0001008E
#define ERR_PIDGET_INV_PARAM_VAL 			0x0001008F
#define ERR_PIDSET_INV_PARAM_VAL 			0x00010090
#define ERR_PLSET_INV_PARAM_NUM 			0x00010091
#define ERR_PLSET_INV_PARAM1_VAL 			0x00010092
#define ERR_PLSET_INV_PARAM2_VAL 			0x00010093
#define ERR_POINTSET_INV_PARAM_NUM 			0x00010094
#define ERR_POINTSET_INV_PARAM1_VAL 		0x00010095
#define ERR_POINTSET_INV_PARAM2_VAL 		0x00010096
#define ERR_POS_INV_PARAM_NUM 				0x00010097
#define ERR_POS_INV_PARAM1_VAL 				0x00010098
#define ERR_POS_INV_PARAM2_VAL 				0x00010099
#define ERR_POINTMODE_INV_PARAM_NUM 		0x0001009A
#define ERR_POINTMODE_INV_PARAM_VAL 		0x0001009B
#define ERR_PUMPON_INV_PARAM_NUM 			0x0001009C
#define ERR_PUMPON_INV_PARAM1_VAL 			0x0001009D
#define ERR_PUMPON_INV_PARAM2_VAL 			0x0001009E
#define ERR_PWRON_INV_PARAM_NUM 			0x0001009F
#define ERR_PWRON_INV_PARAM1_VAL 			0x000100A0
#define ERR_PWRON_INV_PARAM2_VAL 			0x000100A1
#define ERR_RAMDUMP_INV_PARAM_NUM 			0x000100A2
#define ERR_RAMSET_INV_PARAM_NUM 			0x000100A3
#define ERR_RAMTEST_INV_PARAM_NUM 			0x000100A4
#define ERR_REGION_INV_PARAM_NUM 			0x000100A5
#define ERR_REGION_INV_PARAM1_VAL 			0x000100A6
#define ERR_REGION_INV_PARAM2_VAL 			0x000100A7
#define ERR_RETURN_INV_PARAM_NUM 			0x000100A8
#define ERR_RETURN_INV_PARAM_VAL 			0x000100A9
#define ERR_RESEND_INV_PARAM_NUM 			0x000100AA
#define ERR_RESET_INV_PARAM_NUM 			0x000100AB
#define ERR_RESET_INV_PARAM_VAL 			0x000100AC
#define ERR_ROTATE_INV_PARAM_NUM 			0x000100AD
#define ERR_ROTATE_INV_PARAM_VAL 			0x000100AE
#define ERR_SCALE_INV_PARAM_NUM 			0x000100AF
#define ERR_SCALE_INV_PARAM_VAL 			0x000100B0
#define ERR_SCANSTART_INV_PARAM_NUM 		0x000100B1
#define ERR_SCANSTART_INV_PARAM1_VAL 		0x000100B2
#define ERR_SCANSTART_INV_PARAM2_VAL 		0x000100B3
#define ERR_SETAO_INV_PARAM_NUM 			0x000100B4
#define ERR_SETAO_INV_PARAM1_VAL 			0x000100B5
#define ERR_SETAO_INV_PARAM2_VAL 			0x000100B6
#define ERR_SETAO_INV_PARAM3_VAL 			0x000100B7
#define ERR_SETAOIW_INV_PARAM_NUM 			0x000100B8
#define ERR_SETAOI_INV_PARAM_NUM 			0x000100B9
#define ERR_SETAOI_INV_PARAM1_VAL 			0x000100BA
#define ERR_SETAOI_INV_PARAM2_VAL 			0x000100BB
#define ERR_SETAVGNUM_INV_PARAM_NUM			0x000100BC
#define ERR_SETAVGNUM_INV_PARAM_VAL			0x000100BD
#define ERR_SETBRX_INV_PARAM_NUM 			0x000100BE
#define ERR_SETBRX_INV_PARAM1_VAL 			0x000100BF
#define ERR_SETBRX_INV_PARAM2_VAL 			0x000100C0
#define ERR_SETBR_INV_PARAM_NUM 			0x000100C1
#define ERR_SETBR_INV_PARAM_VAL 			0x000100C2
#define ERR_SETCI_INV_PARAM_NUM 			0x000100C3
#define ERR_SETCOX_INV_PARAM_NUM 			0x000100C4
#define ERR_SETCOX_INV_PARAM1_VAL 			0x000100C5
#define ERR_SETCOX_INV_PARAM2_VAL 			0x000100C6
#define ERR_SETCO_INV_PARAM_NUM 			0x000100C7
#define ERR_SETCO_INV_PARAM_VAL 			0x000100C8
#define ERR_SENRSEL_INV_PARAM_NUM 			0x000100C9
#define ERR_SENRSEL_INV_PARAM_VAL 			0x000100CA
#define ERR_SETDO_INV_PARAM_NUM 			0x000100CB
#define ERR_SETDO_INV_PARAM1_VAL 			0x000100CC
#define ERR_SETDOB_INV_PARAM_NUM 			0x000100CD
#define ERR_SETDOB_INV_PARAM1_VAL 			0x000100CE
#define ERR_SETDOB_INV_PARAM2_VAL 			0x000100CF
#define ERR_SETDOB_INV_PARAM3_VAL 			0x000100D0
#define ERR_SETHV_INV_PARAM_NUM 			0x000100D1
#define ERR_SETLDR_INV_PARAM_NUM 			0x000100D2
#define ERR_SETOBJ_INV_PARAM_NUM 			0x000100D3
#define ERR_SETOBJ_INV_PARAM_VAL 			0x000100D4
#define ERR_SETOFSAD_INV_PARAM_NUM 			0x000100D5
#define ERR_SETOFSAD_INV_PARAM1_VAL 		0x000100D6
#define ERR_SETOFSAD_INV_PARAM2_VAL 		0x000100D7
#define ERR_SETOFSAD_INV_PARAM3_VAL 		0x000100D8
#define ERR_SETOFSDA_INV_PARAM_NUM 			0x000100D9
#define ERR_SETOFSDA_INV_PARAM1_VAL 		0x000100DA
#define ERR_SETOFSDA_INV_PARAM2_VAL 		0x000100DB
#define ERR_SETOFSDA_INV_PARAM3_VAL 		0x000100DC
#define ERR_SETSLPDA_INV_PARAM_NUM 			0x000100DD
#define ERR_SETSLPDA_INV_PARAM1_VAL 		0x000100DE
#define ERR_SETSLPDA_INV_PARAM2_VAL 		0x000100DF
#define ERR_SETSLPDA_INV_PARAM3_VAL 		0x000100E0
#define ERR_SETPITR_INV_PARAM_NUM 			0x000100E1
#define ERR_SETPITR_INV_PARAM_VAL 			0x000100E2
#define ERR_SETPIXN_INV_PARAM_NUM 			0x000100E3
#define ERR_SETPIXN_INV_PARAM1_VAL 			0x000100E4
#define ERR_SETPIXN_INV_PARAM2_VAL 			0x000100E5
#define ERR_SETR_INV_PARAM_NUM 				0x000100E6
#define ERR_SETR_INV_PARAM1_VAL 			0x000100E7
#define ERR_SETR_INV_PARAM2_VAL 			0x000100E8
#define ERR_SETSTIGX_INV_PARAM_NUM 			0x000100E9
#define ERR_SETSTIGX_INV_PARAM_VAL 			0x000100EA
#define ERR_SETSTIGY_INV_PARAM_NUM 			0x000100EB
#define ERR_SETSTIGY_INV_PARAM_VAL 			0x000100EC
#define ERR_SETV2IR_INV_PARAM_NUM 			0x000100ED
#define ERR_SETZOOM_INV_PARAM_NUM 			0x000100EE
#define ERR_SETZOOM_INV_PARAM3_VAL 			0x000100EF
#define ERR_SETZOOM_INV_PARAM4_VAL 			0x000100F0
#define ERR_SETZOOM_INV_PARAM5_VAL 			0x000100F1
#define ERR_SETZOOM_INV_PARAM_IS_SCANNING 	0x000100F2
#define ERR_SHADOW_INV_PARAM_NUM 			0x000100F3
#define ERR_SHADOW_INV_PARAM_VAL 			0x000100F4
#define ERR_SIMUVAC_INV_PARAM_NUM 			0x000100F5
#define ERR_SIMUVAC_INV_PARAM1_VAL 			0x000100F6
#define ERR_SIMU_INV_PARAM_NUM 				0x000100F7
#define ERR_SIMU_INV_PARAM_VAL 				0x000100F8
#define ERR_SO_INV_PARAM_NUM 				0x000100F9
#define ERR_SO_INV_PARAM1_VAL 				0x000100FA
#define ERR_SO_INV_PARAM2_VAL 				0x000100FB
#define ERR_SOE_INV_PARAM_NUM 				0x000100FC
#define ERR_SOE_INV_PARAM_VAL 				0x000100FD
#define ERR_START_INV_PARAM_VAL 			0x000100FE
#define ERR_SYSCLK_INV_PARAM_NUM 			0x000100FF
#define ERR_SYSCLK_INV_PARAM_VAL 			0x00010100
#define ERR_TEST_INV_PARAM_NUM 				0x00010101
#define ERR_TEST_INV_PARAM_VAL 				0x00010102
#define ERR_TEMPLIM_INV_PARAM_NUM 			0x00010103
#define ERR_TEMPLIM_INV_PARAM1_VAL 			0x00010104
#define ERR_TEMPLIM_INV_PARAM2_VAL 			0x00010105
#define ERR_TEMPLIM_INV_PARAM3_VAL 			0x00010106
#define ERR_TURBO_INV_PARAM_NUM 			0x00010107
#define ERR_TURBO_INV_PARAM1_VAL 			0x00010108
#define ERR_TURBO_INV_PARAM2_VAL 			0x00010109
#define ERR_TURBOSP_INV_PARAM_NUM 			0x0001010A
#define ERR_TURBOSP_INV_PARAM1_VAL 			0x0001010B
#define ERR_TURBOT_INV_PARAM_NUM 			0x0001010C
#define ERR_TURBOT_INV_PARAM_VAL 			0x0001010D
#define ERR_UP2DN_INV_PARAM_NUM 			0x0001010E
#define ERR_UP2DN_INV_PARAM_VAL 			0x0001010F
#define ERR_UDPPORT_INV_PARAM_NUM 			0x00010110
#define ERR_UDPPORT_INV_PARAM_VAL 			0x00010111
#define ERR_VACERR_INV_PARAM_NUM 			0x00010112
#define ERR_VACOK_INV_PARAM_NUM 			0x00010113
#define ERR_VACOK_INV_PARAM1_VAL 			0x00010114
#define ERR_VACOK_INV_PARAM2_VAL 			0x00010115
#define ERR_VACON_INV_PARAM_NUM 			0x00010116
#define ERR_VACON_INV_PARAM_VAL 			0x00010117
#define ERR_VACPARA_INV_PARAM_NUM 			0x00010118
#define ERR_VACPARA_INV_PARAM1_VAL 			0x00010119
#define ERR_VIEWW_INV_PARAM_NUM 			0x0001011A			
#define ERR_VIEWW_INV_PARAM_VAL 			0x0001011B
#define ERR_WAITEOC_INV_PARAM_NUM 			0x0001011C
#define ERR_WAITEOC_INV_PARAM1_VAL 			0x0001011D
#define ERR_WAITEOC_INV_PARAM2_VAL 			0x0001011E

// =======================================================================
// VACUUM
// =======================================================================
#define ERR_VAC_UNDEF 0x00020000
// =======================================================================
// COMMUNICATION
// =======================================================================
#define ERR_COM_UNDEF 0x00030000
// =======================================================================
// PERIPHERAL
// =======================================================================
#define ERR_PER_UNDEF 0x00040000
// =======================================================================
// SOFTWARE
// =======================================================================
#define ERR_SW_UNDEF 0x00050000

class ErrorMsg
{
  private:
    static const char* ERR_CMD_TXT[];
    static const char* ERR_VAC_TXT[];
    static const char* ERR_COM_TXT[];
    static const char* ERR_PER_TXT[];
    static const char* ERR_SW_TXT[];

    static const unsigned int ERR_TYPE_MASK = 0x000F0000;
    static const unsigned int ERR_VAL_MASK = 0x0000FFFF;
    static const unsigned int ERR_CMD  = 0x00010000; // command
    static const unsigned int ERR_VAC  = 0x00020000; // vacuum
    static const unsigned int ERR_COM  = 0x00030000; // communication
    static const unsigned int ERR_PER  = 0x00040000; // peripheral
    static const unsigned int ERR_SW   = 0x00050000; // software
  public:
    static const char* GetErrorText(unsigned int dwErrCode);
};


#endif
