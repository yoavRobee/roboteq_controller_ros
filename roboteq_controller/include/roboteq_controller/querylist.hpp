#ifndef __querylist__
#define __querylist__
//#include <std_msgs/String.h>
//////////////////////////// RUNTIME COMMANDS ////////////////////////////
/*#define _G				0
#define _GO				0
#define _M				1
#define _MOTCMD			1
#define _P				2
#define _MOTPOS			2
#define _S				3
#define _MOTVEL			3
#define _C				4
#define _SENCNTR		4
#define _CB				5
#define _VAR			6
#define _SBLCNTR		5
#define _AC				7
#define _ACCEL			7
#define _DC				8
#define _DECEL			8
#define _DS				9
#define _DOUT			9
#define _D1				10
#define _DSET			10
#define _D0				11
#define _DRES			11
#define _R				12
#define _H				13
#define _HOME			13
#define _EX				14
#define _ESTOP			14
#define _MG				15
#define _MGO			15
#define _MS				16
#define _MSTOP			16
#define _PR				17
#define _MPOSREL		17
#define _PX				18
#define _NXTPOS			18
#define _PRX			19
#define _NXTPOSR		19
#define _AX				20
#define _NXTACC			20
#define _DX				21
#define _NXTDEC			21
#define _B				22
#define _BOOL			22
#define _SX				23
#define _NXTVEL			23
#define _CS				24
#define _CANSEND		24
#define _CG				25
#define _CANGO			25
#define _RC				26
#define _RCOUT			26
#define _EES			27
#define _EESAV			27
#define _BND			28
#define _BIND			28
#define _AO				29
#define _AOUT			29
#define _TX				30
#define _LEFT			30
#define _TV				31
#define _RIGHT			31
#define _CSW			32
#define _CELLSW			32
#define _PSW			33
#define _BMSPSW			33
#define _ASW			34
#define _BMSASW			34
#define _HS				49
#define _ANG			65
#define _ANGLE			65
#define _CW				86
#define _SWS			81
#define _SPL			87
#define _SAC			88
#define _SDC			89
#define _ROM			90
#define _TC				91
#define _POS			92
#define _PSP			93
#define _PAC			94
#define _PDC			95
#define _TSL			96
#define _CSS			108
#define _STT			112
#define _PXL			117
#define _ZER			121
#define _MM				120
#define _GIQ			122
#define _GID			123
#define _SCO			124
#define _VPM			125
#define _MKM			131
#define _MKO			132
#define _MKP			133
#define _MKD			134
#define _MKT			135
*/
//////////////////////////// RUNTIME QUERIES ////////////////////////////
//std::string ?A;
#define motor_amps	"?A"
#define Motor_Command_Applied	"?M"
#define motor_power	?P
#define motor_speed	?S				

/*#define ?A				0
#define ?M				1
#define ?P				2
#define ?S				3
#define _C				4
#define _ABCNTR			4
#define _CB				5
#define _BLCNTR			5
#define _VAR			6
#define _SR				7
#define _RELSPEED		7
#define _CR				8
#define _RELCNTR		8
#define _BCR			9
#define _BLRCNTR		9
#define _BS				10
#define _BLSPEED		10
#define _BSR			11
#define _BLRSPEED		11
#define _BA				12
#define _BATAMPS		12
#define _V				13
#define _VOLTS			13
#define _D				14
#define _DIGIN			14
//#define _DI			15
#define _DIN			15
#define _AI				16
#define _ANAIN			16
#define _PI				17
#define _PLSIN			17
#define _T				18
#define _TEMP			18
#define _F				19
#define _FEEDBK			19
#define _FS				20
#define _STFLAG			20
#define _FF				21
#define _FLTFLAG		21
#define _B				22
#define _BOOL			22
#define _DO				23
#define _DIGOUT			23
#define _E				24
#define _LPERR			24
#define _CIS			25
#define _CMDSER			25
#define _CIA			26
#define _CMDANA			26
#define _CIP			27
#define _CMDPLS			27
#define _TM				28
#define _TIME			28
#define _LK				29
#define _LOCKED			29
#define _FID			30
#define _TRN			31
#define _TR				32
#define _TRACK			32
#define _K				33
#define _SPEKTRUM		33
#define _DR				34
#define _DREACHED		34
#define _AIC			35
#define _ANAINC			35
#define _PIC			36
#define _PLSINC			36
#define _MA				37
#define _MEMS			37
#define _CL				38
#define _CALIVE			38
#define _CAN			39
#define _CF				40
#define _MGD			41
#define _MGDET			41
#define _MGT			42
#define _MGTRACK		42
#define _MGM			43
#define _MGMRKR			43
#define _MGS			44
#define _MGSTATUS		44
#define _MGY			45
#define _MGYRO			45
#define _MBV			46
#define _MBB			47
#define _FM				48
#define _MOTFLAG		48
#define _HS				49
#define _HSENSE			49
#define _UID			50
#define _ASI			51
#define _RAWSENSADC		51
#define _QO				52
#define _QOUT			52
#define _EO				53
#define _EOUT			53
#define _RMA			54
#define _MACC			54
#define _RMG			55
#define _RGYRO			55
#define _RMM			56
#define _MMAG			56
#define _ML				57
#define _MALL			57
#define _TS				58
#define _LINES			58
#define _MRS			59
#define _MSENS			59
#define _MZ				60
#define _MSZER			60
#define _PK				61
#define _MPEAK			61
#define _RF				62
#define _MREF			62
#define _FIN			63
#define _GY				64
#define _GYRO			64
#define _ANG			65
#define _ANGLE			65
#define _HCT			66
#define _HCOUNTER		66
#define _HDT			67
#define _HDATA			67
#define _CUI			68
#define _SCC			69
#define _ICL			70
#define _FC				71
#define _SL				72
#define _PHA			73
#define _CRT			74
#define _CHD			75
#define _BMC			76
#define _BMF			77
#define _SOH			79
#define _BMS			78
#define _BSC			80
#define _SWS			81
#define _MGX			82
#define _CW				86
#define _SPL			87
#define _SAC			88
#define _SDC			89
#define _ROM			90
#define _TC				91
#define _POS			92
#define _PSP			93
#define _PAC			94
#define _PDC			95
#define _TSL			96
#define _SW				97
#define _RMP			98
#define _AOM			99
#define _SDM			100
#define _VNM			101
#define _SS				106
#define _SSR			107
#define _CSS			108
#define _CSR			109
#define _DPA			110
#define _SCA			111
#define _STT			112
#define _SNS			113
#define _PWR			114
#define _BEM			115
#define _BIN			116
#define _PXL			117
#define _IMQ			118
#define _SMM			119
#define _MM				120
#define _SNA			121
#define _TRQ			122
#define _FLX			123
#define _FLY			124
#define _MGW			125
#define _MKC			126
#define _MKS			127
#define _MKE			128
#define _MKL			129
#define _MKF			130
#define _MKM			131
#define _MKO			132
#define _MKP			133
#define _MKD			134
#define _MKT			135
#define _MKV			136
*/
//////////////////////////// CONFIGURATION ////////////////////////////
/*#define _EE				0
#define _BKD			1
#define _OVL			2
#define _UVL			3
#define _THLD			4
#define _MXMD			5
#define _PWMF			6
#define _CPRI			7
#define _RWD			8
#define _ECHOF			9
#define _RSBR			10
#define _ACS			11
#define _AMS			12
#define _CLIN			13
#define _DFC			14
#define _DINA			15
#define _DINL			16
#define _DOA			17
#define _DOL			18
#define _AMOD			19
#define _AMIN			20
#define _AMAX			21
#define _ACTR			22
#define _ADB			23
#define _ALIN			24
#define _AINA			25
#define _AMINA			26
#define _AMAXA			27
#define _APOL			28
#define _PMOD			29
#define _PMIN			30
#define _PMAX			31
#define _PCTR			32
#define _PDB			33
#define _PLIN			34
#define _PINA			35
#define _PMINA			36
#define _PMAXA			37
#define _PPOL			38
#define _MMOD			39
#define _MXPF			40
#define _MXPR			41
#define _ALIM			42
#define _ATRIG			43
#define _ATGA			44
#define _ATGD			45
#define _KP				46
#define _KI				47
#define _KD				48
#define _PIDM			49
#define _ICAP			50
#define _MAC			51
#define _MDEC			52
#define _MVEL			53
#define _MXRPM			54
#define _MXTRN			55
#define _CLERD			56
#define _BPOL			57
#define _BLSTD			58
#define _BLFB			59
#define _BHOME			60
#define _BLL			61
#define _BHL			62
#define _BLLA			63
#define _BHLA			64
#define _SPOL			65
#define _OVH			66
#define _ZAIC			67
#define _ZPAO			68
#define _ZPAC			69
#define _ZSMC			70
#define _TELS			71
#define _BRUN			72
#define _EMOD			73
#define _EPPR			74
#define _ELL			75
#define _EHL			76
#define _ELLA			77
#define _EHLA			78
#define _EHOME			79
#define _SKUSE			80
#define _SKMIN			81
#define _SKMAX			82
#define _SKCTR			83
#define _SKDB			84
#define _SKLIN			85
#define _CEN			86
#define _CNOD			87
#define _CBR			88
#define _CHB			89
#define _CAS			90
#define _CLSN			91
#define _CSRT			92
#define _CTPS			93
#define _SCRO			94
#define _BMOD			95
#define _BADJ			96
#define _BADV			97
#define _BZPW			98
#define _BFBK			99
#define _BEE			100
#define _DIM			101
#define _EQS			102
#define _WMOD			103
#define _IPA			104
#define _GWA			105
#define _SBM			106
#define _IPP			107
#define _MACA			108
#define _PDNS			109
#define _SDNS			110
#define _DHCP			111
#define _CTT			112
#define _ZMT			113
#define _ZACC			114
#define _ZGYR			115
#define _ZMAG			116
#define _TPOL			117
#define _TWDT			118
#define _MDIR			119
#define _TXOF			120
#define _TINV			121
#define _TMS			122
#define _TWAD			123
#define _ZADJ			124
#define _PWMM			125
#define _PWMI			126
#define _PWMX			127
#define _AADJ			128
#define _TRF2			129
#define _TRF05			130
#define _TRF5			131
#define _TC2			132
#define _TC05			133
#define _TC5			134
#define _TW2			135
#define _TW05			136
#define _TW5			137
#define _TZER			138
#define _GRNG			139
#define _GYZR			140
#define _KPF			141
#define _KIF			142
#define _TID			143
#define _HDEL			144
#define _CTRIM			145
#define _LEG			146
#define _SSP			147
#define _SST			148
#define _VPH			149
#define _MXS			150
#define _MPW			151
#define _RFC			152
#define _IRR			153
#define _ILLR			154
#define _ILM			155
#define _ILR			156
#define _DMOD			161
#define _MNOD			162
#define _HSM			163
#define _SWD			164
#define _HPO			165params
#define _BTNC			166
#define _BCHT			167
#define _BCLT			168
#define _BVHT			169
#define _BVLT			170
#define _BTNT			171
#define _BTHT			172
#define _BTLT			173
#define _BAHT			174
#define _BALT			175params
#define _RPWD			198
#define _FCAL			199
#define _ANAM			200
#define _EDEC			201
#define _MDAL			202
#define _MNRPM			203
#define _FSA			204
#define _SCPR			206
#define _SWMAN			205
#define _STO			207
#define _RS485			208
#define _OTL			209
#define _ZSRM			210
#define _ZCAL			211
#define _ZANG			212
#define _MLX			213
#define _SMOD			214
#define _SLL			215
#define _SHL			216
#define _SLLA			217
#define _SHLA			218
#define _SHOME			219
#define _VIND			220
#define _CCFG			221
#define _SMPT			222
#define _TNM			223
#define _SBEE			224
#define _PSA			225
#define _MSP			226
*/
#endif
