/*
 * debug_module.c
 *
 *  Created on: 2017. 3. 7.
 *      Author: hrjung
 */

#ifndef UNIT_TEST_ENABLED

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
#include "main.h"

#include "build_defs.h"
//#include "parameter.h"
#include "uartstdio.h"
#include "cmdline.h"

#include "inv_param.h"
#include "motor_param.h"

#include "drive.h"
#include "freq.h"
#include "state_func.h"
#include "brake.h"
#include "protect.h"
#include "err_trip.h"
#include "timer_handler.h"


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */

#define MAX_COMMAND_NUM      2
#define NUM_OF_DEBUGCHAR    64

const char *res_str[2] = { "OK", "NOK" };
//const char *res_lmt[2] = { "MIN - MAX", "LOW - HIGH" };

/*******************************************************************************
 * TYPEDEFS
 */

typedef enum {
	DBG_CMD_SHOW_HELP,
	DBG_CMD_ECHO_CONFIG,
	//DBG_CMD_SET_SPEED,
	DBG_CMD_SET_FREQUENCY,
	DBG_CMD_SET_JUMP_FREQ,
	DBG_CMD_SET_ACCEL_TIME,
	DBG_CMD_SET_ENERGY_SAVE,
	DBG_CMD_SET_V_BOOST,

	DBG_CMD_MAIN_CONTROL,

	DBG_CMD_START_MOTOR,
	DBG_CMD_STOP_MOTOR,
	DBG_CMD_END_MOTOR,
	DBG_CMD_SET_DIRECTION,
	DBG_CMD_SHOW_STATUS,

	DBG_CMD_BRAKE_CONTROL,
	DBG_CMD_SET_DCI_BRAKE,

	DBG_CMD_SHOW_MTR_PARAM,
	DBG_CMD_SHOW_TRIP,
	DBG_CMD_DEL_TRIP,
	DBG_CMD_SHOW_MONITOR,
	DBG_CMD_SHOW_TEMP,

	DBG_CMD_PROT_OVERLOAD,
	DBG_CMD_PROT_REGEN,
//	DBG_CMD_PROT_STALL,

	DBG_CMD_STEP_UP,
	DBG_CMD_STEP_DOWN,
	DBG_CMD_SET_STEP,
	DBG_CMD_SET_INIT_RELAY,
	DBG_CMD_SET_SHAFT_BRK,
	DBG_CMD_SET_LED,
	DBG_CMD_SET_PWM_AO,
	DBG_CMD_VERSION,

	// temp test debug command
#ifdef SAMPLE_ADC_VALUE
	DBG_CMD_GET_ADC_I_SAMPLE,
#endif
	DBG_CMD_TEST,

	DBG_CMD_ENUM_MAX
} dbg_cmd_code_e;

enum
{
	FREQ_MAIN,
	FREQ_MIN,
	FREQ_MAX,
	FREQ_LOW,
	FREQ_HIGH
};

/*******************************************************************************
 * LOCAL VARIABLES
 */

int systemVersion = 0x1234;

unsigned char cmdString[NUM_OF_DEBUGCHAR];
int freq_step = 2;


/*******************************************************************************
 * EXTERNS
 */
extern tBoolean g_bNewCmd;
extern monitor_param_st mnt;
extern uint32_t secCnt;
extern MOTOR_working_st m_status;
extern float_t sf4pu_rpm;
extern USER_Params gUserParams;
extern int for_rev_flag;
extern int ovl_alarm_enable;

extern float_t MAIN_getPwmFrequency(void);
extern float_t MAIN_getIu(void);
extern float_t MAIN_getIv(void);
extern float_t MAIN_getIw(void);
extern float_t MAIN_getIave(void);

extern float_t MAIN_getDC_lfp(void);
extern void MAIN_showPidGain(void);

extern void REGEN_start(void);
extern void REGEN_end(void);

extern void STA_printInvState(void);
extern void DCIB_setFlag(void);
extern void UTIL_clearInitRelay(void);
extern float_t UTIL_readIpmTemperature(void);
extern float_t UTIL_readMotorTemperature(void);
#ifdef SUPPORT_AUTO_LOAD_TEST
extern bool UTIL_readSwGpio(void);
extern int TEST_readSwitch(void);
#endif
/*******************************************************************************
 * LOCAL FUNCTIONS
 */


#ifdef SUPPORT_DEBUG_TERMINAL
// Prototype statements for functions found within this file.
STATIC int dbg_processHelp(int argc, char *argv[]);
//STATIC int dbg_setSpeed(int argc, char *argv[]);
STATIC int dbg_setFreq(int argc, char *argv[]);
STATIC int dbg_setJumpFreq(int argc, char *argv[]);
STATIC int dbg_setAccelTime(int argc, char *argv[]);
STATIC int dbg_setEnergySave(int argc, char *argv[]);
STATIC int dbg_setVoltVoost(int argc, char *argv[]);

STATIC int dbg_setDriveControl(int argc, char *argv[]);

STATIC int dbg_runMotor(int argc, char *argv[]);
STATIC int dbg_stopMotor(int argc, char *argv[]);
STATIC int dbg_endMotor(int argc, char *argv[]);
STATIC int dbg_setDirection(int argc, char *argv[]);
STATIC int dbg_showMotorState(int argc, char *argv[]);

STATIC int dbg_setBrakeControl(int argc, char *argv[]);
STATIC int dbg_setDcInjBrake(int argc, char *argv[]);

STATIC int dbg_processMotorParam(int argc, char *argv[]);
STATIC int dbg_showTripInfo(int argc, char *argv[]);
STATIC int dbg_removeTripInfo(int argc, char *argv[]);
STATIC int dbg_showInverterStatus(int argc, char *argv[]);
STATIC int dbg_showTempStatus(int argc, char *argv[]);

STATIC int dbg_setOverload(int argc, char *argv[]);
STATIC int dbg_setRegen(int argc, char *argv[]);

STATIC int dbg_stepUpFreq(int argc, char *argv[]);
STATIC int dbg_stepDownFreq(int argc, char *argv[]);
STATIC int dbg_setStepFreq(int argc, char *argv[]);
STATIC int dbg_setInitRelay(int argc, char *argv[]);
STATIC int dbg_setShaftBrake(int argc, char *argv[]);
STATIC int dbg_setLed(int argc, char *argv[]);
STATIC int dbg_showVersion(int argc, char *argv[]);

#ifdef SAMPLE_ADC_VALUE
STATIC int dbg_getAdcSample(int argc, char *argv[]);
#endif

STATIC int dbg_tmpTest(int argc, char *argv[]);

STATIC int EchoSetting(int argc, char*argv[]);

/*******************************************************************************
 * GLOBAL VARIABLES
 */

//const tCmdLineEntry g_sCmdTable[DBG_CMD_ENUM_MAX] =
tCmdLineEntry g_sCmdTable[DBG_CMD_ENUM_MAX] =
{
#if 0 // reduce 5KB
	{"help", dbg_processHelp, " help : show command list"},
	{"echo", EchoSetting, " echo off/on"},
	//{"spd", dbg_setSpeed, " spd index(0-7) spd(30-180) : set spd for each step"},
	{"freq", dbg_setFreq, " frequency setting\n" \
			"   freq main freq(5-400) : set freq for each step\n"  \
			"   freq show : display freq settings"
	},
	{"jmpf", dbg_setJumpFreq, " jmpf index(0-2) low(1-400) high(1-400) : set jump freq range, low < high"},
	{"time", dbg_setAccelTime, " Accel/Decel time setting\n" \
			"   time acc time(0-300): set accel time \n"  \
			"   time dec time(0-300): set decel time \n"  \
			"   time show : display time settings"
	},
	{"engy", dbg_setEnergySave, " engy 0,1 : energy save off/on"},
	{"vst", dbg_setVoltVoost, " vst rate(0-1000) : v_boost"},
	{"drv", dbg_setDriveControl, " Main Drive Control setting\n" \
				"   drv pwm freq(0~3): set pwm frequency \n"  \
				"   drv show : display drive control settings"
	},

	{"start", dbg_runMotor, " start : run motor"},
	{"stop", dbg_stopMotor, " stop : stop motor"},
	{"end", dbg_endMotor, " end : end PWM"},
	{"dir", dbg_setDirection, " dir 0(forward) or 1(reverse) : set motor direction"},
	{"state", dbg_showMotorState, " state : display running status"},

	{"brk", dbg_setBrakeControl, " Brake control setting" \
			"   brk mth method(0-2) : select stop method\n" \
			"   brk thld threshold(0-50) : set start freq for brake\n" \
			"   brk show : display brake settings"
	},
	{"dcbrk", dbg_setDcInjBrake, " DC injection Brake setting\n" \
			"   dcbrk frq freq(0-50) : set start freq for dc brake\n" \
			"   dcbrk btime time(0-60) : set block time for dc brake\n" \
			"   dcbrk brake rate(0-200) time(0-60) : set dc rate and time for brake\n"  \
			"   dcbrk show : display DC brake settings"
	},

	{"mtrp", dbg_processMotorParam, "  mtrp : show motor parameters"},
	{"rtrip", dbg_showTripInfo, " rtrip : display trip error"},
	{"rmtrip", dbg_removeTripInfo, " rmtrip : delete trip error"},
	{"invs", dbg_showInverterStatus, " invs : display inverter status"},
	{"temp", dbg_showTempStatus, " temp : display temperature status"},

	{"ovl", dbg_setOverload, " overload protect settings\n" 			\
			"   ovl en flag(0, 1) : enable/disable overload trip\n"  		\
			"   ovl wlevel level(30-150) : set warning current level\n"  		\
			"   ovl w_dur dur(0-30) : set warning duration\n"  				\
			"   ovl tlevel level(30-200) : set trip current level\n"  		\
			"   ovl t_dur dur(0-60) : set trip duration\n"  				\
			"   ovl show : display overload settings"
	},

	{"regen", dbg_setRegen, " Regen setting" \
			"   regen res ohm(150-500) power(10-65535) : set resister value and power\n"  \
			"   regen thml thermal(0-65535) reduce(0-150): set thermal and reduce rate\n" \
			"   regen show : display regen settings"
	},

	{"up", dbg_stepUpFreq, " up : Freq step up"},
	{"dn", dbg_stepDownFreq, " up : Freq step down"},
	{"step", dbg_setStepFreq, " step : set step freqquency"},
	{"irel", dbg_setInitRelay, " irel : set Init Relay"},

	{"irel", dbg_setInitRelay, " irel : set Init Relay"},
	{"sbrk", dbg_setShaftBrake, " sbrk (0/1) : set Shaft Brake off/on"},
	{"led", dbg_setLed, " led (0~3) 0/1 : set Led R,G,R2,G2 off/on"},
	{"ver", dbg_showVersion, " ver : show HW, FW version"},

#ifdef SAMPLE_ADC_VALUE
	{"iadc", dbg_getAdcSample, " iadc (0,1) read ADC sample"},
#endif
	{"tmp", dbg_tmpTest, " tmp : test command"}
#else
	{"help", dbg_processHelp, " help : show command list"},
	{"echo", EchoSetting, " echo off/on"},
	{"freq", dbg_setFreq, " frequency setting"},
	{"jmpf", dbg_setJumpFreq, " jump freq setting"},
	{"time", dbg_setAccelTime, " Accel/Decel time"},
	{"engy", dbg_setEnergySave, " energy save off/on"},
	{"vst", dbg_setVoltVoost, " v_boost"},
	{"drv", dbg_setDriveControl, " Main Drive"},

	{"start", dbg_runMotor, " start"},
	{"stop", dbg_stopMotor, " stop"},
	{"end", dbg_endMotor, " end"},
	{"dir", dbg_setDirection, " dir 0/1"},
	{"state", dbg_showMotorState, " state"},

	{"brk", dbg_setBrakeControl, " brk Setting"},
	{"dcbrk", dbg_setDcInjBrake, " DCI Brake setting"},

	{"mtrp", dbg_processMotorParam, "  motor show motor parameters"},
	{"rtrip", dbg_showTripInfo, " read trip info"},
	{"rmtrip", dbg_removeTripInfo, " remove trip info"},
	{"invs", dbg_showInverterStatus, " inverter status"},
	{"temp", dbg_showTempStatus, " temperature status"},

	{"ovl", dbg_setOverload, " overload protect"},
	{"regen", dbg_setRegen, " Regen setting" },

	{"up", dbg_stepUpFreq, " up"},
	{"dn", dbg_stepDownFreq, " dn"},
	{"step", dbg_setStepFreq, " step"},
	{"irel", dbg_setInitRelay, " irel"},
	{"sbrk", dbg_setShaftBrake, " sbrk"},
	{"led", dbg_setLed, " led"},
	{"ver", dbg_showVersion, " ver"},

#ifdef SAMPLE_ADC_VALUE
	{"iadc", dbg_getAdcSample, " read ADC I"},
#endif
	{"tmp", dbg_tmpTest, " tmp"}
#endif
};


/*
 *  ======== function ========
 */

// command function lists
STATIC int dbg_processHelp(int argc, char *argv[])
{
    int i;
//    UARTprintf("cmd <arg1> <arg2>\n");
    for(i=0;i<DBG_CMD_ENUM_MAX;i++)
    {
        UARTprintf("%s\n", g_sCmdTable[i].pcHelp);
        //UARTFlushTx(0);
    }

    return 0;
}

STATIC void dbg_showAccelTimeSetting(void)
{
	UARTprintf(" Accel/Decel time Settings\n");
	UARTprintf("\t accel time %f decel time %f\n", param.ctrl.accel_time, param.ctrl.decel_time);
}


STATIC void dbg_showBrakeControlParam(void)
{
	UARTprintf(" Brake Control Settings\n");
	UARTprintf("\t method %d, brk_freq %f\n", param.brk.method, param.brk.brake_freq);
}

STATIC void dbg_showDciBrakeParam(void)
{
	UARTprintf(" DC injection Brake Settings const=%f\n", dev_const.dci_pwm_rate);
	UARTprintf("\t brake mode %d, start freq %f, block time %f\n", param.brk.method, param.brk.dci_start_freq, param.brk.dci_block_time);
	UARTprintf("\t inject rate %f %%, time %f to brake\n", param.brk.dci_braking_rate, param.brk.dci_braking_time);
}

STATIC void dbg_showMotorParam(void)
{
	UARTprintf(" Motor Parameter Settings\n");
	UARTprintf("\t capacity: %d, poles: %d\n", mtr.capacity, mtr.poles);
	UARTprintf("\t input_volt: %d, rated_freq: %d\n", mtr.input_voltage, mtr.rated_freq);
	UARTprintf("\t slip freq: %f, effectiveness: %f\n", mtr.slip_rate, mtr.effectiveness);
	UARTprintf("\t no load current: %f, max_current: %f\n", mtr.noload_current, mtr.max_current);
	UARTprintf("\t Rs: %f, Rr: %f, Ls: %f\n", mtr.Rs, mtr.Rr, mtr.Ls);
}

STATIC void dbg_showTripData(void)
{
	const char *state_str[5] = { "START", "STOP", "ACCEL", "DECEL", "RUN" };
	UARTprintf(" Error info display\n");
	//for(i=0; i<FAULT_HISTORY_NUM; i++)
	{
		UARTprintf("\t errCode: %d, freq: %f, cur : %f, state=%s \n", \
				param.err_info.code, param.err_info.freq, param.err_info.current, state_str[param.err_info.op_mode]);
	}
}
extern int REGEN_getDuty(void);
extern _iq gVbus_lpf;
STATIC void dbg_showMonitorParam(void)
{
	UARTprintf(" Inverter Status display\n");

	float_t gOver = _IQtoF(_IQdiv(_IQ(1.0),gVbus_lpf));
	UARTprintf("\t Iu: %f, Iv: %f, Iw: %f, DC voltage: %f\n", MAIN_getIu(), MAIN_getIv(), MAIN_getIw(), MAIN_getVdcBus());
	UARTprintf("\t RMS Iu: %f, Iv: %f, Iw: %f\n", internal_status.Irms[0], internal_status.Irms[1], internal_status.Irms[2]);
	UARTprintf("\t Volt: Vu: %f, Vv: %f, Vw: %f \n", internal_status.Vu_inst, internal_status.Vv_inst, internal_status.Vw_inst); //, MAIN_getDC_lfp());
	UARTprintf("\t Volt: U-V: %f, V-W: %f, W-U: %f \n", (internal_status.Vu_inst - internal_status.Vv_inst), (internal_status.Vv_inst-internal_status.Vw_inst), (internal_status.Vw_inst-internal_status.Vu_inst));
//	UARTprintf("\t input status: 0x%x, out status: 0x%x\n", (int)((mnt.dio_status>>16)&0x0F), (int)(mnt.dio_status&0x0F));
	UARTprintf("\t Motor RPM: %f  Freq: %f  target %f dir=%d \n", STA_getCurSpeed(), m_status.cur_freq, m_status.target_freq, (int)m_status.direction);
	UARTprintf("\t Motor status %d, accel: %f  decel: %f gOver=%f \n", m_status.status, m_status.acc_res, m_status.dec_res, gOver);
}

STATIC void dbg_showOverloadParam(void)
{
	UARTprintf(" Overload Setting\n");
	UARTprintf("\t enable: %d\n", param.protect.ovl.enable);
	UARTprintf("\t Warning level: %d, duration: %d\n", param.protect.ovl.wr_limit, param.protect.ovl.wr_duration);
	UARTprintf("\t Trip level: %d, duration: %d\n", param.protect.ovl.tr_limit, param.protect.ovl.tr_duration);
	UARTprintf("\t warn: %f, trip: %f OVC: %f\n", dev_const.warn_level, dev_const.trip_level, dev_const.ovc_level);
}

STATIC void dbg_showRegenParam(void)
{
	float_t value = sqrtf(0.9*150.0*50.0);
	UARTprintf("\t resistance ohm %f power %d \n", param.protect.regen.resistance, param.protect.regen.power);
	UARTprintf("\t thermal %f reduction %d \n", param.protect.regen.thermal, param.protect.regen.band);
	UARTprintf("\t V_max %f %f regen_duty %d \n", dev_const.regen_max_V, value, REGEN_getDuty());
}


#if 1
STATIC int dbg_setFreq(int argc, char *argv[])
{
	int result;
	uint16_t value;
	float_t f_value;
	//int cmd;

    if(argc != 2) goto freq_err;

	value = (uint16_t)atoi(argv[1]);
	f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
	if(f_value > MIN_FREQ_VALUE && f_value < MAX_FREQ_VALUE)
	{
		result = FREQ_setFreqValue(f_value);
		UARTprintf("set frequency=%f, result=%s\n", f_value, res_str[result]);
		UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);
	}
	else
	{
		goto freq_err;
	}

    return 0;

freq_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_FREQUENCY].pcHelp);
	return 1;
}

STATIC int dbg_setJumpFreq(int argc, char *argv[])
{
	int i, index, result=1;
	uint16_t low, high;
	float_t f_low, f_high;

    if(argc != 1 && argc != 4) goto jmp_err;

    if(argc == 1) // show settings
    {
    	UARTprintf("Jump frequency setting\n");
    	for(i=0; i<MAX_JUMP_FREQ_NUM; i++)
    	{
    		if(param.ctrl.jump[i].enable)
    			UARTprintf("  Jump freq[%d]: %f - %f\n", i, param.ctrl.jump[i].low, param.ctrl.jump[i].high);
    	}

    	return 0;
    }

    index = atoi(argv[1]);
    low = (uint16_t)atoi(argv[2]);
    high = (uint16_t)atoi(argv[3]);

    if(low > high) goto jmp_err;

    f_low = (float_t)(low/FREQ_INPUT_RESOLUTION);
    f_high = (float_t)(high/FREQ_INPUT_RESOLUTION);
    result = FREQ_setJumpFreqRange(index, f_low, f_high);
    UARTprintf("set jump frequency range %f - %f at index=%d, result=%s\n", f_low, f_high, index, res_str[result]);
    return 0;

jmp_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_JUMP_FREQ].pcHelp);
    return 1;
}

#else
STATIC int dbg_setSpeed(int argc, char *argv[])
{
	int index, value, result;

	if(argc == 1)
	{
		dbg_showSpeedSettings();
		return 0;
	}
	else if(argc == 3)
	{
		index = atoi(argv[1]);
		value = atoi(argv[2]);
		result = DRV_setSpeedValue(index, value);
		UARTprintf("set speed=%d at index=%d, result=%s\n", value, index, res_str[result]);
	}
	else
	{
		goto spd_err;
	}

    return 0;

spd_err:
	UARTprintf("speed input range : %d ~ %d\n",  (int)dev_const.spd_rpm_min, (int)dev_const.spd_rpm_max);
	return 1;
}
#endif

STATIC int dbg_setAccelTime(int argc, char *argv[])
{
	int value;
	float_t f_val;
	int result;

    if(argc != 2 && argc != 3) goto acc_err;

    if(argc == 2 && strcmp("show", argv[1]) == 0)
    {
    	dbg_showAccelTimeSetting();
		return 0;
    }

    if(argc == 3)
    {
    	value = atoi(argv[2]);
    	f_val = (float_t)value/10.0;
    	if(strcmp("acc", argv[1]) == 0)
    	{
			result = DRV_setAccelTime(f_val);
			UARTprintf("set accel time=%f, result=%s\n", f_val, res_str[result]);
    	}
    	else if(strcmp("dec", argv[1]) == 0)
    	{
			result = DRV_setDecelTime(f_val);
			UARTprintf("set decel time=%f, result=%s\n", f_val, res_str[result]);
    	}
    	else
    		goto acc_err;


		return result;
    }

acc_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_ACCEL_TIME].pcHelp);
	return 1;
}

STATIC int dbg_setEnergySave(int argc, char *argv[])
{
	int on_off, result;

	if(argc != 1 && argc != 2) goto save_err;

	if(argc == 1)
	{
		UARTprintf("energy save is %d\n", param.ctrl.energy_save);
		return 0;
	}

	on_off = atoi(argv[1]);
	result = DRV_setEnergySave(on_off);
	UARTprintf("set Energy_save %d is %s\n", on_off, res_str[result]);

	return 0;

save_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_ENERGY_SAVE].pcHelp);
	return 1;
}

STATIC int dbg_setVoltVoost(int argc, char *argv[])
{
	int value, result;
	float_t f_value;

	if(argc != 1 && argc != 2) goto boost_err;

	if(argc == 1)
	{
		UARTprintf("v_boost is %f\n", param.ctrl.v_boost);
		return 0;
	}

	value = atoi(argv[1]);
	f_value = (float_t)(value/10.0);
	result = DRV_setVoltageBoost(f_value);
	UARTprintf("set voltage boost %f is %s\n", f_value, res_str[result]);

	return 0;

boost_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_V_BOOST].pcHelp);
	return 1;
}

STATIC int dbg_setDriveControl(int argc, char *argv[])
{
	int value;
	int result;
	const char *pwm_str[] = {"6kHz", "9kHz", "12kHz", "15kHz" };

    if(argc != 2 && argc != 3) goto drv_err;

    if(argc == 2)
    {
    	if(strcmp("vf", argv[1]) == 0)
    	{
    		DRV_enableVfControl();
    		UARTprintf("set VF control\n");
    		return 0;
    	}
    	else if(strcmp("foc", argv[1]) == 0)
    	{
    		DRV_enableFocControl();
    		UARTprintf("set FOC control\n");
    		return 0;
    	}
    	else
    		goto drv_err;
    }


    if(argc == 3)
    {
    	value = atoi(argv[2]);
    	if(strcmp("pwm", argv[1]) == 0)
    	{
			result = DRV_setPwmFrequency(value);
			UARTprintf("set pwm freq=%s, result=%s\n", pwm_str[value], res_str[result]);
    	}
    	else
    		goto drv_err;

		return result;
    }

drv_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_MAIN_CONTROL].pcHelp);
	return 1;
}


STATIC int dbg_runMotor(int argc, char *argv[])
{
    if(argc >= 2) goto run_err;

	MAIN_enableSystem(0);
	STA_calcResolution();
	UARTprintf("start running motor\n");

    STA_printInvState();

    return 0;

run_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_START_MOTOR].pcHelp);
    return 1;
}

STATIC int dbg_stopMotor(int argc, char *argv[])
{
	//int result;

    if(argc >= 2) goto stop_err;

	STA_setStopCondition();
	UARTprintf("reduce speed cur_freq=%f to stop\n", m_status.cur_freq);
	UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);

    return 0;

stop_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_STOP_MOTOR].pcHelp);
    return 1;
}

STATIC int dbg_endMotor(int argc, char *argv[])
{
    if(argc >= 2) goto stop_err;

	MAIN_disableSystem();
	UARTprintf("stop running motor\n");

    STA_printInvState();

    return 0;

stop_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_END_MOTOR].pcHelp);
    return 1;
}

STATIC int dbg_setDirection(int argc, char *argv[])
{
	int dir;

    if(argc != 2) goto dir_err;

	dir = atoi(argv[1]);
	if(dir != 0 && dir != 1) goto dir_err;

//	if(m_status.status == STATE_STOP)
	{
        if(dir == 0) //forward direction
        {
			MAIN_setForwardDirection();
			UARTprintf("set direction forward\n");
        }
        else
        {
			MAIN_setReverseDirection();
			UARTprintf("set direction backward\n");
        }
        STA_calcResolution4Reverse(m_status.cur_freq);
	}
//	else
//	{
//	    for_rev_flag = 1;
//	}

    return 0;

dir_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_DIRECTION].pcHelp);
    return 1;
}

STATIC int dbg_showMotorState(int argc, char *argv[])
{
	const char *state_str[5] = { "START", "STOP", "ACCEL", "DECEL", "RUN" };

    if(argc != 1) goto sta_err;

    UARTprintf(" Running speed %d, current %f\n", MAIN_getCurrentSpeed(), STA_getCurrent());

    UARTprintf(" Motor status %s\n", state_str[m_status.status]);

    return 0;

sta_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_STATUS].pcHelp);
    return 1;
}

STATIC int dbg_setBrakeControl(int argc, char *argv[])
{
	int value, result=0;
	float_t f_value;
	char *brk_str[3] = { "Decel", "DC inject", "FreeRun" };

	if(argc < 2 || argc > 3) goto brk_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showBrakeControlParam();
			return 0;
		}
		else
			goto brk_err;
	}

	if(argc == 3)
	{
		value = atoi(argv[2]);
		if(strcmp(argv[1], "mth")==0)
		{
			result = BRK_setBrakeMethod(value);
			if(result)
				UARTprintf("set brake method %d is %s\n", value, res_str[result]);
			else
				UARTprintf("set brake method %d is %s\n", value, brk_str[value]);
			return result;
		}
		else if(strcmp(argv[1], "freq")==0)
		{
			f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
			result = BRK_setBrakeFreq(f_value);
			UARTprintf("set brake start freq %f is %s\n", f_value, res_str[result]);
			return result;
		}
		else
			goto brk_err;
	}

	return 0;

brk_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_BRAKE_CONTROL].pcHelp);
	return 1;
}

STATIC int dbg_setDcInjBrake(int argc, char *argv[])
{
	int value, result;
	float_t f_value;

	if(argc < 2 && argc > 3) goto dcib_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showDciBrakeParam();
			return 0;
		}
		else if(strcmp(argv[1], "on") == 0)
		{
			param.brk.method = DC_INJECT_BRAKE;
		}
		else if(strcmp(argv[1], "off") == 0)
		{
			param.brk.method = REDUCE_SPEED_BRAKE;
		}
		else
			goto dcib_err;
	}

	if(argc == 3)
	{
		value = atoi(argv[2]);
		if(strcmp(argv[1], "frq")==0)
		{
			f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
			result = DCIB_setStartFreq(value);
			UARTprintf("set brake start freq %f is %s\n", f_value, res_str[result]);
			return result;
		}
		else if(strcmp(argv[1], "btime")==0)
		{
			f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
			result = DCIB_setBlockTime(f_value);
			UARTprintf("set brake block time %f is %s\n", f_value, res_str[result]);
			return result;
		}
		else if(strcmp(argv[1], "rate")==0)
		{
			f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
			result = DCIB_setBrakeRate(f_value);
			UARTprintf("set brake rate %f for brake is %s\n", f_value, res_str[result]);
			return result;
		}
		else if(strcmp(argv[1], "time")==0)
		{
			f_value = (float_t)(value/FREQ_INPUT_RESOLUTION);
			result = DCIB_setBrakeTime(f_value);
			UARTprintf("set brake time %f for brake is %s\n", f_value, res_str[result]);
			return result;
		}
		else
			goto dcib_err;

	}

	return 0;

dcib_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_DCI_BRAKE].pcHelp);
	return 1;
}

#if 1
STATIC int dbg_processMotorParam(int argc, char *argv[])
{
	if(argc == 1)
	{
		dbg_showMotorParam();
		return 0;
	}
	else
		goto mtr_err;


mtr_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_MTR_PARAM].pcHelp);
	return 1;
}
#else
STATIC int dbg_processMotorParam(int argc, char *argv[])
{
	int value;
	int f_val;

	if(argc != 2 && argc != 3) goto mtr_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showMotorParam();
			return 0;
		}
		else
			goto mtr_err;
	}

	if(strcmp(argv[1], "slip")==0)
	{
		value = atoi(argv[2]);
		if(value < 0 || value > 10) goto mtr_err;

		mtr.slip_offset = value;
		UARTprintf(" set slip freq %d\n", value);
	}
	else if(strcmp(argv[1], "curr")==0)
	{
		f_val = atoi(argv[2]);
		if(value < 0 || value > 150) goto mtr_err;

		mtr.rated_current = f_val;
		UARTprintf(" set rated current %d\n", (int)f_val);
	}
	else if(strcmp(argv[1], "noload")==0)
	{
		f_val = atoi(argv[2]);
		if(value < 0 || value > 100) goto mtr_err;

		mtr.noload_current = f_val;
		UARTprintf(" set noload current %d\n", (int)f_val);
	}
	else if(strcmp(argv[1], "poles")==0)
	{
		value = atoi(argv[2]);
		if(value < 0 || value > 10) goto mtr_err;

		mtr.poles = value;
		UARTprintf(" set poles %d\n", value);
	}
	else if(strcmp(argv[1], "capa")==0)
	{
		value = atoi(argv[2]);
		if(value < 0 || value > 10) goto mtr_err;

		mtr.capacity = value;
		UARTprintf(" set capacity %d\n", value);
	}
	else if(strcmp(argv[1], "eff")==0)
	{
		value = atoi(argv[2]);
		if(value < 50 || value > 100) goto mtr_err;

		mtr.effectiveness = value;
		UARTprintf(" set effectiveness %d\n", value);
	}
	else if(strcmp(argv[1], "rs")==0)
	{
		f_val = atoi(argv[2]);
		if(value < 0 || value > 28) goto mtr_err;

		mtr.Rs = f_val;
		UARTprintf(" set stator resistance %d\n", (int)f_val);
	}
	else if(strcmp(argv[1], "ls")==0)
	{
		f_val = atoi(argv[2]);
		if(value < 0 || value > 300) goto mtr_err;

		mtr.Ls = f_val;
		UARTprintf(" set stator inductance %d\n", (int)f_val);
	}
	else
		goto mtr_err;

	return 0;

mtr_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_MTR_PARAM].pcHelp);
	return 1;
}
#endif

STATIC int dbg_showTripInfo(int argc, char *argv[])
{
	if(argc > 1) goto tr_err;

	dbg_showTripData();

	return 0;

tr_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_TRIP].pcHelp);
	return 1;
}

STATIC int dbg_removeTripInfo(int argc, char *argv[])
{
	int result;

	if(argc > 1) goto rtr_err;

	result = ERR_clearTripData();
	UARTprintf("clear trip info is %s\n", res_str[result]);

	return 0;

rtr_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_DEL_TRIP].pcHelp);
	return 1;
}

STATIC int dbg_showInverterStatus(int argc, char *argv[])
{
	if(argc > 1) goto mnt_err;

	dbg_showMonitorParam();

	return 0;

mnt_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_MONITOR].pcHelp);
	return 1;
}

STATIC int dbg_showTempStatus(int argc, char *argv[])
{
	float_t ipm_temp, mtr_temp;
	if(argc > 1) goto temp_err;

	ipm_temp = UTIL_readIpmTemperature();
	mtr_temp = UTIL_readMotorTemperature();
	UARTprintf("IPM temp = %f, %d, Motor Temp = %f\n", ipm_temp, internal_status.ipm_temp, mtr_temp);

	return 0;

temp_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SHOW_TEMP].pcHelp);
	return 1;
}


STATIC int dbg_setOverload(int argc, char *argv[])
{
	int value, result;
	const char *en_str[2] = { "DISABLE", "ENABLE" };

	if(argc < 2 && argc > 3) goto ovl_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showOverloadParam();
			return 0;
		}
		else
			goto ovl_err;
	}

	if(argc == 3)
	{
		value = atoi(argv[2]);
		if(strcmp(argv[1], "en")==0)
		{
			if(value != 0 && value != 1) goto ovl_err;

			OVL_enbleOverloadTrip(value);
			UARTprintf("set Overload Trip enable\n", en_str[value]);

			return 0;
		}
		else if(strcmp(argv[1], "wlevel")==0)
		{
			result = OVL_setWarningLevel(value);
			UARTprintf("set Overload waning level %d is %s\n", value, res_str[result]);
			return result;
		}
		else if(strcmp(argv[1], "w_dur")==0)
		{
			result = OVL_setWarningTime(value);
			UARTprintf("set Overload warning duration %d is %s\n", value, res_str[result]);
			return result;
		}
		else if(strcmp(argv[1], "tlevel")==0)
		{
			result = OVL_setTripLevel(value);
			UARTprintf("set Overload Trip level %d is %s\n", value, res_str[result]);
			return result;
		}
		else if(strcmp(argv[1], "t_dur")==0)
		{
			result = OVL_setTripTime(value);
			UARTprintf("set Overload Trip duration %d is %s\n", value, res_str[result]);
			return result;
		}
		else
			goto ovl_err;
	}

	return 0;

ovl_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_PROT_OVERLOAD].pcHelp);
	return 1;
}

STATIC int dbg_setRegen(int argc, char *argv[])
{
	int result;
	uint16_t value, power;

	if(argc < 2 && argc > 4) goto regen_err;

	if(argc == 2)
	{
		if(strcmp(argv[1], "show") == 0)
		{
			dbg_showRegenParam();
			return 0;
		}
		else
			goto regen_err;
	}

	if(argc == 3) goto regen_err;

	if(argc == 4)
	{
		value = (uint16_t)atoi(argv[2]);
		power = (uint16_t)atoi(argv[3]);
		if(strcmp(argv[1], "res")==0)
		{
			result = REGEN_setRegenResistence(value);
			UARTprintf("set regen resistance %d ohm is %s\n", value, res_str[result]);
			result = REGEN_setRegenResistencePower(power);
			UARTprintf("set regen power %d W is %s\n", power, res_str[result]);
		}
		else if(strcmp(argv[1], "thml")==0)
		{
			result = REGEN_setRegenThermal(value);
			UARTprintf("set regen thermal %d is %s\n", value, res_str[result]);
			result = REGEN_setRegenVoltReduction(power);
			UARTprintf("set regen reduce %d is %s\n", power, res_str[result]);
		}
		else
			goto regen_err;
	}

	return 0;

regen_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_PROT_REGEN].pcHelp);
	return 1;
}

STATIC int dbg_stepUpFreq(int argc, char *argv[])
{
	int result;
	float_t f_value = param.ctrl.value;

    if(argc > 2) goto up_err;

    f_value = f_value + (float_t)freq_step;
    result = FREQ_setFreqValue(f_value);
    UARTprintf("set frequency=%f rpm=%d at index=%d, result=%s\n", f_value, 30*(int)f_value, 0, res_str[result]);
    UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);

    return 0;

up_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_STEP_UP].pcHelp);
	return 1;
}

STATIC int dbg_stepDownFreq(int argc, char *argv[])
{
	int result;
	float_t f_value = param.ctrl.value;

    if(argc > 2) goto down_err;

    f_value = f_value - (float_t)freq_step;
    if(f_value < 3.0)
    {
    	UARTprintf("no more step down freq=%f step=%d\n", f_value, freq_step);
    	goto down_err;
    }
    result = FREQ_setFreqValue(f_value);
    UARTprintf("set frequency=%f rpm=%d at index=%d, result=%s\n", f_value, 30*(int)f_value, 0, res_str[result]);
    UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);

    return 0;

down_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_STEP_DOWN].pcHelp);
	return 1;
}

STATIC int dbg_setStepFreq(int argc, char *argv[])
{
	int step;

    if(argc > 3) goto step_err;

    if(argc == 1)
    {
    	UARTprintf(" current step is %d\n", freq_step);
    	return 0;
    }

    step = atoi(argv[1]);
	if(step < 1 || step > 10) goto step_err;

	freq_step = step;
	UARTprintf(" new step is %d\n", freq_step);

	return 0;

step_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_STEP].pcHelp);
	return 1;
}

STATIC int dbg_setInitRelay(int argc, char *argv[])
{
	int on_off;

    if(argc > 2) goto irel_err;

    if(argc == 1)
    {
    	UARTprintf(" Init Relay status %d\n", internal_status.relay_enabled);
    	return 0;
    }

    on_off = atoi(argv[1]);
	if(on_off != 0 && on_off != 1) goto irel_err;

	if(on_off == 0) //release shaft brake
	{
		UTIL_clearInitRelay();
		UARTprintf(" clear Init Relay Low %d\n", internal_status.relay_enabled);
	}
	else
	{
		UTIL_setInitRelay();
		UARTprintf(" set Init Relay High %d\n", internal_status.relay_enabled);
	}

    return 0;

irel_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_INIT_RELAY].pcHelp);
    return 1;
}

STATIC int dbg_setShaftBrake(int argc, char *argv[])
{
	int on_off;

    if(argc != 2) goto sbrk_err;

    on_off = atoi(argv[1]);
	if(on_off != 0 && on_off != 1) goto sbrk_err;

	if(on_off == 0) //release shaft brake
	{
		UTIL_releaseShaftBrake();
		UARTprintf("release shaft brake\n");
    }
    else
    {
		UTIL_setShaftBrake();
		UARTprintf("set shaft brake\n");
    }

    return 0;

sbrk_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_SHAFT_BRK].pcHelp);
    return 1;
}

STATIC int dbg_setLed(int argc, char *argv[])
{
	int type, on_off;
	int led_type[] = {HAL_Gpio_LED_R, HAL_Gpio_LED_G};
	char *led_str[] = {"LED_R", "LED_G"};

    if(argc != 3) goto led_err;

    type = atoi(argv[1]);
    on_off = atoi(argv[2]);

    if(type < 0 || type > 1) goto led_err;
	if(on_off != 0 && on_off != 1) goto led_err;

	if(on_off == 1) //LED on
	{
		UTIL_controlLed(led_type[type], on_off);
		UARTprintf(" %s on\n", led_str[type]);
    }
    else
    {
		UTIL_controlLed(led_type[type], on_off);
		UARTprintf(" %s off\n", led_str[type]);
    }

    return 0;

led_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_SET_LED].pcHelp);
    return 1;
}

STATIC int dbg_showVersion(int argc, char *argv[])
{
    if(argc >= 2) goto ver_err;

    UARTprintf("HW ver %d.%d, FW ver %d.%d\n", HW_VER_MAJ, HW_VER_MIN, DSP_FW_VER_MAJ, DSP_FW_VER_MIN);

    return 0;

ver_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_VERSION].pcHelp);
    return 1;
}


#ifdef SAMPLE_ADC_VALUE
#define V_SAMPLE_COUNT	240

#define I_CURR_SAMPLE_TYPE  	0
#define V_UVW_SAMPLE_TYPE		1
#define V_DC_SAMPLE_TYPE		2
#define V_AB_SAMPLE_TYPE		3
#define PWM_SAMPLE_TYPE			4
#define PWM2_SAMPLE_TYPE		5
#define PHASOR_SAMPLE_TYPE		6
#define V_AB_PHASOR_SAMPLE_TYPE	7
#define PWM_ERR_TYPE			8
#define PWM_PERIOD_COUNT		9
#define I_RMS_SAMPLE_TYPE		10

int sample_type=I_RMS_SAMPLE_TYPE;
int v_count = V_SAMPLE_COUNT;
int sampling_flag=0, stop_sampling_flag=1;
float_t smpl_buff[3][V_SAMPLE_COUNT];
uint16_t smpl_buff_idx=0;
long pwm_err_cnt=0;
float_t pwm_value=0.0, pwm_value_neg=0.0, pwm_value_sat=0.0;

void initSampleBuffer(void)
{
	int i;

	smpl_buff_idx=0;
	for(i=0; i<V_SAMPLE_COUNT; i++)
	{
		smpl_buff[0][i] = 0.0;
		smpl_buff[1][i] = 0.0;
		smpl_buff[2][i] = 0.0;
	}

}

void dbg_getSample(float_t val1, float_t val2, float_t val3)
{
	// for reading V
	if(sampling_flag)
	{
	  smpl_buff[0][smpl_buff_idx] = val1;
	  smpl_buff[1][smpl_buff_idx] = val2;
	  smpl_buff[2][smpl_buff_idx] = val3;

	  smpl_buff_idx++;
	  if(smpl_buff_idx == V_SAMPLE_COUNT)
	  {
		  if(stop_sampling_flag)
		  {
			  sampling_flag=0;
			  UARTprintf(" sampling done\n");
		  }
		  else    smpl_buff_idx=0;
	  }
	}
}

STATIC int dbg_getAdcSample(int argc, char *argv[])
{
	uint16_t i, delay;
	uint16_t cmd=0;
	//float_t f_result[3];

    if(argc != 2) goto iadc_err;

    cmd = (int)atoi(argv[1]);

    switch(cmd)
    {
    case 0: // start sampling
    	initSampleBuffer();
    	stop_sampling_flag = 0;
    	sampling_flag=1;
    	UARTprintf(" start sampling type=%d\n", sample_type);
    	break;
    case 1: // stop sampling
    	stop_sampling_flag = 1;
    	UARTprintf(" stop sampling %d\n", smpl_buff_idx);
    	break;
    case 2: // display sample
    	UARTprintf(" sample type %d\n", sample_type);
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    	{
			for(i=0; i<V_SAMPLE_COUNT; i++)
			{
				for(delay=0; delay<1000; delay++);
				UARTprintf(" %f, %f, %f \n", smpl_buff[0][i], smpl_buff[1][i], smpl_buff[2][i]);
			}
			//UARTprintf(" %f, %f\n", f_result[0], f_result[1]);
    	}
    	else
    		UARTprintf(" sampling data not ready  %d\n", smpl_buff_idx);
    	break;

    case 3:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = I_CURR_SAMPLE_TYPE;
    	UARTprintf("set I samples\n");

    	break;

    case 4:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = V_UVW_SAMPLE_TYPE;
   		UARTprintf("set Vuvw samples\n");

    	break;

    case 5:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = V_DC_SAMPLE_TYPE;
   		UARTprintf("set Vdc sample\n");

    	break;

    case 6:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = V_AB_SAMPLE_TYPE;
    	UARTprintf("set Vab samples\n");

    	break;

    case 7:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = PWM_SAMPLE_TYPE;
    	UARTprintf("set PWM samples\n");

    	break;

    case 8:
    	if(sampling_flag == 0 && stop_sampling_flag == 1)
    		sample_type = PWM2_SAMPLE_TYPE;
    	UARTprintf("set PWM2 samples\n");

    	break;

    case 9:
    	UARTprintf(" sample type %d\n", sample_type);

    	break;

    default:
    	UARTprintf("wrong parameter %d\n", cmd);
       	break;
    }

    return 0;

iadc_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_GET_ADC_I_SAMPLE].pcHelp);
    return 1;
}
#endif


unsigned long b = 0xFFFFFFFE;
//extern float MAIN_convert2InternalSpeedRef(int freq);
//extern int MAIN_convert2Speed(float speed);
extern void initParam(void);

#ifdef PWM_DUTY_TEST
extern uint16_t gFlag_PwmTest;
extern _iq gPwmData_Value;
extern uint16_t gFlagDCIBrake;
extern void dbg_enableSystem(void);
extern void dbg_disableSystem(void);
#endif

STATIC int dbg_tmpTest(int argc, char *argv[])
{
	int i, s_value, index;
//	int value;
//	_iq iq_val;
//	float f_val;

    if(argc > 3) goto tmp_err;

    index = atoi(argv[1]);
    if(index == 0)
    	index = argv[1][0];

    if(index == '0')
    {
#if 0
        for(i=100; i<400; i+=15)
        {
    //    	iq_val = _IQ((float)i);
    //    	value = _IQint(iq_val);
    //		_IQtoa(val_str, "%5.10f", f_val);
    //		UARTprintf("%d iq_int=%d iq=%d\n", i, value, (int)iq_val);

    	    s_value = i;
    	    f_val = MAIN_convert2InternalSpeedRef(s_value);
    	    value = MAIN_convert2Speed(f_val);
    	    UARTprintf("value = %d -> %d\n", s_value, value);
        }

        s_value = 120;
        f_val = MAIN_convert2InternalSpeedRef(s_value);
        value = MAIN_convert2Speed(f_val);
        UARTprintf("value = %d -> %d\n", s_value, value);
#else
        for(i=0; i<400; i+=30)
        {
        	s_value = FREQ_convertToSpeed(i);
        	UARTprintf("value = freq=%d -> rpm=%d\n", i, s_value);
        }
#endif
    }
    else if(index == 1)
    {
    	param.protect.ovl.tr_limit = 110;
    	param.protect.ovl.tr_duration = 10;
    	OVL_enbleOverloadTrip(1);
    	UARTprintf("OVL start = %d\n", (int)secCnt);
    }
    else if(index == 2)
    {
    	param.brk.dci_start_freq = 20.0;
    	param.brk.dci_block_time = 2.0;
    	param.brk.dci_braking_time = 10.0;
    	// enable DCI
    	UARTprintf("DCIB start = %d\n", (int)secCnt);
    }
    else if(index == 3) // check sizeof(int) -> 2byte
    {
    	unsigned int a=0xFFFE;
    	UARTprintf("a=%d b=%d \n", (unsigned int)(a+1), (unsigned int)(a+2));
    }
    else if(index == 4) //
    {
    	initParam();
    }
    else if(index == 5)
    {
//    	float_t f_val=3.14;
//    	UARTprintf("float test %f, sizeof(float)=%d\n", f_val, (int)sizeof(float));
//    	UARTprintf("sizeof(long)=%d sizeof(int)=%d, sizeof(char)=%d\n", (int)sizeof(long), (int)sizeof(int), (int)sizeof(char));
//    	uint16_t period_cycles = (uint16_t)(90.0*1000.0); // overflow
//    	UARTprintf("period = %d\n", period_cycles);
    	UARTprintf("input_voltage = %d, trip=%d\n", mtr.input_voltage, internal_status.trip_happened);
    }
    else if(index == 6)
    {

    	//UTIL_controlLed(HAL_Gpio_LED_R2, 1);
    	UTIL_controlLed(HAL_Gpio_LED_G, 1);

    	// REGEN GPIO test
//    	int on_off;
//    	on_off = atoi(argv[2]);
//    	if(on_off)
//    		UTIL_setRegenBit();
//    	else
//    		UTIL_clearRegenBit();
    }
    else if(index == 'g')
    {
    	UARTprintf(" Trip happened %d \n", internal_status.trip_happened);
    }
    else if(index == 'r')
    {
    	int i;
    	extern float_t array_Iu[];
    	UARTprintf(" Irms =%f \n", MAIN_getIave());
    	for(i=0; i<I_RMS_SAMPLE_COUNT; i++)
    		UARTprintf(" int_Iu %f \n", array_Iu[i]);
    }
    else if(index == 'k')
    {
    	MAIN_showPidGain();
    }
    else if(index == 'v')
    {
    	UARTprintf("OVL warn enable=%d\n", ovl_alarm_enable);
    }
    else if(index == 't')
    {
    	int ms_dur;
    	float_t f_dur;

    	ms_dur = atoi(argv[2]);
    	f_dur = (float_t)ms_dur;
    	TMR_startTimerSig(TIMER_TEST_TSIG, f_dur);
    	UARTprintf(" Test Timer %f sec start at %d \n", f_dur, (int)secCnt);
    }
#ifdef SUPPORT_AUTO_LOAD_TEST
    else if(index == 'l')
    {
    	bool sw_state=0;
    	//sw_state = UTIL_readSwGpio();
    	sw_state = TEST_readSwitch();
    	UARTprintf(" test SW %d \n", (int)sw_state);
    }
#endif
    else if(index == 'f')
    {
#ifdef SUPPORT_REGEN_GPIO
    	int enable;

    	enable = atoi(argv[2]);
    	if(enable == 1)
    		REGEN_start();
    	else
    		REGEN_end();

    	UARTprintf(" set REGEN enable=%d\n", enable);
#else
    	int duty;
    	duty = atoi(argv[2]);
    	UTIL_setRegenPwmDuty(duty);

    	UARTprintf(" set REGEN pwm duty=%d\n", duty);
#endif
    }
#ifdef PWM_DUTY_TEST
    else if(index == 'p')
    {
    	int enable=0;

    	if(argc != 3)
    	{
    		UARTprintf(" PWM test %d\n", gFlag_PwmTest);
    		return 0;
    	}

    	enable = atoi(argv[2]);
    	if(enable == 1)
    	{
			dbg_enableSystem();
			gFlag_PwmTest = true;
			UARTprintf(" enable PWM test \n");
    	}
    	else
    	{
			dbg_disableSystem();
			gFlag_PwmTest = false;
			UARTprintf(" disable PWM test \n");
    	}
    }
    else if(index == 'm')
    {
    	int pwm_data=50;
    	float_t pwm_f=0.0;

    	if(argc != 3)
    	{
    		UARTprintf(" Pwm data %d -> %f \n", pwm_data, pwm_f);
    		return 0;
    	}

    	pwm_data = atoi(argv[2]);
    	if(pwm_data >= 0 && pwm_data <= 100) // input duty as 0 ~ 100%
    	{
    		pwm_f = (float_t)(pwm_data-50)/100.0; // change -0.5 ~ 0.5
    		gPwmData_Value = _IQ(pwm_f);
    		UARTprintf(" Pwm data %d -> %f \n", pwm_data, pwm_f);
    	}
    	else
    		UARTprintf(" Pwm data error %d\n", pwm_data);

    }
    else if(index == 'j')
    {
    	int enable=0;

    	if(argc != 3)
    	{
    		UARTprintf(" PWM DC inject test %d, need >tmp p 1\n", gFlagDCIBrake);
    		return 0;
    	}

    	enable = atoi(argv[2]);
    	if(enable == 1)
    	{
			gFlagDCIBrake = true;
			UARTprintf(" enable PWM DC inject test \n");
    	}
    	else
    	{
			gFlagDCIBrake = false;
			UARTprintf(" disable PWM DC inject test \n");
    	}
    }
    else if(index == 'd')
    {
    	int16_t cur_rate=0.0;
    	float_t pwm_f=0.0, V_duty=0.0;

    	if(argc != 3)
    	{
    		UARTprintf(" set DCI Pwm data, need Current rate \n");
    		return 0;
    	}

    	cur_rate = atoi(argv[2]);
    	if(cur_rate >= 0 && cur_rate <= 200) // input duty as 0 ~ 200%
    	{
    		pwm_f = (float_t)(cur_rate)/100.0 * mtr.max_current*mtr.Rs*2.0;// 2*Rs for Y connection
    		V_duty = pwm_f*100.0 / MAIN_getVdcBus();
    		gPwmData_Value = _IQ(V_duty/100.0);
    		UARTprintf(" DCI Pwm data %d -> V=%f, V_percent=%f\n", cur_rate, (pwm_f/100.0), V_duty);
    	}
    	else
    		UARTprintf(" DCI Pwm a error %d\n", cur_rate);
    }
#endif
#ifdef SAMPLE_ADC_VALUE
    else if(index == 'z')
    {
    	UARTprintf(" PWM zero cnt=%d, val=%f neg=%f sat=%f \n", pwm_err_cnt, pwm_value, pwm_value_neg, pwm_value_sat);
    }
#endif



    return 0;

tmp_err:
	UARTprintf("%s\n", g_sCmdTable[DBG_CMD_VERSION].pcHelp);
    return 1;
}
#endif


void ProcessDebugCommand(void)
{
    //int size = 0;
    //int cnt = 0, cmdCnt;
    int ret;

    if(g_bNewCmd) {
        UARTgets((char*)cmdString, (unsigned long)NUM_OF_DEBUGCHAR);
#ifdef SUPPORT_DEBUG_TERMINAL
        {
            ret = CmdLineProcess((char*)cmdString);
            if(ret==CMDLINE_BAD_CMD)
            	UARTprintf("Invalid command.\n");

            UARTprintf("debug>");
        }
#endif
        g_bNewCmd = false;
    }
}

void dbg_logo(void)
{
	UARTprintf("\n*****************************************************************");
	UARTprintf("\n**%14sCompiled :    %4d/%02d/%02d   %10s %9s**", " ", BUILD_YEAR, BUILD_MONTH, BUILD_DAY, __TIME__, " ");
	UARTprintf("\n**%15sCopyright(C) Nara Control Co., Ltd.%11s**", " ", " ");
	UARTprintf("\n**        Motor control debug program for TMS320F28069M        **\n");
}

int EchoSetting(int argc, char *argv[])
{
    if(strcmp("on", argv[1]) == 0)
        UARTEchoSet(true);
    else if(strcmp("off", argv[1]) == 0)
        UARTEchoSet(false);
    else
        UARTprintf("echo <on/off>\n");

    return 0;
}


#endif
