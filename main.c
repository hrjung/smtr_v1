/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_foc/src/proj_lab05g.c
//! \brief Adjusting the speed controller FPU32
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB05G PROJ_LAB05G
//@{

//! \defgroup PROJ_LAB05G_OVERVIEW Project Overview
//!
//! Adjusting the supplied speed controller FPU32
//!

// **************************************************************************
// the includes

// system includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include "main.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#pragma CODE_SECTION(timer0ISR,"ramfuncs");
#endif

#include "uartstdio.h"

#include "inv_param.h"
#include "drive.h"
#include "state_func.h"
#include "freq.h"
#include "brake.h"

#include "timer_handler.h"
#include "protect.h"
#include "err_trip.h"
#include "nara_inv.h"

#include "motor_param.h"


#ifdef UNIT_TEST_ENABLED
#include "test/unity.h"
#endif

// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   3
//#define LED_BLINK_FREQ_Hz   1

#define COMMAND_NUMBER      2

#define MAIN_CONTROL_VF		0
#define MAIN_CONTROL_FOC	1

#define KRPM_SCALE_FACTOR		(1000.0)
// **************************************************************************
// the extern function
extern void dbg_logo(void);
extern void ProcessDebugCommand(void);
extern void init_test_param(void);
#ifdef SAMPLE_ADC_VALUE
extern void dbg_getSample(float_t val1, float_t val2, float_t val3);
#endif

#ifdef UNIT_TEST_ENABLED

extern void test_setFreqParam(void); // test_freq.c
extern void test_setSpeedParam(void); // test_speed.c
extern void test_setAccelTime(void);

//extern void test_processSpeedScaling(void); //test_resolution.c
//extern void test_processResolution(void);
extern void test_processConvertFreq(void); //test_resolution.c
//extern void test_processResolutionMinMax(void);
extern void test_processResolutionTargetFreq(void);

extern void test_setDciBrakeParam(void);
extern void test_setOverload(void);
extern void test_processDcVoltage(void);

extern void test_controlState(void);
extern void test_controlDrive(void);
extern void test_errorTrip(void); // test_trip.c

#endif
// **************************************************************************
// the globals

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif
HAL_Handle halHandle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif
USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);


#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
#ifdef CSM_ENABLE
#pragma DATA_SECTION(ctrl,"rom_accessed_data");
#endif
CTRL_Obj ctrl;				//v1p7 format
#endif

uint16_t gLEDcnt = 0;


volatile MOTOR_Vars_t gMotorVars; // = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef CSM_ENABLE
extern uint16_t *econst_start, *econst_end, *econst_ram_load;
extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
#endif


#ifdef SUPPORT_VF_CONTROL
// define Angle Generate
ANGLE_GEN_Handle angle_genHandle;
ANGLE_GEN_Obj    angle_gen;

// define Vs per Freq
VS_FREQ_Handle vs_freqHandle;
VS_FREQ_Obj    vs_freq;

_iq gVbus_k_0 = _IQ(0.0);
_iq gVbus_k_1 = _IQ(0.0);
_iq gVbus_lpf = _IQ(0.0);

FILTER_SO_Obj gVbusFilter;
FILTER_SO_Handle gVbusFilterHandle;

_iq gOneOverDcBus;

//#define VOLTAGE_FILTER_BETA (USER_DCBUS_POLE_rps/(float_t)USER_CTRL_FREQ_Hz)
#endif

#ifdef SUPPORT_FIELD_WEAKENING
FW_Obj fw;
FW_Handle fwHandle;

_iq Iq_Max_pu;
#endif

#ifdef SUPPORT_FLYING_START
// define Flying Start (FS) variables
FS_Obj fs;
FS_Handle fsHandle;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;



dev_param_st	dev_param;
dev_const_st	dev_const;
motor_param_st mtr;
inverter_param_st param;
internal_status_st internal_status;

monitor_param_st mnt;
inv_state_st state_param = {STATE_STOP, 0, STOP};

float_t sf4pu_krpm, sf4krpm_pu;
float_t temp_spd_ref=0.0;


float_t direction = 1.0;
int for_rev_flag=0; //flag for forward <-> reverse drive

// test for scaling
_iq conv_ref = _IQ(0.1);
int nv_init=0;

uint16_t block_count=0;
#ifdef SUPPORT_I_RMS_MEASURE
float_t array_Iu[I_RMS_SAMPLE_COUNT], array_Iv[I_RMS_SAMPLE_COUNT], array_Iw[I_RMS_SAMPLE_COUNT];
float_t total_Iu, total_Iv, total_Iw;
int i_pos=0;
int i_ready_flag=0;
#endif

#ifdef PWM_DUTY_TEST
uint16_t gFlag_PwmTest = false;
_iq gPwmData_Value = _IQ(0.);
uint16_t gFlag_PwmStepTest = false;
int pwm_cnt=0;
uint16_t delay_count=10;

int p2n=0;
int n2p=0;
uint16_t p_count=0;
uint16_t n_count=0;
#endif

#ifdef SAMPLE_ADC_VALUE
#define I_SAMPLE_COUNT		100
#define V_SAMPLE_COUNT		270

#define I_CURR_SAMPLE_TYPE  	0
#define V_UVW_SAMPLE_TYPE		1
#define V_DC_SAMPLE_TYPE		2
#define V_AB_SAMPLE_TYPE		3
#define PWM_SAMPLE_TYPE			4
#define PWM2_SAMPLE_TYPE		5
#define PHASOR_SAMPLE_TYPE		6
#define V_AB_PHASOR_SAMPLE_TYPE	7
#define PWM_ERR_TYPE			8
#define PWM_PERIOD_COUNT_TYPE	9
#define I_RMS_SAMPLE_TYPE		10

//#define ADC_AVERAGE_VALUE	2000
extern float_t smpl_buff[3][V_SAMPLE_COUNT];
extern int sampling_flag, stop_sampling_flag;
extern int smpl_buff_idx;
extern int sample_type;
//int start_calc_rms=0;
//int32_t i_adc[3];
//float_t Isample[3][I_SAMPLE_COUNT];
//float_t I_total_sq[3], I_ave[3];
//uint16_t smp_idx;
//void initCurrentBuffer(void);
#endif

#ifdef SUPPORT_VF_CONTROL
void updateGlobalVariables_motor4Vf(CTRL_Handle handle);
#endif

extern uint32_t secCnt;
extern uint16_t MyModbusAddr;

//interrupt void xint1_isr(void);
// **************************************************************************
// the functions

STATIC _iq MAIN_getActualSpeedWithDirection(void)
{
	float_t ref_krpm;

	ref_krpm = FREQ_convertToSpeed(STA_getTargetFreq())/1000.0;

	return _IQ(ref_krpm*direction);
}

#if 0
float MAIN_scaleDown2Krpm(int speed)
{
	return (float)(speed/KRPM_SCALE_FACTOR);
}

int MAIN_scaleUp2rpm(float speed_krpm)
{
	return (int)(speed_krpm*KRPM_SCALE_FACTOR + 0.5); // for round
}

float MAIN_convert2InternalSpeedRef(int speed)
{
	return (MAIN_scaleDown2Krpm(speed)*dev_param.gear_ratio);
}
#endif

int MAIN_convert2Speed(float speedRef)
{
	float speed = speedRef;

	if(speedRef == 0.0) return 0;

	//if(speedRef < 0) speed = -1.0*speedRef;

	speed /= dev_param.gear_ratio;
	speed *= KRPM_SCALE_FACTOR;

//	if(speed > dev_const.spd_rpm_max) speed = dev_const.spd_rpm_max;
//	if(speed < dev_const.spd_rpm_min) speed = dev_const.spd_rpm_min;

	return (int)(speed + 0.5);
}

int MAIN_getCurrentSpeed(void)
{
	float f_spd = _IQtoF(gMotorVars.Speed_krpm);

	// process only positive value, since negative value means reverse direction
	if(gMotorVars.Speed_krpm < 0)
		f_spd *= (-1.0);

	return MAIN_convert2Speed(f_spd);
}

float_t MAIN_convert2Freq(float_t spd_krpm)
{
#ifdef UNIT_TEST_ENABLED
	float_t speed_rpm = spd_krpm*1000;
#else
	float_t speed_rpm;

	if(DRV_isVfControl())
	{
		speed_rpm = spd_krpm; // already rmp
	}
	else
	{
		speed_rpm = _IQtoF(gMotorVars.Speed_krpm) * KRPM_SCALE_FACTOR;
		if(gMotorVars.Speed_krpm < 0.0)
			speed_rpm *= (-1.0);
	}
#endif

	return (speed_rpm*mtr.poles/60.0);
}

_iq MAIN_getAccelRate(void)
{
	float_t value = STA_getTrajResolution();

	if(value == 0.0)
		 return gMotorVars.MaxAccel_krpmps;
	else
		return _IQ(value);
}

float_t MAIN_getVdcBus(void)
{
	return _IQtoF(gAdcData.dcBus)*USER_IQ_FULL_SCALE_VOLTAGE_V;
}

float_t MAIN_getDC_lfp(void)
{
	//return _IQtoF(gAdcData.dcBus)*USER_IQ_FULL_SCALE_VOLTAGE_V;
	return _IQtoF(gVbus_lpf)*USER_IQ_FULL_SCALE_VOLTAGE_V;
}

float_t MAIN_getIu(void)
{
	return internal_status.Iu_inst;
}

float_t MAIN_getIv(void)
{
	return internal_status.Iv_inst;
}

float_t MAIN_getIw(void)
{
	return internal_status.Iw_inst;
}

float_t MAIN_getIave(void)
{
	return internal_status.Iu_rms;
}

inline void MAIN_readCurrent(void)
{
	  // update instant current value
	  internal_status.Iu_inst = _IQtoF(gAdcData.I.value[0])*USER_IQ_FULL_SCALE_CURRENT_A;
	  internal_status.Iv_inst = _IQtoF(gAdcData.I.value[1])*USER_IQ_FULL_SCALE_CURRENT_A;
	  internal_status.Iw_inst = _IQtoF(gAdcData.I.value[2])*USER_IQ_FULL_SCALE_CURRENT_A;

	  if(internal_status.Iu_inst > 15.0) internal_status.Iu_inst = 15.0;
	  else if(internal_status.Iu_inst < -15.0) internal_status.Iu_inst = -15.0;

	  if(internal_status.Iv_inst > 15.0) internal_status.Iv_inst = 15.0;
	  else if(internal_status.Iv_inst < -15.0) internal_status.Iv_inst = -15.0;

	  if(internal_status.Iw_inst > 15.0) internal_status.Iw_inst = 15.0;
	  else if(internal_status.Iw_inst < -15.0) internal_status.Iw_inst = -15.0;
}

#ifdef SUPPORT_I_RMS_MEASURE

int MAIN_getSampleCountLimit(void)
{
	//float_t pwm_freq[] = {4000.0, 8000.0, 12000.0, 16000.0};

#ifdef SUPPORT_USER_VARIABLE
	return (int)(gUserParams.pwmPeriod_kHz*1000.0/(STA_getCurFreq()*(float_t)I_RMS_SAMPLE_COUNT));
#else
	return (int)(USER_PWM_FREQ_kHz*1000.0/(STA_getCurFreq()*(float_t)I_RMS_SAMPLE_COUNT));
#endif
}

inline int MAIN_isSampleRequired(void)
{
	static int sample_cnt=0;
	int sample_limit;

	sample_cnt++;

	sample_limit = MAIN_getSampleCountLimit();
	if(sample_cnt > sample_limit)
	{
		sample_cnt = 0;
		return 1;
	}

	return 0;
}

void MAIN_initIarray(void)
{
	int i;

	for(i=0; i<I_RMS_SAMPLE_COUNT; i++)
	{
		array_Iu[i] = 0.0;
		array_Iv[i] = 0.0;
		array_Iw[i] = 0.0;
	}
	i_pos=0;
	i_ready_flag=0;

	total_Iu = 0.0;
	total_Iv = 0.0;
	total_Iw = 0.0;
}

inline void MAIN_calculateIrms(void)
{
	//int i;
	float_t bk_Iu=0.0, bk_Iw=0.0, bk_Iv=0.0;


	if(i_pos >= I_RMS_SAMPLE_COUNT)
	{
		i_ready_flag=1;
		i_pos=0;
	}

	bk_Iu = array_Iu[i_pos];
	bk_Iv = array_Iv[i_pos];
	bk_Iw = array_Iw[i_pos];

	array_Iu[i_pos] = internal_status.Iu_inst*internal_status.Iu_inst;
	array_Iv[i_pos] = internal_status.Iv_inst*internal_status.Iv_inst;
	array_Iw[i_pos] = internal_status.Iw_inst*internal_status.Iw_inst;

	total_Iu += array_Iu[i_pos];
	total_Iv += array_Iv[i_pos];
	total_Iw += array_Iw[i_pos];
	if(i_ready_flag) // calculate RMS after array is full
	{
		total_Iu -= bk_Iu;
		total_Iv -= bk_Iv;
		total_Iw -= bk_Iw;

		internal_status.Iu_rms = sqrtf(total_Iu/(float_t)I_RMS_SAMPLE_COUNT);
		internal_status.Iv_rms = sqrtf(total_Iv/(float_t)I_RMS_SAMPLE_COUNT);
		internal_status.Iw_rms = sqrtf(total_Iw/(float_t)I_RMS_SAMPLE_COUNT);
	}
	i_pos++;
}
#endif

int MAIN_isOverCurrent(void)
{
#if 0
	if(MAIN_isSystemEnabled()
#ifdef SAMPLE_ADC_VALUE
		&& start_calc_rms
#endif
	)
#endif
	{
		if(fabsf(internal_status.Iu_inst) > OVER_CURRENT_INSTANT_VALUE
			|| fabsf(internal_status.Iw_inst) > OVER_CURRENT_INSTANT_VALUE)
			//|| fabsf(internal_status.Iv_inst) > OVER_CURRENT_INSTANT_VALUE
		{
			internal_status.oc_count++;
			if(internal_status.oc_count > OVER_CURRENT_COUNT_LIMIT)
			{
				ERR_setTripFlag(TRIP_REASON_OVER_CURRENT);
				return 1;
			}
		}
		else
			internal_status.oc_count=0;
	}

	return 0;
}

int MAIN_isMissingIphase(void)
{

	// Iu phase is missing
	if(internal_status.Iu_rms < MISSING_PHASE_RMS_VALUE)
	{
		internal_status.Iu_miss_cnt++;
		if(internal_status.Iu_miss_cnt > CURRENT_MISS_COUNT_LIMIT)
		{
			ERR_setTripFlag(TRIP_REASON_Iu_PHASE_MISS);
			return 1;
		}
	}
	else
		internal_status.Iu_miss_cnt = 0;

	// Iu phase is missing
	if(internal_status.Iw_rms < MISSING_PHASE_RMS_VALUE)
	{
		internal_status.Iw_miss_cnt++;
		if(internal_status.Iw_miss_cnt > CURRENT_MISS_COUNT_LIMIT)
		{
			ERR_setTripFlag(TRIP_REASON_Iw_PHASE_MISS);
			return 1;
		}
	}
	else
		internal_status.Iw_miss_cnt = 0;

	// Iv phase is missing
	if(internal_status.Iw_rms == -internal_status.Iu_rms)
	{
		internal_status.Iw_miss_cnt++;
		if(internal_status.Iw_miss_cnt > CURRENT_MISS_COUNT_LIMIT)
		{
			ERR_setTripFlag(TRIP_REASON_Iv_PHASE_MISS);
			return 1;
		}
	}
	else
		internal_status.Iv_miss_cnt = 0;

	return 0;
}


void MAIN_setCurrentFreq(void)
{
#ifdef UNIT_TEST_ENABLED
	return;
#else
	STA_setCurSpeed(_IQtoF(gMotorVars.Speed_krpm));
//	if(DRV_isVfControl())
//		STA_setCurSpeed(spd_krpm);
//	else
//		STA_setCurSpeed(gMotorVars.Speed_krpm);
#endif
}

void MAIN_setJumpSpeed(int index, float_t low, float_t high)
{
	float_t low_spd, high_spd;

	if(low != 0.0)
	{
		low_spd = FREQ_convertToSpeed(low)/1000.0;
		dev_const.spd_jmp[index].low = _IQ(low_spd*sf4krpm_pu);
	}

	if(high != 0.0)
	{
		high_spd = FREQ_convertToSpeed(high)/1000.0;
		dev_const.spd_jmp[index].high = _IQ(high_spd*sf4krpm_pu);
	}

//	dev_const.spd_jmp[index].low_pu = low_spd*sf4krpm_pu;
//	dev_const.spd_jmp[index].high_pu = high_spd*sf4krpm_pu;
//
//	dev_const.spd_jmp[index].low_spd = low_spd;
//	dev_const.spd_jmp[index].high_spd = high_spd;
}

_iq MAIN_avoidJumpSpeed(_iq spd_pu)
{
	int i;
	_iq conv_spd_pu = spd_pu;

	for(i=0; i<MAX_JUMP_FREQ_NUM; i++)
	{
		if(dev_const.spd_jmp[i].enable == 1
		   && spd_pu > dev_const.spd_jmp[i].low
		   && spd_pu < dev_const.spd_jmp[i].high)
		{
			if(STA_isDecelState())
				return dev_const.spd_jmp[i].high;
			else
				return dev_const.spd_jmp[i].low;
		}
	}

	return conv_spd_pu;
}

void MAIN_setDeviceConstant(void)
{
	memset(&dev_const, 0, sizeof(dev_const));

//	dev_const.spd_rpm_min = (mtr.rpm_min/dev_param.gear_ratio);
//	dev_const.spd_rpm_max = (mtr.rpm_max/dev_param.gear_ratio);
//	dev_const.regen_limit = DC_VOLTAGE_END_REGEN_LEVEL;
	dev_const.trip_level = mtr.max_current*(float_t)param.protect.ovl.tr_limit/100.0;
	dev_const.warn_level = mtr.max_current*(float_t)param.protect.ovl.wr_limit/100.0;
	dev_const.ovc_level = mtr.max_current*2.0;

	dev_const.regen_max_V = sqrtf(0.9*param.protect.regen.resistance*(float_t)param.protect.regen.power);

	//set additional flag

	state_param.inv = STATE_STOP;
	state_param.sel_in = NOT_USED;
	state_param.run = STOP;

	internal_status.Iu_inst = 0.0;
	internal_status.Iv_inst = 0.0;
	internal_status.Iw_inst = 0.0;

	internal_status.Iu_rms = 0.0;
	internal_status.Iv_rms = 0.0;
	internal_status.Iw_rms = 0.0;

	internal_status.Vu_inst = 0.0;
	internal_status.Vv_inst = 0.0;
	internal_status.Vw_inst = 0.0;

	internal_status.Vdc_inst = 0.0;

	internal_status.Vab_pu[0] = 0.0;
	internal_status.Vab_pu[1] = 0.0;

	internal_status.pwmData[0] = 0.0;
	internal_status.pwmData[1] = 0.0;
	internal_status.pwmData[2] = 0.0;

	internal_status.accel_resol = 0.0;
	internal_status.decel_resol = 0.0;

	internal_status.relay_enabled = 0;
	internal_status.regen_enabled = 0;
	internal_status.trip_happened = 0;
	internal_status.shaft_brake_enabled = 0;
	internal_status.emergency_stop = 0;
	internal_status.external_trip = 0;

	internal_status.oc_count = 0;

}

#if 1
int dc_pwm_off=0;
int MAIN_processDCBrake(void)
{
	_iq dc_value=_IQ(0.0);
	static int block_flag=0, dc_brake_flag=0;

	if(param.brk.method != DC_INJECT_BRAKE) return 0;

	switch(DCIB_getState())
	{
	case DCI_NONE_STATE:
		// nothing to do
		block_flag=0;
		dc_brake_flag=0;
		dc_pwm_off=0;
		break;

	case DCI_BLOCK_STATE:
		// PWM off
		if(block_flag == 0)
		{
			HAL_disablePwm(halHandle);
			block_flag = 1;
			//UARTprintf("DCI BLOCK off PWM, at %d\n", (int)secCnt);
		}
		break;

	case DCI_DC_BRAKE_STATE:
		if(dc_brake_flag == 0)
		{
			HAL_enablePwm(halHandle);
			dc_value = _IQ(param.brk.dci_braking_rate/100.0);
			dc_brake_flag = 1;
			//UARTprintf("DCI BRAKE PWM on, at %d\n", (int)secCnt);
		}
		// apply DC voltage
		gPwmData.Tabc.value[0] = dc_value;
		gPwmData.Tabc.value[1] = dc_value;
		gPwmData.Tabc.value[2] = dc_value;
		break;

	case DCI_PWM_OFF_STATE:
		// PWM off
		if(dc_pwm_off == 0)
		{
			MAIN_disableSystem();
			dc_pwm_off = 1;
			//UARTprintf("DCI PWM off, at %d\n", (int)secCnt);
		}

		break;
	}

	return 1;
}
#endif

void initParam(void)
{
	UTIL_setScaleFactor();

	memset(&param, 0, sizeof(param));

	// default ctrl setting
	param.ctrl.value = 10.0;

//	FREQ_setJumpFreqRange(0, 20.0, 30.0);
//	FREQ_setJumpFreqRange(1, 40.0, 50.0);

	DRV_setAccelTime(10.0); // 10.0 sec
	DRV_setDecelTime(10.0);

	DRV_enableVfControl(); //default
	param.ctrl.foc_torque_limit = 180.0;
	param.ctrl.spd_P_gain = 1000.0;
	param.ctrl.spd_I_gain = 100.0;

	DRV_setEnergySave(0); //off
	DRV_setVoltageBoost(0); //off

	//DRV_setPwmFrequency(PWM_6KHz);

	// default brake setting
	BRK_setBrakeMethod(REDUCE_SPEED_BRAKE);
	BRK_setBrakeTIME(5);
	BRK_setThreshold(5.0);

	//default Dci brake
	param.brk.dci_start_freq = 5.0; // start at 5Hz
	param.brk.dci_block_time = 0.5;  // 0.5 sec
	param.brk.dci_braking_rate = 50.0; // 50% rate
	param.brk.dci_braking_time = 2.0; // 1 sec brake

	// default err_info setting
	ERR_clearTripData();

	// default protect setting
	OVL_setWarningLevel(120); // Samyang motor's SF=1.15
	param.protect.ovl.wr_duration = 30;
	param.protect.ovl.enable = 1;
	OVL_setTripLevel(150);
	param.protect.ovl.tr_duration = 10;

	param.protect.regen.resistance = 100.0;
	param.protect.regen.power = 400;
	param.protect.regen.thermal = 0.0;
	param.protect.regen.band = 0;

}

void init_global(void)
{
	gMotorVars.Flag_enableSys = false;
	gMotorVars.Flag_Run_Identify = false;
	gMotorVars.Flag_MotorIdentified = false;
	gMotorVars.Flag_enableForceAngle = false;
	gMotorVars.Flag_enableFieldWeakening = false;
	gMotorVars.Flag_enableRsRecalc = false; // false -> true
	gMotorVars.Flag_enableUserParams = true;
	gMotorVars.Flag_enableOffsetcalc = false; // false -> true
	gMotorVars.Flag_enablePowerWarp = true;
	gMotorVars.Flag_enableSpeedCtrl = false;

	gMotorVars.Flag_enableRun = false;
	gMotorVars.Flag_RunState = false;
	gMotorVars.Flag_enableFlyingStart = false;

	gMotorVars.CtrlState = CTRL_State_Idle;
	gMotorVars.EstState = EST_State_Idle;

	gMotorVars.UserErrorCode = USER_ErrorCode_NoError;

	gMotorVars.CtrlVersion.rsvd = 0;
	gMotorVars.CtrlVersion.targetProc = CTRL_TargetProc_Unknown;
	gMotorVars.CtrlVersion.major = 0;
	gMotorVars.CtrlVersion.minor = 0;

	gMotorVars.IdRef_A = _IQ(0.0);
	gMotorVars.IqRef_A = _IQ(0.0);
	gMotorVars.SpeedRef_pu = _IQ(0.0);
	gMotorVars.SpeedRef_krpm = _IQ(0.1);
	gMotorVars.SpeedTraj_krpm = _IQ(0.0);
	gMotorVars.MaxAccel_krpmps = _IQ(0.2);
	gMotorVars.Speed_krpm = _IQ(0.0);

	gMotorVars.OverModulation = _IQ(USER_MAX_VS_MAG_PU);
	gMotorVars.RsOnLineCurrent_A = _IQ(0.1 * gUserParams.maxCurrent);
	gMotorVars.SvgenMaxModulation_ticks = 400;
	gMotorVars.Flux_Wb = _IQ(0.0);
	gMotorVars.Torque_Nm = _IQ(0.0);

	gMotorVars.MagnCurr_A = 0.0;
	gMotorVars.Rr_Ohm = 0.0;
	gMotorVars.Rs_Ohm = 0.0;
	gMotorVars.RsOnLine_Ohm = 0.0;
	gMotorVars.Lsd_H = 0.0;
	gMotorVars.Lsq_H = 0.0;
	gMotorVars.Flux_VpHz = 0.0;

	//#if 0 //hrjung not used
	gMotorVars.ipd_excFreq_Hz = 0.0;
	gMotorVars.ipd_Kspd = _IQ(0.0);
	gMotorVars.ipd_excMag_coarse_pu = _IQ(0.0);
	gMotorVars.ipd_excMag_fine_pu = _IQ(0.0);
	gMotorVars.ipd_waitTime_coarse_sec = 0.0;
	gMotorVars.ipd_waitTime_fine_sec = 0.0;
	//#endif

	gMotorVars.Kp_spd = _IQ(0.0);
	gMotorVars.Ki_spd = _IQ(0.0);

	gMotorVars.Kp_Idq = _IQ(0.0);
	gMotorVars.Ki_Idq = _IQ(0.0);

	gMotorVars.Vd = _IQ(0.0);
	gMotorVars.Vq = _IQ(0.0);
	gMotorVars.Vs = _IQ(0.0);
	gMotorVars.VsRef = _IQ(0.8 * USER_MAX_VS_MAG_PU);
	gMotorVars.VdcBus_kV = _IQ(0.0);

	gMotorVars.Id_A = _IQ(0.0);
	gMotorVars.Iq_A = _IQ(0.0);
	gMotorVars.Is_A = _IQ(0.0);

	gMotorVars.I_bias.value[0] = 0;
	gMotorVars.I_bias.value[1] = 0;
	gMotorVars.I_bias.value[2] = 0;
	gMotorVars.V_bias.value[0] = 0;
	gMotorVars.V_bias.value[1] = 0;
	gMotorVars.V_bias.value[2] = 0;

	gMotorVars.SpeedSet_krpm = _IQ(0.6);

	//#if 0 //hrjung not used
	gMotorVars.angle_sen_pu = _IQ(0.0);
	gMotorVars.angle_est_pu = _IQ(0.0);
	gMotorVars.speed_sen_pu = _IQ(0.0);
	gMotorVars.speed_est_pu = _IQ(0.0);

	gMotorVars.speedHigh_hall2fast_pu = _IQ(0.0);
	gMotorVars.speedLow_hall2fast_pu = _IQ(0.0);
	gMotorVars.IdSet_A = _IQ(0.0);
	gMotorVars.IqSet_A = _IQ(0.0);
	gMotorVars.IdRef_pu = _IQ(0.0);
	gMotorVars.IqRef_pu = _IQ(0.0);
}

void init_test_param(void)
{
	//device param
	dev_param.nv_ver = 1;
	dev_param.nv_size = sizeof(inverter_param_st)*2;
	dev_param.dev_type = 10;
	dev_param.motor_type = 5;
	dev_param.serial_num = 1111;
	dev_param.hw_ver_maj = 1;
	dev_param.hw_ver_min = 1;
	dev_param.gear_ratio = 1;

	//motor params
	mtr.effectiveness = TEST_MOTOR_EFFECTIVENESS;
	mtr.slip_rate = TEST_MOTOR_SLIP_RATE; // offset speed for VF (krpm)
	mtr.input_voltage = TEST_MOTOR_VOLTAGE_IN;
	mtr.rated_freq = TEST_MOTOR_RATED_FREQ;
//	mtr.rpm_min = 300;
//	mtr.rpm_max = 1800;
	mtr.capacity = TEST_MOTOR_CAPACITY;
	mtr.poles = TEST_MOTOR_NUM_POLE_PAIRS;
	mtr.Rr = TEST_MOTOR_Rr;
	mtr.Rs = TEST_MOTOR_Rs;
	mtr.Ls = TEST_MOTOR_Ls;
	mtr.noload_current = TEST_MOTOR_NOLOAD_CURRENT;
	mtr.max_current = TEST_MOTOR_MAX_CURRENT;
}

void main(void)
{
  //int i;
  //float_t conv_cur=0.0;
  int first_trip_f=1;
  uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);

  #ifdef CSM_ENABLE
  //copy .econst to unsecure RAM
  if(*econst_end - *econst_start)
    {
      memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
    }

  //copy .switch ot unsecure RAM
  if(*switch_end - *switch_start)
    {
      memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
    }
  #endif
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));

  init_global();

  //hrjung read initial parameter from NVM
  init_test_param(); // NV data initialize, will be removed after NV enabled

  //param.ctrl.pwm_freq = PWM_4KHz; //test

  // initialize the user parameters
#ifdef SUPPORT_USER_VARIABLE
  USER_setParams(&gUserParams, &mtr);
#else
  USER_setParams(&gUserParams);
#endif

  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);

  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }

  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);

#ifdef I2C_DEFINE
  // enable I2C
  I2CStdioInit(halHandle);
#endif

#ifdef SPI_DEFINE
  //SPI-A : slave
  spi_fifo_init(halHandle->spiAHandle);
  spi_init(halHandle->spiAHandle);
  //SPI-B : master
  spi_fifo_init(halHandle->spiBHandle);
  spi_init(halHandle->spiBHandle);
#endif

  init_test_param(); // NV data initialize, will be removed after NV enabled
  initParam();

  MAIN_setDeviceConstant();
  UTIL_setRegenPwmDuty(0);
  PROT_init(mtr.input_voltage);
  //DRV_setPwmFrequency(PWM_4KHz); //test
  dev_param.gear_ratio = 1; //test

#ifdef SUPPORT_USER_VARIABLE

  // initialize the user parameters
  USER_setParams(&gUserParams, &mtr);

  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);

  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }

  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);

#endif

  // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif

  controller_obj = (CTRL_Obj *)ctrlHandle;

  {
    CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion = version;
  }

  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);

#ifdef SUPPORT_FIELD_WEAKENING
  // Initialize field weakening
  fwHandle = FW_init(&fw,sizeof(fw));


  // Disable field weakening
  FW_setFlag_enableFw(fwHandle, false);


  // Clear field weakening counter
  FW_clearCounter(fwHandle);


  // Set the number of ISR per field weakening ticks
  FW_setNumIsrTicksPerFwTick(fwHandle, FW_NUM_ISR_TICKS_PER_CTRL_TICK);


  // Set the deltas of field weakening
  FW_setDeltas(fwHandle, FW_INC_DELTA, FW_DEC_DELTA);


  // Set initial output of field weakening to zero
  FW_setOutput(fwHandle, _IQ(0.0));

  // Set the field weakening controller limits
  FW_setMinMax(fwHandle,_IQ(USER_MAX_NEGATIVE_ID_REF_CURRENT_A/USER_IQ_FULL_SCALE_CURRENT_A),_IQ(0.0));
#endif

  // setup faults
  HAL_setupFaults(halHandle);


  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);

#ifdef IPM_DEFINE
  // set GPIO31 to XINT1 for FAULT_IPM
  SetGpioInterrupt();
#endif

  // enable the SCI interrupts
  UARTStdioInit(halHandle, SCI_A);

  //initialize timer variable
  TMR_init();


#ifdef SUPPORT_VF_CONTROL

  // initialize the angle generate module
  angle_genHandle = ANGLE_GEN_init(&angle_gen,sizeof(angle_gen));
  ANGLE_GEN_setParams(angle_genHandle, gUserParams.iqFullScaleFreq_Hz, gUserParams.ctrlPeriod_sec);

	// initialize the Vs per Freq module
  vs_freqHandle = VS_FREQ_init(&vs_freq,sizeof(vs_freq));
  VS_FREQ_setParams(vs_freqHandle,  gUserParams.iqFullScaleFreq_Hz, gUserParams.iqFullScaleVoltage_V, gUserParams.maxVsMag_pu);
  //gUserParams.VF_freq_low = mtr.input_voltage*param.ctrl.v_boost/100.0;
  VS_FREQ_setProfile(vs_freqHandle, gUserParams.VF_freq_low, gUserParams.VF_freq_high, gUserParams.VF_volt_min, gUserParams.VF_volt_max);

  {
	  float_t	voltage_filter_beta = (USER_DCBUS_POLE_rps/(float_t)gUserParams.ctrlFreq_Hz);
     _iq a1 = _IQ((voltage_filter_beta - 2.0)/(voltage_filter_beta + 2.0));
     _iq a2 = _IQ(0.0);
     _iq b0 = _IQ(voltage_filter_beta/(voltage_filter_beta + 2.0));
     _iq b1 = _IQ(voltage_filter_beta/(voltage_filter_beta + 2.0));
     _iq b2 = _IQ(0.0);

	  gVbusFilterHandle = FILTER_SO_init(&(gVbusFilter),sizeof(gVbusFilter));

	  FILTER_SO_setDenCoeffs(gVbusFilterHandle,a1,a2);
	  FILTER_SO_setNumCoeffs(gVbusFilterHandle,b0,b1,b2);

	  FILTER_SO_setInitialConditions(gVbusFilterHandle,_IQ(0.0),_IQ(0.0),_IQ(0.0),_IQ(0.0));
  }
#endif

#ifdef SUPPORT_FLYING_START
  // Initialize Flying Start (FS)
  fsHandle = FS_init(&fs,sizeof(fs));

  // Disable Flying Start (FS)
  FS_setFlag_enableFs(fsHandle, false);

  // Clear Flying Start(FS) check time count
  FS_clearCntCheckTime(fsHandle);

  // Set Flying Start(FS) minimum transition speed
  FS_setSpeedFsMin_krpm(fsHandle, ctrlHandle, FS_SPEED_MIN);

  // set Flying Start(FS) maximum check time
  FS_setMaxCheckTime(fsHandle, FS_MAX_CHECK_TIME);

  gMotorVars.Flag_enableSpeedCtrl = true;		// enable speed close loop control
  gMotorVars.Flag_enableFlyingStart = true;		// enable Flying Start
#endif

#ifdef SUPPORT_DEBUG_GRAPH
  // Initialize Datalog
  datalogHandle = DATALOG_init(&datalog,sizeof(datalog));

  // Connect inputs of the datalog module
  datalog.iptr[0] = (int32_t *)&gMotorVars.Rs_Ohm;		// datalogBuff[1]
  datalog.iptr[1] = (int32_t *)&gMotorVars.RsOnLine_Ohm;	// datalogBuff[2]
  datalog.iptr[2] = (int32_t *)&gAdcData.vdc_adc;		// datalogBuff[2]

  datalog.Flag_EnableLogData = true;
  datalog.Flag_EnableLogOneShot = false;
#endif

  // enable global interrupts
  HAL_enableGlobalInts(halHandle);

  // enable debug interrupts
  HAL_enableDebugInt(halHandle);

  // disable the PWM
  HAL_disablePwm(halHandle);

  //hrjung enable the Timer 0 interrupts
  HAL_enableTimer0Int(halHandle);

  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);

  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

#ifdef UNIT_TEST_ENABLED
    //UARTFlushTx(0);
    UARTprintf("--Unit Test Running\n");

	UNITY_BEGIN();

//	//RUN_TEST(test_setSpeedParam);
//	RUN_TEST(test_setFreqParam); //test_freq.c
//	RUN_TEST(test_setAccelTime); //test_speed.c
//
//	//RUN_TEST(test_processSpeedScaling); //test_resolution.c
//	RUN_TEST(test_processConvertFreq); //test_resolution.c
//	//RUN_TEST(test_processResolution);
//	RUN_TEST(test_processResolutionTargetFreq);
//
//	RUN_TEST(test_setDciBrakeParam); //test_dci_brake.c
//
//	RUN_TEST(test_setOverload); // test_protect.c
//	//RUN_TEST(test_processDcVoltage); not ready
//
//	RUN_TEST(test_controlState); //test_state.c
//
//	RUN_TEST(test_controlDrive); //test_drive.c
//
//	RUN_TEST(test_setMultiControlDin);
//
	RUN_TEST(test_errorTrip); // test_trip.c

	UNITY_END();

	UARTprintf(" End of Unit Test\n");
	//UARTprintf(" regen_resist %f trip=%d %d\n", regen_resistance.value, internal_status.trip_happened, internal_status.nv_error_index);
	//UARTFlushTx(0);
	while(1);
#endif


  // set the voltage bias
//  HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(gUserParams.V_A_Offset));
//  HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(gUserParams.V_B_Offset));
//  HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(gUserParams.V_C_Offset));

  //DRV_enableFocControl(); // FOC test

  // debug command print
  UARTprintf("Please, type help for command list \n");
  dbg_logo();
  UARTprintf("debug>");

  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVars.Flag_enableSys))
	{
        processProtection();

        //TODO : should find correct location
        state_param.inv = STA_control();

        //debug command for Motor stop
        ProcessDebugCommand();

        MAIN_readCurrent();
        //for DC monitoring
        gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));
  	    internal_status.Vdc_inst = _IQtoF(gAdcData.dcBus)*USER_IQ_FULL_SCALE_VOLTAGE_V;

#ifdef SAMPLE_ADC_VALUE
  	    if(sample_type == V_DC_SAMPLE_TYPE)
  	    	dbg_getSample(internal_status.Vdc_inst, 0, 0);
#endif
//  	    MAIN_isOverCurrent();
//  		if(MAIN_isTripHappened() && first_trip_f)
//  		{
//  			first_trip_f=0;
//  	    	UARTprintf("Trip happened %d\n", internal_status.trip_happened);
//  		}
	}

#ifdef SUPPORT_FIELD_WEAKENING
    gMotorVars.Flag_enableFieldWeakening = true;
#endif

    // Enable the Library internal PI.  Iq is referenced by the speed PI now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, true);

    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
      {
        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams);


        // enable/disable Rs recalibration during motor startup
#ifdef SUPPORT_VF_CONTROL
        if(DRV_isVfControl())
        	EST_setFlag_enableRsRecalc(obj->estHandle,false);
        else
#endif
        	EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc);

        // enable/disable automatic calculation of bias values
        //gMotorVars.Flag_enableOffsetcalc = false; //hrjung off
        CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);

#ifdef SUPPORT_FLYING_START
        // Control motor Start or Stop with Flying Start
        motor_RunCtrl(ctrlHandle);
#endif		

        if(CTRL_isError(ctrlHandle))
          {
            // set the enable controller flag to false
            CTRL_setFlag_enableCtrl(ctrlHandle,false);

            // set the enable system flag to false
            gMotorVars.Flag_enableSys = false;

            // disable the PWM
            HAL_disablePwm(halHandle);
          }
        else
          {
            // update the controller state
            bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

            if(flag_ctrlStateChanged)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                if(ctrlState == CTRL_State_OffLine)
                  {
                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_OnLine)
                  {
                    if(gMotorVars.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle);
                    }
                    else
                    {
#ifdef SUPPORT_VF_CONTROL
                      if(!DRV_isVfControl())
#endif
                      {
#ifdef SUPPORT_USER_VARIABLE
						  // set the current bias from setting
						  HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(gUserParams.I_A_Offset));
						  HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(gUserParams.I_B_Offset));
						  HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(gUserParams.I_C_Offset));

						  // set the voltage bias
						  HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(gUserParams.V_A_Offset));
						  HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(gUserParams.V_B_Offset));
						  HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(gUserParams.V_C_Offset));
#else
	                      // set the current bias
	                      HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
	                      HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
	                      HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

	                      // set the voltage bias
	                      HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
	                      HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
	                      HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
#endif
                      }
                    }

#if 1
                    if(!DRV_isVfControl())
                    {
						// Return the bias value for currents
						gMotorVars.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0);
						gMotorVars.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1);
						gMotorVars.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2);

						// Return the bias value for voltages
						gMotorVars.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0);
						gMotorVars.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1);
						gMotorVars.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2);
                    }
#endif

#ifdef SUPPORT_VF_CONTROL
                    if(DRV_isVfControl())
                    {
						// set flag to disable speed controller
					   CTRL_setFlag_enableSpeedCtrl(ctrlHandle, false);

					   // set flag to disable current controller
					   CTRL_setFlag_enableCurrentCtrl(ctrlHandle, false);
                    }
#endif

#ifndef SUPPORT_FLYING_START
                    // enable the PWM
                    HAL_enablePwm(halHandle);
#endif					
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
#ifdef SUPPORT_VF_CONTROL
					if(DRV_isVfControl())
					{
						// clear the speed reference trajectory
						TRAJ_setTargetValue(controller_obj->trajHandle_spd,_IQ(0.0));
						TRAJ_setIntValue(controller_obj->trajHandle_spd,_IQ(0.0));
					}
#endif					
                    gMotorVars.Flag_Run_Identify = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVars.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandle);
                  }
              }
          }


        if(EST_isMotorIdentified(obj->estHandle))
          {
#ifdef SUPPORT_FIELD_WEAKENING
            _iq Is_Max_squared_pu = _IQ((USER_MOTOR_MAX_CURRENT*USER_MOTOR_MAX_CURRENT)/  \
    	      			  (USER_IQ_FULL_SCALE_CURRENT_A*USER_IQ_FULL_SCALE_CURRENT_A));
            _iq Id_squared_pu = _IQmpy(CTRL_getId_ref_pu(ctrlHandle),CTRL_getId_ref_pu(ctrlHandle));

            // Take into consideration that Iq^2+Id^2 = Is^2
            Iq_Max_pu = _IQsqrt(Is_Max_squared_pu-Id_squared_pu);

            //Set new max trajectory
            CTRL_setSpdMax(ctrlHandle, Iq_Max_pu);
#endif
            // set the current ramp
            EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
            gMotorVars.Flag_MotorIdentified = true;

           	gMotorVars.SpeedRef_krpm = MAIN_getActualSpeedWithDirection();

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);

            if(gMotorVars.Speed_krpm > 0) state_param.run = FORWARD;
            else if(gMotorVars.Speed_krpm < 0) state_param.run = REVERSE;

           // STA_setCurSpeed(_IQtoF(gMotorVars.Speed_krpm));
            gMotorVars.MaxAccel_krpmps = MAIN_getAccelRate();

            // set the speed acceleration
            {
#ifdef SUPPORT_USER_VARIABLE
				_iq accel_krpm_sf = _IQ(gUserParams.motor_numPolePairs*1000.0/gUserParams.trajFreq_Hz/USER_IQ_FULL_SCALE_FREQ_Hz/60.0);
				CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(accel_krpm_sf,gMotorVars.MaxAccel_krpmps));
#else
				CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));
#endif
            }


            if(Flag_Latch_softwareUpdate)
            {
              Flag_Latch_softwareUpdate = false;

              USER_calcPIgains(ctrlHandle);

              // initialize the watch window kp and ki current values with pre-calculated values
              gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle,CTRL_Type_PID_Id);
              gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle,CTRL_Type_PID_Id);
            }

          }
        else
          {
            Flag_Latch_softwareUpdate = true;

            // initialize the watch window kp and ki values with pre-calculated values
            gMotorVars.Kp_spd = CTRL_getKp(ctrlHandle,CTRL_Type_PID_spd);
            gMotorVars.Ki_spd = CTRL_getKi(ctrlHandle,CTRL_Type_PID_spd);


            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
          }

        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

#ifdef SUPPORT_VF_CONTROL
            if(DRV_isVfControl())
            	updateGlobalVariables_motor4Vf(ctrlHandle);
            else
#endif
            	updateGlobalVariables_motor(ctrlHandle);
          }

        // update Kp and Ki gains
        updateKpKiGains(ctrlHandle);

        // run Rs online
        if(gMotorVars.Flag_enableRsRecalc)
        	runRsOnLine(ctrlHandle);

#ifdef SUPPORT_FIELD_WEAKENING
        // set field weakening enable flag depending on user's input
        FW_setFlag_enableFw(fwHandle,gMotorVars.Flag_enableFieldWeakening);
#endif

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
#ifdef SUPPORT_VF_CONTROL
        if(DRV_isVfControl())
        	CTRL_setFlag_enablePowerWarp(ctrlHandle,false);
        else
#endif
        {
        	if(param.ctrl.energy_save == ESAVE_UNUSED)
        		gMotorVars.Flag_enablePowerWarp = false;
        	else
        		gMotorVars.Flag_enablePowerWarp = true;

        	CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);
        }


        // protection
        processProtection();

        //DC Injection Brake
        DCIB_processBrakeSigHandler();

        //TODO : should find correct location
        state_param.inv = STA_control();

        //AO_generateOutput();

        // debug command in Motor running
        ProcessDebugCommand();

      } // end of while(gFlag_enableSys) loop

    // disable the PWM
   	HAL_disablePwm(halHandle);

	if(MAIN_isTripHappened() && first_trip_f)
	{
		first_trip_f=0;
		UARTprintf("Trip happened %d\n", internal_status.trip_happened);
	}

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    gMotorVars.Flag_Run_Identify = false;

  } // end of for(;;) loop

} // end of main() function



interrupt void mainISR(void)
{
#ifdef SUPPORT_VF_CONTROL
	MATH_vec2 phasor, Vab_pu;
	_iq vf_speed_pu;
#endif
	//static float_t prev_val=0.0;

	UTIL_testbitG2(0);
	//UTIL_testbit(0);
  // toggle status LED
	//HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
#if 1
#ifdef SUPPORT_USER_VARIABLE
  if(++gLEDcnt >= (uint_least32_t)(gUserParams.isrFreq_Hz / LED_BLINK_FREQ_Hz))
#else
	  if(++gLEDcnt >= (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
#endif
  {
#ifdef SUPPORT_V08_HW
		HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
#else
#ifdef SUPPORT_V0_HW
	    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
#else
	    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
#endif
#endif
	    gLEDcnt = 0;
  }
#endif

  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);

  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);

#if 1
  if(MAIN_isSystemEnabled())
  {
	  if(MAIN_isTripHappened())
	  {
		  MAIN_disableSystem();
		  return;
	  }
  }
#endif

#ifdef SUPPORT_VF_CONTROL
  if(DRV_isVfControl())
  {
  	// filter Vbus voltage
	//gVbus_lpf = FILTER_SO_run_form_1(gVbusFilterHandle, _IQ(520.0/USER_IQ_FULL_SCALE_VOLTAGE_V));
	//gVbus_lpf = gAdcData.dcBus;
	gVbus_lpf = FILTER_SO_run_form_1(gVbusFilterHandle,gAdcData.dcBus);

	//gOneOverDcBus = _IQdiv(_IQ(1.0),gVbus_lpf);
	gOneOverDcBus = _IQ(1.626); //_IQ(2.827);//_IQ(1.626); // _IQ(2.896);

	// run the controller
	uint_least16_t count_isr = CTRL_getCount_isr(ctrlHandle);
	uint_least16_t numIsrTicksPerCtrlTick = CTRL_getNumIsrTicksPerCtrlTick(ctrlHandle);

	// if needed, run the controller
	if(count_isr >= numIsrTicksPerCtrlTick) //USER_NUM_ISR_TICKS_PER_CTRL_TICK
	{
		CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

		// reset the isr count
		CTRL_resetCounter_isr(ctrlHandle);

		// increment the state counter
		CTRL_incrCounter_state(ctrlHandle);

		// increment the trajectory count
		CTRL_incrCounter_traj(ctrlHandle);

		// run the appropriate controller
		if(ctrlState == CTRL_State_OnLine)
		{
			// increment the current count
			CTRL_incrCounter_current(ctrlHandle);

			// increment the speed count
			CTRL_incrCounter_speed(ctrlHandle);

			// run Clarke transform on current
			CLARKE_run(controller_obj->clarkeHandle_I,&gAdcData.I, CTRL_getIab_in_addr(ctrlHandle));

			// run Clarke transform on voltage
			CLARKE_run(controller_obj->clarkeHandle_V,&gAdcData.V, CTRL_getVab_in_addr(ctrlHandle));

			//hrjung add for EST
//			EST_run(controller_obj->estHandle,CTRL_getIab_in_addr(ctrlHandle),CTRL_getVab_in_addr(ctrlHandle),
//									gAdcData.dcBus,TRAJ_getIntValue(controller_obj->trajHandle_spd));


#ifdef SUPPORT_JUMP_FREQ
			vf_speed_pu = TRAJ_getIntValue(controller_obj->trajHandle_spd);

			controller_obj->speed_ref_pu = MAIN_avoidJumpSpeed(vf_speed_pu);
#else
			controller_obj->speed_ref_pu = TRAJ_getIntValue(controller_obj->trajHandle_spd);
#endif
			temp_spd_ref = _IQtoF(controller_obj->speed_ref_pu)*sf4pu_krpm;

			if(fabsf(temp_spd_ref) > 12.0) //krpm (400Hz)
			{
				ERR_setTripFlag(TRIP_REASON_RPM_RANGE_ERR);
				MAIN_disableSystem();
				return ;
			}

			//generate the motor electrical angle
			ANGLE_GEN_run(angle_genHandle, controller_obj->speed_ref_pu);

			//generate the output voltage
			VS_FREQ_run(vs_freqHandle, _IQabs(controller_obj->speed_ref_pu));


			// get the electrical angle
			controller_obj->angle_pu = ANGLE_GEN_getAngle_pu(angle_genHandle);

			// compute the sin/cos phasor
			CTRL_computePhasor(controller_obj->angle_pu,&phasor);

			// compute Valpha, Vbeta with angle and Vout
			Vab_pu.value[0] = _IQmpy(vs_freq.Vs_out,phasor.value[0]);
			Vab_pu.value[1] = _IQmpy(vs_freq.Vs_out,phasor.value[1]);

		    Vab_pu.value[0] = _IQmpy(Vab_pu.value[0],gOneOverDcBus);
		    Vab_pu.value[1] = _IQmpy(Vab_pu.value[1],gOneOverDcBus);

			CTRL_setVab_out_pu(ctrlHandle,&Vab_pu);

			// run the space Vector Generator (SVGEN) module
			SVGEN_run(controller_obj->svgenHandle,CTRL_getVab_out_addr(ctrlHandle),&(gPwmData.Tabc));
		}
		else if(ctrlState == CTRL_State_OffLine)
		{
			// run the offline controller
			CTRL_runOffLine(ctrlHandle,halHandle,&gAdcData,&gPwmData);
		}
		else if(ctrlState == CTRL_State_Idle)
		{
			// set all pwm outputs to zero
			gPwmData.Tabc.value[0] = _IQ(0.0);
			gPwmData.Tabc.value[1] = _IQ(0.0);
			gPwmData.Tabc.value[2] = _IQ(0.0);
		}
	}
	else //count_isr >= numIsrTicksPerCtrlTick
	{
		// increment the isr count
		CTRL_incrCounter_isr(ctrlHandle);
	}
  }
  else // FOC control
#endif  
  { 
  
#ifdef SUPPORT_FLYING_START 
	  // run the flying start
	  FS_run(ctrlHandle, fsHandle);
#endif    

	  // run the controller
	  CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData);

  }

#ifdef SUPPORT_FLYING_START
  if(gMotorVars.Flag_RunState == false)
  {
	gPwmData.Tabc.value[0] = _IQ(0.0);
	gPwmData.Tabc.value[1] = _IQ(0.0);
	gPwmData.Tabc.value[2] = _IQ(0.0);

	// disable the PWM
	HAL_disablePwm(halHandle);
  }
#endif

#ifdef PWM_DUTY_TEST
  if(gFlag_PwmTest)
  {
	  if(gFlag_PwmStepTest)
	  {
		  static int inc=1;
		  //UTIL_testbit(1);
		  if(delay_count == 0)
		  {
#if 1
			  if(pwm_cnt == 100)
				  inc = -1;
			  else if(pwm_cnt == 0)
				  inc = 1;
#else
			  if(pwm_cnt == 6)
				  inc = -1;
			  else if(pwm_cnt == 5)
				  inc = 1;
#endif
			  pwm_cnt += inc;
			  gPwmData_Value = _IQ((float_t)(50-pwm_cnt)/100.0);
			  if(pwm_cnt <= 5)
				  delay_count=200;
			  else
				  delay_count=100;
		  }
		  delay_count--;
		  //UTIL_testbit(0);
	  }
	  gPwmData.Tabc.value[0] = gPwmData_Value;  //~0.5 ~ 0.5
	  gPwmData.Tabc.value[1] = gPwmData_Value;
	  gPwmData.Tabc.value[2] = gPwmData_Value;
  }
#endif

  if( (DRV_isVfControl() && fabsf(temp_spd_ref) < 0.03) // 1Hz
	  ) // ignore below 1Hz
  {
	  gPwmData.Tabc.value[0] = 0.0;
	  gPwmData.Tabc.value[1] = 0.0;
	  gPwmData.Tabc.value[2] = 0.0;
	  block_count++;
  }

  //process PWM for DCI brake
  MAIN_processDCBrake();

  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);

#ifdef SUPPORT_FIELD_WEAKENING
  if(FW_getFlag_enableFw(fwHandle) == true && !DRV_isVfControl())
    {
      FW_incCounter(fwHandle);

      if(FW_getCounter(fwHandle) > FW_getNumIsrTicksPerFwTick(fwHandle))
        {
    	  _iq refValue;
    	  _iq fbackValue;
    	  _iq output;

    	  FW_clearCounter(fwHandle);

    	  refValue = gMotorVars.VsRef;

    	  fbackValue = gMotorVars.Vs;

    	  FW_run(fwHandle, refValue, fbackValue, &output);

    	  CTRL_setId_ref_pu(ctrlHandle, output);

    	  gMotorVars.IdRef_A = _IQmpy(CTRL_getId_ref_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
        }
    }
  else
    {
      CTRL_setId_ref_pu(ctrlHandle, _IQmpy(gMotorVars.IdRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));
    }
#endif

  // setup the controller
  CTRL_setup(ctrlHandle);


#if 0
  if(STA_isStopState()) temp_spd_ref=0.0;
  else if( (STA_getTargetFreq() == 0.0) && STA_isDecelState())
  {
	  if(DRV_isVfControl())
	  {
		  if(fabsf(temp_spd_ref) < 0.03) // 1Hz
		  {
			  //UARTprintf("STOP condition spd_ref %f \n", fabsf(temp_spd_ref));
			  temp_spd_ref=0.0;
			  MAIN_disableSystem();
		  }
	  }
	  else
	  {
		  if(STA_getCurSpeed() < 50.0) // 2Hz
		  {
			  //UARTprintf("STOP FOC  %f\n", STA_getCurSpeed());
			  MAIN_disableSystem();
		  }
	  }
  }
#endif

  MAIN_setCurrentFreq();

  // Irms
  //UTIL_testbit(1);
  if(MAIN_isSampleRequired())
  {
	  MAIN_readCurrent();
	  MAIN_calculateIrms();
  }
  //UTIL_testbit(0);

#if 0
	internal_status.Vu_inst = _IQtoF(gAdcData.V.value[0])*USER_IQ_FULL_SCALE_VOLTAGE_V;
	internal_status.Vv_inst = _IQtoF(gAdcData.V.value[1])*USER_IQ_FULL_SCALE_VOLTAGE_V;
	internal_status.Vw_inst = _IQtoF(gAdcData.V.value[2])*USER_IQ_FULL_SCALE_VOLTAGE_V;

	if(sample_type == V_UVW_SAMPLE_TYPE)
		dbg_getSample(internal_status.Vu_inst, internal_status.Vv_inst, internal_status.Vw_inst);
	  //dbg_getSample(gAdcData.v_adc[0], gAdcData.v_adc[1], gAdcData.v_adc[2]);

	if(sample_type == I_CURR_SAMPLE_TYPE)
		dbg_getSample(internal_status.Iu_inst, internal_status.Iv_inst, internal_status.Iw_inst);
#endif

#if 0
	internal_status.Vdc_inst = _IQtoF(gAdcData.dcBus)*USER_IQ_FULL_SCALE_VOLTAGE_V;
	internal_status.Vdc_lfp = _IQtoF(gVbus_lpf)*USER_IQ_FULL_SCALE_VOLTAGE_V;
#ifdef SAMPLE_ADC_VALUE
	gLEDcnt++;
	if(sample_type == V_DC_SAMPLE_TYPE && gLEDcnt > 1000)
	{
	  dbg_getSample(internal_status.Vdc_inst, internal_status.Vdc_lfp, 0);
	  gLEDcnt = 0;
	}
#endif

	internal_status.Vab_pu[0] = _IQtoF(Vab_pu.value[0]);
	internal_status.Vab_pu[1] = _IQtoF(Vab_pu.value[1]);

	internal_status.phasor[0] = _IQtoF(phasor.value[0]);
	internal_status.phasor[1] = _IQtoF(phasor.value[1]);
	internal_status.angle_pu = _IQtoF(controller_obj->angle_pu);

#ifdef SAMPLE_ADC_VALUE
	if(sample_type == V_AB_SAMPLE_TYPE)
		dbg_getSample(internal_status.Vab_pu[0], internal_status.Vab_pu[1], 0);

	if(sample_type == PHASOR_SAMPLE_TYPE)
		dbg_getSample(internal_status.phasor[0], internal_status.phasor[1], internal_status.angle_pu);

	if(sample_type == V_AB_PHASOR_SAMPLE_TYPE)
		dbg_getSample(internal_status.Vab_pu[0], internal_status.phasor[0], internal_status.angle_pu);
#endif

  internal_status.pwmData[0] = _IQtoF(gPwmData.Tabc.value[0]);
  internal_status.pwmData[1] = _IQtoF(gPwmData.Tabc.value[1]);
  internal_status.pwmData[2] = _IQtoF(gPwmData.Tabc.value[2]);
#ifdef SAMPLE_ADC_VALUE
  if(sample_type == PWM_SAMPLE_TYPE)
	  dbg_getSample(internal_status.pwmData[0], internal_status.pwmData[1], internal_status.pwmData[2]);
#endif

#if 0
  prev_val = internal_status.pwmData[0];
  if(prev_val >= 0.0 && internal_status.pwmData[0] < 0.0)
  {
	  //UTIL_testbit(1);
	  p2n = 1;
	  n2p = 0;
#ifdef SAMPLE_ADC_VALUE
  if(sample_type == PWM_PERIOD_COUNT_TYPE)
	  dbg_getSample((float_t)n_count, 0.0, 0.0);
#endif
  	  n_count=0;
  }
  else if(prev_val < 0.0 && internal_status.pwmData[0] >= 0.0)
  {
	  //UTIL_testbit(0);
	  p2n = 0;
	  n2p = 1;
  }
   n_count += p2n;
#endif
#endif


#ifdef SUPPORT_DEBUG_GRAPH
  DATALOG_update(datalogHandle);
#endif

  UTIL_testbitG2(1);
  //UTIL_testbit(0);

  return;
} // end of mainISR() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  int32_t tmp;

  // get the speed estimate
  gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // when calling EST_ functions that return a float, and fpu32 is enabled, an integer is needed as a return
  // so that the compiler reads the returned value from the accumulator instead of fpu32 registers
  // get the magnetizing current
  tmp = EST_getIdRated(obj->estHandle);
  gMotorVars.MagnCurr_A = *((float_t *)&tmp);

  // get the rotor resistance
  tmp = EST_getRr_Ohm(obj->estHandle);
  gMotorVars.Rr_Ohm = *((float_t *)&tmp);

  // get the stator resistance
  tmp = EST_getRs_Ohm(obj->estHandle);
  gMotorVars.Rs_Ohm = *((float_t *)&tmp);

  // get the stator resistance online
  tmp = EST_getRsOnLine_Ohm(obj->estHandle);
  gMotorVars.RsOnLine_Ohm = *((float_t *)&tmp);

  // get the stator inductance in the direct coordinate direction
  tmp = EST_getLs_d_H(obj->estHandle);
  gMotorVars.Lsd_H = *((float_t *)&tmp);

  // get the stator inductance in the quadrature coordinate direction
  tmp = EST_getLs_q_H(obj->estHandle);
  gMotorVars.Lsq_H = *((float_t *)&tmp);

  // get the flux in V/Hz in floating point
  tmp = EST_getFlux_VpHz(obj->estHandle);
  gMotorVars.Flux_VpHz = *((float_t *)&tmp);

  // get the flux in Wb in fixed point
  gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVars.EstState = EST_getState(obj->estHandle);

#ifdef SUPPORT_FIELD_WEAKENING
  // read Vd and Vq vectors per units
  gMotorVars.Vd = CTRL_getVd_out_pu(ctrlHandle);
  gMotorVars.Vq = CTRL_getVq_out_pu(ctrlHandle);

  // calculate vector Vs in per units
  gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

  // read Id and Iq vectors in amps
  gMotorVars.Id_A = _IQmpy(CTRL_getId_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
  gMotorVars.Iq_A = _IQmpy(CTRL_getIq_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

  // calculate vector Is in amps
  gMotorVars.Is_A = _IQsqrt(_IQmpy(gMotorVars.Id_A, gMotorVars.Id_A) + _IQmpy(gMotorVars.Iq_A, gMotorVars.Iq_A));
#endif

  // Get the DC buss voltage
  //gMotorVars.VdcBus_kV = _IQdiv(gAdcData.dcBus,1000.0);
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  return;
} // end of updateGlobalVariables_motor() function

#ifdef SUPPORT_VF_CONTROL
void updateGlobalVariables_motor4Vf(CTRL_Handle handle)
{
  MATH_vec2 Vab_in_pu, Iab_in_pu;

  gMotorVars.Speed_krpm = _IQ(temp_spd_ref);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // calculate vector Vs in per units
  CTRL_getVab_in_pu(ctrlHandle, &Vab_in_pu);
  gMotorVars.Vs = _IQsqrt(_IQmpy(Vab_in_pu.value[0], Vab_in_pu.value[0]) + _IQmpy(Vab_in_pu.value[1], Vab_in_pu.value[1]));
  //gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

  // read Id and Iq vectors in amps
  gMotorVars.Id_A = _IQmpy(CTRL_getId_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
  gMotorVars.Iq_A = _IQmpy(CTRL_getIq_in_pu(ctrlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

  // calculate vector Is in amps
  CTRL_getIab_in_pu(ctrlHandle, &Iab_in_pu);
  gMotorVars.Is_A = _IQsqrt(_IQmpy(Iab_in_pu.value[0], Iab_in_pu.value[0]) + _IQmpy(Iab_in_pu.value[1], Iab_in_pu.value[1]));
  //gMotorVars.Is_A = _IQsqrt(_IQmpy(gMotorVars.Id_A, gMotorVars.Id_A) + _IQmpy(gMotorVars.Iq_A, gMotorVars.Iq_A));

  // Get the DC buss voltage
  //UTIL_testbit(1);
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));
  //gMotorVars.VdcBus_kV = _IQdiv(gAdcData.dcBus,1000.0);
  //temp_vdc = _IQmpy(gAdcData.temp,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));
  //UTIL_testbit(0);

  return;
}
#endif

void updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
	}

  return;
} // end of updateKpKiGains() function


void runRsOnLine(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // execute Rs OnLine code
  if(gMotorVars.Flag_Run_Identify == true)
    {
      if(EST_getState(obj->estHandle) == EST_State_OnLine)
        {
    	  float_t RsError_Ohm = gMotorVars.RsOnLine_Ohm - gMotorVars.Rs_Ohm;

          EST_setFlag_enableRsOnLine(obj->estHandle,true);
          EST_setRsOnLineId_mag_pu(obj->estHandle,_IQmpy(gMotorVars.RsOnLineCurrent_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

          if(fabsf(RsError_Ohm) < (gMotorVars.Rs_Ohm * 0.05))
            {
              EST_setFlag_updateRs(obj->estHandle,true);
            }
        }
      else
        {
    	  // initialize
          EST_setRsOnLineId_mag_pu(obj->estHandle,_IQ(0.0));
          EST_setRsOnLineId_pu(obj->estHandle,_IQ(0.0));
          EST_setRsOnLine_pu(obj->estHandle,_IQ(0.0));
          EST_setFlag_enableRsOnLine(obj->estHandle,false);
          EST_setFlag_updateRs(obj->estHandle,false);
          EST_setRsOnLine_qFmt(obj->estHandle,EST_getRs_qFmt(obj->estHandle));
        }
    }

  return;
} // end of runRsOnLine() function

#ifdef SUPPORT_FLYING_START
// Control motor running
void motor_RunCtrl(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  bool flag_enableSpeedCtrl;

  gMotorVars.Flag_Run_Identify = true;

  if(gMotorVars.Flag_enableRun)		// Stop to Start
  {
	 gMotorVars.SpeedRef_krpm = gMotorVars.SpeedSet_krpm;

		if(gMotorVars.Flag_RunState == false)
		{
			FS_setFlag_enableFs(fsHandle, gMotorVars.Flag_enableFlyingStart);

			FS_reset(fsHandle);

			gMotorVars.Flag_RunState = true;

			PID_setUi(obj->pidHandle_spd, _IQ(0.0));
			PID_setUi(obj->pidHandle_Id, _IQ(0.0));
			PID_setUi(obj->pidHandle_Iq, _IQ(0.0));

			CTRL_setId_ref_pu(handle, _IQ(0.0));
			CTRL_setIq_ref_pu(handle, _IQ(0.0));
			CTRL_setSpd_out_pu(handle, _IQ(0.0));

			CTRL_setFlag_enableCurrentCtrl(handle,true);

			gPwmData.Tabc.value[0] = _IQ(0.0);
			gPwmData.Tabc.value[1] = _IQ(0.0);
			gPwmData.Tabc.value[2] = _IQ(0.0);

			// write the PWM compare values
			HAL_writePwmData(halHandle,&gPwmData);

			// enable the PWM
			HAL_enablePwm(halHandle);
		}

		flag_enableSpeedCtrl = (gMotorVars.Flag_enableSpeedCtrl) & (FS_getFlag_SpeedCtrl(fsHandle));
   }
   else if(gMotorVars.Flag_RunState == true)  // Run to Stop
   {
		FS_setFlag_enableFs(fsHandle, false);

		gMotorVars.Flag_RunState = false;
		gMotorVars.SpeedRef_krpm = _IQ(0.0);

		// disable the PWM
		HAL_disablePwm(halHandle);

		flag_enableSpeedCtrl = false;

		PID_setUi(obj->pidHandle_spd, _IQ(0.0));
		PID_setUi(obj->pidHandle_Id, _IQ(0.0));
		PID_setUi(obj->pidHandle_Iq, _IQ(0.0));

		gPwmData.Tabc.value[0] = _IQ(0.0);
		gPwmData.Tabc.value[1] = _IQ(0.0);
		gPwmData.Tabc.value[2] = _IQ(0.0);

		CTRL_setId_ref_pu(handle, _IQ(0.0));
		CTRL_setIq_ref_pu(handle, _IQ(0.0));


		CTRL_setFlag_enableCurrentCtrl(handle,false);
  }

  // enable/disable the Library internal PI.  Iq is referenced by the speed PI now
  CTRL_setFlag_enableSpeedCtrl(handle,flag_enableSpeedCtrl);
}
#endif

/*
 *    public function
 */

int MAIN_isSystemEnabled(void)
{
	return (gMotorVars.Flag_enableSys == true);
}

int MAIN_isTripHappened(void)
{
	return (internal_status.trip_happened != TRIP_REASON_NONE);
}

#ifdef SUPPORT_DEBUG_TERMINAL
void dbg_enableSystem(void)
{
	gMotorVars.Flag_enableSys = true;
	gMotorVars.Flag_Run_Identify = true;
}

void dbg_disableSystem(void)
{
	gMotorVars.Flag_enableSys = false;
	gMotorVars.Flag_Run_Identify = false;
}
#endif

int MAIN_enableSystem(int index)
{
	int result=0;

	if(MAIN_isTripHappened()) return 1;

	MAIN_initIarray();

	gMotorVars.Flag_enableSys = true;
	gMotorVars.Flag_Run_Identify = true;

	STA_setNextFreq(param.ctrl.value);

	block_count=0;

	return result;
}

void MAIN_disableSystem(void)
{
	gMotorVars.Flag_enableSys = false;
	gMotorVars.Flag_Run_Identify = false;
	state_param.run = STOP;

	gMotorVars.Speed_krpm = _IQ(0.0);
	STA_setStopStatus();
}

int MAIN_setForwardDirection(void)
{
	direction = 1.0;

	return 0;

}

int MAIN_setReverseDirection(void)
{
	direction = -1.0;

	return 0;
}

int MAIN_applyBoost(void)
{
	gUserParams.VF_freq_low = mtr.input_voltage*param.ctrl.v_boost/100.0;
	UARTprintf("Boost voltage value=%f\n", gUserParams.VF_freq_low);

	VS_FREQ_setProfile(vs_freqHandle, gUserParams.VF_freq_low, gUserParams.VF_freq_high, gUserParams.VF_volt_min, gUserParams.VF_volt_max);

	return 0;
}

float_t MAIN_getPwmFrequency(void)
{
#ifdef SUPPORT_USER_VARIABLE
	return gUserParams.pwmPeriod_kHz;
#else
	return USER_PWM_FREQ_kHz;
#endif
}

int UTIL_controlLed(int type, int on_off)
{
	int result = 0;

	if(type == HAL_Gpio_LED_R || type == HAL_Gpio_LED_G)
	{
		if(on_off == 1)
			HAL_setGpioHigh(halHandle,(GPIO_Number_e)type);
		else
			HAL_setGpioLow(halHandle,(GPIO_Number_e)type);
	}
	else
	{
		UARTprintf("Error : no LED type=%d \n", type);
		result = 1;
	}

	return result;
}

// TODO : debug purpose only, using LED_R2 as test bit
void UTIL_testbit(int on_off) // LD2
{
	if(on_off == 1)
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);
	else
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);
}

void UTIL_testbitG2(int on_off) // LD1
{
	if(on_off == 1)
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
	else
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
}

void UTIL_setInitRelay(void)
{
	HAL_setGpioHigh(halHandle,(GPIO_Number_e)GPIO_Number_20);
	internal_status.relay_enabled = 1;
}

void UTIL_clearInitRelay(void)
{
	HAL_setGpioLow(halHandle,(GPIO_Number_e)GPIO_Number_20);
	internal_status.relay_enabled = 0;
}

void UTIL_setShaftBrake(void)
{
	HAL_setGpioHigh(halHandle,(GPIO_Number_e)GPIO_Number_23);
}

void UTIL_releaseShaftBrake(void)
{
	HAL_setGpioLow(halHandle,(GPIO_Number_e)GPIO_Number_23);
}

void UTIL_setScaleFactor(void)
{
	// scale factor for pu -> krpm
	sf4pu_krpm = (60.0*USER_IQ_FULL_SCALE_FREQ_Hz) / (mtr.poles*1000.0); // 15
	sf4krpm_pu = (mtr.poles*1000.0) / (60.0*USER_IQ_FULL_SCALE_FREQ_Hz);
}

uint16_t UTIL_setRegenPwmDuty(int duty)
{
	float_t pwm_duty;
	uint16_t user_pwm=0;


	pwm_duty = 1.0 - (float_t)duty/100.0; // low active
	user_pwm = HAL_writePwmDataRegen(halHandle, _IQ(pwm_duty));

	return user_pwm;
}

bool UTIL_readOverTemperatureWarning(void)
{
	return HAL_readGpio(halHandle,(GPIO_Number_e)GPIO_Number_21);
}

bool UTIL_readOverTemperatureFault(void)
{
	return HAL_readGpio(halHandle,(GPIO_Number_e)GPIO_Number_25);
}

//@} //defgroup
// end of file



