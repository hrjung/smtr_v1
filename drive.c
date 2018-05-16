/*
 * drive.c
 *
 * 		support 3 types of input
 *		1. potentiometer : 0 ~ 10V
 *		2. voltage input : 0 ~ 10V
 *		3. current input : 0 ~ 20mA
 *
 *		support 1 output can be voltage, current, frequency
 *
 *  Created on: 2017. 4. 5.
 *      Author: hrjung
 */


#include "uartstdio.h"
#include "hal.h"

#include "inv_param.h"
#include "drive.h"
#include "state_func.h"
#include "freq.h"
#include "err_trip.h"

/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

const float_t pwm_tbl[4] = { 6.0, 9.0, 12.0, 15.0 };


/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
extern HAL_Handle halHandle;
extern USER_Params gUserParams;


/*******************************************************************************
 * EXTERNS
 */


/*
 *  ======== local function ========
 */


/*
 *  ======== public function ========
 */

float_t DRV_calculateAccelRate_krpm(float_t time, float_t diff)
{
	float_t spd_rpm, rate_krpm; //spd_range_rpm;
//	int freq_range;

	//calculate based on frequency to RPM
	//spd_range_rpm = (float)STA_getSpeedRange();
	//rate_krpm = spd_range_rpm/(time_100msec * 100); // scale to 1ms unit

	//spd_rpm = FREQ_convertToSpeed(FREQ_getFreqRange()); // max ~ min
	spd_rpm = FREQ_convertToSpeed(diff);
	rate_krpm = spd_rpm/(time * 1000.0); // scale to 1ms unit

	UARTprintf("Accel rate = %f\n", rate_krpm);
	if(rate_krpm == 0.0)
		UARTprintf("Accel rate is too small\n");
	return rate_krpm;
}

int DRV_setAccelTime(float_t value)
{
	if(value < MIN_ACCEL_TIME || value > MAX_ACCEL_TIME) return 1;

	param.ctrl.accel_time = value;

	//STA_setResolution(ACCEL, DRV_calculateAccelRate_krpm(value));

	//return EEP_updateItem(ACCEL_TIME_ADDR, (unsigned char *)&param.ctrl.accel_time);
	return 0;
}

int DRV_setDecelTime(float_t value)
{
	if(value < MIN_ACCEL_TIME || value > MAX_ACCEL_TIME) return 1;

	param.ctrl.decel_time = value;

	//STA_setResolution(DECEL, DRV_calculateAccelRate_krpm(value));

	//return EEP_updateItem(DECEL_TIME_ADDR, (unsigned char *)&param.ctrl.decel_time);
	return 0;
}

int DRV_isVfControl(void)
{
	return (param.ctrl.vf_foc_sel == VF_CONTROL);
}

void DRV_enableVfControl(void)
{
	param.ctrl.vf_foc_sel = VF_CONTROL;
	//EEP_updateItem(VF_FOC_SEL_ADDR, (unsigned char *)&param.ctrl.vf_foc_sel);
}

void DRV_enableFocControl(void)
{
	param.ctrl.vf_foc_sel = FOC_CONTROL;
	//EEP_updateItem(VF_FOC_SEL_ADDR, (unsigned char *)&param.ctrl.vf_foc_sel);
}

int DRV_setTorqueLimit(float_t limit)
{
	if(limit < 100.0 || limit > 220.0) return 0;

	param.ctrl.foc_torque_limit = limit;

	return 0;
}

int DRV_setEnergySave(int method)
{
	if(method < ESAVE_UNUSED || method > ESAVE_BOTH) return 1;

	param.ctrl.energy_save = method;

	//return EEP_updateItem(ENERGY_SAVE_ADDR, (unsigned char *)&param.ctrl.energy_save);
	return 0;
}

int DRV_setVoltageBoost(float_t value)
{
	if(value < 0.0 || value > 100.0) return 1;

	param.ctrl.v_boost = value; // 1000 means 100.0%

	//MAIN_applyBoost();

	//return EEP_updateItem(V_BOOST_ADDR, (unsigned char *)&param.ctrl.v_boost);
	return 0;
}

int DRV_setPwmFrequency(int value)
{
	if(value < PWM_4KHz || value > PWM_16KHz) return 1;

	if(MAIN_isSystemEnabled()) return 1;

	param.ctrl.pwm_freq = value; //

	//gUserParams.pwmPeriod_kHz = pwm_tbl[param.ctrl.pwm_freq];
	//gUserParams.pwmPeriod_usec = 1000.0/gUserParams.pwmPeriod_kHz;

	//return EEP_updateItem(PWM_FREQ_ADDR, (unsigned char *)&param.ctrl.pwm_freq);
	return 0;
}

int DRV_setSpdGainP(float_t value)
{
	if(value < 0.0 || value > 32767.0) return 1;

	param.ctrl.spd_P_gain = value; //

	return 0;
}

int DRV_setSpdGainI(float_t value)
{
	if(value < 0.0 || value > 32767.0) return 1;

	param.ctrl.spd_I_gain = value; //

	return 0;
}

float_t DRV_getPwmFrequency(void)
{
	return pwm_tbl[param.ctrl.pwm_freq];
}


int DRV_runForward(int index)
{
	if(MAIN_isSystemEnabled())
		STA_setNextFreq(param.ctrl.value);
	else
	{
		MAIN_enableSystem(index);
		UARTprintf("start running motor freq=%f\n", param.ctrl.value);
	}

    return 0;
}

int DRV_runBackward(int index)
{
	if(MAIN_isSystemEnabled())
		STA_setNextFreq(param.ctrl.value);
	else
	{
		MAIN_enableSystem(0);
		UARTprintf("start running motor\n");
	}

    return 0;
}

int DRV_stopMotor(void)
{
	int result = 1;

	STA_setStopCondition();
	UARTprintf("reduce speed to stop\n");

    return result;
}
