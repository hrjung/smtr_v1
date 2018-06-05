/*
 * dci_brake.c
 *
 *  Created on: 2017. 3. 17.
 *      Author: hrjung
 */


#include "uartstdio.h"

#include "inv_param.h"
#include "drive.h"
#include "state_func.h"
#include "timer_handler.h"
#include "brake.h"

/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */
#define DC_BRAKE_RATE_MAX	(200.0)

#define DC_BRAKE_TIME_MAX	(60.0)

#define DC_BRAKE_FREQ_LIMIT	(60.0)

/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * LOCAL VARIABLES
 */
static int dci_state_flag = DCI_NONE_STATE;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
extern uint32_t secCnt;
/*******************************************************************************
 * EXTERNS
 */


/*
 *  ======== local function ========
 */



/*
 *  ======== public function ========
 */
int BRK_isDCIBrakeEnabled(void)
{
	return (param.brk.method == DC_INJECT_BRAKE);
}

int BRK_isFreeRunEnabled(void)
{
	if(param.brk.method == FREE_RUN_BRAKE
		&& state_param.inv == STATE_DECEL
		&& STA_getTargetFreq() == 0.0
		&& STA_getCurFreq() != 0.0)
		return 1;

	return 0;
}




int BRK_setBrakeMethod(uint16_t method)
{
	if(method > FREE_RUN_BRAKE) return 1;

	param.brk.method = method;

	return 0;
}

int BRK_setBrakeFreq(float_t freq)
{

	// freq == 0.0, then not use shaft brake
	if(freq != 0.0 && freq < SHAFT_BRAKE_ENABLE_FREQ_MIN) return 1;

	if(freq > SHAFT_BRAKE_ENABLE_FREQ_MAX) return 1;

	// check valid range
//	if(speed <= dev_const.spd_rpm_min || speed >= dev_const.spd_rpm_max)
//		return 1;

	param.brk.brake_freq = freq;

	return 0;
}

//int DCIB_setStartSpeed(int speed)
//{
//	int max_speed = 500; // max start limit speed
//
//	if(speed < 0 || speed > (max_speed/dev_param.gear_ratio)) return 1;
//
//	// check valid range
//	if(speed <= dev_const.spd_rpm_min || speed >= dev_const.spd_rpm_max)
//		return 1;
//
//	param.brk.dci_start_speed = speed;
//	return 0;
//}

int DCIB_setStartFreq(float_t freq)
{
//	int max_speed = 500; // max start limit speed

	// check valid range
	//UARTprintf("freq min=%f max=%f\n", param.ctrl.freq_min, param.ctrl.freq_max);

	if(freq < 0 || freq > DC_BRAKE_FREQ_LIMIT)
		return 1;

	param.brk.dci_start_freq = freq;

	//return EEP_updateItem(BRK_DCI_START_FREQ_ADDR, (unsigned char *)&param.brk.dci_start_freq);
	return 0;
}

int DCIB_setBlockTime(float_t b_time)
{
	uint16_t itime=0;

	if(b_time > DC_BRAKE_TIME_MAX) return 1;
	else if(b_time < 0.0) return 1;

	itime = (uint16_t)(b_time*10.0);
	param.brk.dci_block_time = (float_t)itime/10.0;

	UARTprintf(" DCB block time %f \n", param.brk.dci_block_time);

	return 0;
}

int DCIB_setBrakeRate(float_t rate)
{
	uint16_t itime=0;

	if(rate > DC_BRAKE_RATE_MAX) return 1;
	else if(rate < 0.0) return 1;

	itime = (uint16_t)rate;
	param.brk.dci_braking_rate = (float_t)itime;
	dev_const.dci_pwm_rate = param.brk.dci_braking_rate/100.0 * mtr.max_current*mtr.Rs;

	UARTprintf(" DCB brake rate %f \n", param.brk.dci_braking_rate);

	return 0;
}

int DCIB_setBrakeTime(float_t b_time)
{
	uint16_t itime=0;

	if(b_time > DC_BRAKE_RATE_MAX) return 1;
	else if(b_time < 0.0) return 1;

	itime = (uint16_t)(b_time*10.0);
	param.brk.dci_braking_time = (float_t)itime/10.0;

	UARTprintf(" DCB brake time %f \n", param.brk.dci_braking_time);

	return 0;
}

int DCIB_getState(void)
{
	return dci_state_flag;
}

int DCIB_isBrakeTriggered(void)
{
	if(state_param.inv == STATE_DECEL
		&& STA_getTargetFreq() == 0.0
		&& STA_getCurFreq() != 0.0
		&& STA_getCurFreq() <= param.brk.dci_start_freq)
		return 1;

	return 0;
}

extern int dc_pwm_off;
int DCIB_processBrakeSigHandler(void)
{

	if(param.brk.method != DC_INJECT_BRAKE) return 0;

	switch(dci_state_flag)
	{
	case DCI_NONE_STATE: // check DCI start condition ->
		if(DCIB_isBrakeTriggered())
		{
			TMR_startTimerSig(DCI_BRAKE_SIG_OFF_TSIG, param.brk.dci_block_time);
			dci_state_flag = DCI_BLOCK_STATE;
			UARTprintf("DCI handler NONE -> BLOCK, at %d\n", (int)secCnt);
		}
		break;

	case DCI_BLOCK_STATE:
		// PWM off
		if(TMR_isTimeout(DCI_BRAKE_SIG_OFF_TSIG))
		{
			TMR_disableTimerSig(DCI_BRAKE_SIG_OFF_TSIG);
			TMR_startTimerSig(DCI_BRAKE_SIG_ON_TSIG, param.brk.dci_braking_time);
			dci_state_flag = DCI_DC_BRAKE_STATE;
			UARTprintf("DCI handler BLOCK -> BRAKE, at %d\n", (int)secCnt);
		}
		break;

	case DCI_DC_BRAKE_STATE:
		// DC voltage applied wait timeout
		if(TMR_isTimeout(DCI_BRAKE_SIG_ON_TSIG))
		{
			TMR_disableTimerSig(DCI_BRAKE_SIG_ON_TSIG);
			// if timeout -> PWM_OFF state
			dci_state_flag = DCI_PWM_OFF_STATE;
			UARTprintf("DCI handler BRAKE -> OFF, at %d\n", (int)secCnt);
		}
		break;

	case DCI_PWM_OFF_STATE:
		// PWM off and clear flag
		if(dc_pwm_off && STA_isStopState())
		{
			dci_state_flag = DCI_NONE_STATE;
			//UARTprintf("DCI handler BRAKE -> OFF, at %d\n", (int)secCnt);
		}
		break;

	default:

		break;
	}

	return 1;
}

