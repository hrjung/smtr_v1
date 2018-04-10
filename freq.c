/*
 * freq.c
 *
 *  Created on: 2017. 3. 11.
 *      Author: hrjung
 */
#include <stdint.h>

#include "uartstdio.h"

#include "inv_param.h"
#include "freq.h"
#include "state_func.h"
#include "drive.h"


/*******************************************************************************
 * MACROS
 */
#define FREQ_isValidMin()	(param.ctrl.freq_min != NOT_INITIALIZED)
#define FREQ_isValidMax()	(param.ctrl.freq_max != NOT_INITIALIZED)

//#define FREQ_isAccelTimeBasedMinMaxFreq()	(param.t_const.next_freq == MAX_FREQ_BASE)

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */


/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * EXTERNS
 */


// TODO : add checking working state for avoid change param in running

/*
 *  ======== local function ========
 */



STATIC int FREQ_isInJumpFreq(float_t value)
{
	int i;

	for(i=0; i<MAX_JUMP_FREQ_NUM; i++)
	{
		if(FREQ_isJumpFreqUsed(i))
		{
			if(param.ctrl.jump[i].low <= value && param.ctrl.jump[i].high >= value)
				return 1;
		}
	}

	return 0;
}

// check valid frequency range and return valid value
STATIC float_t FREQ_getRangedFreq(int cond, float_t value)
{
	int i;

	for(i=0; i<MAX_JUMP_FREQ_NUM; i++)
	{
		if(FREQ_isJumpFreqUsed(i))
		{
			if(param.ctrl.jump[i].low < value && param.ctrl.jump[i].high > value)
			{
				if(cond == ACCEL) return param.ctrl.jump[i].low;
				else return param.ctrl.jump[i].high;
			}
		}
	}

	return value;
}


/*
 *  ======== public function ========
 */

int FREQ_isValid(float_t value)
{
	float_t abs_val = fabsf(value);

	if(FREQ_isInJumpFreq(abs_val)) return 0;

	return 1;
}

int Freq_isInWorkingFreqRange(float_t value)
{
	int i;

	for(i=0; i<MAX_JUMP_FREQ_NUM; i++)
	{
		if(param.ctrl.jump[i].enable)
		{
			if(value >= param.ctrl.jump[i].low && value <= param.ctrl.jump[i].high)
				return 0;
		}
	}

	return 1;
}


int FREQ_setFreqValue(float_t value)
{
	param.ctrl.value = value;

	STA_setNextFreq(value);
	STA_calcResolution();

	return 0;
}

int FREQ_clearJumpFreq(int index)
{
	param.ctrl.jump[index].enable = 0;
	param.ctrl.jump[index].low = NOT_INITIALIZED;
	param.ctrl.jump[index].high = NOT_INITIALIZED;

	return 0;
}

int FREQ_setJumpFreqEnable(int index, int enable)
{
	param.ctrl.jump[index].enable = enable;
	dev_const.spd_jmp[index].enable = enable;

	return 0;
}

int FREQ_setJumpFreqLow(int index, float_t low)
{
	if(!Freq_isInWorkingFreqRange(low)) return 1;

	param.ctrl.jump[index].low = low;

	MAIN_setJumpSpeed(index, low, 0.0);

	return 0;
}

int FREQ_setJumpFreqHigh(int index, float_t high)
{
	if(!Freq_isInWorkingFreqRange(high)) return 1;

	param.ctrl.jump[index].high = high;

	MAIN_setJumpSpeed(index, 0.0, high);

	return 0;
}

int FREQ_setJumpFreqRange(int index, float_t low, float_t high)
{
	if(low == 0.0 && high == 0.0) // disable setting
		return FREQ_clearJumpFreq(index);

	if(!Freq_isInWorkingFreqRange(low)) return 1;

	if(!Freq_isInWorkingFreqRange(high)) return 1;

	param.ctrl.jump[index].enable = 1;
	param.ctrl.jump[index].low = low;
	param.ctrl.jump[index].high = high;

	MAIN_setJumpSpeed(index, low, high);

	return 0;
}

float_t FREQ_getVarifiedFreq(float_t current, float_t target)
{
	float_t value = current;
	int cond;


	if(target > current) cond = ACCEL;
	else if(target < current) cond = DECEL;
	else
	{
		UARTprintf("cur = %f, target=%f \n", current, target);
		return value;
	}
	value = FREQ_getRangedFreq(cond, target);

	return value;
}


float_t FREQ_convertToSpeed(float_t freq)
{
    float_t spd_rpm = (freq*60.0 / mtr.poles);

    return spd_rpm;
}

