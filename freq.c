/*
 * freq.c
 *
 *  Created on: 2017. 3. 11.
 *      Author: hrjung
 */
#include <stdint.h>

#include "uartstdio.h"
#include "parameters.h"
//#include "inv_param.h"
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

#define JMP_ENABLE_BASE		JUMP_ENABLE0_INDEX
#define JMP_LOW_BASE		JUMP_LOW0_INDEX
#define JMP_HIGH_BASE		JUMP_HIGH0_INDEX

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

STATIC int FREQ_isJumpFreqUsed(int index)
{
	return (int)iparam[JMP_ENABLE_BASE + index].value.l;
}


STATIC int FREQ_isInJumpFreq(float_t value)
{
	int i;

	for(i=0; i<MAX_JUMP_FREQ_NUM; i++)
	{
		if(FREQ_isJumpFreqUsed(i))
		{
			if(iparam[JMP_LOW_BASE+i].value.f <= value && iparam[JMP_HIGH_BASE+i].value.f >= value)
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
			if(iparam[JMP_LOW_BASE+i].value.f < value && iparam[JMP_HIGH_BASE+i].value.f > value)
			{
				if(cond == ACCEL) return iparam[JMP_LOW_BASE+i].value.f;
				else return iparam[JMP_HIGH_BASE+i].value.f;
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
		if(FREQ_isJumpFreqUsed(i))
		{
			if(value >= iparam[JMP_LOW_BASE+i].value.f && value <= iparam[JMP_HIGH_BASE+i].value.f)
				return 0;
		}
	}

	return 1;
}


int FREQ_setFreqValue(float_t value)
{
	iparam[FREQ_VALUE_INDEX].value.f = value;

	STA_setNextFreq(value);
	STA_calcResolution();

	return 0;
}

int FREQ_clearJumpFreq(uint16_t index)
{
	iparam[JMP_ENABLE_BASE+index].value.l = 0;
	iparam[JMP_LOW_BASE+index].value.f = NOT_INITIALIZED;
	iparam[JMP_HIGH_BASE+index].value.f = NOT_INITIALIZED;

	return 0;
}

int FREQ_setJumpFreqEnable(uint16_t index, uint16_t enable)
{
	iparam[JMP_ENABLE_BASE+index].value.l = (uint32_t)enable;
	dev_const.spd_jmp[index].enable = enable;

	return 0;
}

int FREQ_setJumpFreqLow(uint16_t index, float_t low)
{
	if(!Freq_isInWorkingFreqRange(low)) return 1;

	iparam[JMP_LOW_BASE+index].value.f = low;

	MAIN_setJumpSpeed(index, low, 0.0);

	return 0;
}

int FREQ_setJumpFreqHigh(uint16_t index, float_t high)
{
	if(!Freq_isInWorkingFreqRange(high)) return 1;

	iparam[JMP_HIGH_BASE+index].value.f = high;

	MAIN_setJumpSpeed(index, 0.0, high);

	return 0;
}

int FREQ_setJumpFreqRange(int index, float_t low, float_t high)
{
	if(low == 0.0 && high == 0.0) // disable setting
		return FREQ_clearJumpFreq(index);

	if(!Freq_isInWorkingFreqRange(low)) return 1;

	if(!Freq_isInWorkingFreqRange(high)) return 1;

	iparam[JMP_ENABLE_BASE+index].value.l = 1;
	iparam[JMP_LOW_BASE+index].value.f = low;
	iparam[JMP_HIGH_BASE+index].value.f = high;

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

