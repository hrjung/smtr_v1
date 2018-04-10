/*
 * freq.h
 *
 *  Created on: 2017. 3. 11.
 *      Author: hrjung
 */

#ifndef FREQ_H_
#define FREQ_H_


/*******************************************************************************
 * CONSTANTS
 */
#define 	MAX_FREQ_VALUE	(400.0)
#define		MIN_FREQ_VALUE	(1.0)

#define 	ACCEL	0
#define		DECEL	1

#define FREQ_INPUT_RESOLUTION	(10.0)
//#define FREQ_INPUT_RESOLUTION	(100.0)


#define FREQ_isJumpFreqUsed(index)	(param.ctrl.jump[index].enable == 1)
/*******************************************************************************
 * EXTERNS
 */
extern int FREQ_isValid(float_t value);
extern int Freq_isInWorkingFreqRange(float_t value);

extern int FREQ_setFreqValue(float_t value);

extern int FREQ_setJumpFreqEnable(int index, int enable);
extern int FREQ_setJumpFreqLow(int index, float_t low);
extern int FREQ_setJumpFreqHigh(int index, float_t high);
extern int FREQ_clearJumpFreq(int index);
extern int FREQ_setJumpFreqRange(int index, float_t low, float_t high);

// valid frequency to set at last after checking range, jump freq.
extern float_t FREQ_getVarifiedFreq(float_t current, float_t target);

#if 0
extern int FREQ_setAccelTime(int index, int value);
extern int FREQ_setDecelTime(int index, int value);
extern int FREQ_setTimeUnit(int unit);
extern int FREQ_setTimebase(int value);
#endif

extern float_t FREQ_convertToSpeed(float_t freq);
extern int FREQ_showUserSpeed(float_t rpm);

extern int FREQ_convertAinToFreq(int type, int adc_value);

#endif /* FREQ_H_ */
