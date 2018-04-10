/*
 * dci_brake.h
 *
 *  Created on: 2017. 3. 17.
 *      Author: hrjung
 */

#ifndef BRAKE_H_
#define BRAKE_H_


// DCI_STATE : NONE -> START -> BLOCK ->
enum
{
	DCI_NONE_STATE = 0,
	DCI_BLOCK_STATE,
	DCI_DC_BRAKE_STATE,
	DCI_PWM_OFF_STATE,

};


#define MAX_REGEN_LIMIT_FREQ	(5)


extern int BRK_setBrakeMethod(int method);
extern int BRK_setBrakeTIME(int limit);
extern int BRK_setThreshold(float_t freq);

extern int DCIB_getState(void);
extern int DCIB_setStartFreq(float_t freq);
extern int DCIB_setBlockTime(float_t b_time);
extern int DCIB_setBrakeRate(float_t rate);
extern int DCIB_setBrakeTime(float_t b_time);
extern int DCIB_setBrakeRateStarting(int rate);
extern int DCIB_setBrakeTimeStarting(int b_time);

extern int DCIB_processBrakeSigHandler(void);
#endif /* BRAKE_H_ */
