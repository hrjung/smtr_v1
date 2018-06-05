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

#define SHAFT_BRAKE_ENABLE_FREQ_MIN		(0.1)
#define SHAFT_BRAKE_ENABLE_FREQ_MAX		(60.0)

extern int BRK_isDCIBrakeEnabled(void);
extern int BRK_isFreeRunEnabled(void);

extern int BRK_setBrakeMethod(uint16_t method);
extern int BRK_setBrakeFreq(float_t freq);

extern int DCIB_getState(void);
extern int DCIB_setStartFreq(float_t freq);
extern int DCIB_setBlockTime(float_t b_time);
extern int DCIB_setBrakeRate(float_t rate);
extern int DCIB_setBrakeTime(float_t b_time);
//extern int DCIB_setBrakeRateStarting(uint16_t rate);
//extern int DCIB_setBrakeTimeStarting(uint16_t b_time);

extern int DCIB_processBrakeSigHandler(void);
#endif /* BRAKE_H_ */
