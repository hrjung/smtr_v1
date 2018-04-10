/*
 * timer_handler.h
 *
 *  Created on: 2017. 5. 18.
 *      Author: hrjung
 */

#ifndef TIMER_HANDLER_H_
#define TIMER_HANDLER_H_

enum
{
	DCI_BRAKE_SIG_ON_TSIG = 0, 	// apply brake PWM, 0 ~ 60sec
	DCI_BRAKE_SIG_OFF_TSIG,		// block PWM shortly,  0 ~ 60sec

	OVERLOAD_WARN_START_TSIG, // 0 ~ 30sec
	OVERLOAD_WARN_END_TSIG, // 0 ~ 30sec
	OVERLOAD_TRIP_TSIG,		// 0 ~ 60sec
	OVERLOAD_OVC_TSIG,		// 3sec fix

	ELECTRO_THERMAL_1MIN_TSIG, // 60sec fix

	TIMER_TEST_TSIG,

	MAX_TIMER_TSIG
};

#define OVER_CURRENT_TIMEOUT	1	// 1 sec

#if 0
typedef struct
{
	float TargetSpeed;
	float Tmp;
	float CurrentSpeed;
	float resolution;
} ramp_control_st;
#endif


extern void TMR_clearRunTime(void);
extern void TMR_clearOnTime(void);

extern uint32_t TMR_getRunTime(void);
extern uint32_t TMR_getOnTime(void);

extern int TMR_isTimeout(int type);
extern int TMR_isTimerEnabled(int type);
extern uint32_t TMR_startTimerSig(int type, float_t f_duration);
extern void TMR_disableTimerSig(int type);
extern void TMR_init(void);

#endif /* TIMER_HANDLER_H_ */
