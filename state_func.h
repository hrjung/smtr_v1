/*
 * state_func.h
 *
 *  Created on: 2017. 3. 17.
 *      Author: hrjung
 */

#ifndef STATE_FUNC_H_
#define STATE_FUNC_H_

/*******************************************************************************
 * TYPEDEFS
 */
typedef enum
{
	STATE_START,
	STATE_STOP,
	STATE_ACCEL,
	STATE_DECEL,
	STATE_RUN
} mtr_state_e;

typedef struct
{
	float_t cur_rpm;		// normal ranged value
	float_t target_rpm;	// normal ranged value

	float_t cur_freq;		// normal ranged value
	float_t target_freq;	// normal ranged value

	int direction;
	//int prev_acc_time;
	//int prev_dec_time;
	float_t acc_res; // accelerate resolution per ISR, scale downed value
	float_t dec_res; // decelerate resolution per ISR, scale downed value

	float_t current;
	mtr_state_e status;

} MOTOR_working_st;

/*******************************************************************************
 * MACROS
 */



/*******************************************************************************
 * EXTERNS
 */
//extern MOTOR_working_st sta_freq;

extern int STA_isStopState(void);
extern int STA_isAccelState(void);
extern int STA_isDecelState(void);

extern void STA_setStopStatus(void);
//extern float STA_getCurrentSpeed(void);
//extern float STA_getTargetSpeed_krpm(void);
extern void STA_setCurSpeed(float_t cur_spd);
extern float_t STA_getCurSpeed(void);
//extern void STA_setNextSpeed(int target);
//extern void STA_updateCurrentSpeed(float speed);
//extern int STA_getSpeedRange(void);
extern float_t STA_getTargetFreq(void);
extern float_t STA_getCurFreq(void);
extern void STA_setCurFreq(float_t cur_freq);
extern void STA_setNextFreq(float_t target);
extern void STA_updateCurrentFreq(float_t freq);
extern int STA_getFreqRange(void);
extern float_t STA_getResolution(int flag);
extern void STA_setResolution(int flag, float_t value);
extern void STA_calcResolution(void);
extern float_t STA_getTrajResolution(void);
extern int STA_isSameAccelRate(void);
extern void STA_setStopCondition(void);

extern int STA_control(void);

#endif /* STATE_FUNC_H_ */
