/*
 * drive.h
 *
 *  Created on: 2017. 4. 6.
 *      Author: hrjung
 */

#ifndef DRIVE_H_
#define DRIVE_H_


/*******************************************************************************
 * CONSTANTS
 */

// accel, decel time unit is 100ms, 600 -> 60sec
#define 	MIN_ACCEL_TIME		(1.0)	// 100ms
#define 	MAX_ACCEL_TIME		(600.0)	// 60 sec

// used for actual time in ms unit
//#define 	ACCEL_TIME_SCALE	(100)

#define 	ACCEL	0
#define		DECEL	1


#define 	DRV_SIO_DIN		0
#define		DRV_SIO_DOUT	1

#define		FIELDBUS_EXIST	0x01
#define 	UIO_EXIST		0x02
#define 	NFC_EXIST		0x04

/*******************************************************************************
 * EXTERNS
 */
//extern int DRV_isValidSpeed(int value);
//extern int DRV_setSpeedValue(int index, int value);
//extern int DRV_getActualSpeed(int value);
//extern int DRV_getFreqRangeMin(void);
//extern int DRV_getFreqRangeMax(void);

extern int DRV_setAccelTime(float_t value);
extern int DRV_setDecelTime(float_t value);

extern int DRV_isVfControl(void);
extern void DRV_enableVfControl(void);
extern void DRV_enableFocControl(void);

extern int DRV_setTorqueLimit(float_t limit);
extern int DRV_setEnergySave(int on_off);
extern int DRV_setVoltageBoost(float_t value);
extern int DRV_setPwmFrequency(int value);
extern int DRV_setSpdGainP(float_t value);
extern int DRV_setSpdGainI(float_t value);

extern float_t DRV_calculateAccelRate_krpm(float_t time_100msec, float_t diff);

extern int DRV_runForward(int index);
extern int DRV_runBackward(int index);
extern int DRV_stopMotor(void);

#endif /* DRIVE_H_ */
