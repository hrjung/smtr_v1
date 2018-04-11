/*
 * test_convert.c
 *
 *  Created on: 2017. 5. 12.
 *      Author: hrjung
 */


#ifdef UNIT_TEST_ENABLED

#include <math.h>
#include "main.h"

#include "unity.h"
#include "../inv_param.h"
#include "../drive.h"
#include "../state_func.h"
#include "../freq.h"

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


/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
extern MOTOR_working_st m_status;
/*******************************************************************************
 * EXTERNS
 */
//extern float MAIN_convert2InternalSpeedRef(int speed);
extern int MAIN_convert2Speed(float speedRef);

//extern int STA_calcRampResolution(void);
//extern void RAMP_control(void);

/*
 *  ======== function ========
 */

/*
 *
 * test procedure
 *		spd_min : 300
 *		spd_max : 1800
 *		gear_ratio : 10
 *
 *     convert user speed setting to internal krpm value, considering gear_ratio
 *     ex) user input 100rpm, then actual motor krpm is 1 krpm
 *
 *     actual motor krpm -> user speed rpm
 */

#if 1
void test_processConvertFreq(void)
{
	float exp=0;
	int exp_n, freq;
	float speed_krpm;

/*
 * 		freq <-> rpm
 * 		20		600
 * 		40		1200
 * 		60		1800
 * 		120		3600
 * 		180		5400
 * 		240		7200
 * 		300		9000
 * 		360		10800
 */

	// speed = 0 -> speed_krpm = 0
	freq = 0;
	exp = 0.0;
	speed_krpm = FREQ_convertToSpeed(freq)/1000;
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// freq = 30 -> speed_krpm = 0.3
	freq = 10;
	exp = 0.3;
	speed_krpm = FREQ_convertToSpeed(freq)/1000;
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// freq = 60 -> speed_krpm = 1.0 : center
	freq = 20;
	exp = 0.6;
	speed_krpm = FREQ_convertToSpeed(freq)/1000;
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// freq = 90 -> speed_krpm = 1.2
	freq = 50;
	exp = 1.5;
	speed_krpm = FREQ_convertToSpeed(freq)/1000;
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// freq = 120 -> speed_krpm = 1.8
	freq = 60;
	exp = 1.8;
	speed_krpm = FREQ_convertToSpeed(freq)/1000;
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// freq = 180 -> speed_krpm = 5.4
	freq = 180;
	exp = 5.4;
	speed_krpm = FREQ_convertToSpeed(freq)/1000;
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// freq = 360 -> speed_krpm = 10.8
	freq = 360;
	exp = 10.8;
	speed_krpm = FREQ_convertToSpeed(freq)/1000;
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// speed_krpm = 0 -> freq = 0
	speed_krpm = 0.0;
	exp_n = 0;
	freq = MAIN_convert2Freq(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, freq);

	// speed_krpm = 0.7 -> freq = 70
	speed_krpm = 0.3;
	exp_n = 10;
	freq = MAIN_convert2Freq(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, freq);

	// speed_krpm = 0.9 -> freq = 90
	speed_krpm = 0.9;
	exp_n = 30;
	freq = MAIN_convert2Freq(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, freq);

	// speed_krpm = 1.3 -> freq = 130
	speed_krpm = 1.2;
	exp_n = 40;
	freq = MAIN_convert2Freq(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, freq);

	// speed_krpm = 1.05 -> freq = 300
	speed_krpm = 1.5;
	exp_n = 50;
	freq = MAIN_convert2Freq(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, freq);

	speed_krpm = 1.8;
	exp_n = 60;
	freq = MAIN_convert2Freq(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, freq);

	speed_krpm = 5.4;
	exp_n = 180;
	freq = MAIN_convert2Freq(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, freq);

	speed_krpm = 10.8;
	exp_n = 360;
	freq = MAIN_convert2Freq(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, freq);
}
#else
void test_processSpeedScaling(void)
{
	float exp=0;
	int exp_n, speed;
	float speed_krpm;

	// speed = 0 -> speed_krpm = 0
	speed = 0;
	exp = 0.0;
	speed_krpm = MAIN_convert2InternalSpeedRef(speed);
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// speed = 30 -> speed_krpm = 0.3
	speed = 30;
	exp = 0.3;
	speed_krpm = MAIN_convert2InternalSpeedRef(speed);
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// speed = 100 -> speed_krpm = 1.0 : center
	speed = 80;
	exp = 0.8;
	speed_krpm = MAIN_convert2InternalSpeedRef(speed);
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// speed = 120 -> speed_krpm = 1.2
	speed = 120;
	exp = 1.2;
	speed_krpm = MAIN_convert2InternalSpeedRef(speed);
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// speed = 180 -> speed_krpm = 1.8
	speed = 180;
	exp = 1.8;
	speed_krpm = MAIN_convert2InternalSpeedRef(speed);
	TEST_ASSERT_EQUAL_FLOAT(exp, speed_krpm);

	// speed_krpm = 0 -> speed = 0
	speed_krpm = 0.0;
	exp_n = 0;
	speed = MAIN_convert2Speed(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, speed);

	// speed_krpm = 0.7 -> speed = 70
	speed_krpm = 0.3;
	exp_n = 30;
	speed = MAIN_convert2Speed(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, speed);

	// speed_krpm = 0.9 -> speed = 90
	speed_krpm = 0.9;
	exp_n = 90;
	speed = MAIN_convert2Speed(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, speed);

	// speed_krpm = 1.3 -> speed = 130
	speed_krpm = 1.3;
	exp_n = 130;
	speed = MAIN_convert2Speed(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, speed);

	// speed_krpm = 1.05 -> speed = 300
	speed_krpm = 1.6;
	exp_n = 160;
	speed = MAIN_convert2Speed(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, speed);

	speed_krpm = 1.8;
	exp_n = 180;
	speed = MAIN_convert2Speed(speed_krpm);
	TEST_ASSERT_EQUAL_INT(exp_n, speed);
}
#endif

int compare_float_almost_same(float_t exp, float_t result)
{
	float_t delta = 0.000001f;
	float_t diff;

	if(exp - result >= 0) diff = exp - result;
	else diff = result - exp;

	if(diff < delta) return 1;
	else return 0;
}

void test_processResolutionTargetFreq(void)
{
	float_t exp=0;
	int result;

	//test
	exp = 0.000222f;
	result = compare_float_almost_same(exp, 0x0003f);
	TEST_ASSERT_EQUAL_INT(0, result); //not same

	// 5 sec
	param.ctrl.accel_time = 5.0;
	param.ctrl.decel_time = 5.0;

	m_status.cur_freq = 10.0;
	exp = 0.06; // krpm
	m_status.target_freq = 20.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
	//result = compare_float_almost_same(exp, m_status.acc_res);
	//TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 20.0;
	exp = 0.12; // krpm
	m_status.target_freq = 40.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 40.0;
	exp = 0.24; // krpm
	m_status.target_freq = 80.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 80.0;
	exp = 0.24; // krpm
	m_status.target_freq = 40.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 40.0;
	exp = 0.12; // krpm
	m_status.target_freq = 20.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 20.0;
	exp = 0.06; // krpm
	m_status.target_freq = 10.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	//1 sec
	param.ctrl.accel_time = 1.0;
	param.ctrl.decel_time = 1.0;

	m_status.cur_freq = 10.0;
	exp = 0.3; // krpm
	m_status.target_freq = 20.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 20.0;
	exp = 0.6; // krpm
	m_status.target_freq = 40.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 40.0;
	exp = 1.2; // krpm
	m_status.target_freq = 80.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 80.0;
	exp = 1.2; // krpm
	m_status.target_freq = 40.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 40.0;
	exp = 0.6; // krpm
	m_status.target_freq = 20.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 20.0;
	exp = 0.3; // krpm
	m_status.target_freq = 10.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);


	//10 sec
	param.ctrl.accel_time = 10.0;
	param.ctrl.decel_time = 10.0;

	m_status.cur_freq = 10.0;
	exp = 0.03; // krpm
	m_status.target_freq = 20.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 20.0;
	exp = 0.06; // krpm
	m_status.target_freq = 40.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 40.0;
	exp = 0.12; // krpm
	m_status.target_freq = 80.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 80.0;
	exp = 0.12; // krpm
	m_status.target_freq = 40.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 40.0;
	exp = 0.06; // krpm
	m_status.target_freq = 20.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 20.0;
	exp = 0.03; // krpm
	m_status.target_freq = 10.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);


	// 50 sec
	param.ctrl.accel_time = 50.0;
	param.ctrl.decel_time = 50.0;

	m_status.cur_freq = 10.0;
	exp = 0.006; // krpm
	m_status.target_freq = 20.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 20.0;
	exp = 0.012; // krpm
	m_status.target_freq = 40.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 40.0;
	exp = 0.024; // krpm
	m_status.target_freq = 80.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.acc_res);
//	result = compare_float_almost_same(exp, m_status.acc_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 80.0;
	exp = 0.024; // krpm
	m_status.target_freq = 40.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 40.0;
	exp = 0.012; // krpm
	m_status.target_freq = 20.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur_freq = 20.0;
	exp = 0.006; // krpm
	m_status.target_freq = 10.0;
	STA_calcResolution();
	TEST_ASSERT_EQUAL_FLOAT(exp, m_status.dec_res);
//	result = compare_float_almost_same(exp, m_status.dec_res);
//	TEST_ASSERT_EQUAL_INT(1, result);
}

#if 1
void test_processResolutionMinMax(void)
{
	float_t exp=0;
	int result;
	int acc_time;

	//test
	exp = 0.000222f;
	result = compare_float_almost_same(exp, 0x0003f);
	TEST_ASSERT_EQUAL_INT(0, result); //not same

	// freq range : 0 ~ 360
	// acc_time : resolution
	//    10    :   10.8
	//	  50	:   2.16
	//   100	:   1.08
	//	 300 	:   0.36
	//   400    :   0.27
	acc_time = 10; // 1sec
	DRV_setAccelTime(acc_time);
	exp = 10.8; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 50; // 5sec
	DRV_setAccelTime(acc_time);
	exp = 2.16; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 100; // 10sec
	DRV_setAccelTime(acc_time);
	exp = 1.08; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 300; // 30sec
	DRV_setAccelTime(acc_time);
	exp = 0.36; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 400; // 40sec
	DRV_setAccelTime(acc_time);
	exp = 0.27; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	// decelerate time
	acc_time = 10; // 1sec
	DRV_setDecelTime(acc_time);
	exp = 10.8; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 50; // 5sec
	DRV_setDecelTime(acc_time);
	exp = 2.16; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 100; // 10sec
	DRV_setDecelTime(acc_time);
	exp = 1.08; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 300; // 20sec
	DRV_setDecelTime(acc_time);
	exp = 0.36; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 400; // 40sec
	DRV_setDecelTime(acc_time);
	exp = 0.27; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);
}
#else

void test_processResolution(void)
{
	float exp=0;
	int result;
	int acc_time;

	//test
	exp = 0.000222f;
	result = compare_float_almost_same(exp, 0x0003f);
	TEST_ASSERT_EQUAL_INT(0, result); //not same

	// spd range : 300 ~ 1800
	// acc_time : resolution
	//    10    :   1.5
	//	  50	:   0.3
	//   100	:   0.15
	//	 300 	:   0.05
	acc_time = 10; // 1sec
	DRV_setAccelTime(acc_time);
	exp = 1.5; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 50; // 5sec
	DRV_setAccelTime(acc_time);
	exp = 0.3; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 100; // 10sec
	DRV_setAccelTime(acc_time);
	exp = 0.15; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 300; // 10sec
	DRV_setAccelTime(acc_time);
	exp = 0.05; // krpm
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	// decelerate time
	acc_time = 10; // 1sec
	DRV_setDecelTime(acc_time);
	exp = 1.5; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 50; // 5sec
	DRV_setDecelTime(acc_time);
	exp = 0.3; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 100; // 10sec
	DRV_setDecelTime(acc_time);
	exp = 0.15; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	acc_time = 300; // 10sec
	DRV_setDecelTime(acc_time);
	exp = 0.05; // krpm
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);
}


void test_processResolution(void)
{
	float exp=0;
	int result;

	//set accel time 10sec, speed_range = 1800 - 300
	param.ctrl.accel_time = 100; // 10sec
	m_status.target = 100; // 0 -> 100

	//test
	exp = 0.000222f;
	result = compare_float_almost_same(exp, 0x0003f);
	TEST_ASSERT_EQUAL_INT(0, result); //not same

	m_status.cur = 0;
	exp = 0.0015f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur = 0;
	mtr.rpm_max = 1800;
	mtr.rpm_min = 500;
	exp = 0.0013f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur = 0;
	mtr.rpm_max = 1800;
	mtr.rpm_min = 800;
	exp = 0.001f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	mtr.rpm_max = 1800;
	mtr.rpm_min = 300;
	m_status.cur = 0;
	param.ctrl.accel_time = 50;
	exp = 0.003f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur = 0;
	param.ctrl.accel_time = 300;
	exp = 0.0005f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur = 0;
	param.ctrl.accel_time = 10;
	exp = 0.015f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	// decelerate
	param.ctrl.decel_time = 100;
	m_status.cur = 150;
	exp = 0.0015f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur = 150;
	param.ctrl.decel_time = 300;
	exp = 0.0005f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.dec_res);
	TEST_ASSERT_EQUAL_INT(1, result);

	m_status.cur = 150;
	param.ctrl.decel_time = 10;
	exp = 0.015f;
	result = STA_calcRampResolution();
	result = compare_float_almost_same(exp, m_status.acc_res);
	TEST_ASSERT_EQUAL_INT(1, result);
}
#endif

#endif
