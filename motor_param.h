/*
 * nv_param.h
 *
 *  Created on: 2017. 8. 22.
 *      Author: hrjung
 */

#ifndef NV_PARAM_H
#define NV_PARAM_H

#include "stdint.h"


#ifdef UNIT_TEST_ENABLED
#define STATIC
#else
#define STATIC static
#endif

/*******************************************************************************
 * CONSTANTS
 */

#ifdef TEST_MOTOR // test Motor
#define TEST_MOTOR_NUM_POLE_PAIRS	2
#define TEST_MOTOR_EFFECTIVENESS	90
#define TEST_MOTOR_VOLTAGE_IN		220
#define TEST_MOTOR_RATED_FREQ		60
#define TEST_MOTOR_CAPACITY			(0.25)
#define TEST_MOTOR_SLIP_RATE		(5.0)
#define TEST_MOTOR_NOLOAD_CURRENT	(1.15)
#define TEST_MOTOR_MAX_CURRENT		(3.0)
#define TEST_MOTOR_Rr				(5.574939)
#define TEST_MOTOR_Rs				(10.1598806)
#define TEST_MOTOR_Ls				(0.00392938871)
#endif

// 1.5KW Samyang Motor
#ifdef SAMYANG_1_5K_MOTOR
#define TEST_MOTOR_NUM_POLE_PAIRS	2
#define TEST_MOTOR_EFFECTIVENESS	90
#define TEST_MOTOR_RATED_FREQ		60
#define TEST_MOTOR_CAPACITY			(1.5)
#define TEST_MOTOR_SLIP_RATE		(5.0)
	#if 0
	#define TEST_MOTOR_VOLTAGE_IN		220
	#define TEST_MOTOR_NOLOAD_CURRENT	(3.4)
	#define TEST_MOTOR_MAX_CURRENT		(15.0) //(6.0)
	#else
	#define TEST_MOTOR_VOLTAGE_IN		380
	#define TEST_MOTOR_NOLOAD_CURRENT	(2.0)
	#define TEST_MOTOR_MAX_CURRENT		(3.4) //(3.5)
	#endif
#define TEST_MOTOR_Rr				(2.14568)
#define TEST_MOTOR_Rs				(2.5)
//#define TEST_MOTOR_Ls				(0.02791) // line to line
#define TEST_MOTOR_Ls				(0.013955)
#endif

#ifdef SAMYANG_2_2K_MOTOR
#define TEST_MOTOR_NUM_POLE_PAIRS	2
#define TEST_MOTOR_EFFECTIVENESS	90
#define TEST_MOTOR_RATED_FREQ		60
#define TEST_MOTOR_CAPACITY			(2.2)
#define TEST_MOTOR_SLIP_RATE		(5.0)
	#if 1
	#define TEST_MOTOR_VOLTAGE_IN		220
	#define TEST_MOTOR_NOLOAD_CURRENT	(5.588)
	#define TEST_MOTOR_MAX_CURRENT		(9.8)
	#else
	#define TEST_MOTOR_VOLTAGE_IN		380
	#define TEST_MOTOR_NOLOAD_CURRENT	(3.235)
	#define TEST_MOTOR_MAX_CURRENT		(5.3)
	#endif
#define TEST_MOTOR_Rr				(1.14793)
#define TEST_MOTOR_Rs				(2.86)
#define TEST_MOTOR_Ls				(0.02184)
#endif


#endif /* NV_PARAM_H */
