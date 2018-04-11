/*
 * test_state.c
 *
 *  Created on: 2017. 3. 20.
 *      Author: hrjung
 */


#ifdef UNIT_TEST_ENABLED

#include "unity.h"
#include "../inv_param.h"
#include "../drive.h"
#include "../state_func.h"

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


/*
 *  ======== function ========
 */

/*
 *
 * test procedure
 *
 *  1. check first state is START and goto STOP
 *  2. set target freq, then go to ACCEL state
 *  3. cur == target, then go to RUN state
 *  4. set higher target freq , then go to ACCEL state
 *  5. cur == target, then go to RUN state
 *  6. set target freq =  , then go to DECEL state
 *  7. cur == target, then go to STOP state
 */

void test_controlState(void)
{
	int result=0;
	int exp=0;
	float_t target;

	// first state should be STATE_STOP
	exp = STATE_STOP;
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	//UARTprintf(" stop -> accel \n");
	// accelerate to freq  = 10.0
	exp = STATE_ACCEL;
	target = 100.0;
	STA_setNextFreq(target);
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	//UARTprintf(" accel -> run \n");
	m_status.cur_freq = target; // finally reached target
	//STA_setNextFreq(target);
	exp = STATE_RUN;
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// accelerate to freq  = 200
	exp = STATE_ACCEL;
	target = 200.0;
	STA_setNextFreq(target);
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	m_status.cur_freq = target; // finally reached target
	STA_setNextFreq(target);
	exp = STATE_RUN;
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// decelerate to freq  = 120
	exp = STATE_DECEL;
	target = 120.0;
	STA_setNextFreq(target);
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	m_status.cur_freq = target; // finally reached target
	STA_setNextFreq(target);
	exp = STATE_RUN;
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// back to 200, accelerate
	// accelerate to freq  = 200
	exp = STATE_ACCEL;
	target = 200.0;
	STA_setNextFreq(target);
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	m_status.cur_freq = target; // finally reached target
	STA_setNextFreq(target);
	exp = STATE_RUN;
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// decelerate to freq  = 50
	exp = STATE_DECEL;
	target = 50.0;
	STA_setNextFreq(target);
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	m_status.cur_freq = target; // finally reached target
	STA_setNextFreq(target);
	exp = STATE_RUN;
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// go to stop freq  = 0
	exp = STATE_DECEL;
	target = 0.0;
	STA_setNextFreq(target);
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);

	m_status.cur_freq = target; // finally reached target
	STA_setNextFreq(target);
	exp = STATE_STOP;
	result = STA_control();
	TEST_ASSERT_EQUAL_INT(exp, result);
}

#endif
