/*
 * test_trip.c
 *
 *  Created on: 2017. 7. 27.
 *      Author: hrjung
 */


#ifdef UNIT_TEST_ENABLED

#include "unity.h"
#include "../inv_param.h"
#include "../drive.h"
#include "../state_func.h"
#include "../err_trip.h"

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
extern int ERR_clearTripData(void);

/*
 *  ======== function ========
 */

/*
 *
 * test procedure
 *
 *  1. gen error info, read current error
 *  2. next error written to next index
 *  3. 6th error is written to err[0]
 *  3. cur == target, then go to RUN state

 */

void test_genErrInfo(int err_code, int state)
{
	m_status.cur_freq = 20+err_code;
	m_status.current = 10+err_code;
	m_status.status = (mtr_state_e)state;
}

void test_errorTrip(void)
{
	int result=0;
	int exp=0;
	int err=0;

	ERR_clearTripData();

	// first error;
	test_genErrInfo(err, STATE_ACCEL);
	ERR_setTripInfo(err);
	exp = 0;
	result = ERR_getCurrentErrCode();
	TEST_ASSERT_EQUAL_INT(exp, result);

	// next error
	err = 5;
	test_genErrInfo(err, STATE_ACCEL);
	ERR_setTripInfo(err);
	exp = 5;
	result = ERR_getCurrentErrCode();
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(1, dev_param.err_index);

	// fill all history
	err=10;
	test_genErrInfo(err, STATE_ACCEL);
	ERR_setTripInfo(err);
	err=15;
	test_genErrInfo(err, STATE_ACCEL);
	ERR_setTripInfo(err);
	err=20;
	test_genErrInfo(err, STATE_ACCEL);
	ERR_setTripInfo(err);
	exp=20;
	result = ERR_getCurrentErrCode();
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(4, dev_param.err_index);

	// warp around for 6th error
	err=25;
	test_genErrInfo(err, STATE_ACCEL);
	ERR_setTripInfo(err);
	exp=25;
	result = ERR_getCurrentErrCode();
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(0, dev_param.err_index);
}

#endif
