/*
 * test_dci_brake.c
 *
 *  Created on: 2017. 3. 17.
 *      Author: hrjung
 */

#ifdef UNIT_TEST_ENABLED

#include "unity.h"
#include "../inv_param.h"
#include "../drive.h"


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

/*******************************************************************************
 * EXTERNS
 */

extern int DCIB_setStartFreq(float_t freq);
extern int DCIB_setBlockTime(float_t b_time);
extern int DCIB_setBrakeRate(float_t rate);
extern int DCIB_setBrakeTime(float_t b_time);
//extern int DCIB_setBrakeRateStarting(int rate);
//extern int DCIB_setBrakeTimeStarting(int b_time);
/*
 *  ======== function ========
 */

/*
 *
 * test procedure
 *
 *  1. start_speed range check -> OK
 * 	2. start_speed out of range -> error
 * 	3. compare with min freq, lower than min speed -> error
 * 	4. block_time is in 0 ~ 60 sec, normal -> OK
 * 	5. block_time is out of range -> error
 *
 */
void test_setDciBrakeParam(void)
{
	int result=0;
	int exp=0;
	float_t value;

	param.ctrl.freq_min = 10.0;
	param.ctrl.freq_max = 350.0;

	// start_freq
	value = 50.0; // in range
	exp = 0;
	result = DCIB_setStartFreq(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_FLOAT(value, param.brk.dci_start_freq);

	value = 370.0; // out of range > DC_BRAKE_SPEED_MAX
	exp = 1;
	result = DCIB_setStartFreq(value);
	TEST_ASSERT_EQUAL_INT(result, exp);

	value = 5.0; // lower than min speed
	exp = 1;
	result = DCIB_setStartFreq(value);
	TEST_ASSERT_EQUAL_INT(result, exp);

	// block time
	value = 30.0; // in range
	exp = 0;
	result = DCIB_setBlockTime(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_FLOAT(value, param.brk.dci_block_time);

	value = 100.0; // out of range
	exp = 1;
	result = DCIB_setBlockTime(value);
	TEST_ASSERT_EQUAL_INT(result, exp);

	//brake rate
	value = 150.0; // in range
	exp = 0;
	result = DCIB_setBrakeRate(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_FLOAT(value, param.brk.dci_braking_rate);

	value = 210.0; // out of range
	exp = 1;
	result = DCIB_setBrakeRate(value);
	TEST_ASSERT_EQUAL_INT(result, exp);

	//brake time
	value = 30.0; // in range
	exp = 0;
	result = DCIB_setBrakeTime(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_FLOAT(value, param.brk.dci_braking_time);

	value = 100.0; // out of range
	exp = 1;
	result = DCIB_setBrakeTime(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
}



#endif
