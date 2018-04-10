/*
 * test_protect.c
 *
 *  Created on: 2017. 3. 20.
 *      Author: hrjung
 */


#ifdef UNIT_TEST_ENABLED

#include "unity.h"
#include "../inv_param.h"
#include "drive.h"


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

extern int OVL_setWarningLevel(int level);
extern int OVL_setTripLevel(int level);
extern int OVL_setWarningTime(int dur);
extern int OVL_setTripTime(int dur);

extern int REGEN_isEnabled(void);
extern int REGEN_process(float_t dc_volt);

/*
 *  ======== function ========
 */

/*
 *
 * test procedure
 *
 *  1. set overload % current level for warning
 *  2. set warning time to make warn
 *  3.
 */

void test_setOverload(void)
{
	int result=0;
	int exp=0;
	int value;

	// set max current level OK, 30 ~ 150%
	value = 140;
	exp = 0; //OK
	result = OVL_setWarningLevel(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(value, param.protect.ovl.wr_limit);

	value = 20; //set level under limit -> NOK
	exp = 1; //NOK
	result = OVL_setWarningLevel(value);
	TEST_ASSERT_EQUAL_INT(result, exp);

	value = 160; //set level over limit -> NOK
	exp = 1; //NOK
	result = OVL_setWarningLevel(value);
	TEST_ASSERT_EQUAL_INT(result, exp);


	// set warning duration OK, 0 ~ 30sec
	value = 20;
	exp = 0; //OK
	result = OVL_setWarningTime(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(value, param.protect.ovl.wr_duration);

	value = 40; //set warning duration over limit -> NOK
	exp = 1; //NOK
	result = OVL_setWarningTime(value);
	TEST_ASSERT_EQUAL_INT(result, exp);


	// set trip current level OK : 30 ~ 200%
	value = 140;
	exp = 0; //OK
	result = OVL_setTripLevel(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(value, param.protect.ovl.tr_limit);

	value = 20; //set level under limit -> NOK
	exp = 1; //NOK
	result = OVL_setTripLevel(value);
	TEST_ASSERT_EQUAL_INT(result, exp);

	value = 210; //set trip level over limit -> NOK
	exp = 1; //NOK
	result = OVL_setTripLevel(value);
	TEST_ASSERT_EQUAL_INT(result, exp);

	// set trip time duration OK, 0 ~ 60sec
	value = 50;
	exp = 0; //OK
	result = OVL_setTripTime(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(value, param.protect.ovl.tr_duration);

	value = 70; //set trip duration over limit -> NOK
	exp = 1; //NOK
	result = OVL_setTripTime(value);
	TEST_ASSERT_EQUAL_INT(result, exp);
}


/*
 *
 * test procedure
 *
 *  1. before init relay, DC volt under relay level
 *  2. DC > relay level, relay_enabled
 *  3. DC under volage trip
 */

void test_processDcVoltage(void)
{
	int result=0;
	int exp=0;
	float_t dc_val;

	// before init relay, process OK
	dc_val = 150.0;
	exp = 0; //OK
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(0, internal_status.relay_enabled);

	// init relay on
	dc_val = 160.0;
	exp = 0; //OK
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(1, internal_status.relay_enabled);

	// DC under voltage trip
	dc_val = 150.0;
	exp = 1; // Trip
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);

	// slight below than regen enabled
	dc_val = 370.0;
	exp = 0; // near regen level
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(0, REGEN_isEnabled());

	// regen enabled
	dc_val = 380.0;
	exp = 0; // start regen
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(1, REGEN_isEnabled());

	// regen enabled, below over voltage trip
	dc_val = 400.0;
	exp = 0; // still OK
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(1, REGEN_isEnabled());

	// below regen level, still regen enabled
	dc_val = 370.0;
	exp = 0; // still OK
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(1, REGEN_isEnabled());

	// slightly over regen end level, still regen enabled
	dc_val = 360.0;
	exp = 0; // still OK
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(1, REGEN_isEnabled());

	// under regen end level, regen disabled
	dc_val = 350.0;
	exp = 0; // still OK
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(0, REGEN_isEnabled());

	// over regen end level, still regen disabled
	dc_val = 360.0;
	exp = 0; // still OK
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
	TEST_ASSERT_EQUAL_INT(0, REGEN_isEnabled());

	// DC over voltage trip
	dc_val = 410.0;
	exp = 1; // Trip
	result = REGEN_process(dc_val);
	TEST_ASSERT_EQUAL_INT(result, exp);
}

#endif
