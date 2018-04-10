/*
 * test_freq.c
 *
 *  Created on: 2017. 3. 10.
 *      Author: hrjung
 */

#ifdef UNIT_TEST_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unity.h"
#include "../inv_param.h"
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

/*******************************************************************************
 * EXTERNS
 */

/*
 *  ======== function ========
 */

void setUp(void) {
// set stuff up here
	memset(&param, 0, sizeof(param));
}
void tearDown(void) {
// clean stuff up here
	memset(&param, 0, sizeof(param));
}

/*
 * test procedure
 *
 * 	1. check valid freq range (min ~ max or low ~ high)
 * 	2. avoid jump freq range
 * 	3. default freq range should be applied for no range setting ( 1 ~ 400 )
 * 	4. default freq range should be applied for max, min, low, high limit value
 * 	5. when jump freq range set, check all range conditions.
 * 	6. clear jump range, then clear low, high as well.
 *
 */

void test_setFreqParam(void)
{
	int result=0;
	int exp=0;
	int index=0;
	float_t value, cur, low, high;
//	int i=0;

	// set normal working frequency
	value=10.0;
	exp=0; // OK
	result = FREQ_setFreqValue(index, value);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(value, param.ctrl.value[index]);

	// set higher than default frequency
	value=500.0;
	exp=1; //NOK
	result = FREQ_setFreqValue(index, value);
	TEST_ASSERT_EQUAL_INT(exp, result);


	// set min freq range error < DEFAULT_MIN
//	value=0;
//	exp=1; //NOK
//	result = FREQ_setFreqRangeMin(value);
//	TEST_ASSERT_EQUAL_INT(result, exp);

	// set min freq range error > DEFAULT_MAX
	value=500.0;
	exp=1; //NOK
	result = FREQ_setFreqRangeMin(value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// set min freq range OK
	value=100.0;
	exp=0; //OK
	result = FREQ_setFreqRangeMin(value);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(value, param.ctrl.freq_min);


	// set max freq range error < DEFAULT_MIN
	value=0;
	exp=1; //NOK
	result = FREQ_setFreqRangeMax(value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// set max freq range error > DEFAULT_MAX
	value=500.0;
	exp=1; //NOK
	result = FREQ_setFreqRangeMax(value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// set max freq range OK : 100 ~ 300
	value=300.0;
	exp=0; //OK
	result = FREQ_setFreqRangeMax(value);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(value, param.ctrl.freq_max);

	// set min freq range error > freq.max
	value=350.0;
	exp=1; //NOK
	result = FREQ_setFreqRangeMin(value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// set max freq range error < freq.min
	value=50.0;
	exp=1; //NOK
	result = FREQ_setFreqRangeMax(value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	cur = value;
	value = 90.0;
	exp = 100; // min range
	result = FREQ_getVarifiedFreq(cur, value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	value = 310.0;
	exp = 300; // max range
	result = FREQ_getVarifiedFreq(cur, value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//UARTprintf("min=%f max=%f\n", param.ctrl.freq_min, param.ctrl.freq_max);
	// check min ~ max range
	// current min ~ max : 100 ~ 300
	// set OK in min ~ max range
	value = 250.0;
	exp = 0;
	result = FREQ_setFreqValue(index, value);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(value, param.ctrl.value[index]);

	//set jump freq
	// current min ~ max : 100 ~ 300
	low=50.0; // out of limit
	high=200.0;
	exp = 1; //NOK
	result = FREQ_setJumpFreqRange(0, low, high);
	TEST_ASSERT_EQUAL_INT(result, exp);

	low=170;
	high=310;// out of limit
	exp = 1; //NOK
	result = FREQ_setJumpFreqRange(0, low, high);
	TEST_ASSERT_EQUAL_INT(exp, result);

	low=170.0;
	high=180.0;
	exp = 1; //NOK
	result = FREQ_setJumpFreqRange(3, low, high); // out of index
	TEST_ASSERT_EQUAL_INT(exp, result);

	low=170.0;
	high=180.0;
	exp = 0; //OK
	result = FREQ_setJumpFreqRange(0, low, high);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(param.ctrl.jump[0].enable, 1);
	TEST_ASSERT_EQUAL_FLOAT(param.ctrl.jump[0].low, low);
	TEST_ASSERT_EQUAL_FLOAT(param.ctrl.jump[0].high, high);

	cur = 150.0;
	result = FREQ_setFreqValue(index, cur);
	value = 175.0;
	exp = 170; // low jump
	result = FREQ_getVarifiedFreq(cur, value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	cur = 190.0;
	result = FREQ_setFreqValue(index, cur);
	value = 175.0;
	exp = 180; // high jump
	result = FREQ_getVarifiedFreq(cur, value);
	TEST_ASSERT_EQUAL_INT(exp, result);

	//clear jump range, then OK
	result = FREQ_clearJumpFreq(0);
	value = 175.0;
	exp = 175; //no jump check
	result = FREQ_getVarifiedFreq(cur, value);
	TEST_ASSERT_EQUAL_INT(exp, result);


	// set multi-step frequency
	index=1;
	value=10.0;
	exp=0; // OK
	result = FREQ_setFreqValue(index, value);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(value, param.ctrl.value[index]);

	index=8; // out of array
	value=10.0;
	exp=1; // NOK
	result = FREQ_setFreqValue(index, value);
	TEST_ASSERT_EQUAL_INT(exp, result);
}

#endif
