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
#include "../protect.h"

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



/*
 * test procedure
 *
 * 1. set drive control, check input range
 * 2. according to drive control, then block command from other control input
 * 3. set ban direction, check input range
 * 4. according to ban direction, ignore dir command
 * 5. set multi-func in, check input range
 * 6. for multi-step freq setting, process freq step input
 */
void test_controlDrive(void)
{
	int result=0;
	int exp=0;

	// set energy saving off
	exp = 0;
	result = DRV_setEnergySave(ESAVE_UNUSED);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(0, param.ctrl.energy_save);

	// set energy saving startup
	exp = 0;
	result = DRV_setEnergySave(ESAVE_STARTUP);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(1, param.ctrl.energy_save);

	// set energy saving wrong value
	exp = 1;
	result = DRV_setEnergySave(ESAVE_BOTH+1);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// set Vmin boost correct value
	exp = 0;
	result = DRV_setVoltageBoost(0.0);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(0, param.ctrl.v_boost);

	// set Vmin boost correct value
	exp = 0;
	result = DRV_setVoltageBoost(100.0);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_FLOAT(100.0, param.ctrl.v_boost);

	// set Vmin boost wrong value
	exp = 1;
	result = DRV_setVoltageBoost(100.1);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// set PWM freq correct value
	exp = 0;
	result = DRV_setPwmFrequency(PWM_4KHz);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(PWM_4KHz, param.ctrl.pwm_freq);

	// set PWM freq correct value
	exp = 0;
	result = DRV_setPwmFrequency(PWM_16KHz);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(PWM_16KHz, param.ctrl.pwm_freq);

	// set PWM freq wrong value
	exp = 1;
	result = DRV_setPwmFrequency(PWM_16KHz+1);
	TEST_ASSERT_EQUAL_INT(exp, result);

}

#endif
