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
typedef struct
{
	int bit_L;
	int bit_M;
	int bit_H;
	int freq_step; // ???
	int run_pin;
	int dir_pin;
	int emergency_pin;
	int trip_pin;
	int release_axis;

} multi_din_st;
/*******************************************************************************
 * LOCAL VARIABLES
 */
int run_status=STOP;
int dir_status=FORWARD;

extern int mdin_pin_value[MAX_MULTI_FUNC_DIN_NUM];

/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
extern multi_din_st m_din;
/*******************************************************************************
 * EXTERNS
 */
extern int DRV_convertMultiStep(int input);
extern void DRV_updateMultiDinPinIndex(int index, int func);
extern int DRV_storeMultiFuncDinValue(uint16_t data);
/*
 *  ======== function ========
 */
//int test_readMultiDinBit(int value)
//{
//	int res;
//
//	switch(value)
//	{
//	case 0 : res = 0; break;
//	case 1 : res = 0; break;
//	case 2 : res = 1; break; // bit_M
//	case 3 : res = 1; break; // bit_H
//	case 4 : res = 0; break; // bit_L
//	}
//
//	return res;
//}


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
	int result=0, index;
	int exp=0;

	internal_status.UIO_exist = 1;
	internal_status.FieldBus_exist = 1;


	// set normal value on drive control -> OK
	exp = 0;
	result = DRV_setControlIn(PMETER_IN_DRV);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(PMETER_IN_DRV, param.ctrl.ctrl_in);

	exp = 0;
	result = DRV_setControlIn(COMM_DRV);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(COMM_DRV, param.ctrl.ctrl_in);

	// set wrong value on drive control -> NOK
	exp = 1;
	result = DRV_setControlIn(COMM_DRV+1);
	TEST_ASSERT_EQUAL_INT(exp, result);


	// set normal value on direction ban -> OK
	exp = 0;
	result = DRV_setDirectionBan(ALL_DIR_OK);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(ALL_DIR_OK, param.ctrl.ban_dir);

	exp = 0;
	result = DRV_setDirectionBan(REVERSE_ONLY);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(REVERSE_ONLY, param.ctrl.ban_dir);

	// set wrong value -> NOK
	exp = 1;
	result = DRV_setDirectionBan(REVERSE_ONLY+1);
	TEST_ASSERT_EQUAL_INT(exp, result);

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


	// set normal value on multi-DIN -> OK
	index=0;
	exp = 0;
	result = DRV_setMultiFuncDin(index, DRIVE_CONTROL_RUN);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(DRIVE_CONTROL_RUN, param.ctrl.multi_din[index]);

	exp = 0;
	result = DRV_setMultiFuncDin(index, RELEASE_AXIS);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(RELEASE_AXIS, param.ctrl.multi_din[index]);

	// set normal value on multi-DIN max index -> OK
	index=MAX_MULTI_FUNC_DIN_NUM-1;
	exp = 0;
	result = DRV_setMultiFuncDin(index, DRIVE_CONTROL_RUN);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(DRIVE_CONTROL_RUN, param.ctrl.multi_din[index]);

	exp = 0;
	result = DRV_setMultiFuncDin(index, RELEASE_AXIS);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(RELEASE_AXIS, param.ctrl.multi_din[index]);

	// set wrong value -> NOK
	exp = 1;
	result = DRV_setMultiFuncDin(index, RELEASE_AXIS+1);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// set wrong index -> NOK
	index=MAX_MULTI_FUNC_DIN_NUM;
	exp = 1;
	result = DRV_setMultiFuncDin(index, RELEASE_AXIS);
	TEST_ASSERT_EQUAL_INT(exp, result);


	// set normal value on multi-DOUT -> OK
	index=0;
	exp = 0;
	result = DRV_setMultiFuncDout(index, OVERLOAD_WARN_SIG);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(OVERLOAD_WARN_SIG, param.ctrl.multi_dout[index]);

	exp = 0;
	result = DRV_setMultiFuncDout(index, CONSTANT_RUN_SIG);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(CONSTANT_RUN_SIG, param.ctrl.multi_dout[index]);

	index=MAX_MULTI_FUNC_DOUT_NUM-1;
	exp = 0;
	result = DRV_setMultiFuncDout(index, OVERLOAD_WARN_SIG);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(OVERLOAD_WARN_SIG, param.ctrl.multi_dout[index]);

	exp = 0;
	result = DRV_setMultiFuncDout(index, CONSTANT_RUN_SIG);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(CONSTANT_RUN_SIG, param.ctrl.multi_dout[index]);

	// set wrong value -> NOK
	exp = 1;
	result = DRV_setMultiFuncDout(index, CONSTANT_RUN_SIG+1);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// set wrong index -> NOK
	index=MAX_MULTI_FUNC_DOUT_NUM;
	exp = 1;
	result = DRV_setMultiFuncDout(index, CONSTANT_RUN_SIG);
	TEST_ASSERT_EQUAL_INT(exp, result);

	internal_status.UIO_exist = 0;
	internal_status.FieldBus_exist = 0;
}

/*
 * test procedure
 *
 * 1. set DIN bit as multi-step bit, check setting
 * 2. check converted step index
 * 3. change DIN bit as other than multi-step
 * 4. check converted step index as NOK
 */
void test_setMultiStepDin(void)
{
	int result=0, index;
	int exp=0;

	// multi-step input for multi-func Din, set M bit
	// set bit 4,5,6 as multi-step freq
	index=2;
	exp = 0;
	result = DRV_setMultiFuncDin(index, MULTI_STEP_FREQ_M);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(MULTI_STEP_FREQ_M, param.ctrl.multi_din[index]);
	TEST_ASSERT_EQUAL_INT(index, m_din.bit_M);

	// set H bit
	index=3;
	exp = 0;
	result = DRV_setMultiFuncDin(index, MULTI_STEP_FREQ_H);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(MULTI_STEP_FREQ_H, param.ctrl.multi_din[index]);
	TEST_ASSERT_EQUAL_INT(index, m_din.bit_H);

	// set L bit
	index=4;
	exp = 0;
	result = DRV_setMultiFuncDin(index, MULTI_STEP_FREQ_L);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(MULTI_STEP_FREQ_L, param.ctrl.multi_din[index]);
	TEST_ASSERT_EQUAL_INT(index, m_din.bit_L);

	// conversion bit pattern for multi-step bits input
	exp = 6; // step index
	result = DRV_convertMultiStep(0x0C);
	TEST_ASSERT_EQUAL_INT(exp, result); // convert result is step=6

	// if set other func at multi-step input bit, than clear internal flag
	index=4;
	exp = 0;
	result = DRV_setMultiFuncDin(index, EMERGENCY_STOP);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(EMERGENCY_STOP, param.ctrl.multi_din[index]);
	TEST_ASSERT_EQUAL_INT(MAX_MULTI_FUNC_DIN_NUM, m_din.bit_L);
}

void test_convertMultiStepDin(void)
{
	int result=0;
	int exp=0, input=0xFF;

	// invalid multi-bit setting, than conversion error
	// multi-step count 3
	param.ctrl.multi_din_cnt = 3;
	input = 0xFF;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = MAX_MULTI_FUNC_DIN_NUM;
	exp = 0; // step index
	result = DRV_convertMultiStep(input); // NOK -> return 0
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0, step = 0
	input = 0x0;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = 2;
	exp = 0;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0xF2, step = 2
	input = 0xF2;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = 2;
	exp = 2;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0xF5, step = 5
	input = 0xF5;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = 2;
	exp = 5;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0xFF, step = 7
	input = 0xFF;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = 2;
	exp = 7;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0x0101 0101, step = 6
	input = 0x55;
	m_din.bit_L = 1;
	m_din.bit_M = 0;
	m_din.bit_H = 2;
	exp = 6;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 4
	input = 0x55;
	m_din.bit_L = 1;
	m_din.bit_M = 3;
	m_din.bit_H = 2;
	exp = 4;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 5
	input = 0x55;
	m_din.bit_L = 2;
	m_din.bit_M = 3;
	m_din.bit_H = 0;
	exp = 5;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 5
	input = 0x55;
	m_din.bit_L = 4;
	m_din.bit_M = 3;
	m_din.bit_H = 2;
	exp = 5;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 7
	input = 0x55;
	m_din.bit_L = 4;
	m_din.bit_M = 2;
	m_din.bit_H = 0;
	exp = 7;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 1
	input = 0x55;
	m_din.bit_L = 4;
	m_din.bit_M = 1;
	m_din.bit_H = 3;
	exp = 1;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);


	// multi-step count = 2
	param.ctrl.multi_din_cnt = 2;
	input = 0xFF;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = MAX_MULTI_FUNC_DIN_NUM;
	exp = 3; // step index
	result = DRV_convertMultiStep(input); //
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0, step = 0
	input = 0x0;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = 2;
	exp = 0;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0xF2, step = 2
	input = 0xF2;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = 2;
	exp = 2;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0x0101 0101, step = 6
	input = 0x55;
	m_din.bit_L = 1;
	m_din.bit_M = 0;
	m_din.bit_H = 2;
	exp = 2;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 4
	input = 0x55;
	m_din.bit_L = 1;
	m_din.bit_M = 3;
	m_din.bit_H = 2;
	exp = 0;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 5
	input = 0x55;
	m_din.bit_L = 2;
	m_din.bit_M = 3;
	m_din.bit_H = 0;
	exp = 1;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 5
	input = 0x55;
	m_din.bit_L = 4;
	m_din.bit_M = 3;
	m_din.bit_H = 2;
	exp = 1;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 7
	input = 0x55;
	m_din.bit_L = 4;
	m_din.bit_M = 2;
	m_din.bit_H = 0;
	exp = 3;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 1
	input = 0x55;
	m_din.bit_L = 4;
	m_din.bit_M = 1;
	m_din.bit_H = 3;
	exp = 1;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);


	// multi-step count = 1
	param.ctrl.multi_din_cnt = 1;
	input = 0xFF;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = MAX_MULTI_FUNC_DIN_NUM;
	exp = 1; // step index
	result = DRV_convertMultiStep(input); //
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0, step = 0
	input = 0x0;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = 2;
	exp = 0;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0xF2, step = 2
	input = 0xF2;
	m_din.bit_L = 0;
	m_din.bit_M = 1;
	m_din.bit_H = 2;
	exp = 0;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// read DIN = 0x0101 0101, step = 0
	input = 0x55;
	m_din.bit_L = 1;
	m_din.bit_M = 0;
	m_din.bit_H = 2;
	exp = 0;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 1
	input = 0x55;
	m_din.bit_L = 2;
	m_din.bit_M = 3;
	m_din.bit_H = 0;
	exp = 1;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 1
	input = 0x55;
	m_din.bit_L = 4;
	m_din.bit_M = 3;
	m_din.bit_H = 2;
	exp = 1;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);

	// step = 1
	input = 0x55;
	m_din.bit_L = 4;
	m_din.bit_M = 1;
	m_din.bit_H = 3;
	exp = 1;
	result = DRV_convertMultiStep(input);
	TEST_ASSERT_EQUAL_INT(exp, result);
}


/*
 * test procedure
 *
 * 1. set func at pin index, normal -> OK
 * 2. set same func at other pin -> just change pin index -> OK
 * 3. set other func at same pin -> clear pin for prev func
 * 4. set 1st pin index and check other pin index
 * 5. set last pin index, -> OK
 * 6. change same func at other pin index -> just change index -> OK
 */
void test_setMultiDinPin(void)
{
	int index;

	// multi-function Din pin index
	index=4;
	DRV_updateMultiDinPinIndex(index, DRIVE_CONTROL_RUN);
	TEST_ASSERT_EQUAL_INT(index, m_din.run_pin);

	index=3;
	DRV_updateMultiDinPinIndex(index, DRIVE_CONTROL_RUN);
	TEST_ASSERT_EQUAL_INT(index, m_din.run_pin);

	//change other func, then clear pin index for prev func
	index=3;
	DRV_updateMultiDinPinIndex(index, EXTERNAL_TRIP);
	TEST_ASSERT_EQUAL_INT(MAX_MULTI_FUNC_DIN_NUM, m_din.run_pin); // clear prev setting
	TEST_ASSERT_EQUAL_INT(index, m_din.trip_pin);

	index=0;
	DRV_updateMultiDinPinIndex(index, DRIVE_CONTROL_DIR);
	TEST_ASSERT_EQUAL_INT(index, m_din.dir_pin);
	TEST_ASSERT_EQUAL_INT(3, m_din.trip_pin);

	// set last index
	index=7;
	DRV_updateMultiDinPinIndex(index, EMERGENCY_STOP);
	TEST_ASSERT_EQUAL_INT(index, m_din.emergency_pin);

	//change pin
	index=5;
	DRV_updateMultiDinPinIndex(index, EMERGENCY_STOP);
	TEST_ASSERT_EQUAL_INT(index, m_din.emergency_pin);
}

/*
 * test procedure
 *
 * 1. define Din pin assignment
 * 2. check reading Din correctly set to din_value

 */
void test_convertMultiDinPin(void)
{
	uint16_t din, i;

	DRV_setMultiFuncDin(0, DRIVE_CONTROL_RUN);
	DRV_setMultiFuncDin(1, DRIVE_CONTROL_DIR);
	DRV_setMultiFuncDin(2, MULTI_STEP_FREQ_L);
	DRV_setMultiFuncDin(3, MULTI_STEP_FREQ_M);
	DRV_setMultiFuncDin(4, MULTI_STEP_FREQ_H);
	for(i=0; i<MAX_MULTI_FUNC_DIN_NUM; i++) mdin_pin_value[i] = 0;

	din = 0x15;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.bit_L]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_M]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.bit_H]);

	din = 0x0A;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_L]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.bit_M]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_H]);

	din = 0x03;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_L]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_M]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_H]);

	din = 0x18;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_L]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.bit_M]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.bit_H]);

	din = 0x04;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.bit_L]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_M]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_H]);

	// other pin assign
	DRV_setMultiFuncDin(0, DRIVE_CONTROL_RUN);
	DRV_setMultiFuncDin(1, DRIVE_CONTROL_DIR);
	DRV_setMultiFuncDin(2, EMERGENCY_STOP);
	DRV_setMultiFuncDin(3, EXTERNAL_TRIP);
	DRV_setMultiFuncDin(4, RELEASE_AXIS);
	for(i=0; i<MAX_MULTI_FUNC_DIN_NUM; i++) mdin_pin_value[i] = 0;

	din = 0x0;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.emergency_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.trip_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.release_axis]);

	din = 0x01F;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.emergency_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.trip_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.release_axis]);

	din = 0x07;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.emergency_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.trip_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.release_axis]);

	din = 0x1C;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.emergency_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.trip_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.release_axis]);

	din = 0x1B;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.dir_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.emergency_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.trip_pin]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.release_axis]);

	// other pin assign
	DRV_setMultiFuncDin(0, MULTI_STEP_FREQ_H);
	DRV_setMultiFuncDin(1, RELEASE_AXIS);
	DRV_setMultiFuncDin(2, DIN_UNUSED);
	DRV_setMultiFuncDin(3, DRIVE_CONTROL_RUN);
	DRV_setMultiFuncDin(4, DIN_UNUSED);
	for(i=0; i<MAX_MULTI_FUNC_DIN_NUM; i++) mdin_pin_value[i] = 0;

	din = 0x0;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_H]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.release_axis]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[2]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[4]);

	din = 0x01F;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.bit_H]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.release_axis]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[2]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[4]);

	din = 0x07;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.bit_H]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.release_axis]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[2]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[4]);

	din = 0x18;
	DRV_storeMultiFuncDinValue(din);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.bit_H]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[m_din.release_axis]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[2]);
	TEST_ASSERT_EQUAL_INT(1, mdin_pin_value[m_din.run_pin]);
	TEST_ASSERT_EQUAL_INT(0, mdin_pin_value[4]);
}

/*
 * test procedure
 *
 * 1. set DIN bit as multi-step bit, check setting
 * 2. check converted step index
 * 3. change DIN bit as other than multi-step
 * 4. check converted step index as NOK
 */
void test_setMultiDoutState(void)
{
	int result=0, index;
	int exp=0;

	// multi-function Dout for state
	index=4;
	result = DRV_setMultiFuncDout(index, CONSTANT_RUN_SIG);
	exp = 0;
	//
	TEST_ASSERT_EQUAL_INT(exp, result);


}


/*
 * test procedure
 *
 * 1. set DIN bit as RUN, STOP bit, check setting
 * 2. set RUN, STOP bit then follow control which bit is 1
 * 3. if FX = RX then stop
 * 4. set RUN, DIR bit then follow
 * 5. check status for each bit pattern
 */
void test_setMultiControlDin(void)
{
	int result=0, run_index, stop_index;
	int exp=0, state_exp=0;
	int state;

	// initial setting for multi-function Din for control FX-RX
	exp = 0;
	result = DRV_setControlIn(RUN_DIR_DRV);
	TEST_ASSERT_EQUAL_INT(exp, result); // OK

	// fx, rx pin is not assigned -> NOK
	run_index=2; stop_index=3;
	mdin_pin_value[run_index] = 1;
	mdin_pin_value[stop_index] = 0;
	exp=1;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result); // NOK

	run_index=2;
	exp=0;
	result = DRV_setMultiFuncDin(run_index, DRIVE_CONTROL_RUN);
	TEST_ASSERT_EQUAL_INT(exp, result); // OK
	stop_index=3;
	exp=0;
	result = DRV_setMultiFuncDin(stop_index, DRIVE_CONTROL_DIR);
	TEST_ASSERT_EQUAL_INT(exp, result); // OK
	// varify assign other func at RX pin
	stop_index=3;
	exp=0;
	result = DRV_setMultiFuncDin(stop_index, EMERGENCY_STOP);
	TEST_ASSERT_EQUAL_INT(exp, result); // OK
	TEST_ASSERT_EQUAL_INT(EMERGENCY_STOP, param.ctrl.multi_din[stop_index]);
	TEST_ASSERT_EQUAL_INT(stop_index, m_din.emergency_pin);
	TEST_ASSERT_EQUAL_INT(MAX_MULTI_FUNC_DIN_NUM, m_din.dir_pin); // clear dir_pin
	// reassign for further test
	stop_index=3;
	result = DRV_setMultiFuncDin(stop_index, DRIVE_CONTROL_DIR);
	TEST_ASSERT_EQUAL_INT(exp, result); // OK

	/*
	// FX control, FX=1, RX=0
	mdin_pin_value[run_index] = 1;
	mdin_pin_value[stop_index] = 0;
	exp=0;
	state_exp = FORWARD;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(state_exp, state); // status FOR

	// FX control, FX=0, RX=1
	mdin_pin_value[run_index] = 0;
	mdin_pin_value[stop_index] = 1;
	exp=0;
	state_exp = REVERSE;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(state_exp, state); // status REV

	// FX control, FX=1, RX=1
	mdin_pin_value[run_index] = 1;
	mdin_pin_value[stop_index] = 1;
	exp=0;
	state_exp = STOP;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(state_exp, state); // status STOP

	// FX control, FX=0, RX=0
	mdin_pin_value[run_index] = 0;
	mdin_pin_value[stop_index] = 0;
	exp=0;
	state_exp = STOP;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(state_exp, state); // status STOP

	// control is not Din -> PC loader
	exp = 0;
	result = DRV_setControlIn(LOADER_NFC_DRV);
	TEST_ASSERT_EQUAL_INT(exp, result); // OK

	// normal fx, rx control, but control source is not fx, rx -> NOK
	mdin_pin_value[run_index] = 1;
	mdin_pin_value[stop_index] = 0;
	exp=1;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result); //NOK
*/

	// initial setting for multi-function Din for control RUN-DIR
	exp = 0;
	result = DRV_setControlIn(RUN_DIR_DRV);
	TEST_ASSERT_EQUAL_INT(exp, result); // OK
	run_index=1;
	result = DRV_setMultiFuncDin(run_index, DRIVE_CONTROL_RUN); //RUN
	TEST_ASSERT_EQUAL_INT(exp, result); // OK
	stop_index=2;
	result = DRV_setMultiFuncDin(stop_index, DRIVE_CONTROL_DIR); //DIR
	TEST_ASSERT_EQUAL_INT(exp, result); // OK

	// FX control, RUN=1, DIR=FOR
	mdin_pin_value[run_index] = 1;
	mdin_pin_value[stop_index] = 0;
	exp=0;
	state_exp = FORWARD;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(state_exp, state); // status FORWARD

	// FX control, RUN=1, DIR=REV
	mdin_pin_value[run_index] = 1;
	mdin_pin_value[stop_index] = 1;
	exp=0;
	state_exp = REVERSE;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(state_exp, state); // status REVERSE

	// FX control, RUN=0, DIR=REV
	mdin_pin_value[run_index] = 0;
	mdin_pin_value[stop_index] = 1;
	exp=0;
	state_exp = STOP;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(state_exp, state); // status STOP

	// FX control, RUN=0, DIR=FOR
	mdin_pin_value[run_index] = 0;
	mdin_pin_value[stop_index] = 0;
	exp=0;
	state_exp = STOP;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result);
	TEST_ASSERT_EQUAL_INT(state_exp, state); // status STOP

/*
	// control is not Din -> PC loader
	exp = 0;
	result = DRV_setControlIn(LOADER_NFC_DRV);
	TEST_ASSERT_EQUAL_INT(exp, result); // OK

	// normal fx, rx control, but control source is not fx, rx -> NOK
	mdin_pin_value[run_index] = 1;
	mdin_pin_value[stop_index] = 0;
	exp=1;
	result = DRV_decodeMultiFuncDinControl(&state);
	TEST_ASSERT_EQUAL_INT(exp, result); //NOK
*/
}

#endif
