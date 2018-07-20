//###########################################################################
//
// FILE:   common_tools.c
//
// TITLE:
//
//###########################################################################

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <Assert.h>

#include "main.h"
#include "inv_param.h"
#include "parameters.h"
#include "common_tools.h"
#include "uartstdio.h"

//*****************************************************************************
//
//! \addtogroup
//! @{
//
//*****************************************************************************


inv_parameter_st iparam[INV_PARAM_INDEX_MAX];
inv_parameter_st err_info[ERR_CODE_MAX];
inv_parameter_st inv_status[INV_STATUS_MAX];

//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

void PARAM_init(void)
{
	iparam[FREQ_VALUE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[FREQ_VALUE_INDEX].value.f = 10.0;

	iparam[ACCEL_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[ACCEL_TIME_INDEX].value.f = 10.0;

	iparam[DECEL_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[DECEL_TIME_INDEX].value.f = 10.0;

	iparam[VF_FOC_SEL_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[VF_FOC_SEL_INDEX].value.l = VF_CONTROL;

	iparam[ENERGY_SAVE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[ENERGY_SAVE_INDEX].value.l = ESAVE_UNUSED;

	iparam[PWM_FREQ_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[PWM_FREQ_INDEX].value.l = PWM_4KHz;

	iparam[JUMP_ENABLE0_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE0_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW0_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW0_INDEX].value.f = 0.0;

	iparam[JUMP_HIGH0_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH0_INDEX].value.f = 0.0;

	iparam[JUMP_ENABLE1_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE1_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW1_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW1_INDEX].value.f = 0.0;

	iparam[JUMP_HIGH1_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH1_INDEX].value.f = 0.0;

	iparam[JUMP_ENABLE2_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE2_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW2_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW2_INDEX].value.f = 0.0;

	iparam[JUMP_HIGH2_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH2_INDEX].value.f = 0.0;

	iparam[V_BOOST_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[V_BOOST_INDEX].value.f = 0.0;

	iparam[FOC_TORQUE_LIMIT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[FOC_TORQUE_LIMIT_INDEX].value.f = 180.0;

	iparam[BRK_TYPE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[BRK_TYPE_INDEX].value.l = REDUCE_SPEED_BRAKE;

	iparam[BRK_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_FREQ_INDEX].value.f = 3.0;

	iparam[BRK_DCI_START_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_START_FREQ_INDEX].value.f = REDUCE_SPEED_BRAKE;

	iparam[BRK_DCI_BLOCK_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BLOCK_TIME_INDEX].value.f = 1.0;

	iparam[BRK_DCI_BRAKING_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BRAKING_TIME_INDEX].value.f = 5.0;

	iparam[BRK_DCI_BRAKING_RATE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BRAKING_RATE_INDEX].value.f = 50.0;

	iparam[OVL_WARN_LIMIT_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_WARN_LIMIT_INDEX].value.l = 150;

	iparam[OVL_WR_DURATION_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_WR_DURATION_INDEX].value.l = 10;

	iparam[OVL_ENABLE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_ENABLE_INDEX].value.l = 1; //use

	iparam[OVL_TR_LIMIT_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_TR_LIMIT_INDEX].value.l = 180;

	iparam[OVL_TR_DURATION_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_TR_DURATION_INDEX].value.l = 30;

	iparam[REGEN_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[REGEN_RESISTANCE_INDEX].value.f = 200.0;

	iparam[REGEN_THERMAL_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[REGEN_THERMAL_INDEX].value.f = 5.0;

	iparam[REGEN_POWER_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[REGEN_POWER_INDEX].value.l = 200;

	iparam[REGEN_BAND_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[REGEN_BAND_INDEX].value.l = 0;

	iparam[STATOR_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[STATOR_RESISTANCE_INDEX].value.f = 2.5;

	iparam[ROTATOR_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[ROTATOR_RESISTANCE_INDEX].value.f = 2.14568;

	iparam[INDUCTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[INDUCTANCE_INDEX].value.f = 0.013955;

	iparam[NOLOAD_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[NOLOAD_CURRENT_INDEX].value.f = 2.0;

	iparam[RATED_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[RATED_CURRENT_INDEX].value.f = 3.4;

	iparam[POLES_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[POLES_INDEX].value.l = 2;

	iparam[INPUT_VOLTAGE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[INPUT_VOLTAGE_INDEX].value.l = 380;

	iparam[RATED_FREQ_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[RATED_FREQ_INDEX].value.l = 60;

	// error parameter
	PARAM_initErrInfo();

	// inverter status
	PARAM_initInvStatus();

}


void PARAM_update(uint16_t index, uint16_t *buf)
{
	iparam[index].value.arr[0] = buf[0];
	iparam[index].value.arr[1] = buf[1];
}

uint16_t PARAM_getValue(uint16_t index, uint16_t *buf)
{
#if 1 // test only
	iparam[index].value.f = 3.14;
#endif
	buf[0] = iparam[index].value.arr[0];
	buf[1] = iparam[index].value.arr[1];

	return 2; // size
}

void PARAM_initErrInfo(void)
{
	err_info[ERR_CODE_INDEX].type = PARAMETER_TYPE_LONG;
	err_info[ERR_CODE_INDEX].value.l = 0;

	err_info[ERR_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	err_info[ERR_CURRENT_INDEX].value.f = 0.0;

	err_info[ERR_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	err_info[ERR_FREQ_INDEX].value.f = 0.0;
}

void PARAM_setErrInfo(uint16_t err_code, uint16_t err_status, float_t current, float_t freq)
{
	err_info[ERR_CODE_INDEX].value.arr[0] = err_code;
	err_info[ERR_CODE_INDEX].value.arr[1] = err_status;
	err_info[ERR_CURRENT_INDEX].value.f = current;
	err_info[ERR_FREQ_INDEX].value.f = freq;
}

uint16_t PARAM_getErrorInfo(uint16_t *buf)
{
#if 1 // test only
	err_info[ERR_CODE_INDEX].value.arr[0] = 0;
	err_info[ERR_CODE_INDEX].value.arr[1] = 1;
	err_info[ERR_CURRENT_INDEX].value.f = 10.5;
	err_info[ERR_FREQ_INDEX].value.f = 120.0;
#endif
	buf[0] = err_info[ERR_CODE_INDEX].value.arr[0];
	buf[1] = err_info[ERR_CODE_INDEX].value.arr[1];
	buf[2] = err_info[ERR_CURRENT_INDEX].value.arr[0];
	buf[3] = err_info[ERR_CURRENT_INDEX].value.arr[1];
	buf[4] = err_info[ERR_FREQ_INDEX].value.arr[0];
	buf[5] = err_info[ERR_FREQ_INDEX].value.arr[1];

	return (ERR_CODE_MAX*2);
}

void PARAM_initInvStatus(void)
{
	inv_status[INV_STATUS_INDEX].type = PARAMETER_TYPE_LONG;
	inv_status[INV_STATUS_INDEX].value.l = 0;

	inv_status[INV_I_RMS_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_I_RMS_INDEX].value.f = 0.0;

	inv_status[INV_RUN_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_RUN_FREQ_INDEX].value.f = 0.0;

	inv_status[INV_DC_VOLTAGE_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = 0.0;

	inv_status[INV_IPM_TEMP_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_IPM_TEMP_INDEX].value.f = 0.0;

	inv_status[INV_MOTOR_TEMP_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_MOTOR_TEMP_INDEX].value.f = 0.0;
}

void PARAM_setInvStatus(uint16_t run, uint16_t dir, float_t icurr, float_t freq, float_t vdc, float_t ipm_t, float_t mtr_t)
{
	inv_status[INV_STATUS_INDEX].value.arr[0] = run;
	inv_status[INV_STATUS_INDEX].value.arr[1] = dir;

	inv_status[INV_I_RMS_INDEX].value.f = icurr;
	inv_status[INV_RUN_FREQ_INDEX].value.f = freq;
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = vdc;
	inv_status[INV_IPM_TEMP_INDEX].value.f = ipm_t;
	inv_status[INV_MOTOR_TEMP_INDEX].value.f = mtr_t;
}

uint16_t PARAM_getInvStatus(uint16_t *buf)
{
#if 1 // test only
	inv_status[INV_STATUS_INDEX].value.arr[0] = 0;
	inv_status[INV_STATUS_INDEX].value.arr[1] = 1;
	inv_status[INV_I_RMS_INDEX].value.f = 2.1;
	inv_status[INV_RUN_FREQ_INDEX].value.f = 30.2;
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = 531.3;
	inv_status[INV_IPM_TEMP_INDEX].value.f = 63.4;
	inv_status[INV_MOTOR_TEMP_INDEX].value.f = 54.1;
#endif
	buf[0] = inv_status[INV_STATUS_INDEX].value.arr[0];
	buf[1] = inv_status[INV_STATUS_INDEX].value.arr[1];
	buf[2] = inv_status[INV_I_RMS_INDEX].value.arr[0];
	buf[3] = inv_status[INV_I_RMS_INDEX].value.arr[1];
	buf[4] = inv_status[INV_RUN_FREQ_INDEX].value.arr[0];
	buf[5] = inv_status[INV_RUN_FREQ_INDEX].value.arr[1];
	buf[6] = inv_status[INV_DC_VOLTAGE_INDEX].value.arr[0];
	buf[7] = inv_status[INV_DC_VOLTAGE_INDEX].value.arr[1];
	buf[8] = inv_status[INV_IPM_TEMP_INDEX].value.arr[0];
	buf[9] = inv_status[INV_IPM_TEMP_INDEX].value.arr[1];
	buf[10] = inv_status[INV_MOTOR_TEMP_INDEX].value.arr[0];
	buf[11] = inv_status[INV_MOTOR_TEMP_INDEX].value.arr[1];

	return (INV_STATUS_MAX*2);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


