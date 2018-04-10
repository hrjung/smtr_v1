/*
 * def.h
 *
 *	used as predefined header
 *	including feature definition of global usage
 *
 *  Created on: 2017. 4. 7.
 *      Author: hrjung
 */

#ifndef DEF_H_
#define DEF_H_


/*******************************************************************************
 * CONSTANTS
 */

#define UART_BUFFERED

//timer interrupt : period 1ms
#define SUPPORT_TIMER0_INTERRUPT

// 2KB added
#define SUPPORT_VF_CONTROL

#define SUPPORT_FIELD_WEAKENING

//#define SUPPORT_USER_VARIABLE

#define SUPPORT_JUMP_FREQ

// measure Irms for checking overload
#define SUPPORT_I_RMS_MEASURE

//#define MODBUS_DEBUG_PRINT

//#define SUPPORT_FLYING_START

// enable for unit test only
//#define UNIT_TEST_ENABLED

// for using CCS graph debug
//#define SUPPORT_DEBUG_GRAPH

// enable/disable terminal debug command at compile time
#ifdef UNIT_TEST_ENABLED
#undef SUPPORT_DEBUG_TERMINAL
#undef SAMPLE_ADC_VALUE
#define STATIC
#else
#define SUPPORT_DEBUG_TERMINAL
#define STATIC static
// get ADC value to analyze I_u, I_v, V_u, V_v, V_w
//#define SAMPLE_ADC_VALUE
//#define PWM_DUTY_TEST
#endif


// motor setting, only one setting available
//#define TEST_MOTOR
#define SAMYANG_1_5K_MOTOR
//#define SAMYANG_2_2K_MOTOR

//#define SUPPORT_V0_HW
#define SUPPORT_V08_HW


#ifdef SUPPORT_V0_HW
// external GPIO interrupt for IPM fault signal
#define IPM_DEFINE
//
//#define I2C_DEFINE
////
#define SPI_DEFINE

#define BOOT_DEFINE

#define SUPPORT_V0_HW_PWM //fix polarity of PWM output

#define SUPPORT_V0_HW_USER_PWM // added from HAL tutorial 6.4
//#define SUPPORT_REGEN_GPIO

#endif

// ByPark add for print float value
#define FLOAT_PRINT



/*
 * 		basic parameter definition
 */

#define MAX_MULTI_FUNC_DIN_NUM	5
#define MAX_MULTI_FUNC_DOUT_NUM	2

//#define MAX_SPEED_STEP_NUM	8
#define MAX_FREQ_STEP_NUM	8
//#define MAX_TIME_STEP_NUM	1
//#define MAX_TIME_STEP_NUM	8

#define FAULT_HISTORY_NUM	5
#define MAX_JUMP_FREQ_NUM	3

#endif /* DEF_H_ */