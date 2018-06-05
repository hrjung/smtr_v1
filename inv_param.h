/*
 * inv_param.h
 *
 *  Created on: 2017. 3. 7.
 *      Author: hrjung
 */

#ifndef INV_PARAM_H_
#define INV_PARAM_H_

#include "stdint.h"


#define HW_VER_MAJ	0
#define HW_VER_MIN	8

#define DSP_FW_VER_MAJ	0
#define DSP_FW_VER_MIN	7

#define NOT_INITIALIZED		(0.0)

#define NOT_USED	0

// timer isr freq is 1kHz
#define TIMER0_ISR_FREQ_Hz	((float_t)1000.0)
//#define SPEED_RANGE_KRPM	(1.4)


// definition for trip reason
#define TRIP_REASON_NONE			0
#define	TRIP_REASON_IPM_FAULT		1
#define TRIP_REASON_VDC_UNDER		2
#define TRIP_REASON_VDC_OVER		3
#define TRIP_REASON_OVERLOAD		4
#define TRIP_REASON_IPM_OVER_TEMP	5
#define TRIP_REASON_MTR_OVER_TEMP	6
#define TRIP_REASON_USER_STOP		7
#define TRIP_REASON_EXTERNAL_TRIP	8
#define TRIP_REASON_OVER_CURRENT	9

#define TRIP_REASON_Iu_PHASE_MISS	10
#define TRIP_REASON_Iv_PHASE_MISS	11
#define TRIP_REASON_Iw_PHASE_MISS	12


#define TRIP_REASON_ARRAY_ERR		20
#define TRIP_REASON_ACCEL_ERR		21
#define TRIP_REASON_FREQ_RANGE_ERR	22
#define TRIP_REASON_RPM_RANGE_ERR	23
#define TRIP_REASON_NFC_CARD_ERR	24
#define TRIP_REASON_NV_DATA_ERR		25
#define TRIP_REASON_INPUT_VOLT_ERR	26
#define TRIP_REASON_REGEN_CALC_ERR	27


#define TRIP_REASON_MAX				30
////////////////////////////////////////////

#define MAX_JUMP_FREQ_NUM	3

#define OVER_CURRENT_INSTANT_VALUE	(10.0)
#define OVER_CURRENT_COUNT_LIMIT	10

#define MISSING_PHASE_RMS_VALUE		(0.05)
#define CURRENT_MISS_COUNT_LIMIT	5

#define I_RMS_SAMPLE_COUNT			12

// operation control
enum
{
	REVERSE = -1,
	STOP = 0,
	FORWARD = 1
};

// ctrl.vf_foc_sel
enum
{
	VF_CONTROL = 0,
	FOC_CONTROL,

};

// stop control : brake_method
enum
{
	REDUCE_SPEED_BRAKE,
	DC_INJECT_BRAKE,
	FREE_RUN_BRAKE,
	MAX_BRAKE
};


// PWM frequency setting : pwm_freq
enum
{
	PWM_4KHz,
	PWM_8KHz,
	PWM_12KHz,
	PWM_16KHz
};

// energy save
enum
{
	ESAVE_UNUSED,
	ESAVE_STARTUP,
	ESAVE_RUNNING,
	ESAVE_BOTH
};

typedef struct
{
	uint16_t	enable;
	float_t		low;		// float
	float_t 	high;		// float
} freq_jump_st;

typedef struct
{
	uint16_t 	code; //TODO : need to define error code list
	float_t freq;
	float_t current;
	uint16_t 	op_mode; // mtr_state_e
} trip_err_st;

typedef struct
{
	float_t		value;  // float
	float_t		accel_time;		// xx.x sec
	float_t		decel_time;		// xx.x sec
	freq_jump_st jump[MAX_JUMP_FREQ_NUM];

	uint16_t 		vf_foc_sel;
	float_t		foc_torque_limit; // 100 ~ 220% : 180.0
	float_t 	spd_P_gain; // 0 ~ 32767 : 1000
	float_t 	spd_I_gain; // 0 ~ 32767 : 100

	uint16_t 	energy_save; // evergy_save on/off
	float_t	v_boost;	// 0 ~ 100.0% value of voltage for boost

	uint16_t 	pwm_freq;  // cannot change in run-time

} drive_control_st;


typedef struct
{

	uint16_t 	method;		// decel, DC injection, free run
	float_t 	brake_freq;	// value of brake signal on

	float_t 	dci_start_freq;
	float_t 	dci_block_time;		// 0.1 sec
	float_t		dci_braking_rate;	// %
	float_t		dci_braking_time;	// 0.1 sec

} brake_st;

typedef struct
{
	uint16_t 	wr_limit;
	uint16_t 	wr_duration;

	uint16_t	enable;
	uint16_t	tr_limit;
	uint16_t 	tr_duration;

} overload_st;

typedef struct
{
	float_t 	resistance; // 150.0 ~ 500.0
	float_t		thermal;	// thermal rate : 0 ~ 6553.5kWs
	uint16_t 	power;		// power of regenerate resistance : 10 ~ 65535W
	uint16_t 	band;		// valid band width of regen over regen_limit
} regen_st;

typedef struct
{
	overload_st 	ovl;
	regen_st		regen;

} protection_st ;


// TODO : function not defined precisely yet
//typedef struct
//{
//	int		impact;	//suppress idle oscillation below % : 0 ~ 250
//	int 	filter; // filter time : 2 ~ 250ms
//} osc_damp_st;


typedef struct
{
	int 	inv; // for state_func
	int 	sel_in; // indicate which input source is used
	int 	run; // for motor running status : REVERSE, STOP, FORWARD
} inv_state_st;

// for speed calculation for VF
typedef struct
{
	uint16_t	enable;
	_iq			low;
	_iq 		high;
//	float_t 	low_pu;
//	float_t 	high_pu;
//	float_t 	low_spd;
//	float_t 	high_spd;
} spd_jump_st;

typedef struct
{

	uint16_t 	spd_rpm_min;
	uint16_t	spd_rpm_max;
	float_t 	regen_limit;	// regen stop voltage level
	float_t		warn_level;
	float_t		trip_level;
	float_t		ovc_level;
	float_t		regen_max_V;
	float_t 	dci_pwm_rate;

	spd_jump_st spd_jmp[MAX_JUMP_FREQ_NUM];

} dev_const_st;


typedef struct
{
	uint16_t 	poles;		//USER_MOTOR_NUM_POLE_PAIRS
	uint16_t 	effectiveness;
	uint16_t 	input_voltage;
	uint16_t 	rated_freq;

	float_t 	capacity;
	float_t 	slip_rate;
	float_t 	noload_current;
	float_t 	max_current;	//USER_MOTOR_MAX_CURRENT
	float_t 	Rr;		//USER_MOTOR_Rr
	float_t 	Rs;		//USER_MOTOR_Rs
	float_t 	Ls;		//USER_MOTOR_Ls_d
	//float_t 	magnetize_current;	//USER_MOTOR_MAGNETIZING_CURRENT
	//float_t 	rated_flux; //USER_MOTOR_RATED_FLUX

} motor_param_st;


typedef struct
{
	drive_control_st	ctrl;

	brake_st		brk;

	protection_st	protect;
	//osc_damp_st		osc_damp;

	uint16_t		gear_ratio;
	trip_err_st		err_info;

} inverter_param_st;


typedef struct
{
	float_t		out_curr;
	float_t 	out_freq;
	float_t		out_rpm;
	float_t		dc_voltage;
	float_t		mtr_temperature;

	uint32_t	err_code;

	uint32_t	mtr_onoff_cnt;
	uint32_t  	mtr_elapse_hour;	// motor total run time
	uint32_t  	operating_hour;	// inverter on time
} monitor_param_st;

typedef struct
{
	float_t 	Iu_inst;
	float_t 	Iv_inst;
	float_t 	Iw_inst;

	float_t 	Irms[3];

	uint16_t	Iu_miss_cnt;
	uint16_t	Iv_miss_cnt;
	uint16_t	Iw_miss_cnt;

	float_t		Vu_inst;
	float_t		Vv_inst;
	float_t		Vw_inst;

	float_t		Vab_pu[2];
	float_t 	Vdc_inst;
	float_t 	Vdc_lfp;
	float_t		phasor[2];
	float_t 	angle_pu;

	float_t		pwmData[3];

	float_t		accel_resol;
	float_t		decel_resol;

	uint16_t 	ipm_temp;
	uint16_t 	mtr_temp;

	uint16_t	relay_enabled;
	uint16_t	regen_enabled;
	uint16_t 	trip_happened;
	uint16_t	emergency_stop;
	uint16_t	external_trip;
	uint16_t 	shaft_brake_enabled;

	uint16_t 	oc_count; // over current count


} internal_status_st;


//extern dev_param_st	dev_param;
extern dev_const_st	dev_const;
extern motor_param_st mtr;
extern inverter_param_st param;
extern inv_state_st state_param;
extern internal_status_st internal_status;

extern float_t MAIN_convert2Freq(float_t spd_krpm);
extern void MAIN_setJumpSpeed(int index, float_t low, float_t high);

extern float_t MAIN_getVdcBus(void);
extern int MAIN_isSystemEnabled(void);
extern int MAIN_enableSystem(int index);
extern void MAIN_disableSystem(void);
extern int MAIN_isTripHappened(void);
extern int MAIN_setForwardDirection(void);
extern int MAIN_setReverseDirection(void);
extern int MAIN_getCurrentSpeed(void);
extern int MAIN_applyBoost(void);

extern void UTIL_setInitRelay(void);
extern void UTIL_clearInitRelay(void);
extern void UTIL_setShaftBrake(void);
extern void UTIL_releaseShaftBrake(void);
extern void UTIL_setScaleFactor(void);
extern uint16_t UTIL_setPwmDuty(int duty);
extern int UTIL_controlLed(int type, int on_off);
extern void UTIL_testbit(int on_off);
extern void UTIL_testbitG(int on_off);

extern uint16_t UTIL_setRegenPwmDuty(int duty);
extern float_t UTIL_readIpmTemperature(void);
extern float_t UTIL_readMotorTemperature(void);

#endif /* INV_PARAM_H_ */
