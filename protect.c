/*
 * protect.c
 *
 *  Created on: 2017. 4. 17.
 *      Author: hrjung
 */
#include <stdint.h>
#include <math.h>

#include "uartstdio.h"

#include "inv_param.h"
#include "drive.h"
#include "timer_handler.h"
#include "protect.h"
#include "err_trip.h"

/*******************************************************************************
 * MACROS
 */

#define MAIN_isInitRelayInitialized()   (internal_status.relay_enabled==1)

/*******************************************************************************
 * CONSTANTS
 */
#define ETH_ONE_MINUTE		60

#define TRIP_LEVEL_MIN		30
#define TRIP_LEVEL_WARN		200
#define TRIP_LEVEL_TRIP		200

#define TRIP_TIME_WARN_MAX	30
#define TRIP_TIME_TRIP_MAX	60

#define MOTOR_TEMPERATURE_LIMIT	0x3F00

#define REGEN_RESISTANCE_VALUE_MIN	(150.0)
#define REGEN_RESISTANCE_VALUE_MAX	(500.0)

/*******************************************************************************
 * TYPEDEFS
 */
// 220V
#define DC_VOLTAGE_INIT_RELAY_OFF_220V		(100.0)
#define DC_VOLTAGE_INIT_RELAY_ON_220V		(186.0)
#define DC_VOLTAGE_OVER_TRIP_LEVEL_220V		(404.5)

#define DC_VOLTAGE_START_REGEN_LEVEL_220V 	(373.4)
#define DC_VOLTAGE_END_REGEN_LEVEL_220V		(357.8)

// 380V
#define DC_VOLTAGE_INIT_RELAY_OFF_380V		(100.0)
#define DC_VOLTAGE_INIT_RELAY_ON_380V		(322.4)
#define DC_VOLTAGE_OVER_TRIP_LEVEL_380V		(740.6) //(698.6)

//#define DC_VOLTAGE_START_REGEN_LEVEL_380V 	(550.9) // for test only
//#define DC_VOLTAGE_END_REGEN_LEVEL_380V 	(535.0)
#define DC_VOLTAGE_START_REGEN_LEVEL_380V 	(630.0)//(644.9)
#define DC_VOLTAGE_END_REGEN_LEVEL_380V 	(618.0)

#define REGEN_V_MARGIN			(0.9)
#define MAX_REGEN_CURRENT		(10.0)

typedef struct
{
	//float_t dc_volt_init_relay_off;
	float_t dc_volt_init_relay_on;
	float_t dc_volt_over_trip_level;

	float_t dc_volt_start_regen_level;
	float_t dc_volt_end_regen_level;

} protect_dc_st;

/*******************************************************************************
 * LOCAL VARIABLES
 */

static int evt_flag=0;

#ifdef SUPPORT_REGEN_GPIO
static uint32_t regen_duty=50;
static uint32_t regen_start=0;
extern uint32_t regen_timer;
#endif

static int regen_duty=0;

protect_dc_st protect_dc;

//static float_t overload_warn_level, overload_trip_level;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */




/*******************************************************************************
 * GLOBAL VARIABLES
 */


/*******************************************************************************
 * EXTERNS
 */
extern uint32_t secCnt;


extern bool UTIL_readOverTemperatureWarning(void);
extern bool UTIL_readOverTemperatureFault(void);

extern float_t MAIN_getIave(void);
extern void MAIN_readCurrent(void);
extern int MAIN_isOverCurrent(void);

/*
 *  ======== local function ========
 */


/*
 *  ======== public function ========
 */

int OVL_isOverloadTripEnabled(void)
{
	return (param.protect.ovl.enable == 1);
}

void OVL_enbleOverloadTrip(int enable)
{
	param.protect.ovl.enable = enable;

	//EEP_updateItem(OVL_ENABLE_ADDR, (unsigned char *)&param.protect.ovl.enable);
}

int OVL_setWarningLevel(int level)
{
	if(level < TRIP_LEVEL_MIN || level > TRIP_LEVEL_WARN) return 1;

	param.protect.ovl.wr_limit = level;

	dev_const.warn_level = mtr.max_current*(float_t)param.protect.ovl.wr_limit/100.0;

	//return EEP_updateItem(OVL_WARN_LIMIT_ADDR, (unsigned char *)&param.protect.ovl.wr_limit);
	return 0;
}

int OVL_setTripLevel(int level)
{
	if(level < TRIP_LEVEL_MIN || level > TRIP_LEVEL_TRIP) return 1;

	param.protect.ovl.tr_limit = level;
	dev_const.trip_level = mtr.max_current*(float_t)param.protect.ovl.tr_limit/100.0;

	//return EEP_updateItem(OVL_TR_LIMIT_ADDR, (unsigned char *)&param.protect.ovl.tr_limit);
	return 0;
}

int OVL_setWarningTime(int dur)
{
	if(dur < 0 || dur > TRIP_TIME_WARN_MAX) return 1;

	param.protect.ovl.wr_duration = dur;

	//return EEP_updateItem(OVL_WR_DURATION_ADDR, (unsigned char *)&param.protect.ovl.wr_duration);
	return 0;
}

int OVL_setTripTime(int dur)
{
	if(dur < 0 || dur > TRIP_TIME_TRIP_MAX) return 1;

	param.protect.ovl.tr_duration = dur;

	//return EEP_updateItem(OVL_TR_TIME_ADDR, (unsigned char *)&param.protect.ovl.tr_duration);
	return 0;
}


int OVL_isOverloadWarnCondition(int cur_level)
{
	return (cur_level > dev_const.warn_level);
}

int OVL_processOverloadWarning(int cur_val)
{
	//TODO : need to define working scenario


	return 0;
}


int OVL_isOverloadTripCondition(float_t cur_level)
{
	return (cur_level > dev_const.trip_level);
}

int OVL_processOverloadTrip(float_t cur_val)
{
	//uint32_t st_time;

	if(!OVL_isOverloadTripEnabled()) return 0;

	// start condition ?
	if(OVL_isOverloadTripCondition(cur_val))
	{
		if(!TMR_isTimerEnabled(OVERLOAD_TRIP_TSIG))
		{
			//start timer
			TMR_startTimerSig(OVERLOAD_TRIP_TSIG, param.protect.ovl.tr_duration);
			UARTprintf("OVL trip started %f at %d\n", cur_val, (int)(secCnt/10));
		}
	}

	// condition resolved before timeout
	if(TMR_isTimerEnabled(OVERLOAD_TRIP_TSIG))
	{
		if(!OVL_isOverloadTripCondition(cur_val)) // condition resolved
		{
			TMR_disableTimerSig(OVERLOAD_TRIP_TSIG);
			UARTprintf("OVL trip resolved %f\n", cur_val);
		}
	}

	// timeout ?
	if(TMR_isTimeout(OVERLOAD_TRIP_TSIG))
	{
		TMR_disableTimerSig(OVERLOAD_TRIP_TSIG);
		return 1;
	}

	return 0;
}

int OVL_isOverCurrentCondition(float_t cur_level)
{
	return (cur_level > dev_const.ovc_level);
}

int OVL_processOverCurrentTrip(float_t cur_val)
{
	//uint32_t st_time;

	// start condition ?
	if(OVL_isOverCurrentCondition(cur_val))
	{
		if(!TMR_isTimerEnabled(OVERLOAD_OVC_TSIG))
		{
			//start timer
			TMR_startTimerSig(OVERLOAD_OVC_TSIG, OVER_CURRENT_TIMEOUT); // 1 sec
			UARTprintf("OV current trip started %f at %d\n", cur_val, (int)(secCnt/10));
		}
	}

	// condition resolved before timeout
	if(TMR_isTimerEnabled(OVERLOAD_OVC_TSIG))
	{
		if(!OVL_isOverCurrentCondition(cur_val)) // condition resolved
		{
			TMR_disableTimerSig(OVERLOAD_OVC_TSIG);
			UARTprintf("OV current trip resolved %f\n", cur_val);
		}
	}

	// timeout ?
	if(TMR_isTimeout(OVERLOAD_OVC_TSIG))
	{
		TMR_disableTimerSig(OVERLOAD_OVC_TSIG);
		return 1;
	}

	return 0;
}

// valid resistance 150 ~ 500 ohm
int REGEN_setRegenResistence(float_t resist)
{
	if(resist < REGEN_RESISTANCE_VALUE_MIN || resist > REGEN_RESISTANCE_VALUE_MAX) return 1;

	param.protect.regen.resistance = resist;

	//return EEP_updateRegenResistItem();
	return 0;
}

int REGEN_setRegenResistencePower(uint16_t power)
{

	if(power < 10) return 1;

	param.protect.regen.power = power;

	//return EEP_updateRegenResistItem();
	return 0;
}

int REGEN_setRegenThermal(float_t value)
{
	param.protect.regen.thermal = value;

	//return EEP_updateItem(REGEN_THERMAL_ADDR, (unsigned char *)&param.protect.regen.thermal);
	return 0;
}

int REGEN_setRegenVoltReduction(uint16_t value)
{
	if(value > 150) return 1;

	param.protect.regen.band = value;

	//return EEP_updateItem(REGEN_BAND_ADDR, (unsigned char *)&param.protect.regen.band);
	return 0;
}

int REGEN_getDuty(void)
{
	return regen_duty;
}

int REGEN_setRegenDuty(float_t dc_value)
{
	//int i;
	float_t exp_duty; //exp_I, exp_W

	exp_duty = 0.9*dev_const.regen_max_V/dc_value;
	return (int)(exp_duty*100.0); // rounded integer
}

int REGEN_isEnabled(void)
{
	return internal_status.regen_enabled;
}

#ifdef SUPPORT_REGEN_GPIO
int32_t regen_diff=0;
int REGEN_toggleRegenBit(void)
{
	regen_diff = regen_timer - regen_start;

	if((regen_diff > 0) && (regen_diff < regen_duty))
	{
		UTIL_setRegenBit();
	}
	else if((regen_diff >= regen_duty) && (regen_diff < 100))
	{
		UTIL_clearRegenBit();
	}
	else //if(regen_diff >= 100)
	{
		regen_start = regen_timer;
		UTIL_setRegenBit();
	}

	return 0;
}
#endif

void REGEN_active(float_t dc_value)
{
	//int regen_duty;

	internal_status.regen_enabled = 1;
#ifdef SUPPORT_REGEN_GPIO
	regen_start=regen_timer; // start regen PWM output
	UTIL_setRegenBit();
#else
	regen_duty = REGEN_setRegenDuty(dc_value);
	UTIL_setRegenPwmDuty(regen_duty);
#endif
}

void REGEN_end(void)
{
	internal_status.regen_enabled = 0;
	//regen_duty = 0;
#ifdef SUPPORT_REGEN_GPIO
	UTIL_clearRegenBit(); // stop regen PWM output
#else
	UTIL_setRegenPwmDuty(0);
#endif
}

int REGEN_process(float_t dc_volt)
{
	static int under_flag=0, over_flag=0, regen_flag=0; //, off_flag=0;

	if(!MAIN_isInitRelayInitialized())
	{
		if(dc_volt > protect_dc.dc_volt_init_relay_on)
		{
//			UTIL_setInitRelay();
//			UARTprintf("Relay on at Vdc %f\n", dc_volt);
			under_flag=0; //initialize
			//off_flag=0; //initialize
		}
	}
	else
	{
		if(dc_volt < protect_dc.dc_volt_init_relay_on - 20.0) //&& dc_volt > protect_dc.dc_volt_init_relay_off)
		{
			//TODO : DC under voltage trip
			if(under_flag==0)
			{
				UARTprintf("DC under voltage %f trip event happened at %d\n", dc_volt, (int)(secCnt/10));
				under_flag=1;
			}
			ERR_setTripFlag(TRIP_REASON_VDC_UNDER);
			return 1; // disable PWM after return
		}
#if 0 // remove relay off
		if(dc_volt < protect_dc.dc_volt_init_relay_off) // force to disable relay
		{
			if(off_flag == 0)
			{
				UTIL_clearInitRelay();
				UARTprintf("Relay off at Vdc %f\n", dc_volt);
				off_flag=1;
				under_flag=0; //initialize
			}
		}
#endif
	}

	if(dc_volt > protect_dc.dc_volt_over_trip_level)
	{
		//TODO : DC over voltage trip
		if(over_flag==0)
		{
			UARTprintf("DC over voltage %dV trip event happened at %d\n", dc_volt, (int)(secCnt/10));
			over_flag=1;
		}
		ERR_setTripFlag(TRIP_REASON_VDC_OVER);
		return 1; // disable PWM after return
	}

#if 0
	if(REGEN_isEnabled())
	{
		REGEN_toggleRegenBit();
	}
#else
	if(dc_volt > protect_dc.dc_volt_start_regen_level) //dev_const.regen_limit + param.protect.regen.band)
	{
		if(regen_flag==0)
		{
			UARTprintf("REGEN_start() DC=%f\n", dc_volt);
			regen_flag++;
		}
		REGEN_active(dc_volt);
	}


	if(REGEN_isEnabled())
	{
		if(dc_volt < protect_dc.dc_volt_end_regen_level) //dev_const.regen_limit)
		{
			UARTprintf("REGEN_end() DC=%f\n", dc_volt);
			REGEN_end();
			regen_flag=0;
#ifdef SUPPORT_REGEN_GPIO
		else
			REGEN_toggleRegenBit();
#endif
		}
	}
#endif

	return 0;
}

bool ovTempWarn = 0;
bool ovTempFault = 0;
//bool ipmFault = 0;

int OSC_setDampImpact(int value)
{
	param.osc_damp.impact = value;

	//return EEP_updateItem(OSC_DAMP_IMPACT_ADDR, (unsigned char *)&param.osc_damp.impact);
	return 0;
}

int OSC_setDampFilter(int value)
{
	param.osc_damp.filter = value;

	//return EEP_updateItem(OSC_DAMP_FILTER_ADDR, (unsigned char *)&param.osc_damp.filter);
	return 0;
}

int TEMP_monitorTemperature(void)
{
	static int temp_warn_logged=0, mtr_warn_logged=0;
	uint16_t motor_temp;

	ovTempWarn = UTIL_readOverTemperatureWarning();
	if(ovTempWarn)
	{
		if(temp_warn_logged == 0) // one shot
		{
			// TODO : write warning error at log
			temp_warn_logged = 1;
			UARTprintf(" Temp Warning ! warn=%d fault=%d\n", ovTempWarn, ovTempFault);
		}
		ovTempFault = UTIL_readOverTemperatureFault();
		if(ovTempFault)
		{
			// TODO : write warning error at log, stop inverter
			//
			ERR_setTripFlag(TRIP_REASON_OVER_TEMP_PWR);
			UARTprintf(" Temp Fault ! warn=%d fault=%d\n", ovTempWarn, ovTempFault);
			return 1;
		}
	}
	else
	{
		temp_warn_logged = 0;
	}

#if 0
	ADC_readMotorTemperature(&motor_temp);
	if(motor_temp > MOTOR_TEMPERATURE_LIMIT)
	{
		if(mtr_warn_logged) // one shot
		{
			UARTprintf(" Motor Temp Fault ! temp=%d\n", motor_temp);
			mtr_warn_logged = 1;
		}
		return 1;
	}
	else
		mtr_warn_logged = 0;
#endif

	return 0;
}

void PROT_init(int input)
{
	if(input == 220)
	{
		//protect_dc.dc_volt_init_relay_off = DC_VOLTAGE_INIT_RELAY_OFF_220V;
		protect_dc.dc_volt_init_relay_on = DC_VOLTAGE_INIT_RELAY_ON_220V;
		protect_dc.dc_volt_over_trip_level = DC_VOLTAGE_OVER_TRIP_LEVEL_220V;

		protect_dc.dc_volt_start_regen_level = DC_VOLTAGE_START_REGEN_LEVEL_220V;
		protect_dc.dc_volt_end_regen_level = DC_VOLTAGE_END_REGEN_LEVEL_220V;
	}
	else if(input == 380)
	{
		//protect_dc.dc_volt_init_relay_off = DC_VOLTAGE_INIT_RELAY_OFF_380V;
		protect_dc.dc_volt_init_relay_on = DC_VOLTAGE_INIT_RELAY_ON_380V;
		protect_dc.dc_volt_over_trip_level = DC_VOLTAGE_OVER_TRIP_LEVEL_380V;

		protect_dc.dc_volt_start_regen_level = DC_VOLTAGE_START_REGEN_LEVEL_380V;
		protect_dc.dc_volt_end_regen_level = DC_VOLTAGE_END_REGEN_LEVEL_380V;
	}
	else
	{
		ERR_setTripFlag(TRIP_REASON_INPUT_VOLT_ERR);
	}

}

int processProtection(void)
{
#if 1
	float_t I_rms;

	// one shot
	if(MAIN_isSystemEnabled())
	{
		I_rms = MAIN_getIave();
		if(OVL_processOverloadTrip(I_rms))
		{
			// overload trip enabled
			// trip signal generate
			ERR_setTripFlag(TRIP_REASON_OVERLOAD);
			// set trip error code
			// disable PWM output
			MAIN_disableSystem();

			evt_flag++;
			if(evt_flag == 1)
				UARTprintf("OVL trip event happened at %d\n", (int)(secCnt/10));
		}

		if(OVL_processOverCurrentTrip(I_rms))
		{
			ERR_setTripFlag(TRIP_REASON_OVER_CURRENT);
			MAIN_disableSystem();
			evt_flag++;
			if(evt_flag == 1)
				UARTprintf("OV Current trip event happened at %d\n", (int)(secCnt/10));
		}
	}
#endif

	//if(MAIN_isOverCurrent()) MAIN_disableSystem();

	if( REGEN_process(MAIN_getVdcBus()) ) MAIN_disableSystem();

	//TEMP_monitorTemperature(); // Power module temperature check


	return 0;
}

