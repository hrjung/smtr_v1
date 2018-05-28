/*
 * timer_handler.c
 *
 *  Created on: 2017. 5. 16.
 *      Author: hrjung
 */

#include <stdint.h>

#include "hal.h"

#include "uartstdio.h"

#include "inv_param.h"
#include "drive.h"
#include "state_func.h"
#include "timer_handler.h"

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
	int 		enable;
	uint32_t	st_time;
	uint32_t	duration;
	int			timeout_flag;
} timer_handler_st;

/*******************************************************************************
 * LOCAL VARIABLES
 */

uint32_t on_time=0;
uint32_t run_time=0;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */

uint32_t gTimerCount = 0;
uint32_t secCnt = 0;

#ifdef SUPPORT_REGEN_GPIO
uint32_t regen_timer = 0;
#endif

timer_handler_st time_sig[MAX_TIMER_TSIG];
/*******************************************************************************
 * EXTERNS
 */
extern HAL_Handle halHandle;

extern int FREQ_setFreqValue(float_t value);
/*
 *  ======== local function ========
 */



/*
 *  ======== public function ========
 */

void TMR_clearRunTime(void)
{
	run_time = 0;
}

void TMR_clearOnTime(void)
{
	on_time = 0;
}

uint32_t TMR_getRunTime(void)
{
	return run_time;
}

uint32_t TMR_getOnTime(void)
{
	return on_time;
}

int TMR_isTimerEnabled(int type)
{
	return time_sig[type].enable;
}

void TMR_disableTimerSig(int type)
{
	time_sig[type].enable = 0;
	time_sig[type].duration = 0;
	time_sig[type].st_time = 0;
	time_sig[type].timeout_flag = 0;
}

// unit of duration is 100ms
uint32_t TMR_startTimerSig(int type, float_t f_duration)
{
	if(TMR_isTimerEnabled(type)) return 0;

	time_sig[type].enable = 1;
	time_sig[type].duration = (uint32_t)(f_duration*10.0); // convert to 100ms unit
	time_sig[type].st_time = secCnt;
	time_sig[type].timeout_flag = 0;

	return secCnt;
}

void TMR_init(void)
{
	int i;

	on_time = 0;
	run_time = 0;

	gTimerCount=0;
	secCnt=0;
	for(i=0; i<MAX_TIMER_TSIG; i++)
	{
		TMR_disableTimerSig(i);
	}
}

int TMR_isTimeout(int type)
{
	return (time_sig[type].enable && time_sig[type].timeout_flag);
}

int TMR_isTimeOutCondition(int type)
{
	return (time_sig[type].duration <=  secCnt - time_sig[type].st_time) && (time_sig[type].timeout_flag == 0);
}

//hrjung add for timer0 interrupt 1ms period
//uint16_t adc_data;
//extern int ADC_readCurrentControl(uint16_t *value);
interrupt void timer0ISR(void)
{

	//UTIL_testbit(1);
	// toggle status LED
	gTimerCount++;
	if(gTimerCount%100 == 0) secCnt++; // 100ms

#if 0
	if(gTimerCount%1000 == 0)
	{
#ifdef SUPPORT_HW_COMMON
		HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);
#else
		HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED3);
#endif
//    	ADC_readCurrentControl(&adc_data);
    	//UARTprintf(" %d, %d\n", (int)gTimerCount, (int)secCnt);
	}
#endif

	// acknowledge the Timer 0 interrupt
	HAL_acqTimer0Int(halHandle);

#ifdef SUPPORT_REGEN_GPIO
	{
		regen_timer = gTimerCount; // 10Hz period, minimum pulse width is 100us
	}
#endif

#if 0 // only for test without debug connection
	if(internal_status.relay_enabled)
	{
		static uint32_t test_start=0, test_duration=0;
		static int freq_set=0, test_end=0;

		// set start time
		if(test_start == 0) test_start = secCnt;

		// wait 3 sec
		if(secCnt - test_start > 30)
		{
			if(freq_set == 0)
			{
				//set frequency
				FREQ_setFreqValue(25.0);
				// start
				MAIN_enableSystem(0);
				STA_calcResolution();
				freq_set = 1;
				test_duration = secCnt;
				UARTprintf("set freq 20Hz at %f\n", (float_t)(secCnt/10.0));
			}

			// run 10s including acceleration
			if(freq_set == 1 && (secCnt - test_duration > 100))
			{
				// stop motor
				if(test_end == 0)
				{
					STA_setStopCondition();
					test_end = 1;
					UARTprintf("stop at %f\n", (float_t)(secCnt/10.0));
				}
			}
		}
	}
#endif

	if(time_sig[DCI_BRAKE_SIG_ON_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(DCI_BRAKE_SIG_ON_TSIG))
		{
			time_sig[DCI_BRAKE_SIG_ON_TSIG].timeout_flag = 1;
			//UARTprintf(" DCI_BRAKE_SIG_ON timeout at %d \n", (int)secCnt);

		}
	}

	if(time_sig[DCI_BRAKE_SIG_OFF_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(DCI_BRAKE_SIG_OFF_TSIG))
		{
			time_sig[DCI_BRAKE_SIG_OFF_TSIG].timeout_flag = 1;
			//UARTprintf(" DCI_BRAKE_SIG_OFF timeout at %d \n", (int)secCnt);

		}
	}

	if(time_sig[OVERLOAD_WARN_START_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(OVERLOAD_WARN_START_TSIG))
		{
			time_sig[OVERLOAD_WARN_START_TSIG].timeout_flag = 1;
			//MAIN_disableSystem();
		}
	}

	if(time_sig[OVERLOAD_TRIP_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(OVERLOAD_TRIP_TSIG))
		{
			time_sig[OVERLOAD_TRIP_TSIG].timeout_flag = 1;
			MAIN_disableSystem();
		}
	}

	if(time_sig[OVERLOAD_OVC_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(OVERLOAD_OVC_TSIG))
		{
			time_sig[OVERLOAD_OVC_TSIG].timeout_flag = 1;
			MAIN_disableSystem();
		}
	}

	if(time_sig[TIMER_TEST_TSIG].enable)
	{
		if(TMR_isTimeOutCondition(TIMER_TEST_TSIG))
		{
			time_sig[TIMER_TEST_TSIG].timeout_flag = 1;
			UARTprintf(" Test Timer timeout at %d \n", (int)secCnt);
			TMR_disableTimerSig(TIMER_TEST_TSIG);
		}
	}

	// record run-time


	// record inverter on-time

	//UTIL_testbit(0);

	return;
} // end of timer0ISR() function


