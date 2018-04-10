/*
 * err_trip.c
 *
 *  Created on: 2017. 7. 27.
 *      Author: hrjung
 */
#include "stdint.h"

#include "uartstdio.h"
//#include "nv_param.h"
#include "inv_param.h"
#include "drive.h"
#include "state_func.h"



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

//static int err_index=0;
int ERR_setTripInfo(int code);

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * EXTERNS
 */
extern MOTOR_working_st m_status;

/*
 *  ======== local function ========
 */


/*
 *  ======== public function ========
 */

void ERR_setTripFlag(int cause)
{
	if(internal_status.trip_happened != cause) // avoid duplicated
	{
		internal_status.trip_happened = cause;
		ERR_setTripInfo(cause);
	}
}

#if 0
int ERR_setTripFlagAtInitNV(int cause)
{
	internal_status.trip_happened = cause;
	ERR_setTripInfo(cause);

	return 0;
}
#endif

int ERR_clearTripData(void)
{
	dev_param.err_info.code = 0;
	dev_param.err_info.freq = 0;
	dev_param.err_info.current = 0;
	dev_param.err_info.op_mode = 0;

	return 0;
}

int ERR_setTripInfo(int code)
{
	dev_param.err_info.code = code;
	dev_param.err_info.freq = m_status.cur_freq;
	dev_param.err_info.current = m_status.current;
	dev_param.err_info.op_mode = m_status.status;

	return 0;
}

int ERR_getCurrentErrCode(void)
{
	return dev_param.err_info.code;
}
