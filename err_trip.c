/*
 * err_trip.c
 *
 *  Created on: 2017. 7. 27.
 *      Author: hrjung
 */
#include "stdint.h"

#include "uartstdio.h"
//#include "nv_param.h"
#include "parameters.h"
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
		PARAM_setErrInfo(cause, m_status.status, m_status.current, m_status.cur_freq);
	}
}

