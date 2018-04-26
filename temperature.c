/*
 * temperature.c
 *
 *  Created on: 2018. 4. 26.
 *      Author: hrjung
 */

#include <stdint.h>

#include "hal.h"

#include "uartstdio.h"

#include "inv_param.h"


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
uint16_t ipm_value[10];
uint16_t mtr_value[10];

uint16_t ipm_index=0;
uint16_t mtr_index=0;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */



/*******************************************************************************
 * EXTERNS
 */


// **************************************************************************
// definition needed for MODBUS
// **************************************************************************

/*
 *  ======== local function ========
 */

void UTIL_initIpmTemperatureArray(void)
{
	int i;

	for(i=0; i<10; i++) ipm_value[i] = 0;
}

void UTIL_initMotorTemperatureArray(void)
{
	int i;

	for(i=0; i<10; i++) mtr_value[i] = 0;
}

float_t UTIL_readIpmTemperature(void)
{



	return ((float_t)internal_status.ipm_temp * 0.0328 - 13.261);
}

float_t UTIL_readMotorTemperature(void)
{
	return (float_t)0.0;
}
/*
 *  ======== public function ========
 */



