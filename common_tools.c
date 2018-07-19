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
#include "common_tools.h"
#include "uartstdio.h"
//*****************************************************************************
//
//! \addtogroup
//! @{
//
//*****************************************************************************


extern HAL_Handle halHandle;

//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

int UTIL_controlLed(int type, int on_off)
{
	int result = 0;

	if(type == HAL_Gpio_LED_R || type == HAL_Gpio_LED_G)
	{
		if(on_off == 1)
			HAL_setGpioHigh(halHandle,(GPIO_Number_e)type);
		else
			HAL_setGpioLow(halHandle,(GPIO_Number_e)type);
	}
	else
	{
		UARTprintf("Error : no LED type=%d \n", type);
		result = 1;
	}

	return result;
}

// TODO : debug purpose only, using LED_R2 as test bit
void UTIL_testbit(int on_off) // LD2
{
	if(on_off == 1)
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);
	else
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);
}

void UTIL_testbitG(int on_off) // LD1
{
	if(on_off == 1)
		HAL_setGpioHigh(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
	else
		HAL_setGpioLow(halHandle,(GPIO_Number_e)HAL_Gpio_LED_G);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


