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

typedef union
{
  uint16_t iword[2];
  float_t f_data;
} union_fdata;

typedef union
{
  uint16_t iword[2];
  uint32_t l_data;
} union_ldata;


extern HAL_Handle halHandle;

//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************
void UTIL_packingFloat(uint16_t index, float_t fval, uint16_t *buf)
{
	union_fdata f;

	f.f_data = fval;

	buf[0] = f.iword[0];
	buf[1] = f.iword[1];
}

void UTIL_packingLong(uint16_t index, uint32_t lval, uint16_t *buf)
{
	union_ldata l;

	l.l_data = lval;

	buf[0] = l.iword[0];
	buf[1] = l.iword[1];
}

void UTIL_packingParam(uint16_t index, uint16_t *buf)
{
	float_t fval = 3.14;

	UTIL_packingFloat(index, fval, buf);
}

float_t UTIL_unpackingFloat(uint16_t index, uint16_t *buf)
{
	union_fdata f;

	f.iword[0] = buf[0];
	f.iword[1] = buf[1];

	return f.f_data;
}

uint32_t UTIL_unpackingLong(uint16_t index, uint16_t *buf)
{
	union_ldata l;

	l.iword[0] = buf[0];
	l.iword[1] = buf[1];

	return l.l_data;
}

int UTIL_unpackingParam(uint16_t index, uint16_t *buf, uint32_t *lval, float_t *fval)
{

	*fval = UTIL_unpackingFloat(index, buf);

	return 1;
}

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


