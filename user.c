/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_foc/src/user.c
//! \brief Contains the function for setting initialization data to the CTRL, HAL, and EST modules
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include <math.h>
#include "user.h"
#include "sw/modules/ctrl/src/32b/ctrl.h"

#ifdef SUPPORT_USER_VARIABLE
#include "inv_param.h"


extern float_t DRV_getPwmFrequency(void);

// **************************************************************************
// the defines


//! \brief ADC current offsets for A, B, and C phases
//! \brief One-time hardware dependent, though the calibration can be done at run-time as well
//! \brief After initial board calibration these values should be updated for your specific hardware so they are available after compile in the binary to be loaded to the controller
//#define   I_A_offset    (0.9966602325)
//#define   I_B_offset    (0.9960474968)
//#define   I_C_offset    (0.997712791)

#define   I_A_offset    (0.0)
#define   I_B_offset    (0.0)
#define   I_C_offset    (0.0)

//! \brief ADC voltage offsets for A, B, and C phases
//! \brief One-time hardware dependent, though the calibration can be done at run-time as well
//! \brief After initial board calibration these values should be updated for your specific hardware so they are available after compile in the binary to be loaded to the controller
//#define   V_A_offset    (0.3384106159)
//#define   V_B_offset    (0.3370528817)
//#define   V_C_offset    (0.3377450705)

#define   V_A_offset    (0.32325)  // PU value of 278V
#define   V_B_offset    (0.324419)
#define   V_C_offset    (0.32325)

//#define USER_MOTOR_FREQ_LOW				(5.0)			// Hz - suggested to set to 10% of rated motor frequency
//#define USER_MOTOR_FREQ_HIGH			(60.0)			// Hz - suggested to set to 100% of rated motor frequency
//#define USER_MOTOR_FREQ_MAX				(60.0)			// Hz - suggested to set to 120% of rated motor frequency
//#define USER_MOTOR_VOLT_MIN				(10.0)			// Volt - suggested to set to ~20% of rated motor voltage
//#define USER_MOTOR_VOLT_MAX				(240.0/1.732051)// Volt - suggested to set to 100% of rated motor voltage

#define USER_MOTOR_TYPE                 MOTOR_Type_Induction


// **************************************************************************
// the typedefs

extern USER_Params gUserParams;
#endif

#define USER_INPUT_VOLTAGE	(380)
// **************************************************************************
// the functions

#ifdef SUPPORT_USER_VARIABLE
void USER_setParams(USER_Params *pUserParams, motor_param_st *pmtr)
#else
void USER_setParams(USER_Params *pUserParams)
#endif
{
  pUserParams->iqFullScaleCurrent_A = USER_IQ_FULL_SCALE_CURRENT_A;
  pUserParams->iqFullScaleVoltage_V = USER_IQ_FULL_SCALE_VOLTAGE_V;

  pUserParams->iqFullScaleFreq_Hz = USER_IQ_FULL_SCALE_FREQ_Hz;

  pUserParams->numIsrTicksPerCtrlTick = USER_NUM_ISR_TICKS_PER_CTRL_TICK;
  pUserParams->numCtrlTicksPerCurrentTick = USER_NUM_CTRL_TICKS_PER_CURRENT_TICK;
  pUserParams->numCtrlTicksPerEstTick = USER_NUM_CTRL_TICKS_PER_EST_TICK;
#ifdef SUPPORT_USER_VARIABLE
  pUserParams->numCtrlTicksPerSpeedTick = 4; //(uint_least16_t)DRV_getPwmFrequency(); //USER_NUM_CTRL_TICKS_PER_SPEED_TICK;
  pUserParams->numCtrlTicksPerTrajTick = 4; //(uint_least16_t)DRV_getPwmFrequency(); //USER_NUM_CTRL_TICKS_PER_TRAJ_TICK;
#else
  pUserParams->numCtrlTicksPerSpeedTick = USER_NUM_CTRL_TICKS_PER_SPEED_TICK;
  pUserParams->numCtrlTicksPerTrajTick = USER_NUM_CTRL_TICKS_PER_TRAJ_TICK;
#endif  

  pUserParams->numCurrentSensors = USER_NUM_CURRENT_SENSORS;
  pUserParams->numVoltageSensors = USER_NUM_VOLTAGE_SENSORS;

  pUserParams->offsetPole_rps = USER_OFFSET_POLE_rps;
  pUserParams->fluxPole_rps = USER_FLUX_POLE_rps;

  pUserParams->zeroSpeedLimit = USER_ZEROSPEEDLIMIT;

  pUserParams->forceAngleFreq_Hz = USER_FORCE_ANGLE_FREQ_Hz;

  pUserParams->maxAccel_Hzps = USER_MAX_ACCEL_Hzps;

  pUserParams->maxAccel_est_Hzps = USER_MAX_ACCEL_EST_Hzps;

  pUserParams->directionPole_rps = USER_DIRECTION_POLE_rps;

  pUserParams->speedPole_rps = USER_SPEED_POLE_rps;

  pUserParams->dcBusPole_rps = USER_DCBUS_POLE_rps;

  pUserParams->fluxFraction = USER_FLUX_FRACTION;

  pUserParams->indEst_speedMaxFraction = USER_SPEEDMAX_FRACTION_FOR_L_IDENT;

  pUserParams->systemFreq_MHz = USER_SYSTEM_FREQ_MHz;

#ifdef SUPPORT_USER_VARIABLE
  pUserParams->pwmPeriod_kHz = (4.0); //USER_PWM_FREQ_kHz;
  //pUserParams->pwmPeriod_kHz = DRV_getPwmFrequency();
  pUserParams->pwmPeriod_usec = (1000.0/pUserParams->pwmPeriod_kHz);

  pUserParams->isrFreq_Hz = (uint_least32_t)(pUserParams->pwmPeriod_kHz * 1000.0 / USER_NUM_PWM_TICKS_PER_ISR_TICK);
  pUserParams->isrPeriod_usec = (pUserParams->pwmPeriod_usec * (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK);

  pUserParams->ctrlFreq_Hz = (uint_least32_t)(pUserParams->isrFreq_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK);
  pUserParams->ctrlPeriod_usec = (pUserParams->isrPeriod_usec * USER_NUM_ISR_TICKS_PER_CTRL_TICK);

  pUserParams->estFreq_Hz = (uint_least32_t)(pUserParams->ctrlFreq_Hz/USER_NUM_CTRL_TICKS_PER_EST_TICK);

  pUserParams->trajFreq_Hz = (uint_least32_t)(pUserParams->ctrlFreq_Hz/pUserParams->numCtrlTicksPerTrajTick);

  pUserParams->ctrlPeriod_sec = (pUserParams->ctrlPeriod_usec/(float_t)1000000.0);
#else
  pUserParams->pwmPeriod_usec = USER_PWM_PERIOD_usec;
#endif

  pUserParams->voltage_sf = USER_VOLTAGE_SF;

  pUserParams->current_sf = USER_CURRENT_SF;

  pUserParams->voltageFilterPole_rps = USER_VOLTAGE_FILTER_POLE_rps;

  pUserParams->maxVsMag_pu = USER_MAX_VS_MAG_PU;

  pUserParams->estKappa = USER_EST_KAPPAQ;

#ifdef SUPPORT_USER_VARIABLE
  pUserParams->motor_type = USER_MOTOR_TYPE;
  pUserParams->motor_numPolePairs = pmtr->poles;
  pUserParams->motor_ratedFlux = (0.8165*(float_t)mtr.input_voltage)/(float_t)mtr.rated_freq;;
  pUserParams->motor_Rr = pmtr->Rr;
  pUserParams->motor_Rs = pmtr->Rs;
  pUserParams->motor_Ls_d = pmtr->Ls;
  pUserParams->motor_Ls_q = pmtr->Ls;
#else
  pUserParams->motor_type = USER_MOTOR_TYPE;
  pUserParams->motor_numPolePairs = USER_MOTOR_NUM_POLE_PAIRS;
  pUserParams->motor_ratedFlux = USER_MOTOR_RATED_FLUX;
  pUserParams->motor_Rr = USER_MOTOR_Rr;
  pUserParams->motor_Rs = USER_MOTOR_Rs;
  pUserParams->motor_Ls_d = USER_MOTOR_Ls_d;
  pUserParams->motor_Ls_q = USER_MOTOR_Ls_q;
#endif

/*  if((pUserParams->motor_Rr > (float_t)0.0) && (pUserParams->motor_Rs > (float_t)0.0))
    {
      pUserParams->powerWarpGain = sqrt((float_t)1.0 + pUserParams->motor_Rr/pUserParams->motor_Rs);
    }
  else
*/    {
      pUserParams->powerWarpGain = USER_POWERWARP_GAIN;
    }

  pUserParams->maxCurrent_resEst = USER_MOTOR_RES_EST_CURRENT;
  pUserParams->maxCurrent_indEst = USER_MOTOR_IND_EST_CURRENT;
#ifdef SUPPORT_USER_VARIABLE
  pUserParams->maxCurrent = pmtr->max_current;
  pUserParams->maxCurrentSlope = (USER_MOTOR_RES_EST_CURRENT/USER_IQ_FULL_SCALE_CURRENT_A/pUserParams->trajFreq_Hz);
  pUserParams->maxCurrentSlope_powerWarp = (0.3*pUserParams->maxCurrentSlope);
  pUserParams->IdRated = (1.4142*pmtr->noload_current);
#else
  pUserParams->maxCurrent = USER_MOTOR_MAX_CURRENT;
  pUserParams->maxCurrentSlope = USER_MAX_CURRENT_SLOPE;
  pUserParams->maxCurrentSlope_powerWarp = USER_MAX_CURRENT_SLOPE_POWERWARP;
  pUserParams->IdRated = USER_MOTOR_MAGNETIZING_CURRENT;
#endif
  pUserParams->IdRatedFraction_ratedFlux = USER_IDRATED_FRACTION_FOR_RATED_FLUX;
  pUserParams->IdRatedFraction_indEst = USER_IDRATED_FRACTION_FOR_L_IDENT;
  pUserParams->IdRated_delta = USER_IDRATED_DELTA;

  pUserParams->fluxEstFreq_Hz = USER_MOTOR_FLUX_EST_FREQ_Hz;

#ifdef SUPPORT_USER_VARIABLE
  pUserParams->ctrlWaitTime[CTRL_State_Error]         = 0;
  pUserParams->ctrlWaitTime[CTRL_State_Idle]          = 0;
  pUserParams->ctrlWaitTime[CTRL_State_OffLine]       = (uint_least32_t)( 5.0 * pUserParams->ctrlFreq_Hz);
  pUserParams->ctrlWaitTime[CTRL_State_OnLine]        = 0;

  pUserParams->estWaitTime[EST_State_Error]           = 0;
  pUserParams->estWaitTime[EST_State_Idle]            = 0;
  pUserParams->estWaitTime[EST_State_RoverL]          = (uint_least32_t)( 8.0 * pUserParams->estFreq_Hz);
  pUserParams->estWaitTime[EST_State_Rs]              = 0;
  pUserParams->estWaitTime[EST_State_RampUp]          = (uint_least32_t)((5.0 + USER_MOTOR_FLUX_EST_FREQ_Hz / USER_MAX_ACCEL_EST_Hzps) * pUserParams->estFreq_Hz);
  pUserParams->estWaitTime[EST_State_IdRated]         = (uint_least32_t)(30.0 * pUserParams->estFreq_Hz);
  pUserParams->estWaitTime[EST_State_RatedFlux_OL]    = (uint_least32_t)( 0.2 * pUserParams->estFreq_Hz);
  pUserParams->estWaitTime[EST_State_RatedFlux]       = 0;
  pUserParams->estWaitTime[EST_State_RampDown]        = (uint_least32_t)( 2.0 * pUserParams->estFreq_Hz);
  pUserParams->estWaitTime[EST_State_LockRotor]       = 0;
  pUserParams->estWaitTime[EST_State_Ls]              = 0;
  pUserParams->estWaitTime[EST_State_Rr]              = (uint_least32_t)(20.0 * pUserParams->estFreq_Hz);
  pUserParams->estWaitTime[EST_State_MotorIdentified] = 0;
  pUserParams->estWaitTime[EST_State_OnLine]          = 0;

  pUserParams->FluxWaitTime[EST_Flux_State_Error]     = 0;
  pUserParams->FluxWaitTime[EST_Flux_State_Idle]      = 0;
  pUserParams->FluxWaitTime[EST_Flux_State_CL1]       = (uint_least32_t)(10.0 * pUserParams->estFreq_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_CL2]       = (uint_least32_t)( 0.2 * pUserParams->estFreq_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_Fine]      = (uint_least32_t)( 4.0 * pUserParams->estFreq_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_Done]      = 0;

  pUserParams->LsWaitTime[EST_Ls_State_Error]        = 0;
  pUserParams->LsWaitTime[EST_Ls_State_Idle]         = 0;
  pUserParams->LsWaitTime[EST_Ls_State_RampUp]       = (uint_least32_t)( 3.0 * pUserParams->estFreq_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Init]         = (uint_least32_t)( 3.0 * pUserParams->estFreq_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Coarse]       = (uint_least32_t)( 0.2 * pUserParams->estFreq_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Fine]         = (uint_least32_t)(30.0 * pUserParams->estFreq_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Done]         = 0;

  pUserParams->RsWaitTime[EST_Rs_State_Error]        = 0;
  pUserParams->RsWaitTime[EST_Rs_State_Idle]         = 0;
  pUserParams->RsWaitTime[EST_Rs_State_RampUp]       = (uint_least32_t)( 1.0 * pUserParams->estFreq_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Coarse]       = (uint_least32_t)( 2.0 * pUserParams->estFreq_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Fine]         = (uint_least32_t)( 7.0 * pUserParams->estFreq_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Done]         = 0;  


  pUserParams->RoverL_estFreq_Hz = USER_R_OVER_L_EST_FREQ_Hz;
#else
  pUserParams->ctrlWaitTime[CTRL_State_Error]         = 0;
  pUserParams->ctrlWaitTime[CTRL_State_Idle]          = 0;
  pUserParams->ctrlWaitTime[CTRL_State_OffLine]       = (uint_least32_t)( 5.0 * USER_CTRL_FREQ_Hz);
  pUserParams->ctrlWaitTime[CTRL_State_OnLine]        = 0;

  pUserParams->estWaitTime[EST_State_Error]           = 0;
  pUserParams->estWaitTime[EST_State_Idle]            = 0;
  pUserParams->estWaitTime[EST_State_RoverL]          = (uint_least32_t)( 8.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_Rs]              = 0;
  pUserParams->estWaitTime[EST_State_RampUp]          = (uint_least32_t)((5.0 + USER_MOTOR_FLUX_EST_FREQ_Hz / USER_MAX_ACCEL_EST_Hzps) * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_IdRated]         = (uint_least32_t)(30.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_RatedFlux_OL]    = (uint_least32_t)( 0.2 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_RatedFlux]       = 0;
  pUserParams->estWaitTime[EST_State_RampDown]        = (uint_least32_t)( 2.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_LockRotor]       = 0;
  pUserParams->estWaitTime[EST_State_Ls]              = 0;
  pUserParams->estWaitTime[EST_State_Rr]              = (uint_least32_t)(20.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_MotorIdentified] = 0;
  pUserParams->estWaitTime[EST_State_OnLine]          = 0;

  pUserParams->FluxWaitTime[EST_Flux_State_Error]     = 0;
  pUserParams->FluxWaitTime[EST_Flux_State_Idle]      = 0;
  pUserParams->FluxWaitTime[EST_Flux_State_CL1]       = (uint_least32_t)(10.0 * USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_CL2]       = (uint_least32_t)( 0.2 * USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_Fine]      = (uint_least32_t)( 4.0 * USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_Done]      = 0;

  pUserParams->LsWaitTime[EST_Ls_State_Error]        = 0;
  pUserParams->LsWaitTime[EST_Ls_State_Idle]         = 0;
  pUserParams->LsWaitTime[EST_Ls_State_RampUp]       = (uint_least32_t)( 3.0 * USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Init]         = (uint_least32_t)( 3.0 * USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Coarse]       = (uint_least32_t)( 0.2 * USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Fine]         = (uint_least32_t)(30.0 * USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Done]         = 0;

  pUserParams->RsWaitTime[EST_Rs_State_Error]        = 0;
  pUserParams->RsWaitTime[EST_Rs_State_Idle]         = 0;
  pUserParams->RsWaitTime[EST_Rs_State_RampUp]       = (uint_least32_t)( 1.0 * USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Coarse]       = (uint_least32_t)( 2.0 * USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Fine]         = (uint_least32_t)( 7.0 * USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Done]         = 0;

  pUserParams->ctrlFreq_Hz = USER_CTRL_FREQ_Hz;

  pUserParams->estFreq_Hz = USER_EST_FREQ_Hz;

  pUserParams->RoverL_estFreq_Hz = USER_R_OVER_L_EST_FREQ_Hz;

  pUserParams->trajFreq_Hz = USER_TRAJ_FREQ_Hz;

  pUserParams->ctrlPeriod_sec = USER_CTRL_PERIOD_sec;
#endif  

#ifdef SUPPORT_USER_VARIABLE
  pUserParams->maxNegativeIdCurrent_a = (-0.5 * pUserParams->maxCurrent);
#else
  pUserParams->maxNegativeIdCurrent_a = USER_MAX_NEGATIVE_ID_REF_CURRENT_A;
#endif

#ifdef SUPPORT_USER_VARIABLE
  pUserParams->I_A_Offset = I_A_offset;
  pUserParams->I_B_Offset = I_B_offset;
  pUserParams->I_C_Offset = I_C_offset;

  pUserParams->V_A_Offset = V_A_offset;
  pUserParams->V_B_Offset = V_B_offset;
  pUserParams->V_C_Offset = V_C_offset;
#endif  

  pUserParams->VF_freq_low = USER_MOTOR_FREQ_LOW;
  pUserParams->VF_freq_high = USER_MOTOR_FREQ_HIGH;

//#define PWM_DEADBAND_LIMITATION  (1.0)
#define PWM_DEADBAND_LIMITATION  (0.95)
//#define PWM_DEADBAND_LIMITATION  (0.93)

#if 1
#ifdef SUPPORT_USER_VARIABLE
  pUserParams->VF_volt_max = ((pmtr->input_voltage*1.35)/1.732051)*PWM_DEADBAND_LIMITATION;
#else
  pUserParams->VF_volt_max = ((USER_INPUT_VOLTAGE*1.35)/1.732051)*PWM_DEADBAND_LIMITATION;
#endif
  //pUserParams->VF_volt_min = 20.0;
  pUserParams->VF_volt_min = (pUserParams->VF_volt_max*(USER_MOTOR_FREQ_LOW/USER_MOTOR_FREQ_HIGH));
#else
  if(pmtr->input_voltage == 220) // standard x1.35+10.0
  {
	  pUserParams->VF_volt_max = ((pmtr->input_voltage*1.35)/1.732051 + 10.0);//USER_MOTOR_VOLT_MAX;
	  pUserParams->VF_volt_min = (10.0 + pUserParams->VF_volt_max*(USER_MOTOR_FREQ_LOW/USER_MOTOR_FREQ_HIGH));//USER_MOTOR_VOLT_MIN; //pmtr->input_voltage*0.1;
  }
  else if(pmtr->input_voltage == 380)
  {
	  pUserParams->VF_volt_max = ((pmtr->input_voltage*1.5)/1.732051 + 10.0);//USER_MOTOR_VOLT_MAX;
	  pUserParams->VF_volt_min = (10.0 + pUserParams->VF_volt_max*(USER_MOTOR_FREQ_LOW/USER_MOTOR_FREQ_HIGH));//USER_MOTOR_VOLT_MIN; //pmtr->input_voltage*0.1;
  }
  else
	  internal_status.trip_happened = TRIP_REASON_INPUT_VOLT_ERR;
#endif
  //pUserParams->VF_volt_min = (80.0/1.732051);//USER_MOTOR_VOLT_MIN;
  //pUserParams->VF_volt_min = (10.0/1.732051) + pUserParams->VF_volt_max*(USER_MOTOR_FREQ_LOW/USER_MOTOR_FREQ_HIGH);//USER_MOTOR_VOLT_MIN; //pmtr->input_voltage*0.1;

  return;
} // end of USER_setParams() function

#ifdef SUPPORT_USER_VARIABLE
void USER_checkForErrors(USER_Params *pUserParams)
{
  USER_setErrorCode(pUserParams, USER_ErrorCode_NoError);

  if((USER_IQ_FULL_SCALE_CURRENT_A <= 0.0) ||
    (USER_IQ_FULL_SCALE_CURRENT_A <= (0.02 * pUserParams->maxCurrent * USER_IQ_FULL_SCALE_FREQ_Hz / 128.0)) ||
    (USER_IQ_FULL_SCALE_CURRENT_A <= (2.0 * pUserParams->maxCurrent * USER_IQ_FULL_SCALE_FREQ_Hz * pUserParams->ctrlPeriod_sec / 128.0)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

  if((USER_IQ_FULL_SCALE_CURRENT_A < pUserParams->IdRated) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_RES_EST_CURRENT) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_IND_EST_CURRENT) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < pUserParams->maxCurrent))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

#if 0 //hrjung
  if((USER_MOTOR_RATED_FLUX > 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
      if(USER_IQ_FULL_SCALE_VOLTAGE_V >= ((float_t)USER_EST_FREQ_Hz * USER_MOTOR_RATED_FLUX * 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }
#endif

  if((pUserParams->motor_ratedFlux > 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(USER_IQ_FULL_SCALE_VOLTAGE_V >= ((float_t)pUserParams->estFreq_Hz * pUserParams->motor_ratedFlux * 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }

  if((USER_IQ_FULL_SCALE_VOLTAGE_V <= 0.0) ||
    (USER_IQ_FULL_SCALE_VOLTAGE_V <= (0.5 * pUserParams->maxCurrent * pUserParams->motor_Ls_d * USER_VOLTAGE_FILTER_POLE_rps)) ||
    (USER_IQ_FULL_SCALE_VOLTAGE_V <= (0.5 * pUserParams->maxCurrent * pUserParams->motor_Ls_q * USER_VOLTAGE_FILTER_POLE_rps)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_Low);
    }

  if((USER_IQ_FULL_SCALE_FREQ_Hz > (4.0 * USER_VOLTAGE_FILTER_POLE_Hz)) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= ((128.0 * USER_IQ_FULL_SCALE_CURRENT_A) / (0.02 * pUserParams->maxCurrent))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= ((128.0 * USER_IQ_FULL_SCALE_CURRENT_A) / (2.0 * pUserParams->maxCurrent * pUserParams->ctrlPeriod_sec))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= (128.0 * (float_t)pUserParams->motor_numPolePairs * 1000.0 / 60.0)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_High);
    }

  if((USER_IQ_FULL_SCALE_FREQ_Hz < 50.0) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < USER_MOTOR_FLUX_EST_FREQ_Hz) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < USER_SPEED_POLE_rps) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz <= ((float_t)pUserParams->motor_numPolePairs * 1000.0 / (60.0 * 128.0))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < (USER_MAX_ACCEL_Hzps / ((float_t)pUserParams->trajFreq_Hz))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < (USER_MAX_ACCEL_EST_Hzps / ((float_t)pUserParams->trajFreq_Hz))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < ((float_t)USER_R_OVER_L_EST_FREQ_Hz)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_Low);
    }

  if(USER_NUM_PWM_TICKS_PER_ISR_TICK > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_High);
    }

  if(USER_NUM_PWM_TICKS_PER_ISR_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_Low);
    }

  if(USER_NUM_ISR_TICKS_PER_CTRL_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numIsrTicksPerCtrlTick_Low);
    }

  if(USER_NUM_CTRL_TICKS_PER_CURRENT_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerCurrentTick_Low);
    }

  if(USER_NUM_CTRL_TICKS_PER_EST_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerEstTick_Low);
    }

  if((pUserParams->numCtrlTicksPerSpeedTick < 1) ||
    (pUserParams->numCtrlTicksPerSpeedTick < USER_NUM_CTRL_TICKS_PER_CURRENT_TICK))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerSpeedTick_Low);
    }

  if(pUserParams->numCtrlTicksPerTrajTick < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerTrajTick_Low);
    }

  if(USER_NUM_CURRENT_SENSORS > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_High);
    }

  if(USER_NUM_CURRENT_SENSORS < 2)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_Low);
    }

  if(USER_NUM_VOLTAGE_SENSORS > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_High);
    }

  if(USER_NUM_VOLTAGE_SENSORS < 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_Low);
    }

  if(USER_OFFSET_POLE_rps > ((float_t)pUserParams->ctrlFreq_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_High);
    }

  if(USER_OFFSET_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_Low);
    }

  if(USER_FLUX_POLE_rps > ((float_t)pUserParams->estFreq_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_High);
    }

  if(USER_FLUX_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_Low);
    }

  if(USER_ZEROSPEEDLIMIT > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_High);
    }

  if(USER_ZEROSPEEDLIMIT <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_Low);
    }

  if(USER_FORCE_ANGLE_FREQ_Hz > ((float_t)pUserParams->estFreq_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_High);
    }

  if(USER_FORCE_ANGLE_FREQ_Hz <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_Low);
    }

  if(USER_MAX_ACCEL_Hzps > ((float_t)pUserParams->trajFreq_Hz * USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_High);
    }

  if(USER_MAX_ACCEL_Hzps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_Low);
    }

  if(USER_MAX_ACCEL_EST_Hzps > ((float_t)pUserParams->trajFreq_Hz * USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_High);
    }

  if(USER_MAX_ACCEL_EST_Hzps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_Low);
    }

  if(USER_DIRECTION_POLE_rps > ((float_t)pUserParams->estFreq_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_High);
    }

  if(USER_DIRECTION_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_Low);
    }

  if((USER_SPEED_POLE_rps > USER_IQ_FULL_SCALE_FREQ_Hz) ||
    (USER_SPEED_POLE_rps > ((float_t)pUserParams->estFreq_Hz)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_High);
    }

  if(USER_SPEED_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_Low);
    }

  if(USER_DCBUS_POLE_rps > ((float_t)pUserParams->estFreq_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_High);
    }

  if(USER_DCBUS_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_Low);
    }

  if(USER_FLUX_FRACTION > 1.2)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_High);
    }

  if(USER_FLUX_FRACTION < 0.05)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_Low);
    }

  if(USER_SPEEDMAX_FRACTION_FOR_L_IDENT > (USER_IQ_FULL_SCALE_CURRENT_A / pUserParams->maxCurrent))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_High);
    }

  if(USER_SPEEDMAX_FRACTION_FOR_L_IDENT <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_Low);
    }

  if(USER_POWERWARP_GAIN > 2.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_High);
    }

  if(USER_POWERWARP_GAIN < 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_Low);
    }

  if(USER_SYSTEM_FREQ_MHz > 90.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_High);
    }

  if(USER_SYSTEM_FREQ_MHz <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_Low);
    }

  if(pUserParams->pwmPeriod_kHz > (1000.0 * USER_SYSTEM_FREQ_MHz / 100.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_High);
    }

  if(pUserParams->pwmPeriod_kHz < (1000.0 * USER_SYSTEM_FREQ_MHz / 65536.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_Low);
    }

  if(USER_VOLTAGE_SF >= 128.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_High);
    }

  if(USER_VOLTAGE_SF < 0.1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_Low);
    }

  if(USER_CURRENT_SF >= 128.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_High);
    }

  if(USER_CURRENT_SF < 0.1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_Low);
    }

  if(USER_VOLTAGE_FILTER_POLE_Hz > ((float_t)pUserParams->estFreq_Hz / MATH_PI))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_High);
    }

  if(USER_VOLTAGE_FILTER_POLE_Hz < (USER_IQ_FULL_SCALE_FREQ_Hz / 4.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_Low);
    }

  if(USER_MAX_VS_MAG_PU > (4.0 / 3.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_High);
    }

  if(USER_MAX_VS_MAG_PU <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_Low);
    }

  if(USER_EST_KAPPAQ > 1.5)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_High);
    }

  if(USER_EST_KAPPAQ < 1.5)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_Low);
    }

  if((USER_MOTOR_TYPE != MOTOR_Type_Induction) && (USER_MOTOR_TYPE != MOTOR_Type_Pm))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_motor_type_Unknown);
    }

  if(pUserParams->motor_numPolePairs < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_motor_numPolePairs_Low);
    }

#if 0
  if((pUserParams->motor_ratedFlux != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
      if(pUserParams->motor_ratedFlux > (USER_IQ_FULL_SCALE_FREQ_Hz * 65536.0 / (float_t)USER_EST_FREQ_Hz / 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

      if(pUserParams->motor_ratedFlux < (USER_IQ_FULL_SCALE_VOLTAGE_V / (float_t)USER_EST_FREQ_Hz / 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }
#endif

  if((pUserParams->motor_ratedFlux != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(pUserParams->motor_ratedFlux > (USER_IQ_FULL_SCALE_FREQ_Hz * 65536.0 / (float_t)pUserParams->estFreq_Hz / 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

      if(pUserParams->motor_ratedFlux < (USER_IQ_FULL_SCALE_VOLTAGE_V / (float_t)pUserParams->estFreq_Hz / 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }

#if 0 //hrjung
  if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(USER_MOTOR_Rr > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

      if(USER_MOTOR_Rr < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }
#endif

  if((pUserParams->motor_Rr != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(pUserParams->motor_Rr > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / USER_IQ_FULL_SCALE_CURRENT_A))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

      if(pUserParams->motor_Rr < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }

  if(pUserParams->motor_Rs != 0.0)
    {
      if(pUserParams->motor_Rs > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / USER_IQ_FULL_SCALE_CURRENT_A))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_High);
        }

      if(pUserParams->motor_Rs < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_Low);
        }
    }

  if(pUserParams->motor_Ls_d != 0.0)
    {
      if(pUserParams->motor_Ls_d > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_High);
        }

      if(pUserParams->motor_Ls_d < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_Low);
        }
    }

  if(pUserParams->motor_Ls_q != 0.0)
    {
      if(pUserParams->motor_Ls_q > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_High);
        }

      if(pUserParams->motor_Ls_q < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_Low);
        }
    }

  if(USER_MOTOR_RES_EST_CURRENT > pUserParams->maxCurrent)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_High);
    }

  if(USER_MOTOR_RES_EST_CURRENT < 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_Low);
    }
#if 0 //hrjung
  if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(USER_MOTOR_IND_EST_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

      if(USER_MOTOR_IND_EST_CURRENT < (-USER_MOTOR_MAX_CURRENT))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }
#endif

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_MOTOR_IND_EST_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

      if(USER_MOTOR_IND_EST_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }

  if(pUserParams->maxCurrent > USER_IQ_FULL_SCALE_CURRENT_A)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_High);
    }

  if(pUserParams->maxCurrent <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_Low);
    }

  if(pUserParams->maxCurrentSlope > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_High);
    }

  if(pUserParams->maxCurrentSlope <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_Low);
    }

  if(pUserParams->maxCurrentSlope_powerWarp > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_High);
    }

  if(pUserParams->maxCurrentSlope_powerWarp <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_Low);
    }

#if 0 //hrjung
  if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(USER_MOTOR_MAGNETIZING_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

      if(USER_MOTOR_MAGNETIZING_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }
#endif

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(pUserParams->IdRated > pUserParams->maxCurrent)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

      if(pUserParams->IdRated < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_IDRATED_FRACTION_FOR_RATED_FLUX > (USER_IQ_FULL_SCALE_CURRENT_A / (1.2 * pUserParams->maxCurrent)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_High);
        }

      if(USER_IDRATED_FRACTION_FOR_RATED_FLUX < 0.1)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_Low);
        }
    }

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_IDRATED_FRACTION_FOR_L_IDENT > (USER_IQ_FULL_SCALE_CURRENT_A / pUserParams->maxCurrent))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_High);
        }

      if(USER_IDRATED_FRACTION_FOR_L_IDENT < 0.1)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_Low);
        }
    }

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_IDRATED_DELTA > (USER_IQ_FULL_SCALE_CURRENT_A / ((float_t)USER_NUM_ISR_TICKS_PER_CTRL_TICK * pUserParams->maxCurrent)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_High);
        }

      if(USER_IDRATED_DELTA < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_Low);
        }
    }

  if(USER_MOTOR_FLUX_EST_FREQ_Hz > USER_IQ_FULL_SCALE_FREQ_Hz)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_High);
    }

  if((USER_MOTOR_FLUX_EST_FREQ_Hz < 0.0) ||
    (USER_MOTOR_FLUX_EST_FREQ_Hz < (USER_ZEROSPEEDLIMIT * USER_IQ_FULL_SCALE_FREQ_Hz)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_Low);
    }

  if(pUserParams->motor_Ls_d != 0.0)
    {
      if(((float_t)pUserParams->ctrlFreq_Hz >= (128.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (0.5 * (pUserParams->motor_Ls_d + 1e-9) * USER_IQ_FULL_SCALE_CURRENT_A))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

  if(pUserParams->motor_Ls_q != 0.0)
    {
      if(((float_t)pUserParams->ctrlFreq_Hz >= (128.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (0.5 * (pUserParams->motor_Ls_q + 1e-9) * USER_IQ_FULL_SCALE_CURRENT_A))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

  if(((float_t)pUserParams->ctrlFreq_Hz < USER_IQ_FULL_SCALE_FREQ_Hz) ||
    ((float_t)pUserParams->ctrlFreq_Hz < USER_OFFSET_POLE_rps) ||
    ((float_t)pUserParams->ctrlFreq_Hz < 250.0) ||
    ((float_t)pUserParams->ctrlFreq_Hz <= (2.0 * USER_IQ_FULL_SCALE_FREQ_Hz * pUserParams->maxCurrent / (128.0 * USER_IQ_FULL_SCALE_CURRENT_A))))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
    }

  if((pUserParams->motor_Rs != 0.0) && (pUserParams->motor_Ls_d != 0.0) && (pUserParams->motor_Ls_q != 0.0))
    {
      if(((float_t)pUserParams->ctrlFreq_Hz <= (pUserParams->motor_Rs / (pUserParams->motor_Ls_d + 1e-9))) ||
        ((float_t)pUserParams->ctrlFreq_Hz <= (pUserParams->motor_Rs / (pUserParams->motor_Ls_q + 1e-9))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
        }
    }

  if(((float_t)pUserParams->estFreq_Hz < USER_FORCE_ANGLE_FREQ_Hz) ||
    ((float_t)pUserParams->estFreq_Hz < USER_VOLTAGE_FILTER_POLE_rps) ||
    ((float_t)pUserParams->estFreq_Hz < USER_DCBUS_POLE_rps) ||
    ((float_t)pUserParams->estFreq_Hz < USER_FLUX_POLE_rps) ||
    ((float_t)pUserParams->estFreq_Hz < USER_DIRECTION_POLE_rps) ||
    ((float_t)pUserParams->estFreq_Hz < USER_SPEED_POLE_rps) ||
    ((float_t)pUserParams->estFreq_Hz < 0.2))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estFreq_Hz_Low);
    }

  if(USER_R_OVER_L_EST_FREQ_Hz > USER_IQ_FULL_SCALE_FREQ_Hz)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_RoverL_estFreq_Hz_High);
    }

  if(((float_t)pUserParams->trajFreq_Hz < 1.0) ||
    ((float_t)pUserParams->trajFreq_Hz < USER_MAX_ACCEL_Hzps / USER_IQ_FULL_SCALE_FREQ_Hz) ||
    ((float_t)pUserParams->trajFreq_Hz < USER_MAX_ACCEL_EST_Hzps / USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_trajFreq_Hz_Low);
    }

  if(pUserParams->maxNegativeIdCurrent_a > 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_High);
    }

  if(pUserParams->maxNegativeIdCurrent_a < (-pUserParams->maxCurrent))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_Low);
    }

  // Only for debug testing, only be commented
//  USER_setErrorCode(pUserParams, USER_ErrorCode_NoError);

  return;
} // end of USER_checkForErrors() function
#else
void USER_checkForErrors(USER_Params *pUserParams)
{
  USER_setErrorCode(pUserParams, USER_ErrorCode_NoError);

  if((USER_IQ_FULL_SCALE_CURRENT_A <= 0.0) ||
    (USER_IQ_FULL_SCALE_CURRENT_A <= (0.02 * USER_MOTOR_MAX_CURRENT * USER_IQ_FULL_SCALE_FREQ_Hz / 128.0)) ||
    (USER_IQ_FULL_SCALE_CURRENT_A <= (2.0 * USER_MOTOR_MAX_CURRENT * USER_IQ_FULL_SCALE_FREQ_Hz * USER_CTRL_PERIOD_sec / 128.0)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

  if((USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_MAGNETIZING_CURRENT) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_RES_EST_CURRENT) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_IND_EST_CURRENT) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_MAX_CURRENT))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

#if 0 //hrjung
  if((USER_MOTOR_RATED_FLUX > 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
      if(USER_IQ_FULL_SCALE_VOLTAGE_V >= ((float_t)USER_EST_FREQ_Hz * USER_MOTOR_RATED_FLUX * 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }
#endif	

  if((USER_MOTOR_RATED_FLUX > 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(USER_IQ_FULL_SCALE_VOLTAGE_V >= ((float_t)USER_EST_FREQ_Hz * USER_MOTOR_RATED_FLUX * 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }

  if((USER_IQ_FULL_SCALE_VOLTAGE_V <= 0.0) ||
    (USER_IQ_FULL_SCALE_VOLTAGE_V <= (0.5 * USER_MOTOR_MAX_CURRENT * USER_MOTOR_Ls_d * USER_VOLTAGE_FILTER_POLE_rps)) ||
    (USER_IQ_FULL_SCALE_VOLTAGE_V <= (0.5 * USER_MOTOR_MAX_CURRENT * USER_MOTOR_Ls_q * USER_VOLTAGE_FILTER_POLE_rps)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_Low);
    }

  if((USER_IQ_FULL_SCALE_FREQ_Hz > (4.0 * USER_VOLTAGE_FILTER_POLE_Hz)) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= ((128.0 * USER_IQ_FULL_SCALE_CURRENT_A) / (0.02 * USER_MOTOR_MAX_CURRENT))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= ((128.0 * USER_IQ_FULL_SCALE_CURRENT_A) / (2.0 * USER_MOTOR_MAX_CURRENT * USER_CTRL_PERIOD_sec))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= (128.0 * (float_t)USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / 60.0)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_High);
    }

  if((USER_IQ_FULL_SCALE_FREQ_Hz < 50.0) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < USER_MOTOR_FLUX_EST_FREQ_Hz) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < USER_SPEED_POLE_rps) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz <= ((float_t)USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / (60.0 * 128.0))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < (USER_MAX_ACCEL_Hzps / ((float_t)USER_TRAJ_FREQ_Hz))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < (USER_MAX_ACCEL_EST_Hzps / ((float_t)USER_TRAJ_FREQ_Hz))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < ((float_t)USER_R_OVER_L_EST_FREQ_Hz)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_Low);
    }

  if(USER_NUM_PWM_TICKS_PER_ISR_TICK > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_High);
    }

  if(USER_NUM_PWM_TICKS_PER_ISR_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_Low);
    }

  if(USER_NUM_ISR_TICKS_PER_CTRL_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numIsrTicksPerCtrlTick_Low);
    }

  if(USER_NUM_CTRL_TICKS_PER_CURRENT_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerCurrentTick_Low);
    }

  if(USER_NUM_CTRL_TICKS_PER_EST_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerEstTick_Low);
    }

  if((USER_NUM_CTRL_TICKS_PER_SPEED_TICK < 1) ||
    (USER_NUM_CTRL_TICKS_PER_SPEED_TICK < USER_NUM_CTRL_TICKS_PER_CURRENT_TICK))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerSpeedTick_Low);
    }

  if(USER_NUM_CTRL_TICKS_PER_TRAJ_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerTrajTick_Low);
    }

  if(USER_NUM_CURRENT_SENSORS > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_High);
    }

  if(USER_NUM_CURRENT_SENSORS < 2)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_Low);
    }

  if(USER_NUM_VOLTAGE_SENSORS > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_High);
    }

  if(USER_NUM_VOLTAGE_SENSORS < 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_Low);
    }

  if(USER_OFFSET_POLE_rps > ((float_t)USER_CTRL_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_High);
    }

  if(USER_OFFSET_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_Low);
    }

  if(USER_FLUX_POLE_rps > ((float_t)USER_EST_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_High);
    }

  if(USER_FLUX_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_Low);
    }

  if(USER_ZEROSPEEDLIMIT > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_High);
    }

  if(USER_ZEROSPEEDLIMIT <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_Low);
    }

  if(USER_FORCE_ANGLE_FREQ_Hz > ((float_t)USER_EST_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_High);
    }

  if(USER_FORCE_ANGLE_FREQ_Hz <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_Low);
    }

  if(USER_MAX_ACCEL_Hzps > ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_High);
    }

  if(USER_MAX_ACCEL_Hzps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_Low);
    }

  if(USER_MAX_ACCEL_EST_Hzps > ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_High);
    }

  if(USER_MAX_ACCEL_EST_Hzps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_Low);
    }

  if(USER_DIRECTION_POLE_rps > ((float_t)USER_EST_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_High);
    }

  if(USER_DIRECTION_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_Low);
    }

  if((USER_SPEED_POLE_rps > USER_IQ_FULL_SCALE_FREQ_Hz) ||
    (USER_SPEED_POLE_rps > ((float_t)USER_EST_FREQ_Hz)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_High);
    }

  if(USER_SPEED_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_Low);
    }

  if(USER_DCBUS_POLE_rps > ((float_t)USER_EST_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_High);
    }

  if(USER_DCBUS_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_Low);
    }

  if(USER_FLUX_FRACTION > 1.2)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_High);
    }

  if(USER_FLUX_FRACTION < 0.05)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_Low);
    }

  if(USER_SPEEDMAX_FRACTION_FOR_L_IDENT > (USER_IQ_FULL_SCALE_CURRENT_A / USER_MOTOR_MAX_CURRENT))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_High);
    }

  if(USER_SPEEDMAX_FRACTION_FOR_L_IDENT <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_Low);
    }

  if(USER_POWERWARP_GAIN > 2.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_High);
    }

  if(USER_POWERWARP_GAIN < 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_Low);
    }

  if(USER_SYSTEM_FREQ_MHz > 90.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_High);
    }

  if(USER_SYSTEM_FREQ_MHz <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_Low);
    }

  if(USER_PWM_FREQ_kHz > (1000.0 * USER_SYSTEM_FREQ_MHz / 100.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_High);
    }

  if(USER_PWM_FREQ_kHz < (1000.0 * USER_SYSTEM_FREQ_MHz / 65536.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_Low);
    }

  if(USER_VOLTAGE_SF >= 128.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_High);
    }

  if(USER_VOLTAGE_SF < 0.1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_Low);
    }

  if(USER_CURRENT_SF >= 128.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_High);
    }

  if(USER_CURRENT_SF < 0.1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_Low);
    }

  if(USER_VOLTAGE_FILTER_POLE_Hz > ((float_t)USER_EST_FREQ_Hz / MATH_PI))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_High);
    }

  if(USER_VOLTAGE_FILTER_POLE_Hz < (USER_IQ_FULL_SCALE_FREQ_Hz / 4.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_Low);
    }

  if(USER_MAX_VS_MAG_PU > (4.0 / 3.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_High);
    }

  if(USER_MAX_VS_MAG_PU <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_Low);
    }

  if(USER_EST_KAPPAQ > 1.5)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_High);
    }

  if(USER_EST_KAPPAQ < 1.5)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_Low);
    }

#if 0 //hrjung
  if((USER_MOTOR_TYPE != MOTOR_Type_Induction) && (USER_MOTOR_TYPE != MOTOR_Type_Pm))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_motor_type_Unknown);
    }
#endif	

  if(USER_MOTOR_NUM_POLE_PAIRS < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_motor_numPolePairs_Low);
    }

#if 0 //hrjung
  if((USER_MOTOR_RATED_FLUX != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
      if(USER_MOTOR_RATED_FLUX > (USER_IQ_FULL_SCALE_FREQ_Hz * 65536.0 / (float_t)USER_EST_FREQ_Hz / 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

      if(USER_MOTOR_RATED_FLUX < (USER_IQ_FULL_SCALE_VOLTAGE_V / (float_t)USER_EST_FREQ_Hz / 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }
#endif	

  if((USER_MOTOR_RATED_FLUX != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(USER_MOTOR_RATED_FLUX > (USER_IQ_FULL_SCALE_FREQ_Hz * 65536.0 / (float_t)USER_EST_FREQ_Hz / 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

      if(USER_MOTOR_RATED_FLUX < (USER_IQ_FULL_SCALE_VOLTAGE_V / (float_t)USER_EST_FREQ_Hz / 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }

#if 0 //hrjung
  if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(USER_MOTOR_Rr > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

      if(USER_MOTOR_Rr < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }
#endif	

  if((USER_MOTOR_Rr != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(USER_MOTOR_Rr > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / USER_IQ_FULL_SCALE_CURRENT_A))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

      if(USER_MOTOR_Rr < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }

  if(USER_MOTOR_Rs != 0.0)
    {
      if(USER_MOTOR_Rs > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / USER_IQ_FULL_SCALE_CURRENT_A))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_High);
        }

      if(USER_MOTOR_Rs < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_Low);
        }
    }

  if(USER_MOTOR_Ls_d != 0.0)
    {
      if(USER_MOTOR_Ls_d > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_High);
        }

      if(USER_MOTOR_Ls_d < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_Low);
        }
    }

  if(USER_MOTOR_Ls_q != 0.0)
    {
      if(USER_MOTOR_Ls_q > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_High);
        }

      if(USER_MOTOR_Ls_q < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_Low);
        }
    }

  if(USER_MOTOR_RES_EST_CURRENT > USER_MOTOR_MAX_CURRENT)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_High);
    }

  if(USER_MOTOR_RES_EST_CURRENT < 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_Low);
    }

#if 0 //hrjung
  if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(USER_MOTOR_IND_EST_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

      if(USER_MOTOR_IND_EST_CURRENT < (-USER_MOTOR_MAX_CURRENT))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }
#endif	

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_MOTOR_IND_EST_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

      if(USER_MOTOR_IND_EST_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }

  if(USER_MOTOR_MAX_CURRENT > USER_IQ_FULL_SCALE_CURRENT_A)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_High);
    }

  if(USER_MOTOR_MAX_CURRENT <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_Low);
    }

  if(USER_MAX_CURRENT_SLOPE > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_High);
    }

  if(USER_MAX_CURRENT_SLOPE <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_Low);
    }

  if(USER_MAX_CURRENT_SLOPE_POWERWARP > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_High);
    }

  if(USER_MAX_CURRENT_SLOPE_POWERWARP <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_Low);
    }

#if 0 //hrjung
  if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(USER_MOTOR_MAGNETIZING_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

      if(USER_MOTOR_MAGNETIZING_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }
#endif

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_MOTOR_MAGNETIZING_CURRENT > USER_MOTOR_MAX_CURRENT)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

      if(USER_MOTOR_MAGNETIZING_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_IDRATED_FRACTION_FOR_RATED_FLUX > (USER_IQ_FULL_SCALE_CURRENT_A / (1.2 * USER_MOTOR_MAX_CURRENT)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_High);
        }

      if(USER_IDRATED_FRACTION_FOR_RATED_FLUX < 0.1)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_Low);
        }
    }

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_IDRATED_FRACTION_FOR_L_IDENT > (USER_IQ_FULL_SCALE_CURRENT_A / USER_MOTOR_MAX_CURRENT))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_High);
        }

      if(USER_IDRATED_FRACTION_FOR_L_IDENT < 0.1)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_Low);
        }
    }

  if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(USER_IDRATED_DELTA > (USER_IQ_FULL_SCALE_CURRENT_A / ((float_t)USER_NUM_ISR_TICKS_PER_CTRL_TICK * USER_MOTOR_MAX_CURRENT)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_High);
        }

      if(USER_IDRATED_DELTA < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_Low);
        }
    }

  if(USER_MOTOR_FLUX_EST_FREQ_Hz > USER_IQ_FULL_SCALE_FREQ_Hz)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_High);
    }

  if((USER_MOTOR_FLUX_EST_FREQ_Hz < 0.0) ||
    (USER_MOTOR_FLUX_EST_FREQ_Hz < (USER_ZEROSPEEDLIMIT * USER_IQ_FULL_SCALE_FREQ_Hz)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_Low);
    }

  if(USER_MOTOR_Ls_d != 0.0)
    {
      if(((float_t)USER_CTRL_FREQ_Hz >= (128.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (0.5 * (USER_MOTOR_Ls_d + 1e-9) * USER_IQ_FULL_SCALE_CURRENT_A))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

  if(USER_MOTOR_Ls_q != 0.0)
    {
      if(((float_t)USER_CTRL_FREQ_Hz >= (128.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (0.5 * (USER_MOTOR_Ls_q + 1e-9) * USER_IQ_FULL_SCALE_CURRENT_A))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

  if(((float_t)USER_CTRL_FREQ_Hz < USER_IQ_FULL_SCALE_FREQ_Hz) ||
    ((float_t)USER_CTRL_FREQ_Hz < USER_OFFSET_POLE_rps) ||
    ((float_t)USER_CTRL_FREQ_Hz < 250.0) ||
    ((float_t)USER_CTRL_FREQ_Hz <= (2.0 * USER_IQ_FULL_SCALE_FREQ_Hz * USER_MOTOR_MAX_CURRENT / (128.0 * USER_IQ_FULL_SCALE_CURRENT_A))))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
    }

  if((USER_MOTOR_Rs != 0.0) && (USER_MOTOR_Ls_d != 0.0) && (USER_MOTOR_Ls_q != 0.0))
    {
      if(((float_t)USER_CTRL_FREQ_Hz <= (USER_MOTOR_Rs / (USER_MOTOR_Ls_d + 1e-9))) ||
        ((float_t)USER_CTRL_FREQ_Hz <= (USER_MOTOR_Rs / (USER_MOTOR_Ls_q + 1e-9))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
        }
    }

  if(((float_t)USER_EST_FREQ_Hz < USER_FORCE_ANGLE_FREQ_Hz) ||
    ((float_t)USER_EST_FREQ_Hz < USER_VOLTAGE_FILTER_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < USER_DCBUS_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < USER_FLUX_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < USER_DIRECTION_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < USER_SPEED_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < 0.2))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estFreq_Hz_Low);
    }

  if(USER_R_OVER_L_EST_FREQ_Hz > USER_IQ_FULL_SCALE_FREQ_Hz)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_RoverL_estFreq_Hz_High);
    }

  if(((float_t)USER_TRAJ_FREQ_Hz < 1.0) ||
    ((float_t)USER_TRAJ_FREQ_Hz < USER_MAX_ACCEL_Hzps / USER_IQ_FULL_SCALE_FREQ_Hz) ||
    ((float_t)USER_TRAJ_FREQ_Hz < USER_MAX_ACCEL_EST_Hzps / USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_trajFreq_Hz_Low);
    }

  if(USER_MAX_NEGATIVE_ID_REF_CURRENT_A > 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_High);
    }

  if(USER_MAX_NEGATIVE_ID_REF_CURRENT_A < (-USER_MOTOR_MAX_CURRENT))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_Low);
    }

  // Only for debug testing, only be commented
//  USER_setErrorCode(pUserParams, USER_ErrorCode_NoError);

  return;
} // end of USER_checkForErrors() function
#endif


USER_ErrorCode_e USER_getErrorCode(USER_Params *pUserParams)
{
  return(pUserParams->errorCode);
} // end of USER_getErrorCode() function


void USER_setErrorCode(USER_Params *pUserParams,const USER_ErrorCode_e errorCode)
{
  pUserParams->errorCode = errorCode;

  return;
} // end of USER_setErrorCode() function


void USER_softwareUpdate1p6(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t fullScaleInductance = USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A*USER_VOLTAGE_FILTER_POLE_rps);
  float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(obj->estHandle));
  int_least8_t lShift = ceil(log(obj->motorParams.Ls_d_H/(Ls_coarse_max*fullScaleInductance))/log(2.0));
  uint_least8_t Ls_qFmt = 30 - lShift;
  float_t L_max = fullScaleInductance * pow(2.0,lShift);
  _iq Ls_d_pu = _IQ30(obj->motorParams.Ls_d_H / L_max);
  _iq Ls_q_pu = _IQ30(obj->motorParams.Ls_q_H / L_max);


  // store the results
  EST_setLs_d_pu(obj->estHandle,Ls_d_pu);
  EST_setLs_q_pu(obj->estHandle,Ls_q_pu);
  EST_setLs_qFmt(obj->estHandle,Ls_qFmt);

  return;
} // end of softwareUpdate1p6() function


#ifndef NO_CTRL
void USER_calcPIgains(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t fullScaleCurrent = USER_IQ_FULL_SCALE_CURRENT_A;
  float_t fullScaleVoltage = USER_IQ_FULL_SCALE_VOLTAGE_V;
  float_t ctrlPeriod_sec = CTRL_getCtrlPeriod_sec(handle);
  float_t Ls_d;
  float_t Ls_q;
  float_t Rs;
  float_t RoverLs_d;
  float_t RoverLs_q;
  _iq Kp_Id;
  _iq Ki_Id;
  _iq Kp_Iq;
  _iq Ki_Iq;
  _iq Kd;

#ifdef __TMS320C28XX_FPU32__
  int32_t tmp;

  // when calling EST_ functions that return a float, and fpu32 is enabled, an integer is needed as a return
  // so that the compiler reads the returned value from the accumulator instead of fpu32 registers
  tmp = EST_getLs_d_H(obj->estHandle);
  Ls_d = *((float_t *)&tmp);

  tmp = EST_getLs_q_H(obj->estHandle);
  Ls_q = *((float_t *)&tmp);

  tmp = EST_getRs_Ohm(obj->estHandle);
  Rs = *((float_t *)&tmp);
#else
  Ls_d = EST_getLs_d_H(obj->estHandle);

  Ls_q = EST_getLs_q_H(obj->estHandle);

  Rs = EST_getRs_Ohm(obj->estHandle);
#endif

  RoverLs_d = Rs/Ls_d;
  Kp_Id = _IQ((0.25*Ls_d*fullScaleCurrent)/(ctrlPeriod_sec*fullScaleVoltage));
  Ki_Id = _IQ(RoverLs_d*ctrlPeriod_sec);

  RoverLs_q = Rs/Ls_q;
  Kp_Iq = _IQ((0.25*Ls_q*fullScaleCurrent)/(ctrlPeriod_sec*fullScaleVoltage));
  Ki_Iq = _IQ(RoverLs_q*ctrlPeriod_sec);

  Kd = _IQ(0.0);

  // set the Id controller gains
  PID_setKi(obj->pidHandle_Id,Ki_Id);
  CTRL_setGains(handle,CTRL_Type_PID_Id,Kp_Id,Ki_Id,Kd);

  // set the Iq controller gains
  PID_setKi(obj->pidHandle_Iq,Ki_Iq);
  CTRL_setGains(handle,CTRL_Type_PID_Iq,Kp_Iq,Ki_Iq,Kd);

  return;
} // end of calcPIgains() function
#endif


//! \brief     Computes the scale factor needed to convert from torque created by Ld, Lq, Id and Iq, from per unit to Nm
//!
_iq USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(void)
{
  float_t FullScaleInductance = (USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A*USER_VOLTAGE_FILTER_POLE_rps));
  float_t FullScaleCurrent = (USER_IQ_FULL_SCALE_CURRENT_A);
#ifdef SUPPORT_USER_VARIABLE 
  float_t lShift = ceil(log(gUserParams.motor_Ls_d/(0.7*FullScaleInductance))/log(2.0));

  return(_IQ(FullScaleInductance*FullScaleCurrent*FullScaleCurrent*gUserParams.motor_numPolePairs*1.5*pow(2.0,lShift)));
#else
  float_t lShift = ceil(log(USER_MOTOR_Ls_d/(0.7*FullScaleInductance))/log(2.0));

  return(_IQ(FullScaleInductance*FullScaleCurrent*FullScaleCurrent*USER_MOTOR_NUM_POLE_PAIRS*1.5*pow(2.0,lShift)));
#endif  
} // end of USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf() function


//! \brief     Computes the scale factor needed to convert from torque created by flux and Iq, from per unit to Nm
//!
_iq USER_computeTorque_Flux_Iq_pu_to_Nm_sf(void)
{
#ifdef SUPPORT_USER_VARIABLE
  float_t FullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)gUserParams.estFreq_Hz);
  float_t FullScaleCurrent = (USER_IQ_FULL_SCALE_CURRENT_A);
  float_t maxFlux = (gUserParams.motor_ratedFlux*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
  float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));

  return(_IQ(FullScaleFlux/(2.0*MATH_PI)*FullScaleCurrent*gUserParams.motor_numPolePairs*1.5*pow(2.0,lShift)));
#else
  float_t FullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)USER_EST_FREQ_Hz);
  float_t FullScaleCurrent = (USER_IQ_FULL_SCALE_CURRENT_A);
  float_t maxFlux = (USER_MOTOR_RATED_FLUX*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
  float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));

  return(_IQ(FullScaleFlux/(2.0*MATH_PI)*FullScaleCurrent*USER_MOTOR_NUM_POLE_PAIRS*1.5*pow(2.0,lShift)));
#endif  
} // end of USER_computeTorque_Flux_Iq_pu_to_Nm_sf() function


//! \brief     Computes the scale factor needed to convert from per unit to Wb
//!
_iq USER_computeFlux_pu_to_Wb_sf(void)
{
#ifdef SUPPORT_USER_VARIABLE
  float_t FullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)gUserParams.estFreq_Hz);
  float_t maxFlux = (gUserParams.motor_ratedFlux*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
#else
  float_t FullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)USER_EST_FREQ_Hz);
  float_t maxFlux = (USER_MOTOR_RATED_FLUX*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
#endif  
  float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));

  return(_IQ(FullScaleFlux/(2.0*MATH_PI)*pow(2.0,lShift)));
} // end of USER_computeFlux_pu_to_Wb_sf() function


//! \brief     Computes the scale factor needed to convert from per unit to V/Hz
//!
_iq USER_computeFlux_pu_to_VpHz_sf(void)
{
#ifdef SUPPORT_USER_VARIABLE
  float_t FullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)gUserParams.estFreq_Hz);
  float_t maxFlux = (gUserParams.motor_ratedFlux*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
#else
  float_t FullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)USER_EST_FREQ_Hz);
  float_t maxFlux = (USER_MOTOR_RATED_FLUX*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
#endif  
  float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));

  return(_IQ(FullScaleFlux*pow(2.0,lShift)));
} // end of USER_computeFlux_pu_to_VpHz_sf() function


//! \brief     Computes Flux in Wb or V/Hz depending on the scale factor sent as parameter
//!
_iq USER_computeFlux(CTRL_Handle handle, const _iq sf)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(_IQmpy(EST_getFlux_pu(obj->estHandle),sf));
} // end of USER_computeFlux() function


//! \brief     Computes Torque in Nm
//!
_iq USER_computeTorque_Nm(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  _iq Flux_pu = EST_getFlux_pu(obj->estHandle);
  _iq Id_pu = PID_getFbackValue(obj->pidHandle_Id);
  _iq Iq_pu = PID_getFbackValue(obj->pidHandle_Iq);
  _iq Ld_minus_Lq_pu = _IQ30toIQ(EST_getLs_d_pu(obj->estHandle)-EST_getLs_q_pu(obj->estHandle));
  _iq Torque_Flux_Iq_Nm = _IQmpy(_IQmpy(Flux_pu,Iq_pu),torque_Flux_sf);
  _iq Torque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(Ld_minus_Lq_pu,Id_pu),Iq_pu),torque_Ls_sf);
  _iq Torque_Nm = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

  return(Torque_Nm);
} // end of USER_computeTorque_Nm() function


//! \brief     Computes Torque in Nm
//!
_iq USER_computeTorque_lbin(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  _iq Flux_pu = EST_getFlux_pu(obj->estHandle);
  _iq Id_pu = PID_getFbackValue(obj->pidHandle_Id);
  _iq Iq_pu = PID_getFbackValue(obj->pidHandle_Iq);
  _iq Ld_minus_Lq_pu = _IQ30toIQ(EST_getLs_d_pu(obj->estHandle)-EST_getLs_q_pu(obj->estHandle));
  _iq Torque_Flux_Iq_Nm = _IQmpy(_IQmpy(Flux_pu,Iq_pu),torque_Flux_sf);
  _iq Torque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(Ld_minus_Lq_pu,Id_pu),Iq_pu),torque_Ls_sf);
  _iq Torque_Nm = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

  return(_IQmpy(Torque_Nm, _IQ(MATH_Nm_TO_lbin_SF)));
} // end of USER_computeTorque_lbin() function


// end of file

