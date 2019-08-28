/**
    \file: pll_lmx249x.c
    \author: Assad Ali, Thomas Finke
 	\brief: This file implements the function declarations for Texas Instrument's PLL named LMX249x
 */

/* ===========================================================================
 ** Copyright (C) 2017-2018 Infineon Technologies AG
 ** All rights reserved.
 ** ===========================================================================
 **
 ** ===========================================================================
 ** This document contains proprietary information of Infineon Technologies AG.
 ** Passing on and copying of this document, and communication of its contents
 ** is not permitted without Infineon's prior written authorization.
 ** ===========================================================================
 */

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <external_dev_drivers/PLL/include/pll.h>
#include "config.h"


/*
==============================================================================
   2. DATA
==============================================================================
*/
#define false 0
#define true 1

LMX249x_Object_s  	gLmx249x_pll;
void* pll_handle = &gLmx249x_pll;

volatile  double   g_chirp_time_us = CHIRP_TIME_us;

volatile  float   g_bandwidth_MHz = BANDWIDTH_MHz;

volatile  float  g_pll_lower_freq_MHz = PLL_LOWER_FREQUENCY_MHz;

volatile  float  g_pll_upper_freq_MHz = PLL_UPPER_FREQUENCY_MHz;

volatile  float  g_pll_base_freq_MHz = PLL_LOWER_FREQUENCY_MHz;


volatile  uint8_t    g_update_pll_configurations = false;

volatile  uint32_t   g_num_of_chirps = NUM_OF_CHIRPS;

//volatile  uint8_t 	 g_modulation_type = 0;


/*
==============================================================================
   3. API FUNCTIONS
==============================================================================
*/
//typedef void (*sendSPIFunction)(const uint8_t* data_ptr, uint8_t num_of_bytes);

void pll_transmitData(uint8_t* data_ptr, uint32_t num_of_bytes)
{
	//_debug("Exchanging %i bytes of data with device %i", num_of_bytes ,PLL_SPI_DEVICE_CS);
	module_SPI_setDevice( PLL_SPI_DEVICE_CS);
	module_SPI_setMode(PLL_SPI_MODE);
	module_SPI_sendMulti(data_ptr, num_of_bytes);
}


void pll_init(void* pll_handle)
{
	LMX249x_Object_s* lmx249x_pll = (LMX249x_Object_s*)pll_handle;
	LMX249x_HardwareSetup_s s_sPLLHardwareSetup;

	/* send general configuration to PLL */
	/* --------------------------------- */
	s_sPLLHardwareSetup.dReferenceFreq         =  40.0;
	s_sPLLHardwareSetup.uReferenceDivider      = 1;
	s_sPLLHardwareSetup.eReferenceDoubler      = LMX249x_OSCIN_DOUBLER_OFF;
	s_sPLLHardwareSetup.eOscInMode             = LMX249x_OSCIN_SINGLE_ENDED;
	s_sPLLHardwareSetup.uExternalDivider       = 16;

	s_sPLLHardwareSetup.eTrig1PinFunction      = LMX249x_MUX_IN_TRIG1;
	s_sPLLHardwareSetup.eTrig1PinDriveMode     = LMX249x_PIN_FCN_INPUT;
	s_sPLLHardwareSetup.eTrig2PinFunction      = LMX249x_MUX_OUT_FLAG0_FROM_RAMP;
	s_sPLLHardwareSetup.eTrig2PinDriveMode     = LMX249x_PIN_FCN_PULLUPDN_OUT;
	s_sPLLHardwareSetup.eModPinFunction        = LMX249x_MUX_OUT_FLAG1_FROM_RAMP;
	s_sPLLHardwareSetup.eModPinDriveMode       = LMX249x_PIN_FCN_PULLUPDN_OUT;
	s_sPLLHardwareSetup.eMUXoutPinFunction     = LMX249x_MUX_OUT_READ_BACK;
	s_sPLLHardwareSetup.eMUXoutPinDriveMode    = LMX249x_PIN_FCN_PULLUPDN_OUT;
	s_sPLLHardwareSetup.eChargePumpPolarity    = LMX249x_CPPOL_NEGATIVE;

	s_sPLLHardwareSetup.uChargePumpCurrent     = 31;
	s_sPLLHardwareSetup.uChargePumpCurrentFS   = 0;
	s_sPLLHardwareSetup.uChargePumpThresholdLo = 6;
	s_sPLLHardwareSetup.uChargePumpThresholdHi = 42;
	s_sPLLHardwareSetup.eChargePumpPulseWidth  = LMX249x_CP_PULSE_860PS;
	s_sPLLHardwareSetup.eCycleSlipReduction    = LMX249x_CSR_DISABLED;
	s_sPLLHardwareSetup.uFastLockTimer         = 0;
	s_sPLLHardwareSetup.uLockDetectNumGoodEdge = 32;
	s_sPLLHardwareSetup.uLockDetectNumBadEdge  = 4;
	s_sPLLHardwareSetup.eLockDetectWindow      = LMX249x_DLD_TOL_10NS;

	LMX249x_init(lmx249x_pll, &s_sPLLHardwareSetup, pll_transmitData);

	LMX249x_setPowerState(lmx249x_pll, LMX249x_POWER_CE);
}

void pll_configure_ramps(void* pll_handle)
{
	// setup data configuration data for PLL chip
	LMX249x_Object_s*	lmx249x_pll = (LMX249x_Object_s*)pll_handle;

	LMX249x_RampGlobal_s pllGlobalRampSetup;

	LMX249x_RampSection_s pllRampSections[3];

	pllGlobalRampSetup.dBaseFrequency   = (double)g_pll_base_freq_MHz;    /*!< base frequency */

	pllGlobalRampSetup.dMinFrequency 	= (double)(g_pll_lower_freq_MHz - (float)PLL_RAMP_GUARD_FREQ_MHz);		/*!< Minimum frequency that can't be underrun by a ramp */
	pllGlobalRampSetup.dMaxFrequency 	= (double)(g_pll_upper_freq_MHz + (float)PLL_RAMP_GUARD_FREQ_MHz);		/*!< Maximum frequency that can't be overrun by a ramp */
	pllGlobalRampSetup.dComp0Freq    	= (double)g_pll_lower_freq_MHz;						/*!< Frequency that is used by Comparator 0 to compare the ramp with. */
	pllGlobalRampSetup.dComp1Freq    	= (double)g_pll_upper_freq_MHz;						/*!< Frequency that is used by Comparator 0 to compare the ramp with. */
	pllGlobalRampSetup.dDeviationFrequency = 0;  			/*!< FSK deviation frequency */

	//pllGlobalRampSetup.uNumRamps 		= 3;							/*!< Number of ramps to do before ramp is disabled (set to 0 for infinite number of ramps) */
	pllGlobalRampSetup.uNumRamps 		= NUM_OF_CHIRPS + 1;

	pllGlobalRampSetup.eTriggerA 		= LMX249x_RAMP_TRIG_NEVER_TRIGGERS;					/*!< Define the source for Trigger A */
	pllGlobalRampSetup.eTriggerB 		= LMX249x_RAMP_TRIG_TRIG1_TERMINAL_RISING_EDGE;		/*!< Define the source for Trigger B */
	pllGlobalRampSetup.eTriggerC 		= LMX249x_RAMP_TRIG_NEVER_TRIGGERS;          		/*!< Define the source for Trigger C */

	pllGlobalRampSetup.eRampClock 		= LMX249x_RAMP_CLOCK_INTERNAL;         				/*!< determines if internal or external clock for ramp timing is used */
	pllGlobalRampSetup.eModulation 		= LMX249x_RAMP_MODULATION_FM;        				/*!< Kind of modulation */
	pllGlobalRampSetup.eAutoOff 		= LMX249x_RAMP_AUTO_OFF_ENABLE;           			/*!< Defines, if the Ramp should be turned off after a certain amount of ramps */
	pllGlobalRampSetup.eRampCountTrigger= LMX249x_RAMP_COUNT_INCREMENT_SEGMENT_TRANSITION;	/*!< Defines the increment trigger for the ramp counter */
	pllGlobalRampSetup.eDevTrigger 		= LMX249x_TRIGGER_A;						/*!< Defines the deviation trigger for FSK operation */

	pllGlobalRampSetup.eDitherMode 		= LMX249x_FRAC_DITHER_DISABLED;
	pllGlobalRampSetup.eFracOrder 		= LMX249x_FRAC_ORD_SECOND;

	/* calculate real frequency shift based on PLL clock */
	double bandwidth_MHz = LMX249x_getRealFrequencyShift(lmx249x_pll, g_bandwidth_MHz, g_chirp_time_us);

	/* reset ramp  */
	pllRampSections[0].dFreqShift   = 0.0;
	pllRampSections[0].eFastlock    = LMX249x_RAMP_FASTLOCK_DISABLED;
	pllRampSections[0].dTramp       = 0;
	pllRampSections[0].eFlag        = LMX249x_RAMP_FLAG_CLR_BOTH;
	pllRampSections[0].eReset       = LMX249x_RAMP_RST_ENABLE;
	pllRampSections[0].eNextTrig    = LMX249x_RAMP_NEXT_TRIG_TRIGB;
	pllRampSections[0].uNext        = 1;
	pllRampSections[0].eComparators = LMX249x_RAMP_NO_COMPARATOR;

	/* ramp down to lower frequency */
/*	pllRampSections[1].dFreqShift   = -bandwidth_MHz;
	pllRampSections[1].eFastlock    = LMX249x_RAMP_FASTLOCK_DISABLED;
	pllRampSections[1].dTramp       = (double)RAMP_DOWM_TIME_usec;
	pllRampSections[1].eFlag        = LMX249x_RAMP_FLAG_CLR_BOTH;
	pllRampSections[1].eReset       = LMX249x_RAMP_RST_DISABLE;
	pllRampSections[1].eNextTrig    = LMX249x_RAMP_NEXT_TRIG_RAMPX_LEN;
	pllRampSections[1].uNext        = 2;
	pllRampSections[1].eComparators = LMX249x_RAMP_NO_COMPARATOR;*/

	/* wait time between ramps */
/*	pllRampSections[2].dFreqShift   = 0;
	pllRampSections[2].eFastlock    = LMX249x_RAMP_FASTLOCK_DISABLED;
	pllRampSections[2].dTramp       = (double)PLL_STEADY_STATE_usec;
	pllRampSections[2].eFlag        = LMX249x_RAMP_FLAG_CLR_BOTH;
	pllRampSections[2].eReset       = LMX249x_RAMP_RST_DISABLE;
	pllRampSections[2].eNextTrig    = LMX249x_RAMP_NEXT_TRIG_RAMPX_LEN;
	pllRampSections[2].uNext        = 3;
	pllRampSections[2].eComparators = LMX249x_RAMP_NO_COMPARATOR;*/

	/* ramp up and trigger ADC sampling*/
	pllRampSections[1].dFreqShift   = bandwidth_MHz;
	pllRampSections[1].eFastlock    = LMX249x_RAMP_FASTLOCK_DISABLED;
	pllRampSections[1].dTramp       = (double)g_chirp_time_us;
	pllRampSections[1].eFlag        = LMX249x_RAMP_FLAG_SET_BOTH;
	pllRampSections[1].eReset       = LMX249x_RAMP_RST_ENABLE;
	pllRampSections[1].eNextTrig    = LMX249x_RAMP_NEXT_TRIG_RAMPX_LEN;
	pllRampSections[1].uNext        = 2;
	pllRampSections[1].eComparators = LMX249x_RAMP_NO_COMPARATOR;

	/* Clear ADC sampling flags*/
	pllRampSections[2].dFreqShift   = bandwidth_MHz;
	pllRampSections[2].eFastlock    = LMX249x_RAMP_FASTLOCK_DISABLED;
	pllRampSections[2].dTramp       = (double)g_chirp_time_us;
	pllRampSections[2].eFlag        = LMX249x_RAMP_FLAG_CLR_BOTH;
	pllRampSections[2].eReset       = LMX249x_RAMP_RST_ENABLE;
	pllRampSections[2].eNextTrig    = LMX249x_RAMP_NEXT_TRIG_RAMPX_LEN;
	pllRampSections[2].uNext        = 1;
	pllRampSections[2].eComparators = LMX249x_RAMP_NO_COMPARATOR;

	LMX249x_configureRamps(lmx249x_pll, &pllGlobalRampSetup, pllRampSections, 3);
}

void pll_enable_ramps(void* pll_handle)
{
	LMX249x_Object_s* lmx249x_pll = (LMX249x_Object_s*)pll_handle;
	LMX249x_enableRamps(lmx249x_pll, true);
}

void pll_disable_ramps(void* pll_handle)
{
	LMX249x_Object_s* lmx249x_pll = (LMX249x_Object_s*)pll_handle;
	LMX249x_enableRamps(lmx249x_pll, false);
}

void pll_trigger_ramp(void)
{
	// wait until PLL is ready to generate the next chirp
	/*TODO: назначить GPIO для PLL_MOD*/
	//while (DIGITAL_IO_GetInput(&DIGITAL_IO_PLL_MOD) != 0);

	// now start the chirp
	/*TODO: назначить GPIO для PLL_TRIG1*/
	//DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_PLL_TRIG1);
	module_GPIO_SetHigh(PLL_TRIG1_PIN);
}

void pll_release_ramp_trigger(void)
{
	/*TODO: назначить GPIO для PLL_TRIG1*/
//	DIGITAL_IO_SetOutputLow(&DIGITAL_IO_PLL_TRIG1);
	module_GPIO_SetLow(PLL_TRIG1_PIN);
}

void pll_trigger_deviation(void)
{
	//DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_PLL_TRIG2);
}

void pll_release_deviation_trigger(void)
{
	//DIGITAL_IO_SetOutputLow(&DIGITAL_IO_PLL_TRIG2);
}

void pll_enable(void)
{
	/*TODO: назначить GPIO для PLL_CE*/
	//DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_PLL_CE);

}

void pll_disable(void)
{
	/*TODO: назначить GPIO для PLL_CE*/
//	DIGITAL_IO_SetOutputLow(&DIGITAL_IO_PLL_CE);
}


void pll_update_configuration(void* pll_handle)
{
	pll_configure_ramps(pll_handle);

	//if (g_modulation_type == MODULATION_FMCW)
	//{
		pll_enable_ramps(pll_handle);
	//}
	//else
	//{
		//LMX249x_setFrequency(pll_handle, (double)g_pll_base_freq_MHz, LMX249x_FRAC_ORD_THIRD, LMX249x_FRAC_DITHER_WEAK);
	//}

	g_update_pll_configurations = false;
}

void pll_set_update_config_flag(uint8_t flag)
{
	if (flag == 0)
	{
		g_update_pll_configurations = false;
	}
	else
	{
		g_update_pll_configurations = true;
	}
}

