/**
    \file config.h
    \author Assad Ali
    \brief Configuration file for Distance2Go project.
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

#ifndef CONFIG_GAURD_H
#define CONFIG_GAURD_H

/*
==============================================================================
   1. INCLUDE
==============================================================================
*/

#include <stdint.h>

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

//================================ GENERAL CONFIG ================================//

//#define FW_MODULATION_TYPE 		(1U)		/**< FW Modulation Type ==>  0: Doppler;  1: FMCW  [0 - 1] */

//#define	NUM_OF_CHIRPS			(16U)		/**< Valid range of chirps in relation to the SAMPLES_PER_CHIRP ==> [1 - 64], Default: 8 */

//#define	SAMPLES_PER_CHIRP 		(64U)		/**< Size of IQ raw ADC buffer ==> [32,64,128,256], Default: 128  */

//#define FRAME_PERIOD_ms 		(100U)    	/**< Time period of one frame to capture data [5 - Infinity] */

//================================ BGT CONFIG ====================================//

#define DUTY_CYCLE_ENABLE		(1U)		/**< Enable[1] and disable[0] for duty cycling of Distance2Go via BGT & PLL On/Off */

#define BGT_TX_POWER			(7U)		/**< BGT TX Power levels [1 - 7], Minimum = 1 & Maximum = 7 */

#define LNA_GAIN_ENABLE			(1U)		/**< Enable and disable LNA Gain in BGT TX [0 - 1] */

//================================ DSP CONFIG ====================================//

//#define	RANGE_FFT_SIZE 			(256U)		/**< FFT length for FMCW mode, with zero padding */

//#define	DOPPLER_FFT_SIZE 		(32U)		/**< FFT length for FMCW mode, with zero padding */

//#define	FFT_INPUT_TYPE			(2U)        /**< FFT input: Default = complex input IQ; real input I = 0, real input Q = 1 */

//#define	NUM_OF_TARGETS			(5U)		/**< Maximum number of targets to be detected, [1 - 8] */

//#define RANGE_THRESH_TYPE		(0U)		/**< Constant Threshold = 0U, Adaptive Threshold = 1U */

//#define ADAPTIVE_THRESH_OFFSET	(50U)		/**< Adaptive Threshold offset, should be above noise floor */

//================================ FMCW CONFIG ====================================//

//#define BANDWIDTH_MHz 			(150U)    	/**< Bandwidth in MHz (0 - 200MHz) */

//#define CHIRP_TIME_us 			(100U)    	/**< Chirp time in microseconds (1000us - 3000us), additional ramp down time (200usec)
 //	 	 	 	 	 	 	 	 	 	 	 	 and steady state (300usec) time is added to it */

//#define MINIMUM_RANGE_cm 		(90U)    	/**< Used in FMCW to exclude targets below this distance (units in cm) */

//#define MAXIMUM_RANGE_cm 		(1000U)    	/**< Used in FMCW to exclude targets beyond this distance (units in cm) */

//#define RANGE_DETECTION_THRESHOLD (100U)   	/**< FFT spectrum threshold to detect a target in FMCW */

//================================ DOPPLER CONFIG ====================================//

//#define DOPPLER_SAMPLING_FREQ_Hz (20000U) 	/**< Sampling frequency in Hz for Doppler */

//#define MINIMUM_SPEED_kmh 		 (0U)    	/**< Used in Doppler to exclude targets below this speed (units in kmh) */

//#define MAXIMUM_SPEED_kmh 		 (4U)    	/**< Used in FMCW to exclude targets above this speed (units in kmh) */

//#define SPEED_DETECTION_THRESHOLD (50U)  	/**< FFT spectrum threshold to detect a target in Doppler */

#endif

/* --- End of File ------------------------------------------------ */

