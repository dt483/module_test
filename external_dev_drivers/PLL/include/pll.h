/**
    \file   pll.h
    \author Assad Ali, Thomas Finke
 	\brief  This file includes the function declarations for PLL operation
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

#ifndef PLL_GUARD_H_
#define PLL_GUARD_H_

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/
#include <external_dev_drivers/PLL/include/LMX249x.h>
#include "module-spi.h"
#include "module-gpio.h"


/*
==============================================================================
   2. GLOBAL DATA
==============================================================================
*/



#define		BANDWIDTH_MHz 			(200U)    	/**< Bandwidth in MHz (0 - 200MHz) */

#define		CHIRP_TIME_us 			(51.2)    	/**< Chirp time in microseconds */

#define		NUM_OF_CHIRPS			(256U)

#define 	PLL_LOWER_FREQUENCY_MHz 	(24025U)

#define 	PLL_UPPER_FREQUENCY_MHz 	(24225U)



//#define 	RAMP_DOWM_TIME_usec		(200U)		/**< Ramp down time for SawTooth Chirps in the PLL */

//#define 	PLL_STEADY_STATE_usec	(400U) 		/**< Steady state delay before starting next chirp in the PLL */

#define     PLL_RAMP_GUARD_FREQ_MHz	(0.5) 		/**< PLL lower and upper frequency guard band needed for identical multiple ramps*/


//================================ HARDWARE DEPENDED DEFINES ====================//
//#define MC7601 //TODO: replace to top config
#ifdef MC7601
	#define PLL_SPI_DEVICE_CS	module_GPIO_SPI_CS4
	#define PLL_SPI_MODE		SPI_MODE_3
	//#define PLL_POWER_EN_GPIO	module_GPIO0 //not used
	#define PLL_MOD_PIN			module_GPIO10
	#define PLL_TRIG1_PIN		module_GPIO11
	//#define PLL_TRIG2_PIN		module_GPIO0 //not used

#else
	#define PLL_SPI_DEVICE_CS	module_GPIO_SPI_CS2
	#define PLL_SPI_MODE		SPI_MODE_3
	#define PLL_CHIP_EN_GPIO	module_GPIO2
	#define PLL_MOD_PIN			module_GPIO11
	#define PLL_TRIG1_PIN		module_GPIO8
	#define PLL_TRIG2_PIN		//xint1
	#define PLL_MUXOUT_PIN		module_GPIO10
#endif

/*
==============================================================================
   3. GLOBAL DATA
==============================================================================
*/

extern void* pll_handle;	/**< Global handle for PLL, all PLL settings are controlled through this handle */

extern volatile  float   g_pll_base_freq_MHz;	/**< Global variable used to set the PLL base frequency for Doppler and FMCW modulation defined in config.h  */

extern volatile  float   g_pll_lower_freq_MHz; 	/**< Global variable used to set the PLL lower frequency for FMCW modulation */

extern volatile  float   g_pll_upper_freq_MHz; 	/**< Global variable used to set the PLL upper frequency for FMCW modulation */


typedef enum
{
   MODULATION_DOPPLER  = 0U,  	/**< Doppler Modulation for speed calculation */
   MODULATION_FMCW     = 1U		/**< FMCW Modulation for range calculation*/

}  Modulation_t;

/*
==============================================================================
   3. API FUNCTIONS
==============================================================================
*/

/**
 * \brief  This function sends the general configurations to PLL via SPI.
 *
 *  It is used after every power up of PLL in duty cycling mode.
 *
 * \param[in]	*pll_handle		Pointer to the handle for PLL, all PLL settings are controlled through this handle
 *
 */
void pll_init(void* pll_handle);

/**
 * \brief  This function configure the ramps generation pattern i.e. up chirp, down chirp and their order of generation.
 *
 *  It used bandwidth and chirp time parameters in to considerations to configure ramps.
 *
 * \param[in]	*pll_handle		Pointer to the handle for PLL, all PLL settings are controlled through this handle
 *
 */
void pll_configure_ramps(void* pll_handle);

/**
 * \brief  This function enables the ramps generation through SPI command to the PLL.
 *
 * Ramps are started by enabling the DIGITAL_IO_PLL_TRIG1 pin.
 *
 * \param[in]	*pll_handle		Pointer to the handle for PLL, all PLL settings are controlled through this handle
 *
 */
void pll_enable_ramps(void* pll_handle);

/**
 * \brief  This function disables the ramps generation through SPI command to the PLL.
 *
 * \param[in]	*pll_handle		Pointer to the handle for PLL, all PLL settings are controlled through this handle
 *
 */
void pll_disable_ramps(void* pll_handle);

/**
 * \brief  This function sets DIGITAL_IO_PLL_TRIG1 pin to high to trigger the start generation of ramps.
 */
void pll_trigger_ramp(void);

/**
 * \brief  This function sets DIGITAL_IO_PLL_TRIG1 pin to low to bring the trigger to down state.
 */
void pll_release_ramp_trigger(void);

/**
 * \brief  This function sets DIGITAL_IO_PLL_TRIG2 pin to high to trigger the frequency deviation used in FSK.
 */
void pll_trigger_deviation(void);

/**
 * \brief  This function sets DIGITAL_IO_PLL_TRIG2 pin to low to release the trigger.
 */
void pll_release_deviation_trigger(void);

/**
 * \brief  This function sets DIGITAL_IO_PLL_CE pin to high to enable the CE of PLL, used in power-up phase of duty cycling.
 */
void pll_enable(void);

/**
 * \brief  This function sets DIGITAL_IO_PLL_CE pin to low to disable the CE of PLL, used in power-down phase of duty cycling.
 */
void pll_disable(void);

/**
 * \brief  This function defines the protocol for updating the PLL configurations in case bandwidth or chirp-time or modulation changes.
 */

void pll_update_configuration(void* pll_handle);

/**
 * \brief  This function sets a flag to update the PLL configurations in case bandwidth or chirp-time or modulation changes.
 *
 * \param[in]	flag			'0' for false and non-zero for true
 *
 */
void pll_set_update_config_flag(uint8_t flag);

/**
 * \brief  This function reads out LMX2491 PLL registers.
 *
 * \param[in]	address			16-bit address value, pointing to the first register to start reading out
 * \param[in]	num_of_bytes	Number of consecutive bytes to be read out starting form the given address
 * \param[out]	*out_data_ptr	Pointer of type uint8_t to a memory location where output bytes to be written
 *
 */
void pll_read_register(uint16_t address, uint8_t* out_data_ptr, uint8_t num_of_bytes);

#endif /* PLL_GUARD_H_ */
