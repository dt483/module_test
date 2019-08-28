/**
    \file bgt24mtr11.h
    \author Assad Ali
  	\brief File includes definitions for SPI Data Bits in the control register of the BGT24. Ref. BGT24MTR11 Data Sheet Rev. 3.1

*/

/*THERE iS SOME CHANGES IN THIS FILE */

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

#ifndef BGT24MTR11_GUARD_H
#define BGT24MTR11_GUARD_H

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdint.h>

#include "module-spi.h"
#include "module-gpio.h"

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

#define BGT24_BASE_CONF		(0x1848u)		/**< BGT24MTR11 default Configuration : 0x1848 = 0001 1000  0100 1000 */

#define BGT24_POWER_CONF	(0x1847u)		/**< BGT24MTR12 default Configuration : 0x1847 = 0001 1000  0100 0111 */

// SPI control word settings
#define BGT24_ENA_MASK     	(0x7FFFu)   	/**< LNA Gain increase; 0 = increase LNA gain */
#define BGT24_DIS_MASK     	(0x8000u)   	/**< LNA Gain reduction; 1 = reduce LNA gain */

#define BGT24_ENA_PA_MASK   (0xEFFFu)  		/**< ENABLE PA - 0 = turn on TX power */
#define BGT24_DIS_PA_MASK 	(0x1000u)  		/**< DISABLE PA - 1 = turn off TX power */

#define BGT24_DIS_DIV64K_MASK (0x0040u)  	/**< 1 = Disable 64K divider for Q2 */

#define BGT24_DIS_DIV16_MASK (0x0020u)   	/**< 1 = Disable 16 divider for Q2 */

#define BGT24_PC2_BUF_MASK  (0x0010u)		/**< PC2_BUF 1 = High LO buffer output power */
#define BGT24_PC1_BUF_MASK 	(0x0008u)		/**< PC1_BUF 1 = High TX buffer output power */

#define BGT24_PC_PA_MASK  	(0x0007u)		/**< PC0_PA, PC1_PA, PC2_PA,  1 = reduce TX power */

// AMUX settings

#define BGT24_AMUX_0   ((0x0001u << 7U))	/**< BGT AUMUX_0 Mask */
#define BGT24_AMUX_1   ((0x0001u << 8U))	/**< BGT AUMUX_1 Mask */
#define BGT24_AMUX_2   ((0x0001u << 11U))	/**< BGT AUMUX_2 Mask */

#define BGT24_AMUX_VOUT_TX   (0xF67Fu)  	/**< BGT AUMUX_VOUT Mask */

// TX PA settings
#define  BGT24_PC_PA_0  (0x0000u)    		/**< Max Tx power, no reduction */
#define  BGT24_PC_PA_1  (0x0001u)	  		/**< Max Tx power, no reduction */
#define  BGT24_PC_PA_2  (0x0002u)			/**< Reduction by 0.8dBm */
#define  BGT24_PC_PA_3  (0x0003u)			/**< Reduction by 1.4dBm */
#define  BGT24_PC_PA_4  (0x0004u)			/**< Reduction by 2.5dBm */
#define  BGT24_PC_PA_5  (0x0005u)			/**< Reduction by 4dBm */
#define  BGT24_PC_PA_6  (0x0006u)			/**< Reduction by 6dBm */
#define  BGT24_PC_PA_7  (0x0007u)			/**< Reduction by 9dBm */

//================================ BGT CONFIG ====================================//

#define DUTY_CYCLE_ENABLE		(1U)		/**< Enable[1] and disable[0] for duty cycling of Distance2Go via BGT & PLL On/Off */

#define BGT_TX_POWER			(7U)		/**< BGT TX Power levels [1 - 7], Minimum = 1 & Maximum = 7 */

#define LNA_GAIN_ENABLE			(1U)		/**< Enable and disable LNA Gain in BGT TX [0 - 1] */

//================================ HARDWARE DEPENDED DEFINES ====================//
//#define MC7601 //TODO: replace to top config

#ifdef MC7601
	#define BGT_SPI_DEVICE_CS		module_GPIO_SPI_CS5
	#define BGT_SPI_MODE			SPI_MODE_1
	#define BGT_POWER_EN_GPIO		module_GPIO5
#else
	#define BGT_SPI_DEVICE_CS		module_GPIO_SPI_CS1
	#define BGT_SPI_MODE			SPI_MODE_1
	#define BGT_POWER_EN_GPIO		module_GPIO9
#endif


/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 *
 * \brief This enum defines possible power levels of BGT Tx power amplifier. Use type BGT_Power_t for this enum.
 *
 * @{
 */
typedef enum
{
	TX_MIN 		= 1U,		/**< Reduction by 9dBm */
	TX_LEVEL_1 	= 2U,		/**< Reduction by 6dBm */
	TX_LEVEL_2 	= 3U,		/**< Reduction by 4dBm */
	TX_MID 		= 4U,		/**< Reduction by 2.5dBm */
	TX_LEVEL_4 	= 5U,		/**< Reduction by 1.4dBm */
	TX_LEVEL_5 	= 6U,		/**< Reduction by 0.8dBm */
	TX_MAX 		= 7U		/**< No Reduction, Max Tx Power */

} BGT_Power_t;

/** @} */

/**
 *
 * \brief This enum defines possible power levels of BGT Tx power amplifier. Use type BGT_Power_t for this enum.
 *
 * @{
 */
typedef struct
{
	uint8_t		spi_gs;               	/**< LNA gain reduction */
	uint8_t     spi_amux2;            	/**< Analog multiplexer control bit 2 */
	uint8_t     spi_disable_pwr_amp;    /**< Disable power amplifier */
	uint8_t     spi_amux1;				/**< Analog multiplexer control bit 1 */
	uint8_t     spi_amux0;  			/**< Analog multiplexer control bit 0 */
	uint8_t     spi_disable_div_64k;    /**< Disable 64k divider */
	uint8_t     spi_disable_div_16;     /**< Disable 16 divider */
	uint8_t     spi_pc2_buf;			/**< High LO buffer output power */
	uint8_t     spi_pc1_buf;			/**< High TX buffer output power */
	uint8_t     spi_pc2_pa; 			/**< TX power reduction bit 2 */
	uint8_t     spi_pc1_pa; 			/**< TX power reduction bit 1 */
	uint8_t     spi_pc0_pa;             /**< TX power reduction bit 0 */

} BGT_Spi_t;

/** @} */

/*
==============================================================================
   5. FUNCTION PROTOTYPES
==============================================================================
*/

/**
 * \brief This function initializes the BGT24MTR11 16-bit SPI shadow register and sent BGT settings defined in config.h.
 *
 * RX LNA enable or disable settings defined by \ref LNA_GAIN_ENABLE and Tx out put power level defined by \ref BGT_TX_POWER in config.h
 * are also set in this function.
 *
 * Bit description of 16-bit SPI shadow register in BGT is explained below;
 *  - 15		 GS LNA Gain reduction (low)
 *  - 14	 	 Not used (low)
 *  - 13	 	 Not used (low)
 *  - 12 		 DIS_PA TX power disabled (high)
 *  - 11 		 AMUX2 Analog multiplexer control bit 2 (high)
 *  - 10 		 Test bit, must be low otherwise malfunction (low)
 *  - 9 		 Test bit, must be low otherwise malfunction (low)
 *  - 8 		 AMUX1 Analog multiplexer control bit 1 (low)
 *  - 7 		 AMUX0 Analog multiplexer control bit 0 (low)
 *  - 6 		 DIS_DIV64k Disable 64k divider Q2 (low)
 *  - 5 		 DIS_DIV16 Disable 16 divider Q1 (low)
 *  - 4 		 PC2_BUF High LO buffer output power in �high� mode otherwise typ. 4dB reduced LO-output power (low)
 *  - 3 		 PC1_BUF High TX buffer output power (low)
 *  - 2 		 PC2_PA TX power reduction bit 2 (high)
 *  - 1 		 PC1_PA TX power reduction bit 1 (high)
 *  - 0 		 PC0_PA TX power reduction bit 0 (high)
 *
 */
void bgt_init(void);

/**
 * \brief This function enable the Power amplifier of TX of the BGT24MTR11 to start transmission.
 *
 * 12th bit of SPI shadow register in BGT24MTR11 is enabled (high)
 *
 */
void bgt_start_tx(void);

/**
 * \brief This function disable the Power amplifier of TX of the BGT24MTR11 to start transmission.
 *
 * 12th bit of SPI shadow register in BGT24MTR11 is disabled (low)
 *
 */
void bgt_stop_tx(void);

/**
 * \brief This function enable the GPIO to connect the switch between BGT24MTR11 and VCC.
 *
 * This function is used in duty cycling mode. Before data acquisition this should be enabled with at least 1 msec of delay for settling up BGT.
 *
 */
void bgt_power_up(void);

/**
 * \brief This function disable the GPIO to disconnect the switch between BGT24MTR11 and VCC.
 *
 * This function is used in duty cycling mode. After data acquisition this should be disabled.
 *
 */
void bgt_power_down(void);

/**
 * \brief This function is used to set the power level of Power amplifier of TX of the BGT24MTR11.
 *
 * Setting TX power level will make sense only if 12th-bit of BGT24MTR11 SPI register is enabled.
 *
 * \param[in]  power_level     Unsigned 8-bit integer, whose values are matched to the Power levels defined by enum \ref BGT_Power_t.
 *
 */

void bgt_set_tx_power(uint8_t power_level);

/**
 * \brief This function is used to get the power level of Power amplifier of TX of the BGT24MTR11.
 *
 * \return     Unsigned 8-bit integer, whose values are matched to the Power levels defined by enum \ref BGT_Power_t.
 *
 */
uint8_t bgt_get_tx_power(void);

/**
 * \brief This function enables the Receiver LNA gain, which gives almost 6dB gain increase.
 *
 * By default this LNA gain is enabled/disabled based on config.h in the bgt_init() function. Changing this setting requires to do the calibration again.
 *
 * - 15th-bit		Set this bit to low to disable the LNA Gain reduction which means enabling the 6dB LNA gain.
 *
 */
void bgt_lna_gain_enable(void);

/**
 * \brief This function disables the Receiver LNA gain, which gives almost 6dB gain reduction.
 *
 * By default this LNA gain is enabled/disabled based on config.h in the bgt_init() function. Changing this setting requires to do the calibration again.
 *
 * - 15th-bit		Set this bit to high to enable the LNA Gain reduction which means reducing the 6dB LNA gain.
 *
 */
void bgt_lna_gain_disable(void);

/**
 * \brief This function returns the current status of the Receiver LNA gain, if enabled it returns true else false.
 *
 *\return	Unsigned 8-bit integer, 0 for false and non-zero for true case.
 */
uint8_t bgt_lna_gain_is_enable(void);

/**
 * \brief This function transmits the 16-bit SPI settings for BGT24MTR11 defined by the shadow register \ref g_bgt_conf.
 *
 */
void bgt_set_config(uint16_t);

/**
 * \brief This function returns the 16-bit SPI settings for BGT24MTR11 defined by the shadow register \ref g_bgt_conf.
 *
 *\return	Unsigned 16-bit integers containing current SPI settigns for BGT24MTR11.
 */
uint16_t bgt_get_config(void);


/**
 * \brief This function set the SPI settings for BGT24MTR11 to read the temperature output from AMUX pin.
 */
void bgt_ana_temp(void);

/**
 * \brief This function set the SPI settings for BGT24MTR11 to read the output TX power from AMUX pin.
 */
void bgt_ana_vout_tx(void);

/**
 * \brief This function set the SPI settings for BGT24MTR11 to read the reference TX power from AMUX pin.
 */
void bgt_ana_vref_tx(void);


void bgt_lowest_power_with_q2_disable(void);

#endif /* BGT24MTR11_H_ */

