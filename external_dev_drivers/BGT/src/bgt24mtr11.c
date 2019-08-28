/**
    \file: bgt24mtr11.c
    \author: Assad Ali
  	\brief: File implements BGT base functions to configure the user settings via SPI Data Bits in the control register of the BGT24MTR11.

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

#include <external_dev_drivers/BGT/include/bgt24mtr11.h>



/*
==============================================================================
   2. DATA
==============================================================================
 */

static volatile  uint16_t   g_bgt_conf = BGT24_BASE_CONF;

volatile  uint16_t   bgt_ana_command = 2;

volatile  uint16_t   bgt_ana_out[3] = {0,0,0};

volatile  float  g_bgt_vout_tx_power = 0;

volatile  float  g_bgt_temperature_C = 0;



//static module_GPIO_SPICS_numDevice_t device_spi_CS =  (module_GPIO_SPICS_numDevice_t) BGT_SPI_DEVICE_CS;

//static module_SPI_mode_t device_spi_mode =  (module_SPI_mode_t) BGT_SPI_MODE;

//static module_GPIO_pin_t power_en_pin = (module_GPIO_pin_t) BGT_POWER_EN_GPIO;

/*
==============================================================================
   3. API FUNCTIONS
==============================================================================
 */

void bgt_transmitData(uint8_t* data_ptr, uint32_t num_of_bytes)
{
	//_debug("Exchanging %i bytes of data with device %i", num_of_bytes ,BGT_SPI_DEVICE_CS);
	module_SPI_setDevice( BGT_SPI_DEVICE_CS);
	module_SPI_setMode(BGT_SPI_MODE);
	module_SPI_sendMulti(data_ptr, num_of_bytes);

}

void bgt_init(void)
{
	g_bgt_conf = BGT24_BASE_CONF;

	if (LNA_GAIN_ENABLE)
	{
		g_bgt_conf &= BGT24_ENA_MASK;
	}
	else
	{
		g_bgt_conf |= BGT24_DIS_MASK;
	}

	bgt_set_tx_power((uint8_t)BGT_TX_POWER);		// configure BGT24

	bgt_ana_vref_tx();
}

//------------------------------------------//

/*
 * start the transmitter on the BGT24MTR11
 */
void bgt_start_tx(void)
{
	g_bgt_conf &= BGT24_ENA_PA_MASK;

	bgt_set_config(g_bgt_conf);

}  // end of BGTStartTX()

//------------------------------------------//

/*
 * stop the transmitter on the BGT24MTR11
 */
void bgt_stop_tx(void)
{
	g_bgt_conf |= BGT24_DIS_PA_MASK;

	bgt_set_config(g_bgt_conf);

}  // end of BGTStopTX()

//------------------------------------------//

void bgt_ana_temp(void)
{
	bgt_ana_command = 0;

	g_bgt_conf &= BGT24_AMUX_VOUT_TX;

	g_bgt_conf |= BGT24_AMUX_2;
}

//------------------------------------------//

void bgt_ana_vout_tx(void)
{
	bgt_ana_command = 1;

	g_bgt_conf &= BGT24_AMUX_VOUT_TX;
}

//------------------------------------------//

void bgt_ana_vref_tx(void)
{
	bgt_ana_command = 2;

	g_bgt_conf &= BGT24_AMUX_VOUT_TX;

	g_bgt_conf |= BGT24_AMUX_0;
}

//------------------------------------------//

/*
 * Enable the LNA on the BGT24MTR11
 */
void bgt_lna_gain_enable(void)
{
	g_bgt_conf &= BGT24_ENA_MASK;
}

//------------------------------------------//

/*
 * Disable the LNA on the BGT24MTR11
 */
void bgt_lna_gain_disable(void)
{
	g_bgt_conf |= BGT24_DIS_MASK;
}

//------------------------------------------//

/*
 * Status (Enable or Disable) of the LNA on the BGT24MTR12
 */
uint8_t bgt_lna_gain_is_enable(void)
{
	return (uint8_t)((g_bgt_conf & BGT24_DIS_MASK) == 0 ? 1 : 0);
}

//------------------------------------------//

/*
 * get 16-bit SPI values to the BGT24MTR11
 */
uint16_t bgt_get_config(void)
{
	return g_bgt_conf;
}

//------------------------------------------//

/*
 * get the BGT Tx power level from [0 - 7]
 */
uint8_t bgt_get_tx_power(void)
{
	return (uint8_t)(g_bgt_conf & 0x07);	// lower byte contains the Tx power levels info
}

//------------------------------------------//

/*
 * set the BGT Tx power level from [0 - 7]
 */
void bgt_set_tx_power(uint8_t power_level)
{
	g_bgt_conf &= 0xFFF8;		// clears the last 3-bits

	switch (power_level)
	{
	case 0:
		g_bgt_conf |= BGT24_PC_PA_7;	// Reduction by 9dBm
		break;

	case 1:
		g_bgt_conf |= BGT24_PC_PA_6;  // Reduction by 6dBm
		break;

	case 2:
		g_bgt_conf |= BGT24_PC_PA_5;  // Reduction by 4dBm
		break;

	case 3:
		g_bgt_conf |= BGT24_PC_PA_4;  // Reduction by 2.5dBm
		break;

	case 4:
		g_bgt_conf |= BGT24_PC_PA_3;  // Reduction by 1.4dBm
		break;

	case 5:
		g_bgt_conf |= BGT24_PC_PA_2;  // Reduction by 0.8dBm
		break;

	case 6:
		g_bgt_conf |= BGT24_PC_PA_1;  // Reduction by 0.4dBm
		break;

	case 7:
	default:
		g_bgt_conf |= BGT24_PC_PA_0;  // TX on with maximum power

	}

	bgt_ana_vout_tx();
}

//------------------------------------------//

void bgt_power_up(void)
{
	/* After turning on BGT, then We should keep CE pin high.
	 * CE pin is active low, so it should keep high until it is activated.
	 *  */

//	DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_SPI_M_CS_BGT24);
//	DIGITAL_IO_SetOutputLow(&DIGITAL_IO_BGT_POWER_ENABLE);
	module_GPIO_SetLow(BGT_POWER_EN_GPIO);
}

/*THERE iS SOME CHANGES IN THIS FILE */

//------------------------------------------//

void bgt_power_down(void)
{
	/* Before turning off BGT, we should keep SPI's signals low
	 * to avoid offset voltage at BGT's Vcc.
	 *
	 * If they are above 0.3V which is bias voltage inside BGT, then inside BGT turns on.
	 * It makes offset voltage at BGT's Vcc.
	 *  */

//	DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_BGT_POWER_ENABLE);
//	DIGITAL_IO_SetOutputLow(&DIGITAL_IO_SPI_M_CS_BGT24);
	module_GPIO_SetHigh(BGT_POWER_EN_GPIO);
//	TODO: setup GPIO for BGT

}

//------------------------------------------//
void bgt_lowest_power_with_q2_disable(void)
{
	bgt_set_config((uint16_t)BGT24_POWER_CONF);
}

/*
 * send 16-bit SPI values to the BGT
 */
void bgt_set_config(uint16_t SPIdata)
{
	uint16_t shuffled_data = 0;

	shuffled_data  = (SPIdata << 8);		// lower byte goes to upper byte position

	shuffled_data |= (SPIdata >> 8);		// upper byte goes to lower byte position


	//spi_transmit_data( (uint8_t *)&shuffled_data , 2U, (void*) &DIGITAL_IO_SPI_M_CS_BGT24);
	bgt_transmitData( (uint8_t *)&shuffled_data , 2U);
}

