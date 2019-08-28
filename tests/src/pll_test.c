/*
 * pll_test.c
 *
 *  Created on: 22 мар. 2019 г.
 *      Author: aleksey
 */

#include <external_dev_drivers/PLL/include/pll.h>
#include <module-spi.h>
#include <module-dit.h>

LMX249x_Object_s  	gLmx249x_pll;

void* pll_handle = &gLmx249x_pll;

static void setup_SPI()
{
	module_SPI_init();

	    module_SPI_setDividers(255,254);
		//module_SPI_setDividers(10,4);
		uint32_t freq =  module_SPI_getFrequency();
		uint32_t period = (1000000000/freq);
		_printf("Frequency set: %lu Hz, clk width: %lu ns", (uint32_t) freq, (uint32_t) period/2);
}

int pll_test()
{
	module_GPIO_ledOn();
	setup_SPI();
	module_GPIO_ledOff();
	pll_init(pll_handle);
	pll_configure_ramps(pll_handle);

	/*TODO: Добавить тесты, когда будет распаяна pll*/

	return 0;
}
