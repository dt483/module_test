/*
 * bgt_test.c
 *
 *  Created on: 22 мар. 2019 г.
 *      Author: aleksey
 */

#include <external_dev_drivers/BGT/include/bgt24mtr11.h>
#include <external_dev_drivers/PLL/include/pll.h>
#include "module-spi.h"
#include "module-dit.h"

static void setup_SPI()
{
	module_SPI_init();

	    module_SPI_setDividers(255,254);
		//module_SPI_setDividers(10,4);
		uint32_t freq =  module_SPI_getFrequency();
		uint32_t period = (1000000000/freq);
		_printf("Frequency set: %lu Hz, clk width: %lu ns", (uint32_t) freq, (uint32_t) period/2);
}

int bgt_test()
{
	_printf("******** BGT_TEST START*********");
	module_DIT_controller_t * timer1 = module_DIT_getInstance(TIMER_1);

	module_GPIO_ledOn();

	setup_SPI();



	bgt_power_up();

	module_DIT_Wait(timer1, 10, DIT_timeExp_millSeconds);

	module_GPIO_ledOff();
	bgt_lowest_power_with_q2_disable();
	module_GPIO_ledOn();

	//module_DIT_Wait(timer1, 10, DIT_timeExp_millSeconds);

	//pll_enable();

	module_DIT_Wait(timer1, 10, DIT_timeExp_millSeconds);
	bgt_init();

	module_GPIO_ledOff();



	bgt_start_tx();
	module_GPIO_ledOn();

	/*TODO: тестировании после распайки платы*/


	return 0;
}
