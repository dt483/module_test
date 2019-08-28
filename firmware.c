

//#include <common/include/dispatcher.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdarg.h>

/* local includes*/
#include <common/include/mutex.h>
#include <common/include/queue.h>
#include "module-armcore.h"
#include "module-armsc.h"
#include "module-dit.h"
#include "module-dmac.h"
#include "module-extirc.h"
#include "module-gpio.h"
#include "module-spi.h"
#include "module-uart.h"
#include "module-vic.h"
#include "module-nmcload.h"

/* module api functions */
//#include "module-serial.h"
//#include "serial_protocol_test.h"

#define ENABLE_TESTING

/* Tests includes*/
#ifdef ENABLE_TESTING

#endif
/******                  */
#include <external_dev_drivers/BGT/include/bgt24mtr11.h>
#include <external_dev_drivers/PLL/include/pll.h>

extern void* pll_handle;

void main (void) __attribute__ (( section (".text.main") ));
void main (void)
{

	module_UART_controller_t * console_uart = module_UART_getInstance(UART_0); //defined by module_base
	module_UART_defaultInit (console_uart); //setting 115200 8n1


	module_DIT_controller_t * timer1 = module_DIT_getInstance(TIMER_1);

	//for SPI output to X19 X20 on mc7601
	//module_GPIO_SetDirection(module_GPIO7, module_GPIO_DIRECTION_OUTPUT);
	//module_GPIO_SetHigh(module_GPIO7);
	module_SPI_init();
	module_SPI_setDividers(64, 64);

	module_GPIO_SetDirection(PLL_MOD_PIN, module_GPIO_DIRECTION_INPUT);
	module_GPIO_SetDirection(PLL_TRIG1_PIN, module_GPIO_DIRECTION_OUTPUT);
	module_GPIO_SetDirection(BGT_POWER_EN_GPIO, module_GPIO_DIRECTION_OUTPUT);

	bgt_power_down();
	pll_release_ramp_trigger();
	// pll_soft_reset( pll_handle);
	//LMX249x_setPowerState(pll_handle, LMX249x_POWER_DOWN);


	module_DIT_Wait(timer1, 1000, DIT_timeExp_microSeconds);

	//===================== BGT / PLL Power-up ===================
	bgt_power_up();
	//LMX249x_setPowerState(pll_handle, LMX249x_POWER_UP);
	module_DIT_Wait(timer1, 100, DIT_timeExp_microSeconds);
	bgt_lowest_power_with_q2_disable();
	module_DIT_Wait(timer1, 50, DIT_timeExp_microSeconds);
	//pll_enable();
	module_DIT_Wait(timer1, 100, DIT_timeExp_microSeconds);
	bgt_init();
	//---------------------  PLL init -----------------------
	pll_init(pll_handle);
	module_DIT_Wait(timer1, 100, DIT_timeExp_microSeconds);
	pll_update_configuration(pll_handle);
	module_DIT_Wait(timer1, 100, DIT_timeExp_microSeconds);


	bgt_start_tx();
	//pll_trigger_ramp();

#define TIME_BTWN_FRAMES_us 30000
	while( 1 )
    {



		//while ( module_GPIO_GetValue(PLL_MOD_PIN) ) {}


	//	pll_update_configuration(pll_handle);
		pll_enable_ramps(pll_handle);

		module_DIT_Wait(timer1, TIME_BTWN_FRAMES_us, DIT_timeExp_microSeconds);
		pll_trigger_ramp();

		module_DIT_Wait(timer1, CHIRP_TIME_us*NUM_OF_CHIRPS, DIT_timeExp_microSeconds);

		pll_release_ramp_trigger();


		//module_DIT_Wait(timer1, 1000, DIT_timeExp_microSeconds);


    }

}

