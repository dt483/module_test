/*

    Part of the Raspberry-Pi Bare Metal Tutorials
    Copyright (c) 2015, Brian Sidebotham
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

*/

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "lls/initnmc_mini_abs.h"
#include "lls/nmcload.h"
#include "module-dit.h"
#include "module-gpio.h"
#include "module-uart.h"
//#include "module_lib/xmodem-1k/xmodem.h"
#include "module-vic.h"
#include "module-arm.h"
#include "module-spi.h"
#include "module-extirc.h"
#include "module-dmac.h"

#include "h.h"

#define status_ok (0)
#define status_err (-1)

#define NM_ELF_LOCATION AMB1_START_ADDR

static int bounce = 0;
static int counter = 0;

static module_NMC_descriptor_t * NM_core_1 = &NMcore1_desc;

//typedef int (*command_fhandler_t)(uint32_t bufferAddr, uint32_t bufferLen);
int _nm_printf(uint32_t bufferAddr, uint32_t bufferLen)
{
	uint32_t * external_buffer = (uint32_t*) bufferAddr;
	uint32_t tmp;
	int length = bufferLen, i =0;
	_printf("** Message from NM%i: ",NM_core_1->boardNumber);
	for (i=1; i<=length; i++)
	{
		tmp = external_buffer[i];
		module_UART_send(console_uart, (char) tmp);
	}
	printf("\r\n");
	return 0;
}



void _timer1_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void _timer1_interrupt_handler (void)
{
	_printf ("TIMER 1 counter: %i", counter++);

	module_DIT_controller_t * timerInstance = module_DIT_getInstance (TIMER_1);
	module_DIT_clearInterrupt(timerInstance);

	module_VIC_finishHandling ();
}

void _timer2_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void _timer2_interrupt_handler (void)
{
	//_printf ("TIMER 2 ALARM");
	//module_GPIO_assertBlink();
	bounce = 0;

	module_DIT_controller_t * timerInstance = module_DIT_getInstance (TIMER_2);
	module_DIT_clearInterrupt(timerInstance);

	module_VIC_finishHandling();
}

void _extInt0_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void _extInt0_interrupt_handler (void)
{

	//module_GPIO_assertBlink();

	if (bounce == 0)
	{
		_printf ("EXT0 interrupt ALARM");
		module_GPIO_toggle(LED0_GPIO);
		//anti bounce
		module_DIT_controller_t * timer2 = module_DIT_getInstance(TIMER_2);
		module_DIT_EnableInterrupt(timer2);
		uint32_t ticks = module_DIT_countTicks(400,DIT_timeExp_millSeconds, DIT_divider_16);
		module_DIT_runOneShotCounter(timer2,DIT_divider_16, ticks);
		bounce = 1;
	}
	module_EXTIRC_clearInterrupt(EXTIRC_XINT0);
	module_VIC_finishHandling();
}

void _dmac_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void _dmac_interrupt_handler (void)
{
	module_DMAC_chNum_t irq_src  = module_DMAC_getInterruptSource();;
	//_printf ("DMAC TRANSFER COMPLETE:");
	//

	module_DMAC_channel_t* chRx =  module_DMAC_getChannelInstance( irq_src);


	module_DMAC_clearInterrupt(irq_src);
	module_VIC_finishHandling();
}


static int st_counter = 0;
//static int IRQ_counter = 0;


/*void printDouble(double v, int decimalDigits)
{
  int i = 1;
  int intPart, fractPart;
  for (;decimalDigits!=0; i*=10, decimalDigits--);
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);
  if(fractPart < 0) fractPart *= -1;
  _printf("%i.%i", intPart, fractPart);
}*/

//void  main (void) __attribute__ ((noreturn));


void main (void)
{
	/** Main function - we'll never return from here */

	module_UART_controller_t * console_uart = module_UART_getInstance(UART_0); //defined by module_base
	module_UART_defaultInit (console_uart); //setting 115200 8n1
	module_UART_setConsole(console_uart); //setting default uart console

	_printf("UART console initialized");



	module_DIT_controller_t * timer1 = module_DIT_getInstance(TIMER_1);


	module_VIC_set_interrupt_handler (NMC0HP, (void*) module_NMCLOAD_commandHandler_core1 );
	//module_VIC_set_interrupt_handler (ITMS, (void*)_NewMS_interrupt_handler);
	module_NMCLOAD_linkHandler (0, (void *) _nm_printf);

	module_ARM_vicEnable();
	//module_VIC_set_interrupt_handler (TIMINT1, (void*)_timer1_interrupt_handler);
	module_VIC_set_interrupt_handler (TIMINT2, (void*)_timer2_interrupt_handler);

	module_VIC_set_interrupt_handler (EXTINT0, (void*)_extInt0_interrupt_handler);

	module_VIC_set_interrupt_handler (DMACSENDINT, (void*) _dmac_interrupt_handler);
	module_VIC_set_interrupt_handler (DMACRECVINT, (void*) _dmac_interrupt_handler);

	module_GPIO_ledOn();
	module_EXTIRC_Setup(EXTIRC_XINT0, EXTIRC_LEVEL_Front);

	int i,j;

	module_DIT_EnableInterrupt(timer1);
	uint32_t ticks = module_DIT_countTicks(1,DIT_timeExp_Seconds, DIT_divider_16);
	module_DIT_runPeriodicCounter(timer1,DIT_divider_16, ticks);


	//char * arr = malloc(20);
	//char arr1[100] = {"1234567890\n\r1234567890\n\r"};

	//char arr2_tx[100] __attribute__ ((aligned (64))) = {"dddddfffffddfddf\n"};
	char arr2_rx[100] __attribute__ ((aligned (32))) = {0};


	module_UART_controller_t * dat_uart = module_UART_getInstance(UART_1);
	module_UART_defaultInit (dat_uart);
	_printf ("UART1 initialized");
	//dat_uart->MCR |= MCR_LOOP;

	//uint8_t test[10] = {'\n','\r','-','T','E','S','T','-','\n','\r'};
	/*i=0;
	//while (arr1[i]!= '\n'){
	while (i < 100){
		dat_uart->RFR_TFR_DLL = arr1[i++];
	}
	i=0;*/
	//uint8_t bytes[950] __attribute__ ((aligned (64)));

	module_DMAC_channel_t* chTx = module_DMAC_getChannelInstance( DMAC_CHANNEL_UART1_MemToPeri);
	module_DMAC_channel_t* chRx = module_DMAC_getChannelInstance( DMAC_CHANNEL_UART1_PeriToMem);

	module_DMAC_channelConfig_t uartTxdmacConf;
	uartTxdmacConf.PackType = DMAC_Pack_8bit;
	uartTxdmacConf.channelNumber = DMAC_CHANNEL_UART1_MemToPeri;
	uartTxdmacConf.srcAddr = (uint32_t*) hello;
	uartTxdmacConf.dstAddr = (uint32_t*) &dat_uart->RFR_TFR_DLL;
	uartTxdmacConf.byteNumber = hello_size;

	module_DMAC_channelConfig_t uartRxdmacConf;
	uartRxdmacConf.PackType = DMAC_Pack_8bit;
	uartRxdmacConf.channelNumber = DMAC_CHANNEL_UART1_PeriToMem;
	uartRxdmacConf.srcAddr = (uint32_t*) &dat_uart->RFR_TFR_DLL;
	uartRxdmacConf.dstAddr = (uint32_t*) arr2_rx;
	uartRxdmacConf.byteNumber = 22;

	int statusTx, statusRx;

//	module_DMAC_stopChannel(uartRxdmacConf.channelNumber);
//	module_DMAC_clearInterrupt(uartRxdmacConf.channelNumber);
//	statusRx = module_DMAC_setupChannel(uartRxdmacConf);
//	if (statusRx == status_ok)_printf("OK"); else _printf("ERR");
//
//	if (statusRx == status_ok) module_DMAC_startChannel(uartRxdmacConf.channelNumber);


	module_DMAC_stopChannel(uartTxdmacConf.channelNumber);
	module_DMAC_clearInterrupt(uartTxdmacConf.channelNumber);
	statusTx = module_DMAC_setupChannel(uartTxdmacConf);
	if (statusTx == status_ok)_printf("OK"); else _printf("ERR");

	if (statusTx == status_ok) module_DMAC_startChannel(uartTxdmacConf.channelNumber);



	//dat_uart->RFR_TFR_DLL = '\n';
	//dat_uart->RFR_TFR_DLL = '\r';

	/*module_DIT_EnableInterrupt(timer2);
	ticks = module_DIT_countTicks(5,DIT_timeExp_Seconds, DIT_divider_16);
	module_DIT_runPeriodicCounter(timer2,DIT_divider_16, ticks);*/


	//module_DIT_runOneShotCounter(timer2, DIT_divider_16, ticks);

	//module_DIT_Wait(timer1, 1000000, DIT_timeExp_microSeconds);
//	_printf("Waiting 2..."); module_DIT_WaitMillSeconds(timer1, 1000);
//	_printf("Waiting 1..."); module_DIT_WaitMillSeconds(timer1, 1000);

//	module_SPI_init();
	//module_SPI_setLoopBack(SPI_LOOPBACK_MODE);
//	module_SPI_setLoopBack(SPI_NORMAL_MODE);

	/*module_SPI_setDevice( module_GPIO_SPI_CS6);
	#define DATASIZE 10

	uint8_t tx_data[DATASIZE], rx_data[DATASIZE];
	int i, res;
	_printf("sizeof data: %i",sizeof(tx_data));
	for (i = 0; i < DATASIZE; i++)
	{
		tx_data[i] = i;
		rx_data[i] = 0;
	}


	_printf ("Exchanging %i data bytes in loopback mode...", DATASIZE);

		//res = module_SPI_exchangeMulti(tx_data, 0,  (uint32_t) DATASIZE);

	for (i = 0; i < DATASIZE; i++) module_SPI_exchangeSingle(tx_data, module_SPI_NoRecieve);

	_printf("sizeof data: %i",sizeof(tx_data));
	for (i = 0; i < DATASIZE; i++)
	{
		_printf ("tx: %i \t rx: %i \t res: %i", tx_data[i], rx_data[i], res);
	}

*///
	//_printf("Entering main.");






/*	module_SystimerClearInterrupt (2);
	//printf("DBG: VICADDRESS = 0x%X \n\r",read_reg(VICADDRESS));
	write_reg(0x20, VICINTENCLEAR);
	write_reg((uint32_t) _interrupt_handler_, VICVECTADDR_5);
	write_reg(0x20, VICINTENABLE);
	//printf("DBG: VICADDRESS = 0x%X \n\r",read_reg(VICADDRESS));

	_enable_interrupts();

*/

	   _printf("Initialization complete. Start NM cores");

		printf("Core 0 initializing descriptor ");
		if (module_NMCLOAD_GetBoardDesc(1, NM_core_1) == NMCLOAD_OK)
			_printf("Core 0 initializing descriptor- OK\n\r");
		else
			_printf("Core 0 initializing descriptor - FAIL\n\r");

		_printf("Core 0 loading nmloader ");
		if (module_NMCLOAD_LoadInitCode(NM_core_1, (uint32_t) Debug_initnmc_mini_abs) == NMCLOAD_OK)
			_printf("Core 0 init - OK\n\r");
		else
			_printf("Core 0 init - FAIL\n\r");

		_printf("Core 0 loading user program");
		if (module_NMCLOAD_LoadProgramFile(NM_core_1, (uint32_t) Debug_nmc_abs ) == NMCLOAD_OK)
			_printf("Core 0 loading- OK \n\r");
		else
			_printf("Core 0 loading - FAIL \n\r");




		_printf( "SYNC POINT 1");
		int sync_val;
		module_NMCLOAD_Sync(NM_core_1, (int) 0xdedab0b, &sync_val);


	printf("Entering endless loop... \n\r");
	while( 1 )
    {
			module_DIT_Wait(timer1, 1, DIT_timeExp_Seconds);
    		_printf("loop: %i ",st_counter++);
    		//dat_uart->RFR_TFR_DLL = '|';
    		/*module_DIT_Wait(timer1, 0.5, DIT_timeExp_Seconds);
    		module_GPIO_ledOn();

            module_GPIO_ledOff();*/
    }

}

