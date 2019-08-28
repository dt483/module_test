/*
 * module-serial.c
 *
 *  Created on: 1 апр. 2019 г.
 *      Author: aleksey
 */
#include <common/include/module-serial.h>

static char serial_print_buffer[PRINT_BUFFER_SIZE] = {0};

#define MSG_LEN 11
static const char arm_label[MSG_LEN] = "[ARM CORE:]";
static const char nm1_label[MSG_LEN] = "[NM1 CORE:]";
static const char nm2_label[MSG_LEN] = "[NM2 CORE:]";

module_UART_controller_t * serial_console = 0;
module_UART_controller_t * serial_data = 0;

module_DMAC_channel_t* dmac_tx = 0;
module_DMAC_channel_t* dmac_rx = 0;

static module_DMAC_channelConfig_t dmac_tx_config;
static module_DMAC_channelConfig_t dmac_rx_config;

static module_SERIAL_package_t recieve_buffer __attribute__ ((aligned (32))) = {0}  ;
static module_SERIAL_package_t transmit_buffer  __attribute__ ((aligned (32))) = {0} ;


#ifdef DEBUG_PRINT_NUMBERS
#define DBG_BUFFER_SIZE 10
	char dbg_buff[DBG_BUFFER_SIZE] = {0};
	static int print_counter_arm = 0;
	static int print_counter_nm1 = 0;
	static int print_counter_nm2 = 0;
#endif


void module_SERIAL_init()
{

	/* Initialising serial console */
	serial_console = module_UART_getInstance (CONSOLE);
#ifdef SERIAL_CONSOLE_USE_DEFAULT_SETTINGS
	module_UART_defaultInit (serial_console);
#else
	module_UART_init (	serial_console,
			SERIAL_CONSOLE_BAUD			,
			SERIAL_CONSOLE_START_BITS	,
			SERIAL_CONSOLE_PARITY		,
			SERIAL_CONSOLE_STOP_BITS	);
#endif

	/* Initialising serial data */
	serial_data = module_UART_getInstance (DATA);
#ifdef	SERIAL_DATA_USE_DEFAULT_SETTINGS
	module_UART_defaultInit (serial_data);
#else
	module_UART_init (	serial_data,
			SERIAL_DATA_BAUD		,
			SERIAL_DATA_START_BITS	,
			SERIAL_DATA_PARITY		,
			SERIAL_DATA_STOP_BITS	);

#endif



#if DATA == UART_0
	#define DATA_DMAC_TX DMAC_CHANNEL_UART0_MemToPeri
	#define DATA_DMAC_RX DMAC_CHANNEL_UART0_PeriToMem
#elif DATA == UART_1
	#define DATA_DMAC_TX DMAC_CHANNEL_UART1_MemToPeri
	#define DATA_DMAC_RX DMAC_CHANNEL_UART1_PeriToMem
#endif

	/*TODO: setup DMACS*/
	/* Initialising DMACs for serial DATA port*/
	dmac_tx =  module_DMAC_getChannelInstance( DATA_DMAC_TX);
	dmac_rx =  module_DMAC_getChannelInstance( DATA_DMAC_RX);


	dmac_tx_config.PackType = DMAC_Pack_8bit;
	dmac_tx_config.channelNumber = DATA_DMAC_TX;
	//uartTxdmacConf.srcAddr = (uint32_t*) hello;
	//uartTxdmacConf.dstAddr = (uint32_t*) &dat_uart->RFR_TFR_DLL;
	//uartTxdmacConf.byteNumber = hello_size;


	dmac_rx_config.PackType = DMAC_Pack_8bit;
	dmac_rx_config.channelNumber = DATA_DMAC_RX;

	//uartRxdmacConf.srcAddr = (uint32_t*) &dat_uart->RFR_TFR_DLL;
	//uartRxdmacConf.dstAddr = (uint32_t*) arr2_rx;
	//uartRxdmacConf.byteNumber = 22;
}


void module_SERIAL_printCoreLabel (module_SERIAL_printSource_t source)
{
#ifdef USE_CORE_LABELES
	int i;
	if (source == PRINT_FROM_ARM)
	{
		for (i=0; i<MSG_LEN; i++) module_UART_send(serial_console, arm_label[i]);
	}
	else if (source == PRINT_FROM_NM1)
	{
		for (i=0; i<MSG_LEN; i++) module_UART_send(serial_console, nm1_label[i]);
	}
	else if (source == PRINT_FROM_NM2)
	{
		for (i=0; i<MSG_LEN; i++) module_UART_send(serial_console, nm2_label[i]);
	}
#endif
}

int module_SERIAL_NM_printf(uint32_t bufferAddr, uint32_t bufferLen)
{
	/*Передавать указатель на ПЕРВЫЙ байт массива. НУЛЕВОЙ байт использовать для индикации чтения массива.
	 * Если массив прочитан - записать в НУЛЕВОЙ элемент значение*/
	if (!serial_console) module_SERIAL_init();
	//uint32_t * external_buffer = (uint32_t*) bufferAddr;
	module_SERIAL_NMprintHeader_t * nm_data = (module_SERIAL_NMprintHeader_t *) bufferAddr;
	uint32_t * formatted_input_pointer = &nm_data->array_pointer;
	uint32_t tmp;
	int length = bufferLen, i =0;
	/*TODO: Добавить поддержку второго ядра*/
#ifdef DEBUG_PRINT_NUMBERS
	_debug("NM_printf: buffer state %i",  nm_data->status);
	_debug("NM_printf: used buffer %i",  nm_data->buffer);
#endif
#ifdef DEBUG_PRINT_NUMBERS
	int dbg_msg_len = sprintf(dbg_buff,"[#:%i]", print_counter_nm1++);
	int dbg_i;
	for (dbg_i=0; dbg_i<dbg_msg_len; dbg_i++)
		module_UART_send(serial_console, dbg_buff[dbg_i]);
#endif

	module_SERIAL_printCoreLabel(PRINT_FROM_NM1); //Print CORE label
	for (i=0; i<=length; i++)
	{
		tmp = formatted_input_pointer[i];
		module_UART_send(serial_console, (char) tmp);
	}
	nm_data->status = 0x30;

	return 0;
}

void module_SERIAL_ARM_printf(const char* format,...)
{
	if (!serial_console) module_SERIAL_init();

	va_list argptr;
	va_start (argptr, format);
	int buff_len = vsprintf(serial_print_buffer, format, argptr);
	va_end( argptr );

	int i;
	module_SERIAL_printCoreLabel(PRINT_FROM_ARM); //Print CORE label

#ifdef DEBUG_PRINT_NUMBERS
	int dbg_msg_len = sprintf(dbg_buff,"[#:%i]", print_counter_arm++);
	int dbg_i;
	for (dbg_i=0; dbg_i<dbg_msg_len; dbg_i++)
		module_UART_send(serial_console, dbg_buff[dbg_i]);
#endif

	for (i=0; i < buff_len; i++) module_UART_send(serial_console, serial_print_buffer[i]);
}

void module_SERIAL_send_data(uint32_t bufferAddr, uint32_t bufferLen32 )
{
//#ifndef USE_DMAC
	if (!serial_data) module_SERIAL_init();

	uint32_t * external_buffer = (uint32_t*) bufferAddr;
	uint32_t tmp32;
	uint8_t * tmp8_p;
	uint32_t i,j =0;

	for (i=0; i < bufferLen32; i++)
	{
		tmp32 = external_buffer[i];
		tmp8_p = (uint8_t*) &tmp32;
		for (j=0; j < 4; j++) module_UART_send(serial_data, (char) tmp8_p[j]);
	}
//#else


//#endif

}

void module_SERIAL_send_dataBytes(char * bufferAddr, uint32_t bufferLen )
{
#ifndef USE_DMAC
	if (!serial_data) module_SERIAL_init();

	uint32_t i =0;
	for (i=0; i < bufferLen; i++)
	{
		module_UART_send(serial_data, bufferAddr[i]);
	}

#else

	dmac_tx_config.dstAddr = (uint32_t*) &serial_data->RFR_TFR_DLL;
	dmac_tx_config.srcAddr = (uint32_t*) bufferAddr;
	dmac_tx_config.byteNumber = bufferLen;

	uint32_t dmac_ch_busy_counter = 0;
	while (dmac_tx->Busy | (dmac_ch_busy_counter > 100000) ) dmac_ch_busy_counter++;
	if (dmac_ch_busy_counter > 100000)
	{
		_runtime_error ("SERIAL: UART tx dmac channel wait time exceeded");
		return;
	}

	module_DMAC_stopChannel(dmac_tx_config.channelNumber);
	module_DMAC_clearInterrupt(dmac_tx_config.channelNumber);

	if ( module_DMAC_setupChannel(dmac_tx_config))
	{
		_runtime_error ("SERIAL: dmac setting fail");
		return;
	}
	module_DMAC_startChannel(dmac_tx_config.channelNumber);
#endif
}

void module_SERIAL_processPackage (void * arg)
{
	_printf("Invoked module_SERIAL_processPackage handler");

	module_SERIAL_package_t * recieved_package =  (module_SERIAL_package_t *) arg;

	_printf("Package parameters: ");
	_printf(".sender_address         = 0x%X", recieved_package->header.sender_address        );
	_printf(".reciever_address       = 0x%X", recieved_package->header.reciever_address      );
	_printf(".transaction_id         = 0x%X", recieved_package->header.transaction_id        );
	_printf(".current_package_number = 0x%X", recieved_package->header.current_package_number);
	_printf(".num_of_packages        = 0x%X", recieved_package->header.num_of_packages       );
	_printf(".reserved0              = 0x%X", recieved_package->header.reserved0             );
	_printf(".reserved1              = 0x%X", recieved_package->header.reserved1             );

}

typedef enum
{
    FSM_IDLE = 0,
    FSM_DETECTED_PACKAGE,
} module_SERIAL_fsm_state_t;


static module_SERIAL_fsm_state_t fsm_state = FSM_IDLE;

module_SERIAL_fsm_state_t module_SERIAL_detectPrefix(char sByte)
{

	static int nPrefixByte = 0;
	if (sByte == module_SERIAL_serialPackPrefix[nPrefixByte])
		nPrefixByte++;
	else
		nPrefixByte = 0;

    if (nPrefixByte == 4) fsm_state = FSM_DETECTED_PACKAGE;
    //else fsm_state = FSM_IDLE;

    return fsm_state;
}
module_SERIAL_fsm_state_t module_SERIAL_detectSuffix(char sByte)
{
	static int nSuffixByte = 0;
	if (sByte == module_SERIAL_serialPackSuffix[nSuffixByte])
		nSuffixByte++;
	else
		nSuffixByte = 0;
	/* next state selection */
	if (nSuffixByte == 4)
		fsm_state = FSM_IDLE;
	//else fsm_state = FSM_IDLE;

    return fsm_state;
}



void module_SERIAL_rxDataReady_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void module_SERIAL_rxDataReady_interrupt_handler (void)
{

	char sByte;
	do
	{
		if (! module_UART_recieve(serial_data, &sByte, 100000) )
		{
			module_VIC_finishHandling ();
			return;
		}

	} while (module_SERIAL_detectPrefix (sByte) != FSM_DETECTED_PACKAGE);


#ifndef USE_DMAC
	/*BLOCKING METHOD!*/
#else
	dmac_rx_config.srcAddr = (uint32_t*) serial_data->RFR_TFR_DLL;
	dmac_rx_config.dstAddr = (uint32_t*) &recieve_buffer;
	dmac_rx_config.byteNumber = sizeof(module_SERIAL_package_t);

	uint32_t dmac_ch_busy_counter = 0;
	while (dmac_rx->Busy | (dmac_ch_busy_counter > 100000) ) dmac_ch_busy_counter++;
	if (dmac_ch_busy_counter > 100000)
	{
		_runtime_error ("SERIAL: UART rx dmac channel wait time exceeded");
		return;
	}

	module_DMAC_stopChannel(dmac_rx_config.channelNumber);
	module_DMAC_clearInterrupt(dmac_rx_config.channelNumber);

	if ( module_DMAC_setupChannel(dmac_rx_config))
	{
		_runtime_error ("SERIAL: dmac setting fail");
		return;
	}
	module_DMAC_startChannel(dmac_tx_config.channelNumber);

#endif
	module_VIC_finishHandling ();
}


static uint32_t _dmac_interrupt_event_counter = 0;
void module_SERIAL_dmacRx_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void module_SERIAL_dmacRx_interrupt_handler (void)
{
	_dmac_interrupt_event_counter++;
	module_DMAC_chNum_t irq_src =  module_DMAC_getInterruptSource();

	if (irq_src == DATA_DMAC_RX)
	{
		int b=0;
		module_SERIAL_fsm_state_t fsmSt;
		char sByte;

		for (b=0; b<4; b++)
		{
			if (! module_UART_recieve(serial_data, &sByte, 100000) )
			{
				_runtime_error ("SERIAL: not enough bytes for approve package suffix!");
				module_DMAC_clearInterrupt(irq_src);
				module_VIC_finishHandling ();
				return;
			}
			fsmSt = module_SERIAL_detectPrefix (sByte);

		}

		if (fsmSt != FSM_IDLE)
		{
			_runtime_error ("SERIAL: package suffix not approved!");
			module_DMAC_clearInterrupt(irq_src);
			module_VIC_finishHandling ();
			return;
		}
		queue_enqueue (handle_recieved_package_task->task_requests_queue, &recieve_buffer);
	}
	else
		_assert("SERIAL: have not handler");

	module_DMAC_clearInterrupt(irq_src);
	module_VIC_finishHandling ();
}



