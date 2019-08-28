
#include <common/include/mutex.h>
#include <common/include/queue.h>
#include <common/include/task_dispatcher.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdarg.h>

/* low level drivers*/
#include "module-arm.h"
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
#include "module-serial.h"
#include "serial_protocol_test.h"


//#include "lls/initnmc_mini_abs.h"
#include "nmc_startup/Debug/initnmc_mini_abs.h" //Init code in hex c-header file
#include "nmc/Debug/nmc_abs.h"
//#define NM_ELF_LOCATION SMB1_START_ADDR





static int st_counter = 0;


module_DIT_controller_t * timer1 = 0;
module_DIT_controller_t * timer2 = 0;

/* Core descriptors  */
module_NMC_descriptor_t NMcore1_desc;
module_NMC_descriptor_t NMcore2_desc;

// Hellord world in ASCII banner
// #include "h.h"


/* System tasks */
task_t * NM_print0;
task_t * NM_dataSend;
task_t * HOST_requests;
task_t * timer1_regular_task;
task_t * NM0_sync_task;


/* System queues  */
/*module_QUEUE_t * NM_print0_queue;
module_QUEUE_t * NM_dataSend_queue;
module_QUEUE_t * HOST_requests_queue;
module_QUEUE_t * timer1_regular_task_queue;*/




/* Extern test functions */
extern int spi_test();
extern int pll_test();
extern int bgt_test();
/******                  */

static uint32_t array_send_flag = 0;



typedef struct {
    uint32_t handshaking_word;
    uint32_t message_type;
    uint32_t message_length;
    uint32_t reserved;
} serial_header_t;




enum request_type{
    REQ_TEST_1 = 1,
    REQ_TEST_2 = 2
};


#define messageSize 256

char data_array[messageSize] __attribute__((aligned (32)));;// = "abc def ghi jkl mno pqr srt uvw\0";

void _serial_data_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void _serial_data_interrupt_handler (void)
{
	module_UART_controller_t * uart_data = module_UART_getInstance (DATA);
	//uint32_t irq_id = GET_BIT_FIELD (uart_data->IIR_FCR, IIR_ID); //get irq id

	serial_header_t msg_h_in;
	uint8_t * rcv_data_pointer = (uint8_t*) &msg_h_in;
	int rcv_data_counter = 0;
	//uint32_t lsr = uart_data->LSR;
	while ( (rcv_data_counter < sizeof(serial_header_t)))
	{
		while (!(uart_data->LSR & 0x1)){}
		rcv_data_pointer [rcv_data_counter++] = uart_data->RFR_TFR_DLL;
	}



	if (msg_h_in.message_type == REQ_TEST_1)
	{
		queue_enqueue (HOST_requests->task_requests_queue, &msg_h_in);
		_debug("NM_print0_queue now has %i tasks from %i",
				NM_print0->task_requests_queue->current_length, NM_print0->task_requests_queue->depth);
	}

	/*_printf("Recieved data: size = %i, data: ", rcv_data_counter);
	_printf(".handshaking_word = 0x%X",msg_h_in.handshaking_word );
	_printf(".message_type = 0x%X" 	,msg_h_in.message_type );
	_printf(".message_length = 0x%X" ,msg_h_in.message_length);
	_printf(".reserved = 0x%X" 		,msg_h_in.reserved );*/

	//module_DIT_clearInterrupt(timerInstance);
	module_VIC_finishHandling ();
}

static uint32_t  _timer1_interrupt_event_counter = 0;
void _timer1_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void _timer1_interrupt_handler (void)
{
	queue_enqueue (timer1_regular_task->task_requests_queue, &_timer1_interrupt_event_counter);
	_timer1_interrupt_event_counter++;

	module_DIT_clearInterrupt (timer1);
	module_VIC_finishHandling ();
}


void _NM1_interrupt_handler (void) __attribute__((interrupt ("IRQ")));
void _NM1_interrupt_handler (void)
{
	uint32_t * cmdBlckAddr = 0;
	cmdBlckAddr = (uint32_t *) NMcore1_desc.localNM_startAddr_ARM + CommandToArm;

	module_NMCLOAD_commandBlock_t * cmdBlck = (module_NMCLOAD_commandBlock_t *) cmdBlckAddr;
	cmdBlck->handlerStatus = 0x1;


	module_NMCLOAD_commandBlock_t cmd_local_data;
	module_NMCLOAD_getCommandData ((module_NMC_descriptor_t *) &NMcore1_desc, &cmd_local_data);

	if (cmd_local_data.commantType == 0x1)
	{
		queue_enqueue (NM_print0->task_requests_queue, &cmd_local_data);
		_debug("NM_print0_queue now has %i tasks from %i",
				NM_print0->task_requests_queue->current_length, NM_print0->task_requests_queue->depth);
	}
	else if (cmd_local_data.commantType == 0x5)
	{
		queue_enqueue (NM_dataSend->task_requests_queue, &cmd_local_data);
		_debug("NM_dataSend_queue now has %i tasks from %i",
				NM_dataSend->task_requests_queue->current_length, NM_dataSend->task_requests_queue->depth);
	}
	else if (cmd_local_data.commantType == 0x77)
		{
			queue_enqueue (NM0_sync_task->task_requests_queue, &cmd_local_data);
			/*_debug("NM_dataSend_queue now has %i tasks from %i",
					NM0_sync_task->task_requests_queue->current_length, NM0_sync_task->task_requests_queue->depth);*/
		}


	module_ARMSC_clear_NMU_interrupt(ARMSC_INT_NMC0HP);
	module_VIC_finishHandling();

	cmdBlck->handlerStatus = 0x0;
	cmdBlck->nmRequestStatus = 0x0;
}


// Command Interrupt handler


void NM_print0_handler (void * arg)
{
	_printf("Invoked NM_print0_handler ");
	module_NMCLOAD_commandBlock_t * cmd_data = arg;
	//uint32_t* data =  (uint32_t*) cmd_data->bufferAddr;

	module_SERIAL_NM_printf(cmd_data->bufferAddr, cmd_data->bufferLen);
}

static int NM_dataSend_handler_call_counter = 0;
void NM_dataSend_handler (void * arg)
{

	_printf("Invoked NM_dataSend_handler: %i ", NM_dataSend_handler_call_counter++);
	module_NMCLOAD_commandBlock_t * cmd_data = arg;

	serial_header_t ack_to_request = {
	  .handshaking_word = 0xBEFED0CC,
	  .message_type = 0xABBA,
	  .message_length = cmd_data->bufferLen,
	  .reserved = cmd_data->bufferAddr
	};

	_printf("Sending request: ");
	_printf("    .handshaking_word =0x%x",ack_to_request.handshaking_word );
	_printf("    .message_type     =0x%x",ack_to_request.message_type);
	_printf("    .message_length   =0x%x",ack_to_request.message_length );
	_printf("    .reserved         =0x%x",ack_to_request.reserved );

	module_SERIAL_send_data((uint32_t) &ack_to_request, sizeof(serial_header_t));

}

void NM0_sync_handler (void * arg)
{
	module_NMCLOAD_commandBlock_t * cmd_data = arg;
	_printf("Sync with NM : %u", (uint32_t) cmd_data->bufferLen);
	int value;
	module_NMCLOAD_Sync(&NMcore1_desc, 0 , &value);
	_printf("Recieved value = 0x%X", value);

}


serial_package_t package  __attribute__ ((aligned (32)));

static int transaction_id = 0;
int packages_num = 10;

uint8_t serialPackStart[4]  __attribute__ ((aligned (32))) = {0xAA, 0xFE, 0xDC, 0xCD} ;
uint8_t serialPackEnd[4]  __attribute__ ((aligned (32))) = {0xCF, 0xAC, 0xBB, 0xDE} ;

uint8_t dummyPack[16]  __attribute__ ((aligned (32))) =
	{0xCF, 0xCF, 0xBB , 0x44 ,0x0, 0x0, 0x11, 0xAC, 0xFF,  0x1B, 0xDE, 0xAC, 0xAC, 0x45, 0x68, 0x44} ;

static int timer1_regular_task_handler_call_counter = 0;
void timer1_regular_task_handler (void * arg)
{

	_printf("Invoked timer1_regular_task_handler: %i, param: %i ", timer1_regular_task_handler_call_counter++, *((uint32_t*) arg));
	int i,j,k;
	for (i=0;i<packages_num;i++)
	{
		//if (pack_num == packages_num) pack_num = 0;
		_printf("Sending package %i from %i...", i, packages_num);
		serial_package_fill (&package, i, packages_num, transaction_id);

		module_SERIAL_send_dataBytes( (char*) &dummyPack, 16 );

		module_SERIAL_send_dataBytes( (char*) &serialPackStart, 4 );
		module_SERIAL_send_dataBytes( (char*) &package, sizeof(serial_package_t) );
		module_SERIAL_send_dataBytes( (char*) &serialPackEnd, 4 );

//		if (pack_num >= packages_num)
//			{
//				pack_num = 0;
//
//			}


		//for (j=0;j<1000000;j++){}
	}
	transaction_id++;

}


module_DISPATCHER_t system_dispatcher;

void main (void) __attribute__ (( section (".text.main") ));
void main (void)
{
	/** Main function - we'll never return from here */
	/* ************************************ INITIALISING PARAMETERS ************************ */
	module_NMCLOAD_init(&NMcore1_desc, &NMcore2_desc);
	module_SERIAL_init();
	module_ARM_vicEnable();
	module_ARM_irqBlock();

	_printf("");
	_printf("UART console initialized");

	/*TODO: не нравится порядок работы с таймерами... Добавить мьютекс (или отслеживание аппаратной работы)
	 *  для одного таймера и настроить второй таймер в качестве системного таймера, который постоянно работает
	 *  и любой может отслеживать по нему системное время. Добавить API для работы с таймерами*/
	timer1 = module_DIT_getInstance(TIMER_1);
	timer2 = module_DIT_getInstance(TIMER_2);

	module_VIC_set_interrupt_handler (TIMINT1, (void*) _timer1_interrupt_handler );
	module_VIC_set_interrupt_handler (NMC0HP, (void*) _NM1_interrupt_handler );
	module_VIC_set_interrupt_handler (DMACRECVINT, (void*) module_SERIAL_dmacRx_interrupt_handler);

	/*TODO: добавить настройку прерыванийй при инициализации последовательных портов в бибилиотеке*/
	//module_VIC_set_interrupt_handler (UART0INT, (void*) _serial_data_interrupt_handler  );
//	module_VIC_set_interrupt_handler (UART1INT, (void*) _serial_data_interrupt_handler );


	module_DISPATCHER_init(&system_dispatcher);

	module_GPIO_ledOn();
	/* ****************** END OF INITIALISING PARAMETERS ************************************ */

	_printf("Initialization complete. Start NM cores");

	/* ************************************ STARTING NM CORES ********************************** */

	_printf("Core 0 loading nmloader ");
	if (module_NMCLOAD_LoadInitCode(&NMcore1_desc, (uint32_t) Debug_initnmc_mini_abs) == NMCLOAD_OK)
		_printf("Core 0 init - OK");
	else
		_printf("Core 0 init - FAIL");

	_printf("Core 0 loading user program");
	if (module_NMCLOAD_LoadProgramFile(&NMcore1_desc, (uint32_t) Debug_nmc_abs ) == NMCLOAD_OK)
		_printf("Core 0 loading- OK ");
	else
		_printf("Core 0 loading - FAIL ");

	/* ************************** END OF STARTING NM CORES ************************************ */

	/* ************************** System Tasks creating ************************************** */
	/*NM_print0_queue = module_QUEUE_create (5, sizeof(module_NMCLOAD_commandBlock_t) );
	module_DISPATCHER_task_t * NM_print0_task =   module_DISPATCHER_createTask
			(NM_print0_queue, (module_DISPATCHER_task_handler_t) NM_print0_handler);*/

	NM0_sync_task = module_DISPATCHER_createTask (&system_dispatcher,
			(task_handler_t) NM0_sync_handler, 5, sizeof(uint32_t) );

	handle_recieved_package_task = module_DISPATCHER_createTask (&system_dispatcher,
			(task_handler_t) module_SERIAL_processPackage, /*depth*/5, sizeof(module_SERIAL_package_t));

	NM_print0 =  module_DISPATCHER_createTask (&system_dispatcher,
			(task_handler_t) NM_print0_handler, /*depth*/5, sizeof(module_NMCLOAD_commandBlock_t));

	NM_dataSend = module_DISPATCHER_createTask (&system_dispatcher,
			(task_handler_t) NM_dataSend_handler, 5, sizeof(serial_header_t) );

	timer1_regular_task = module_DISPATCHER_createTask (&system_dispatcher,
			(task_handler_t) timer1_regular_task_handler, 5, sizeof(uint32_t) );



	/* *****************************************************************************************/



	_printf("Sizeof serial_package_t: %i", sizeof(serial_package_t));


	module_DIT_EnableInterrupt(timer1);
	uint32_t timer_value = module_DIT_countTicks (3 ,DIT_timeExp_Seconds, DIT_divider_256);
	module_DIT_runPeriodicCounter(timer1, DIT_divider_256, timer_value);

	module_DISPATCHER_start(&system_dispatcher);



	while( 1 )
    {
			module_DIT_Wait(timer1, 500, DIT_timeExp_millSeconds);
			//_printf("loop: %i",st_counter++);
			/*_printf(
					" \r\n ------------------------------------------------------------------------ \r\n"
					"Breath. Few think about the fact that apart from the formation \r\n"
    				"of carbon dioxide that we exhale, with each breath of air in our \r\n"
    				" body a huge number of different compounds are formed, due to the \r\n"
    				"influence of oxygen. In turn, these compounds become the basis for \r\n"
    				"the synthesis of vitamins, amino acids, proteins, \r\n"
    				"and fats.: %i \r\n"
    				"------------------------------------------------------------------------ \r\n"
    				,st_counter++);*/
    		module_GPIO_toggle(module_GPIO0);

    		//if (array_send_flag == 0xAABB00) send_header(&send_array_header);
    		//if (array_send_flag == 0xAABBCC) send_array(&data_array, messageSize);
    		//dat_uart->RFR_TFR_DLL = '|';
    		/*module_DIT_Wait(timer1, 0.5, DIT_timeExp_Seconds);
    		module_GPIO_ledOn();

            module_GPIO_ledOff();*/
    }

}

