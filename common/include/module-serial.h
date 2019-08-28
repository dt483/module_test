/*
 * module-serial.h
 *
 *  Created on: 1 апр. 2019 г.
 *      Author: aleksey
 */

#ifndef COMMON_INCLUDE_MODULE_SERIAL_H_
#define COMMON_INCLUDE_MODULE_SERIAL_H_

#include <common/include/dispatcher.h>
#include <common/include/mutex.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#include "module-uart.h"
#include "module-vic.h"
#include "module-dmac.h"




/* DEBUG */
#define DEBUG_PRINT_NUMBERS
#define USE_CORE_LABELES

#define CONSOLE		UART_1
#define DATA		UART_0

#define PRINT_BUFFER_SIZE 1024
#define PACKAGE_PAYLOAD_SIZE_BYTES 128

#define USE_DMAC

#define SERIAL_CONSOLE_USE_DEFAULT_SETTINGS
#ifndef SERIAL_CONSOLE_USE_DEFAULT_SETTINGS
#define SERIAL_CONSOLE_BAUD			UART_Speed_115200
#define SERIAL_CONSOLE_START_BITS	UART_wordLength_8bit
#define SERIAL_CONSOLE_PARITY		UART_Parity_NoControl
#define SERIAL_CONSOLE_STOP_BITS	UART_StopBits_1
#endif

#define SERIAL_DATA_USE_DEFAULT_SETTINGS
#ifndef SERIAL_DATA_USE_DEFAULT_SETTINGS
#define SERIAL_DATA_BAUD		UART_Speed_115200
#define SERIAL_DATA_START_BITS	UART_wordLength_8bit
#define SERIAL_DATA_PARITY		UART_Parity_NoControl
#define SERIAL_DATA_STOP_BITS	UART_StopBits_1
#endif

typedef struct  {
	uint32_t status;
	uint32_t buffer;
	uint32_t array_pointer;
} module_SERIAL_NMprintHeader_t;

typedef enum module_SERIAL_printSource{
	PRINT_FROM_ARM = 0,
	PRINT_FROM_NM1 = 1,
	PRINT_FROM_NM2 = 2
} module_SERIAL_printSource_t;


typedef struct {
	uint16_t	sender_address;
	uint16_t	reciever_address;
	uint32_t	transaction_id;
	uint8_t		current_package_number;
	uint8_t		num_of_packages;
	uint8_t		reserved0;
	uint8_t		reserved1;
} module_SERIAL_package_header_t;


typedef struct {
	module_SERIAL_package_header_t	header;
	uint8_t							payload [PACKAGE_PAYLOAD_SIZE_BYTES];
	uint32_t						reserved0;
} module_SERIAL_package_t;


/* Task for handling recived packages from UART throw DMA RX channel*/
task_t * handle_recieved_package_task;

#define PREFIX_WORD_SIZE 4
#define SUFFIX_WORD_SIZE 4

uint8_t module_SERIAL_serialPackPrefix[PREFIX_WORD_SIZE]  __attribute__ ((aligned (32))) = {0xAA, 0xFE, 0xDC, 0xCD} ;
uint8_t module_SERIAL_serialPackSuffix[SUFFIX_WORD_SIZE]  __attribute__ ((aligned (32))) = {0xCF, 0xAC, 0xBB, 0xDE} ;


/* *** Function prototypes *** */
/**
 * @brief Init both serial UART interfaces for console and for data exchange
 */
void module_SERIAL_init();

/**
 * @brief Print function for using as print command handler from NM in NMCLOAD
 */
int module_SERIAL_NM_printf(uint32_t bufferAddr, uint32_t bufferLen);

/**
 * @brief Print function for printing formatted output to serial console (like printf())
 */
void module_SERIAL_ARM_printf(const char* format,...);


void module_SERIAL_send_data(uint32_t bufferAddr, uint32_t bufferLen32 );


void module_SERIAL_send_dataBytes(char * bufferAddr, uint32_t bufferLen );


void module_SERIAL_dmacRx_interrupt_handler (void);

void module_SERIAL_processPackage (void * arg);



#endif /* COMMON_INCLUDE_MODULE_SERIAL_H_ */
