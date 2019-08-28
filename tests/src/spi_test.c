/*
 * spi_test.c
 *
 *  Created on: 21 мар. 2019 г.
 *      Author: aleksey
 */
#include <stdint.h>
#include <module-spi.h>
#include <module-dit.h>

#define DATASIZE 10

typedef enum
{
	slave0 = module_GPIO_SPI_CS5,
	slave1 = module_GPIO_SPI_CS6,
	slave2 = module_GPIO_SPI_CS7
} spi_slave_t;


int send_to_slave(uint8_t* TxDataPointer, uint8_t* RxDataPointer, uint32_t DataLen , spi_slave_t device)
{
	module_SPI_setDevice( device);
	//_printf("Selected device %i", device);
	int i;
	for (i=0;i<DataLen;i++){
		module_SPI_exchangeSingle(TxDataPointer+i, 0);
	}

	//module_SPI_exchangeMulti (TxDataPointer, RxDataPointer , DataLen);
	//_printf("Exchanged %i bytes of data with device %i", DataLen ,device);
	return 0;
}

int spi_test()
{
	_printf("******** SPI_TEST START*********");
	uint8_t tx_data[DATASIZE], rx_data[DATASIZE];
	int i, res;
	_printf("Size of data: %i",sizeof(tx_data));
	for (i = 0; i < DATASIZE; i++)
	{
		tx_data[i] = DATASIZE-i;
		rx_data[i] = 0;
	}
	module_DIT_controller_t * timer1 = module_DIT_getInstance(TIMER_1);

	module_SPI_init();

	module_SPI_setDividers(10,4);
	uint32_t freq =  module_SPI_getFrequency();
	uint32_t period = (1000000000/freq);
	_printf("Frequency set: %lu Hz, clk width: %lu ns", (uint32_t) freq, (uint32_t) period/2);
	//module_SPI_setDevice( module_GPIO_SPI_CS7);
	//module_SPI_exchangeSingle(tx_data, 0); //dummy transcation
	//module_DIT_Wait(timer1, 100, DIT_timeExp_millSeconds);

	module_GPIO_ledOff();
	int  slave;
	for (slave=slave0; slave<= slave2; slave++)
	{
		module_DIT_Wait(timer1, 1, DIT_timeExp_millSeconds);
		send_to_slave(tx_data, 0, (uint32_t) DATASIZE, slave);


	}

	_printf("******** SPI_TEST COMPLETE*********");
	return 0;
}
