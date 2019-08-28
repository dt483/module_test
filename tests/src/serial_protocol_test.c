#include "serial_protocol_test.h"

void serial_package_fill (serial_package_t * package, uint8_t current_pack, uint8_t packages_num, uint32_t transaction_id)
{

	package->header. sender_address = 0xBEFE;
	package->header. reciever_address = 0xABDA;
	package->header.  transaction_id = transaction_id;
	package->header.current_package_number = current_pack;
	package->header.num_of_packages = packages_num;
	package->header.reserved0 = 0;
	package->header.reserved1 = 0;

	uint32_t i = 0;
	for (i=0; i<PACKAGE_PAYLOAD_SIZE_BYTES; i++)
	{
		package->payload[i] = 255-i;
	}

	package->reserved0 = 0xC7;





}
