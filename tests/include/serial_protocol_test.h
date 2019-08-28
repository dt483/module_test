/*
 * serial_protocol_test.h
 *
 *  Created on: 12 апр. 2019 г.
 *      Author: aleksey
 */

#ifndef SERIAL_PROTOCOL_TEST_H_
#define SERIAL_PROTOCOL_TEST_H_

#include <stdint.h>

typedef struct {
    uint16_t sender_address;
    uint16_t reciever_address;
    uint32_t  transaction_id;
    uint8_t current_package_number;
    uint8_t num_of_packages;
    uint8_t reserved0;
    uint8_t reserved1;
} serial_package_header_t;


#define PACKAGE_PAYLOAD_SIZE_BYTES 128
typedef struct {
    serial_package_header_t header;
    uint8_t                  payload [PACKAGE_PAYLOAD_SIZE_BYTES];
    uint32_t                  reserved0;
} serial_package_t;




void serial_package_fill (serial_package_t * package, uint8_t current_pack, uint8_t packages_num, uint32_t transaction_id);

#endif /* SERIAL_PROTOCOL_TEST_H_ */
