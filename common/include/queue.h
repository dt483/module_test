/*
 * module-queue.h
 *
 *  Created on: 2 апр. 2019 г.
 *      Author: aleksey
 */

#ifndef COMMON_INCLUDE_QUEUE_H_
#define COMMON_INCLUDE_QUEUE_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <module-base.h>

#define Q_DEPTH 1



typedef struct {
	uint32_t					depth;
	uint32_t					current_length;
	uint32_t					current_position;
	void * 						linked_array;
	size_t 						element_size;
} queue_t;



/* *** Funtion prototypes *** */
int queue_enqueue (queue_t * queue, void * element);
int queue_dequeue (queue_t * queue, void * element);
queue_t * queue_create (uint32_t depth, size_t element_size );
int queue_destroy (queue_t * queue_ponter);
int queue_test (queue_t * queue);


#endif /* COMMON_INCLUDE_QUEUE_H_ */
