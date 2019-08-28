/*
 * module-queue.c
 *
 *  Created on: 2 апр. 2019 г.
 *      Author: aleksey
 */

#include <common/include/queue.h>


int queue_enqueue (queue_t * queue, void * element)
{
		if (queue->current_length == queue->depth)
			return 1; //fifo full!

		if (queue->current_position > queue->depth-1) queue->current_position = 0;

		uint8_t * addr_in_arr = (uint8_t*) (queue->linked_array + queue->current_position*queue->element_size);
		uint8_t * e_ch = (uint8_t*) element;
		int i;
		for (i=0; i< queue->element_size;i++)
		{
			*addr_in_arr++ = *e_ch++;
		}
		queue->current_length++;
		queue->current_position++;
	return 0;
}

int queue_test (queue_t * queue)
{
	int i, element, element_byte;

	// Allocation memory for test elements
	void * dummy_elements_array = (void *) malloc (queue->element_size * queue->depth);

	_debug ("QUEUE:  ** Testing Queue **");

	for (element = 0; element < queue->depth; element++)
	{
		void * dummy_element = dummy_elements_array + (element*queue->element_size);
		_debug ("QUEUE:  Pushing  to position: %i element:" , queue->current_position);
		for (element_byte = 0; element_byte < queue->element_size; element_byte ++)
		{
			*((uint8_t *) dummy_element + element_byte) = (element<<4) | element_byte;
			_debug ("QUEUE:  	[%i]: 0x%X ",element_byte, *((uint8_t *) dummy_element + element_byte));
		}
		queue_enqueue (queue, (void*) dummy_element);
	}

	void * picked_element= (void *) malloc (queue->element_size);

	while (queue->current_length > 0)
	{

		queue_dequeue (queue, picked_element);
		_debug ("QUEUE: Picked up element:");
		for (element_byte = 0; element_byte < queue->element_size; element_byte ++)
		{
			_debug ("QUEUE:  	[%i]: 0x%X ",element_byte, *((uint8_t *) picked_element + element_byte));
		}

		_debug ("QUEUE: Elements left in queue: %i", queue->current_length);
	}

	free (picked_element);
	free (dummy_elements_array);
	return 0;
}

int queue_dequeue (queue_t * queue, void * element)
{
	if (queue->current_length == 0)
		return 1;

	int pos = 0;
	pos = (queue->depth+queue->current_position) - queue->current_length;
	if (pos > queue->depth-1) pos -= queue->depth;

	uint8_t * addr_in_arr = (uint8_t*) (queue->linked_array + pos*queue->element_size);
	uint8_t * e_ch = (uint8_t*) element;
	int i;
	for (i=0; i< queue->element_size;i++)
	{
		*e_ch++ = *addr_in_arr++;
	}

	queue->current_length--;
	return 0;
}

queue_t * queue_create (uint32_t depth, size_t element_size )
{
#ifdef DEBUG_QUEUE
	_debug ("QUEUE: Creating Queue object with parameters:");
	_debug ("QUEUE: priority = %u", priority);
	_debug ("QUEUE: depth = %u",depth);
	_debug ("QUEUE: element_size = %u", element_size);
#endif

	queue_t * queue_addr = ( queue_t *) malloc(sizeof(queue_t));
	queue_addr->depth = depth;
	queue_addr->element_size = element_size;
	queue_addr->current_position = 0;
	queue_addr->current_length = 0;


	void * arr_addr = ( queue_t *) malloc(depth * sizeof(queue_t));
	queue_addr->linked_array = arr_addr;

#ifdef DEBUG_QUEUE
	_debug ("QUEUE: Creating Queue object:");
	_debug ("QUEUE: queue pointer : %p", queue_addr);
	_debug ("QUEUE: linked array pointer : %p", queue_addr->linked_array);

	_debug ("QUEUE:  depth            = %u", queue_addr->depth            );
	_debug ("QUEUE:  current_length   = %u", queue_addr->current_length   );
	_debug ("QUEUE:  current_position = %u", queue_addr->current_position );
	_debug ("QUEUE:  linked_array     = %u", queue_addr->linked_array     );
	_debug ("QUEUE:  element_size     = %u", queue_addr->element_size     );
#endif

	return queue_addr;
}

int queue_destroy (queue_t * queue_ptr)
{
#ifdef DEBUG_QUEUE
	_debug ("QUEUE: Destroying Queue object.");
#endif

	free(queue_ptr->linked_array);
	free(queue_ptr);
	return 0;
}

