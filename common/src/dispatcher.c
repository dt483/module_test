/*
 * module-dispatcher.c
 *
 *  Created on: 5 апр. 2019 г.
 *      Author: aleksey
 */

#include <common/include/dispatcher.h>



void dispatcher_init(module_DISPATCHER_t * dispatcher_pointer)
{
	dispatcher_pointer->registered_tasks_num = 0;
	dispatcher_pointer->status = DISPATCHER_IDLE;
	_debug ("sizeof(module_DISPATCHER_task_t *) = %i", sizeof(task_t *));
	dispatcher_pointer->task_pointer = (task_t **) malloc (DISPATCHER_MAX_TASKS*(sizeof(task_t *)));

}

task_t * dispatcher_linkTask (queue_t * task_queue, task_handler_t f_handler)
{
	if (f_handler == 0)
	{
		_runtime_error ("DISPATCHER: null pointer function handler");
		return (task_t *) 0;
	}

	if (task_queue == 0)
	{
		_runtime_error ("DISPATCHER: null pointer of task data queue. Create queue then create task");
		return (task_t *) 0;
	}

	task_t * task_pointer = (task_t *) malloc (sizeof(task_t));
	task_pointer->task_handler = (task_handler_t) f_handler;
	task_pointer->task_requests_queue = (queue_t *) task_queue;

	return (task_t *) task_pointer;
}


int dispatcher_registerTask(module_DISPATCHER_t * dispatcher_pointer, task_t * task_pointer)
{
	if (dispatcher_pointer->status == DISPATCHER_ACTIVE)
	{
		_runtime_error ("DISPATCHER: dispatcher is in active state");
		return 1;
	}

	if (task_pointer == 0)
	{
		_runtime_error ("DISPATCHER: null pointer of task. Create task then register task in dispatcher");
		return 1;
	}
	if (dispatcher_pointer->registered_tasks_num + 1 > DISPATCHER_MAX_TASKS)
	{
		_runtime_error ("DISPATCHER: max tasks value overflow");
	}


	dispatcher_pointer->task_pointer[dispatcher_pointer->registered_tasks_num] = task_pointer;

#ifdef DISPATCHER_DEBUG
	_debug("DISPATCHER: task registered with priority %i", registered_tasks_num);
#endif

	dispatcher_pointer->registered_tasks_num++;
	return 0;
}

task_t * dispather_createTask(module_DISPATCHER_t * dispatcher_pointer,
		task_handler_t f_handler, uint32_t task_queue_depth, size_t datasize)
{
	if (!dispatcher_pointer) dispatcher_init(dispatcher_pointer);
	queue_t * queue_pointer = queue_create (task_queue_depth, datasize);
	task_t * task_pointer = dispatcher_linkTask(queue_pointer, f_handler);
	dispatcher_registerTask(dispatcher_pointer, task_pointer);
	return task_pointer;
}

void disaptcher_start(module_DISPATCHER_t * dispatcher_pointer)
{
	/*Бесконечный цикл, в котором будут крутиться обработчики заданий*/
	/*TODO: предусмотреть возможность остановки и запуска*/


	dispatcher_pointer->status = DISPATCHER_ACTIVE;
	module_ARM_irqUnblock();

	void * task_arg = 0;
	int task_number;
	while (1)
	{
		for (task_number = 0; task_number < dispatcher_pointer->registered_tasks_num; task_number++){
			if  (! dispatcher_pointer->task_pointer[task_number]->task_requests_queue->current_length == 0)
			{
				module_ARM_irqBlock();
				task_arg = malloc(dispatcher_pointer->task_pointer[task_number]->task_requests_queue->element_size);
				if ( queue_dequeue( dispatcher_pointer->task_pointer[task_number]->task_requests_queue,task_arg) )
					continue; //in case of data pick error go to next task

				//invoke task handler
				module_ARM_irqUnblock();
				dispatcher_pointer->task_pointer[task_number]->task_handler(task_arg);
				free(task_arg );

				break;
			}
		}
	}

}


