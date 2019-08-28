/*
 * task.c
 *
 *  Created on: 4 июн. 2019 г.
 *      Author: aleksey
 */

#include <task.h>

task_t * task_create(const char * name,
					task_handler_t f_handler,
					size_t size_of_used_data,
					uint32_t data_queue_size)
{

	if (name == 0)
	{
		_runtime_error("TASK: name string pointer is null");
		return -1;
	}

	if (strlen(name) > 128)
	{
		_runtime_error("TASK: name string too long or it is not the c-string");
		return -1;
	}

	if (f_handler == 0)
	{
		_runtime_error ("TASK: null pointer function handler");
		return -1;
	}

	task_t * new_task_pointer = (task_t *) malloc (sizeof(task_t));
	new_task_pointer->name =


	int t=0;
	if (!((data_queue_size == 0) && (size_of_used_data == 0) ))
	{
		if ( data_queue_size > (uint32_t) MAX_DATA_QUEUE_SIZE )
		{
			return -1;
		}
	}


	queue_t * queue_pointer = queue_create (task_queue_depth, datasize);

	task_t * task_pointer = dispatcher_linkTask(queue_pointer, f_handler);
	dispatcher_registerTask(dispatcher_pointer, task_pointer);
	return task_pointer;
}



task_needExec_t task_needExec(task_t * task)
{
	if (task->activity == TASK_ACTIVE)
		return NEED_EXEC;
	else if (task->activity == TASK_DEACTIVE)
		return NO_NEED_EXEC;

	if (task->data_queue->current_length > 0)
		return NEED_EXEC;
	else return NO_NEED_EXEC;
}



int task_pushData(task_t * task, void * data_pointer, size_t data_size)
{
	if (! data_size == task->data_queue->element_size)
	{
		_runtime_error("TASK: incorrect data size");
		return -1;
	}

	if (!queue_enqueue(task->data_queue, data_pointer))
			return 0;
	else return -1;
}



int task_setActivity(task_t * task, task_activity_t act)
{
	if (act==IF_DATA_AVAILABLE && task->data_queue->depth < 0)
	{
		_assert("TASK: task has no data queue");
		return -1;
	}

	task->activity = act;
	return 0;
}
