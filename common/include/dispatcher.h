/*
 * module-dispatcher.h
 *
 *  Created on: 5 апр. 2019 г.
 *      Author: aleksey
 */

#ifndef TASK_DISPATCHER_H_
#define TASK_DISPATCHER_H_

#include <common/include/queue.h>
#include <common/include/task.h>
#include "module-armcore.h"

/*TODO: опционально. Можно сделать автоматическое перевыделение слотов под задачи Пока определим
 * фиксированное числослотов под определенное количество задач*/
#define DISPATCHER_MAX_TASKS 5


typedef enum {
	DISPATCHER_IDLE = 0,
	DISPATCHER_ACTIVE = 1
} module_DISPATCHER_status_t;





typedef struct {
	module_DISPATCHER_status_t status;
	task_t ** task_pointer;
	uint32_t registered_tasks_num;
	//uint32_t active_tasks_num;

} module_DISPATCHER_t;

/* *** Funtion prototypes *** */
void module_DISPATCHER_init(module_DISPATCHER_t * dispatcher_pointer);

//module_DISPATCHER_task_t * module_DISPATCHER_linkTask (module_QUEUE_t * task_queue, module_DISPATCHER_task_handler_t f_handler);

//int module_DISPATCHER_registerTask(module_DISPATCHER_t * dispatcher_pointer, module_DISPATCHER_task_t * task_pointer);

task_t * module_DISPATCHER_createTask(module_DISPATCHER_t * dispatcher_pointer,
		task_handler_t f_handler, uint32_t task_queue_depth, size_t datasize);

void module_DISPATCHER_start(module_DISPATCHER_t * dispatcher_pointer);

#endif /* TASK_DISPATCHER_H_ */
