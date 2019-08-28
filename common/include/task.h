/*
 * task.h
 *
 *  Created on: 4 июн. 2019 г.
 *      Author: aleksey
 */

#ifndef TASK_H_
#define TASK_H_

#define TASK_DEBUG

#include <string.h>

#include <module-armcore.h>
#include <module-base.h>


#include <common/include/queue.h>

#define MAX_DATA_QUEUE_SIZE = 15


typedef void (*task_handler_t)(void * arg);

typedef enum {
	TASK_ACTIVE = 1,
	TASK_DEACTIVE = 0,
	IF_DATA_AVAILABLE = 3
} task_activity_t;

typedef enum {
	NEED_EXEC = 1,
	NO_NEED_EXEC = 0,
} task_needExec_t;

static inline void task_dummyHandler () {
#ifdef TASK_DEBUG
 _assert("TASK: used dummy handler");
#endif
	NOP();
}

typedef struct {
	const char * name;
	task_handler_t handler; //function-handler pointer
	task_activity_t activity;
	queue_t * data_queue;

} task_t;

#endif /* TASK */

/**
 * Возвращает указатель на строку, содержащую имя
 */
char * task_getNameString (task_t * task);


/*
getting task status
