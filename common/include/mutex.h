/*
 * module-mutex.h
 *
 *  Created on: 1 апр. 2019 г.
 *      Author: aleksey
 */

#ifndef COMMON_INCLUDE_MUTEX_H_
#define COMMON_INCLUDE_MUTEX_H_

#include <stdint.h>

#define MUTEX_DEBUG

typedef enum {
	MUTEX_FREE = 0,
	MUTEX_CAPTURED =1
} mutex_status_t;

typedef enum {
	MUTEX_CONTROL_SUCC = 0,
	MUTEX_CONTROL_FAIL  = 1
} mutex_control_res_t;

typedef struct {
	uint32_t status;
	uint32_t owner_task_id;
	char mutex_name[24];
} mutex_t;

/* *** Function prototypes *** */

/**
 * @brief Free mutex function
 */
mutex_control_res_t mutex_tryFree (mutex_t * mtx, uint32_t owner_task_id);

/**
 * @brief Capture mutex function
 */
mutex_control_res_t mutex_tryCapture (mutex_t * mtx, uint32_t owner_task_id);


#endif /* COMMON_INCLUDE_MUTEX_H_ */
