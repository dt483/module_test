/*
 * module-mutex.c
 *
 *  Created on: 1 апр. 2019 г.
 *      Author: aleksey
 */

#include mutex.h"
#include "module-armcore.h"




mutex_control_res_t mutex_tryCapture (mutex_t * mtx, uint32_t owner_task_id)
{
	if (mtx->status == MUTEX_CAPTURED) {
		if (mtx->owner_task_id == owner_task_id)
			_assert ("MUTEX: mutex %s is already by this task", mtx->mutex_name);
		else
			_assert ("MUTEX: mutex %s is already captured by task %u", mtx->mutex_name, owner_task_id);
		return MUTEX_CONTROL_FAIL;
	}

	mtx->status = MUTEX_CAPTURED;
	mtx->owner_task_id = owner_task_id;

#ifdef MUTEX_DEBUG
	_debug ("MUTEX: mutex succesfully captured by task %u", mtx->mutex_name);
#endif

	return MUTEX_CONTROL_SUCC;
}

mutex_control_res_t mutex_tryFree (mutex_t * mtx, uint32_t owner_task_id)
{
	if (mtx->status == MUTEX_CAPTURED)
	{
		if (mtx->owner_task_id == owner_task_id)
		{
			mtx->status = MUTEX_FREE;
			//mtx->owner_task_id = -1;
			return MUTEX_CONTROL_SUCC;
		}
		else {
			_assert ("MUTEX: mutex %s has captured by another task", mtx->mutex_name);
			return MUTEX_CONTROL_FAIL;
		}
	}
	else
		_assert ("MUTEX: mutex %s is already free", mtx->mutex_name);
		return MUTEX_CONTROL_FAIL;
}
