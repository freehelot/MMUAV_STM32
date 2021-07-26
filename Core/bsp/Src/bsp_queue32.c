/*
 * @brief Common queue support functions
 * unused currently
 */

#include <string.h>
#include "bsp_queue32.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define Q_INDH(q)                ((q)->head & ((q)->size - 1))
#define Q_INDT(q)                ((q)->tail & ((q)->size - 1))

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize queue */
uint32_t queue32_init(queue32_t *p_queue, uint32_t *p_buffer, uint32_t size)
{
	uint32_t result = (NULL != p_queue) && (NULL != p_buffer);

	if (result)
	{
		p_queue->data = p_buffer;
		p_queue->size = size;
		p_queue->head = p_queue->tail = 0;
	}

	return result;
}

/* Insert a single item into queue */
uint32_t queue32_write(queue32_t *p_queue, const uint32_t *p_data)
{
	uint32_t result = !queue32_full_check(p_queue);

	if (result)
	{
		p_queue->data[Q_INDH(p_queue)] = *p_data;
		p_queue->head++;
	}

	return result;
}


/* Pop single item from queue */
uint32_t queue32_read(queue32_t *p_queue, uint32_t *p_data)
{
	uint32_t result = !queue32_empty_check(p_queue);

	if (result)
	{
		*p_data = p_queue->data[Q_INDT(p_queue)];
		p_queue->tail++;
	}

	return result;
}

