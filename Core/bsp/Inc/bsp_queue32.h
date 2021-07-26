/*
 * bsp_queue32.h
 *
 *  Created on: Jul 26, 2021
 *      Author: hzmavc
 */

#ifndef BSP_INC_BSP_QUEUE32_H_
#define BSP_INC_BSP_QUEUE32_H_




/*
 * @brief Common queue support functions
 */



#include <stdint.h>

/**
 * @brief queue structure
 */
typedef struct {
	uint32_t *data;
	uint32_t size;
	uint32_t head;
	uint32_t tail;
} queue32_t;

#define STATIC					static
#define INLINE					inline

/**
 * @def		Q_VHEAD(rb)
 * volatile typecasted head index
 */
#define Q_VHEAD(q)              (*(volatile uint32_t *) &(q)->head)

/**
 * @def		Q_VTAIL(rb)
 * volatile typecasted tail index
 */
#define Q_VTAIL(q)              (*(volatile uint32_t *) &(q)->tail)

/**
 * @brief	Initialize queue
 * @param	p_queue		: Pointer to queue to initialize
 * @param	p_buffer	: Pointer to buffer to associate with queue
 * @param	size		: Size of ring buffer
 * @note	Memory pointed by @a p_buffer must have a size power of 2 and must at
 * 			least be 2 or greater.
 * @return	1 on success, 0 otherwise
 */
uint32_t queue32_init(queue32_t *p_queue, uint32_t *p_buffer, uint32_t size);


STATIC INLINE void queue32_flush(queue32_t *p_queue)
{
	p_queue->head = p_queue->tail = 0;
}


STATIC INLINE uint32_t queue32_size_get(queue32_t *p_queue)
{
	return p_queue->size;
}


STATIC INLINE uint32_t queue32_count_get(queue32_t *p_queue)
{
	return Q_VHEAD(p_queue) - Q_VTAIL(p_queue);
}


STATIC INLINE uint32_t queue32_free_get(queue32_t *p_queue)
{
	return p_queue->size - queue32_count_get(p_queue);
}


STATIC INLINE uint32_t queue32_full_check(queue32_t *p_queue)
{
	return (queue32_count_get(p_queue) >= p_queue->size);
}


STATIC INLINE uint32_t queue32_empty_check(queue32_t *p_queue)
{
	return Q_VHEAD(p_queue) == Q_VTAIL(p_queue);
}


uint32_t queue32_write(queue32_t *p_queue, const uint32_t *p_data);
uint32_t queue32_read(queue32_t *p_queue, uint32_t *p_data);

#endif /* BSP_INC_BSP_QUEUE32_H_ */
