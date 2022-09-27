/*
 * queue.h
 *
 *  Created on: Sep 15, 2022
 *      Author: Kuprin_IV
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include "stm32f4xx_hal.h"
#include "usbd_conf.h"

#define DATA_SIZE 		USBD_CUSTOMHID_OUTREPORT_BUF_SIZE
#define OUEUE_DEPTH 	10

typedef struct
{
	uint8_t* (*Peek)(void);
	uint8_t (*IsEmpty)(void);
	uint8_t (*IsFull)(void) ;
	uint8_t (*Size)(void);
	void (*Insert)(uint8_t* data);
	uint8_t* (*Poll)(void);
}Queue;

extern Queue* commands_queue;

#endif /* INC_QUEUE_H_ */
