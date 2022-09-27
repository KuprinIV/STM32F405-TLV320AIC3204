/*
 * queue.c
 *
 *  Created on: Sep 15, 2022
 *      Author: Kuprin_IV
 */
#include "queue.h"
#include <string.h>

// driver functions
static uint8_t* peek();
static uint8_t isEmpty();
static uint8_t isFull() ;
static uint8_t size();
static void insert(uint8_t* data);
static uint8_t* poll();

// init driver
Queue cmd_queue = {
		peek,
		isEmpty,
		isFull,
		size,
		insert,
		poll,
};

Queue* commands_queue = &cmd_queue;

// queue variables
uint8_t commandsArray[OUEUE_DEPTH][DATA_SIZE];
uint8_t front = 0;
uint8_t rear = -1;
uint8_t itemCount = 0;

/*
 * command queue realization
 */

/**
  * @brief  Get value from queue without removing
  * @param  None
  * @retval Command data, received via Bluetooth
  */
uint8_t* peek()
{
   return commandsArray[front];
}

/**
  * @brief  Check is queue is empty
  * @param  None
  * @retval 0 - queue isn't empty, 1 - queue is empty
  */
uint8_t isEmpty()
{
	uint8_t res = (itemCount == 0) ? 1 : 0;
	return res;
}

/**
  * @brief  Check is queue is full
  * @param  None
  * @retval 0 - queue isn't full, 1 - queue is full
  */
uint8_t isFull()
{
	uint8_t res = (itemCount == OUEUE_DEPTH) ? 1 : 0;
   return res;
}

/**
  * @brief  Get queue size
  * @param  None
  * @retval Queue length
  */
uint8_t size()
{
   return itemCount;
}

/**
  * @brief  Insert value into the queue
  * @param  Command data, received via Bluetooth
  * @retval None
  */
void insert(uint8_t* data)
{

   if(!isFull())
   {

      if(rear == OUEUE_DEPTH-1)
      {
         rear = -1;
      }

      memcpy(commandsArray[++rear], data, DATA_SIZE);
      itemCount++;
   }
}

/**
  * @brief  Get value from queue with removing
  * @param  None
  * @retval Command data, received via Bluetooth
  */
uint8_t* poll()
{
   uint8_t* data = commandsArray[front++];

   if(front == OUEUE_DEPTH)
   {
      front = 0;
   }

   itemCount--;
   return data;
}


