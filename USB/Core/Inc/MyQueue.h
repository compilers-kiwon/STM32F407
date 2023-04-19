/*
 * MyQueue.h
 *
 *  Created on: Apr 19, 2023
 *      Author: kkwon
 */

#ifndef INC_MYQUEUE_H_
#define INC_MYQUEUE_H_

typedef struct
{
    uint8_t *data;
    int max_size;
    int front;
    int rear;
} _stMyQueue;

_stMyQueue * MyQueue_Create (int size);
int MyQueue_Size(_stMyQueue *myQueue);
int MyQueue_IsEmpty(_stMyQueue *myQueue);
int MyQueue_IsFull(_stMyQueue *myQueue);
int MyQueue_Push(_stMyQueue *myQueue, uint8_t data);
int MyQueue_MultiPush(_stMyQueue *myQueue, uint8_t *data, int len);
int MyQueue_Pop(_stMyQueue *myQueue, uint8_t *data);

#endif /* INC_MYQUEUE_H_ */
