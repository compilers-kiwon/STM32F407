/*
 * MyQueue.c
 *
 *  Created on: Apr 19, 2023
 *      Author: kkwon
 */

#include <stdio.h>
#include <stdlib.h>
#include "MyQueue.h"

_stMyQueue * MyQueue_Create (int size)
{
    _stMyQueue *myQueue;

    myQueue = (_stMyQueue *)malloc(sizeof(_stMyQueue));
    myQueue->data = (uint8_t *)malloc(sizeof(uint8_t) * size);
    myQueue->max_size = size;
    myQueue->front = myQueue->rear = 0;

    return myQueue;
}
int	MyQueue_Size(_stMyQueue *myQueue)
{
    if (myQueue->front > myQueue->rear) {
        return (myQueue->max_size - myQueue->front) + myQueue->rear;
    }

    return (myQueue->rear - myQueue->front);
}

int	MyQueue_IsEmpty(_stMyQueue *myQueue)
{
    return (myQueue->rear == myQueue->front);
}

int	MyQueue_IsFull(_stMyQueue *myQueue)
{
    return (((myQueue->rear + 1) % myQueue->max_size) == myQueue->front);
}

int	MyQueue_Push(_stMyQueue *myQueue, uint8_t data)
{
    if(MyQueue_IsFull(myQueue))
        return -1;

    myQueue->data[myQueue->rear] = data;
    myQueue->rear = (myQueue->rear + 1) %  myQueue->max_size;

    return 0;
}

int	MyQueue_MultiPush(_stMyQueue *myQueue, uint8_t *data, int len)
{
    int n;

    for(n = 0;n<len;n++) {
        if(MyQueue_IsFull(myQueue))
            return -1;

        myQueue->data[myQueue->rear] = *data++;
        myQueue->rear = (myQueue->rear + 1) %  myQueue->max_size;
    }

    return 0;
}

int	MyQueue_Pop(_stMyQueue *myQueue, uint8_t *data)
{
    if(MyQueue_IsEmpty(myQueue))
        return -1;

    *data = myQueue->data[myQueue->front];
    myQueue->front = (myQueue->front + 1) %  myQueue->max_size;

    return 0;
}
