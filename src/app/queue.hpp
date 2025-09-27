#ifndef __QUEUE_HPP__
#define __QUEUE_HPP__

#include "librm.hpp"

using namespace rm;

#define QSIZE 517

typedef struct {
  u8 RW_Lock;
  u8 arr[QSIZE];
  u32 front;
  i32 rear;
  u32 counter;
} Queue_t;

void QueueInit(Queue_t *q);
u8 IsFull(Queue_t *q);
u8 IsEmpty(Queue_t *q);
void EnQueue(Queue_t *q, u8 *val, u8 lenth);
int Pop(Queue_t *buffer1, Queue_t *buffer2, u8 data[11]);
void UI_EnQueue(Queue_t *q, u8 *val, u8 lenth);
int UI_Pop(Queue_t *buffer1, u8 *data);

#endif  /* __QUEUE_HPP__ */ 