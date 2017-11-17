//
// Created by smart on 10/30/17.
//

#ifndef SMARTBOX_QUEUE_LIST_H
#define SMARTBOX_QUEUE_LIST_H

#include <stdint.h>

typedef uint32_t Item;
typedef struct node * PNode;
typedef struct node
{
    Item data;
    PNode next;
}Node;

typedef struct
{
    PNode front;
    PNode rear;
    int size;
}Queue;

/* construct a queue */
Queue *InitQueue();

/* destroy queue */
void DestroyQueue(Queue *pqueue);

/* clear queue */
void ClearQueue(Queue *pqueue);

/* check if  queue is empty */
int IsEmpty(Queue *pqueue);

/*return size of queue */
int GetSize(Queue *pqueue);

/* return the first node of queue */
PNode GetFront(Queue *pqueue,Item *pitem);

/* return the tail node of queue */
PNode GetRear(Queue *pqueue,Item *pitem);

/*insert a new node */
PNode EnQueue(Queue *pqueue,Item item);

/* remove node form head of queue */
PNode DeQueue(Queue *pqueue,Item *pitem);

/* through queue by call visit function */
void QueueTraverse(Queue *pqueue,void (*visit)());




#endif //SMARTBOX_QUEUE_LIST_H
