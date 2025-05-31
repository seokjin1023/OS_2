#ifndef PROJECTS_CROSSROADS_PRIORITY_SYNCH_H
#define PROJECTS_CROSSROADS_PRIORITY_SYNCH_H

#include <list.h>
#include <stdbool.h>
#include "threads/thread.h"

/* A counting semaphore with priority. */
struct priority_semaphore
{
  unsigned value;
  struct list waiters; /* List of waiting threads */
};

void priority_sema_init(struct priority_semaphore *, unsigned value);
void priority_sema_down(struct priority_semaphore *);
bool priority_sema_try_down(struct priority_semaphore *);
void priority_sema_up(struct priority_semaphore *);
void priority_sema_self_test(void);

/* Priority Lock */
struct priority_lock
{
  struct thread *holder;
  struct priority_semaphore semaphore;
};

void priority_lock_init(struct priority_lock *);
void priority_lock_acquire(struct priority_lock *);
bool priority_lock_try_acquire(struct priority_lock *);
void priority_lock_release(struct priority_lock *);
bool priority_lock_held_by_current_thread(const struct priority_lock *);

/* Priority Condition */
struct priority_condition
{
  struct list waiters;
};

void priority_cond_init(struct priority_condition *);
void priority_cond_wait(struct priority_condition *, struct priority_lock *);
void priority_cond_signal(struct priority_condition *, struct priority_lock *);
void priority_cond_broadcast(struct priority_condition *, struct priority_lock *);

#define priority_barrier() asm volatile("" : : : "memory")

#endif /* PROJECTS_CROSSROADS_PRIORITY_SYNCH_H */