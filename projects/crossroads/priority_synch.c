#include <stdio.h>
#include <string.h>
#include "projects/crossroads/priority_synch.h"
#include "threads/interrupt.h"
#include "threads/thread.h"

/* Priority comparison function for sorting waiters list. */
static bool priority_sema_priority_less(const struct list_elem *a,
                                        const struct list_elem *b,
                                        void *aux UNUSED)
{
    return list_entry(a, struct thread, elem)->priority >
           list_entry(b, struct thread, elem)->priority;
}

void priority_sema_init(struct priority_semaphore *sema, unsigned value)
{
    ASSERT(sema != NULL);
    sema->value = value;
    list_init(&sema->waiters);
}

void priority_sema_down(struct priority_semaphore *sema)
{
    enum intr_level old_level;

    ASSERT(sema != NULL);
    ASSERT(!intr_context());

    old_level = intr_disable();
    while (sema->value == 0)
    {
        list_insert_ordered(&sema->waiters, &thread_current()->elem,
                            priority_sema_priority_less, NULL);
        thread_block();
    }
    sema->value--;
    intr_set_level(old_level);
}

bool priority_sema_try_down(struct priority_semaphore *sema)
{
    enum intr_level old_level;
    bool success;

    ASSERT(sema != NULL);

    old_level = intr_disable();
    if (sema->value > 0)
    {
        sema->value--;
        success = true;
    }
    else
        success = false;
    intr_set_level(old_level);

    return success;
}

void priority_sema_up(struct priority_semaphore *sema)
{
    enum intr_level old_level;

    ASSERT(sema != NULL);

    old_level = intr_disable();
    if (!list_empty(&sema->waiters))
    {
        list_sort(&sema->waiters, priority_sema_priority_less, NULL);
        thread_unblock(list_entry(list_pop_front(&sema->waiters),
                                  struct thread, elem));
    }
    sema->value++;
    if (!intr_context())
        thread_yield();
    intr_set_level(old_level);
}

static void priority_sema_test_helper(void *sema_);

void priority_sema_self_test(void)
{
    struct priority_semaphore sema[2];
    int i;

    printf("Testing semaphores...");
    priority_sema_init(&sema[0], 0);
    priority_sema_init(&sema[1], 0);
    thread_create("sema-test", PRI_DEFAULT, priority_sema_test_helper, &sema);
    for (i = 0; i < 10; i++)
    {
        priority_sema_up(&sema[0]);
        priority_sema_down(&sema[1]);
    }
    printf("done.\n");
}

static void priority_sema_test_helper(void *sema_)
{
    struct priority_semaphore *sema = sema_;
    int i;

    for (i = 0; i < 10; i++)
    {
        priority_sema_down(&sema[0]);
        priority_sema_up(&sema[1]);
    }
}

void priority_lock_init(struct priority_lock *lock)
{
    ASSERT(lock != NULL);
    lock->holder = NULL;
    priority_sema_init(&lock->semaphore, 1);
}

void priority_lock_acquire(struct priority_lock *lock)
{
    ASSERT(lock != NULL);
    ASSERT(!intr_context());
    ASSERT(!priority_lock_held_by_current_thread(lock));

    priority_sema_down(&lock->semaphore);
    lock->holder = thread_current();
}

bool priority_lock_try_acquire(struct priority_lock *lock)
{
    bool success;

    ASSERT(lock != NULL);
    ASSERT(!priority_lock_held_by_current_thread(lock));

    success = priority_sema_try_down(&lock->semaphore);
    if (success)
        lock->holder = thread_current();
    return success;
}

void priority_lock_release(struct priority_lock *lock)
{
    ASSERT(lock != NULL);
    ASSERT(priority_lock_held_by_current_thread(lock));

    lock->holder = NULL;
    priority_sema_up(&lock->semaphore);
}

bool priority_lock_held_by_current_thread(const struct priority_lock *lock)
{
    ASSERT(lock != NULL);
    return lock->holder == thread_current();
}

struct priority_semaphore_elem
{
    struct list_elem elem;
    struct priority_semaphore semaphore;
    int priority;
};

void priority_cond_init(struct priority_condition *cond)
{
    ASSERT(cond != NULL);
    list_init(&cond->waiters);
}

void priority_cond_wait(struct priority_condition *cond, struct priority_lock *lock)
{
    struct priority_semaphore_elem waiter;
    ASSERT(cond != NULL);
    ASSERT(lock != NULL);
    ASSERT(!intr_context());
    ASSERT(priority_lock_held_by_current_thread(lock));

    priority_sema_init(&waiter.semaphore, 0);
    waiter.priority = thread_current()->priority;
    list_insert_ordered(&cond->waiters, &waiter.elem,
                        priority_sema_priority_less, NULL);

    priority_lock_release(lock);
    priority_sema_down(&waiter.semaphore);
    priority_lock_acquire(lock);
}

void priority_cond_signal(struct priority_condition *cond, struct priority_lock *lock UNUSED)
{
    ASSERT(cond != NULL);
    ASSERT(lock != NULL);
    ASSERT(!intr_context());
    ASSERT(priority_lock_held_by_current_thread(lock));

    if (!list_empty(&cond->waiters))
    {
        list_sort(&cond->waiters, priority_sema_priority_less, NULL);
        priority_sema_up(&list_entry(list_pop_front(&cond->waiters),
                                     struct priority_semaphore_elem, elem)
                              ->semaphore);
    }
}

void priority_cond_broadcast(struct priority_condition *cond, struct priority_lock *lock)
{
    ASSERT(cond != NULL);
    ASSERT(lock != NULL);

    while (!list_empty(&cond->waiters))
        priority_cond_signal(cond, lock);
}
