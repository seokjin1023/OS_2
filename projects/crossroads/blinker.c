
#include <list.h>
#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/blinker.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/position.h"
#include "projects/crossroads/priority_synch.h"

struct blinker_info *blinker;
struct vehicle_info *vehicles;

struct priority_semaphore v_sema;
struct lock blinker_lock;
/*
교차로 진입을 위한 차들을 관리하는 semaphore 하나
각 blinker가 중앙 semaphore를 조회하고 각자 판단 후 entry 허용
- lock을 통해서 진입 가능한지 확인할 예정임.
*/

struct position blinker_cell[4] = {
    {2, 2}, {2, 4}, {4, 4}, {4, 2}
};

void init_blinker(struct blinker_info *blinkers, struct lock **map_locks, struct vehicle_info *vehicle_info)
{
    for (int i = 0; i < NUM_BLINKER; i++)
    {
        sema_init(&blinkers->blinker_sema);
    }
    priority_sema_init(&v_sema, 4);
    lock_init(&blinker_lock);
    vehicles = vehicle_info;
    blinker = blinkers;
}

//본인 담당 구역만 맞는지 확인
bool can_enter_partial(struct vehicle_info *vi, int blinker_id) {
    int start = vi->start - 'A';
    int dest = vi->dest - 'A';
    int blinker_row = blinker_cell[blinker_id].row;
    int blinker_col = blinker_cell[blinker_id].col;
    for(int i = 0; i < 12; i++) {
        if(blinker_row == vehicle_path[start][dest][i].row 
            && blinker_col == vehicle_path[start][dest][i].col) {
            //check can get lock
        }
    }
    return true;
}

void blinker_thread(void *arg) {
    int id = (int)arg;
    
    while (true) {
        lock_acquire(&blinker_lock);

        struct list_elem *e = list_begin(&v_sema.waiters);
        while (e != list_end(&v_sema.waiters)) {
            struct vehicle_info *vi = list_entry(e, struct vehicle_info, elem);

            if (vi->being_considered)
                goto next;

            if (can_enter_partial(vi, id)) {
                vi->passed_blinkers++;
                if (vi->passed_blinkers == NUM_BLINKER) {
                    vi->being_considered = true;
                    acquire_all_locks(vi);
                    list_remove(e);
                    thread_unblock(vi->t);
                }
            }

        next:
            e = list_next(e);
        }

        lock_release(&blinker_lock);
        thread_yield();
    }
}
void start_blinker()
{
    for (int i = 0; i < NUM_BLINKER; i++)
    {
        char name[16];
        snprintf(name, sizeof name, "blinker_%d", i);
        thread_create(name, PRI_DEFAULT, blinker_thread, i);
    }
}
