
#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/blinker.h"
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

void init_blinker(struct blinker_info *blinkers, struct lock **map_locks, struct vehicle_info *vehicle_info)
{
    for (int i = 0; i < NUM_BLINKER; i++)
    {
        blinkers[i].map_locks = map_locks;
        blinkers[i].vehicle_info = vehicle_info;
    }
    priority_sema_init(&v_sema, 4);
    lock_init(&blinker_lock);
    vehicles = vehicle_info;
    blinker = blinkers;
}
/* 주어진 차량의 경로가 map_locks 기준으로 모두 비어 있는지 확인 */
bool can_enter(struct vehicle_info *vi, struct lock **map_locks)
{
}

/* 교차로 내부 경로에 해당하는 모든 map_locks를 lock_acquire */
void acquire_all_locks(struct vehicle_info *vi, struct lock **map_locks);

void blinker_thread(void *arg)
{
    int id = (int)(uintptr_t)arg;
    struct blinker_info *bi = &blinker[id];

    while (true)
    {
        lock_acquire(&blinker_lock);

        if (!list_empty(&v_sema.waiters))
        {
            // 우선순위 정렬된 대기자 중에서 차량 thread를 가져옴
            struct thread *t = list_entry(list_front(&v_sema.waiters), struct thread, elem);
            struct vehicle_info *vi = t->vehicle_info;

            // ✅ 여기서 진입 가능한지 판단하는 함수는 네가 구현
            if (can_enter(vi, bi->map_locks))
            {
                // 경로 lock들을 확보
                acquire_all_locks(vi, bi->map_locks);

                // 큐에서 제거하고 unblock
                list_pop_front(&v_sema.waiters);
                thread_unblock(t); // vehicle thread 진입 허용
                lock_release(&blinker_lock);

                // vehicle이 교차로를 지나도록 대기 또는 로그
                thread_yield(); // 블링커는 다음 차량 보기 위해 잠시 양보
                continue;
            }
        }

        lock_release(&blinker_lock);
        thread_yield(); // 아무것도 못 했으면 양보
    }
}

void start_blinker()
{
    for (int i = 0; i < NUM_BLINKER; i++)
    {
        char name[16];
        snprintf(name, sizeof name, "blinker_%d", i);
        thread_create(name, PRI_DEFAULT, blinker_thread, (void *)(uintptr_t)i);
    }
}
