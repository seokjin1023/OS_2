#include <stdio.h>
#include "projects/crossroads/blinker.h"
#include "threads/thread.h"
#include "projects/crossroads/vehicle.h"

struct blinker_info *blinker;
struct lock is_green_lock;
// 이걸 통해 확인
int is_green[NUM_BLINKER] = {1, 1, 1, 1};

const struct position blinker_cell[4] = {
    {2, 2}, {2, 4}, {4, 4}, {4, 2}};

void init_blinker(struct blinker_info *blinkers, struct lock **map_locks, struct vehicle_info *vehicle_info)
{
    for (int i = 0; i < NUM_BLINKER; i++)
    {
        blinkers[i].map_locks = map_locks;
        blinkers[i].vehicle_info = vehicle_info;
    }
    blinker = blinkers;
    lock_init(&is_green_lock);
}

void blinker_thread(void *arg)
{
    int id = (int)arg;
    struct blinker_info *b = &blinker[id];
    struct lock **map_locks = b->map_locks;

    /* 이 블링커가 담당하는 단일 셀 좌표 */
    int row = blinker_cell[id].row;
    int col = blinker_cell[id].col;

    while (true)
    {
        /* 1) 해당 셀이 비었는지(lock_try_acquire 성공 여부 확인) */
        if (lock_try_acquire(&map_locks[row][col]))
        {
            /* 락을 단기간 빌려온 뒤, 곧바로 release → “지금 비어 있다” 표시 */
            lock_acquire(&is_green_lock);
            is_green[id] = 1;
            lock_release(&is_green_lock);
            lock_release(&map_locks[row][col]);
        }
        else
        {
            /* 락을 못 잡았다는 것은 “차가 이미 있다” → 적색(signal off) */
            lock_acquire(&is_green_lock);
            is_green[id] = 0;
            lock_release(&is_green_lock);
        }

        /*
         * (필수) CPU 점유를 조금이라도 완화하기 위해 즉시 yield.
         * 임의의 msleep은 발생시키지 않습니다.
         */
        thread_yield();
    }
}
void start_blinker(void)
{
    for (int i = 0; i < NUM_BLINKER; i++)
    {
        char name[16];
        snprintf(name, sizeof name, "blinker_%d", i);
        thread_create(name, PRI_DEFAULT, blinker_thread, i);
    }
}
