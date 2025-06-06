#ifndef __PROJECTS_PROJECT2_BLINKER_H__
#define __PROJECTS_PROJECT2_BLINKER_H__

#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/position.h"

/** you can change the number of blinkers */
#define NUM_BLINKER 4

extern struct blinker_info *blinker;
extern const struct position blinker_cell[4];
extern struct lock is_green_lock;
// {2,2}, {2,4}, {4,4}, {4,2} 순서대로 맞는지 확인함.
extern int is_green[NUM_BLINKER];

struct blinker_info
{
    struct lock **map_locks;
    struct vechicle_info *vehicle_info;
};

void init_blinker(struct blinker_info *blinkers, struct lock **map_locks, struct vehicle_info *vehicle_info);
void start_blinker(void);

#endif /* __PROJECTS_PROJECT2_BLINKER_H__ */