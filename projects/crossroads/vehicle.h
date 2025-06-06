#ifndef __PROJECTS_PROJECT2_VEHICLE_H__
#define __PROJECTS_PROJECT2_VEHICLE_H__

#include "projects/crossroads/position.h"

#define VEHICLE_STATUS_READY 0
#define VEHICLE_STATUS_RUNNING 1
#define VEHICLE_STATUS_FINISHED 2

#define VEHICL_TYPE_NORMAL 0
#define VEHICL_TYPE_AMBULANCE 1

struct vehicle_info
{
    char id;
    char state;
    char start;
    char dest;

    char type;
    int arrival;
    int golden_time;

    struct position position;
    struct lock **map_locks;

    /* ========== 아래부터 새로 추가된 필드들 ========== */
    struct position crossroads_entry_point; /* 교차로 진입점*/
    struct position crossroads_exit_point;  /* 교차로 탈출점*/
    /* ============================================== */
};

extern const struct position vehicle_path[4][4][12];

void vehicle_loop(void *vi);

void parse_vehicles(struct vehicle_info *vehicle_info, char *input);

#endif /* __PROJECTS_PROJECT2_VEHICLE_H__ */