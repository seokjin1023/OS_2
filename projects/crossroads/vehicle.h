#ifndef __PROJECTS_PROJECT2_VEHICLE_H__
#define __PROJECTS_PROJECT2_VEHICLE_H__

#include "projects/crossroads/position.h"

#define VEHICLE_STATUS_READY 0
#define VEHICLE_STATUS_RUNNING 1
#define VEHICLE_STATUS_FINISHED 2

#define VEHICL_TYPE_NORMAL 0
#define VEHICL_TYPE_AMBULANCE 1

struct vehicle_info {
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
    int            passed_blinkers;    /* blinker 스레드가 몇 개 통과했는지 */
    bool           being_considered;   /* blinker 스레드가 전체 lock 획득 시도 중인지 */
    struct position *needed_locks;       /* 이 차량이 교차로 진입 전에 미리 잡아야 하는 lock 목록 */
    int            needed_lock_cnt;    /* needed_locks 배열 길이(0~NUM_BLINKER 사이) */
	struct position crossroads_entry_point; /* 교차로 진입점*/
	struct position crossroads_exit_point; /* 교차로 탈출점*/
    /* ============================================== */
};


extern const struct position vehicle_path[4][4][12];

void vehicle_loop(void *vi);

void parse_vehicles(struct vehicle_info *vehicle_info, char *input);

#endif /* __PROJECTS_PROJECT2_VEHICLE_H__ */