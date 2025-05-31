
#include <stdio.h>
#include <string.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
    /* from A */ {
        /* to A */
        {
            {4, 0},
            {4, 1},
            {4, 2},
            {4, 3},
            {4, 4},
            {3, 4},
            {2, 4},
            {2, 3},
            {2, 2},
            {2, 1},
            {2, 0},
            {-1, -1},
        },
        /* to B */
        {
            {4, 0},
            {4, 1},
            {4, 2},
            {5, 2},
            {6, 2},
            {-1, -1},
        },
        /* to C */
        {
            {4, 0},
            {4, 1},
            {4, 2},
            {4, 3},
            {4, 4},
            {4, 5},
            {4, 6},
            {-1, -1},
        },
        /* to D */
        {
            {4, 0},
            {4, 1},
            {4, 2},
            {4, 3},
            {4, 4},
            {3, 4},
            {2, 4},
            {1, 4},
            {0, 4},
            {-1, -1},
        }},
    /* from B */ {/* to A */
                  {
                      {6, 4},
                      {5, 4},
                      {4, 4},
                      {3, 4},
                      {2, 4},
                      {2, 3},
                      {2, 2},
                      {2, 1},
                      {2, 0},
                      {-1, -1},
                  },
                  /* to B */
                  {
                      {6, 4},
                      {5, 4},
                      {4, 4},
                      {3, 4},
                      {2, 4},
                      {2, 3},
                      {2, 2},
                      {3, 2},
                      {4, 2},
                      {5, 2},
                      {6, 2},
                      {-1, -1},
                  },
                  /* to C */
                  {
                      {6, 4},
                      {5, 4},
                      {4, 4},
                      {4, 5},
                      {4, 6},
                      {-1, -1},
                  },
                  /* to D */
                  {
                      {6, 4},
                      {5, 4},
                      {4, 4},
                      {3, 4},
                      {2, 4},
                      {1, 4},
                      {0, 4},
                      {-1, -1},
                  }},
    /* from C */ {/* to A */
                  {
                      {2, 6},
                      {2, 5},
                      {2, 4},
                      {2, 3},
                      {2, 2},
                      {2, 1},
                      {2, 0},
                      {-1, -1},
                  },
                  /* to B */
                  {
                      {2, 6},
                      {2, 5},
                      {2, 4},
                      {2, 3},
                      {2, 2},
                      {3, 2},
                      {4, 2},
                      {5, 2},
                      {6, 2},
                      {-1, -1},
                  },
                  /* to C */
                  {
                      {2, 6},
                      {2, 5},
                      {2, 4},
                      {2, 3},
                      {2, 2},
                      {3, 2},
                      {4, 2},
                      {4, 3},
                      {4, 4},
                      {4, 5},
                      {4, 6},
                      {-1, -1},
                  },
                  /* to D */
                  {
                      {2, 6},
                      {2, 5},
                      {2, 4},
                      {1, 4},
                      {0, 4},
                      {-1, -1},
                  }},
    /* from D */ {/* to A */
                  {
                      {0, 2},
                      {1, 2},
                      {2, 2},
                      {2, 1},
                      {2, 0},
                      {-1, -1},
                  },
                  /* to B */
                  {
                      {0, 2},
                      {1, 2},
                      {2, 2},
                      {3, 2},
                      {4, 2},
                      {5, 2},
                      {6, 2},
                      {-1, -1},
                  },
                  /* to C */
                  {
                      {0, 2},
                      {1, 2},
                      {2, 2},
                      {3, 2},
                      {4, 2},
                      {4, 3},
                      {4, 4},
                      {4, 5},
                      {4, 6},
                      {-1, -1},
                  },
                  /* to D */
                  {
                      {0, 2},
                      {1, 2},
                      {2, 2},
                      {3, 2},
                      {4, 2},
                      {4, 3},
                      {4, 4},
                      {3, 4},
                      {2, 4},
                      {1, 4},
                      {0, 4},
                      {-1, -1},
                  }}};

void parse_vehicles(struct vehicle_info *vehicle_info, char *input)
{
    int thread_cnt = 1;
    for (int i = 0; (size_t)i < strlen(input); i++)
    {
        if (input[i] == ':')
        {
            thread_cnt++;
        }
    }

    // 2) strtok_r로 ':'을 구분자로 사용하여 토큰 분리
    char *token;
    char *save_ptr = NULL;

    token = strtok_r(input, ":", &save_ptr);
    for (int idx = 0; idx < thread_cnt; idx++)
    {
        // 토큰이 NULL일 경우(잘못된 입력) erro출력 및 시스템 종료
        if (token == NULL)
        {
            printf("Error: Not enough vehicle data provided.\n");
            shutdown_power_off();
        }
        size_t len = strlen(token);

        // 공통 필드: id, start, dest, state
        // token[0] = id, token[1] = start, token[2] = dest
        vehicle_info[idx].id = token[0];
        vehicle_info[idx].start = token[1];
        vehicle_info[idx].dest = token[2];
        vehicle_info[idx].state = VEHICLE_STATUS_READY; // 상태는 항상 READY로 고정

        if (len == 3)
        {
            // 길이가 3인 경우: 일반 차량
            vehicle_info[idx].type = VEHICL_TYPE_NORMAL;
            vehicle_info[idx].arrival = 0;
            vehicle_info[idx].golden_time = 0;
        }
        else
        {
            // 길이가 3보다 큰 경우: "fAB5.12"
            // type은 구급차(AMBULANCE)
            vehicle_info[idx].type = VEHICL_TYPE_AMBULANCE;

            // token + 3 위치부터 "arrival.golden_time" 형태
            char *numbers = token + 3;
            char *dot_pos = strchr(numbers, '.');
            if (dot_pos != NULL)
            {
                // arrival 문자열의 길이
                size_t arr_len = (size_t)(dot_pos - numbers);

                // arrival 부분
                char arrival_str[16] = {0};
                strlcpy(arrival_str, numbers, arr_len + 1);

                // golden_time 부분
                char *golden_str = dot_pos + 1;

                // 문자열을 정수로 변환 후 저장
                vehicle_info[idx].arrival = atoi(arrival_str);
                vehicle_info[idx].golden_time = atoi(golden_str);
            }
            else
            {
                // 혹시 '.'이 없을 경우 오류 출력 및 시스템 종료
                printf("Error: ambulance must have golden time.\n");
                shutdown_power_off();
            }
        }

        // 다음 토큰으로 이동
        token = strtok_r(NULL, ":", &save_ptr);
    }
}

static int is_position_outside(struct position pos)
{
    return (pos.row == -1 || pos.col == -1);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
    struct position pos_cur, pos_next;

    pos_next = vehicle_path[start][dest][step];
    pos_cur = vi->position;

    if (vi->state == VEHICLE_STATUS_RUNNING)
    {
        /* check termination */
        if (is_position_outside(pos_next))
        {
            /* actual move */
            vi->position.row = vi->position.col = -1;
            /* release previous */
            lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
            return 0;
        }
    }

    /* lock next position */
    lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
    if (vi->state == VEHICLE_STATUS_READY)
    {
        /* start this vehicle */
        vi->state = VEHICLE_STATUS_RUNNING;
    }
    else
    {
        /* release current position */
        lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
    }
    /* update position */
    vi->position = pos_next;

    return 1;
}

void init_on_mainthread(int thread_cnt)
{
    /* Called once before spawning threads */
}

void vehicle_loop(void *_vi)
{
    int res;
    int start, dest, step;

    struct vehicle_info *vi = _vi;

    start = vi->start - 'A';
    dest = vi->dest - 'A';

    vi->position.row = vi->position.col = -1;
    vi->state = VEHICLE_STATUS_READY;

    step = 0;
    while (1)
    {
        /* vehicle main code */
        res = try_move(start, dest, step, vi);
        if (res == 1)
        {
            step++;
        }

        /* termination condition. */
        if (res == 0)
        {
            break;
        }

        /* unitstep change! */
        unitstep_changed();
    }

    /* status transition must happen before sema_up */
    vi->state = VEHICLE_STATUS_FINISHED;
}
