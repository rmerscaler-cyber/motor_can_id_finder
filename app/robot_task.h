#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include <stdint.h>

#define ROBOT_TASK_INIT_TIME_MS  201u
#define ROBOT_TASK_PERIOD_MS       1u

/* 仅供调试观察任务调度时间；robot_task 是这些字段的唯一写者。 */
typedef struct
{
    uint32_t cycle_count;
    uint32_t last_start_tick;
    uint32_t last_run_ticks;
    uint32_t max_run_ticks;
    uint32_t overrun_count;
} robot_task_statistics_t;

extern volatile robot_task_statistics_t robot_task_statistics;

/* H7 使用 CMSIS-RTOS2 的 void * 任务参数签名。 */
void robot_task(void *pvParameters);

#endif /* ROBOT_TASK_H */
