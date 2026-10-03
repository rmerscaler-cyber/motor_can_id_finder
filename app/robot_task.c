#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "robot_task.h"
#include "robot_application.h"
#include "task.h"

/*
 * robot_task 是 FreeRTOS 外壳，不实现控制决策。它唯一负责：
 * 1) 等待底层启动窗口结束；
 * 2) 按固定周期调用 robot_application_step；
 * 3) 记录调度耗时，便于在调试器中判断是否超期。
 */
volatile robot_task_statistics_t robot_task_statistics;

static void robot_task_debug_update(TickType_t start_tick,
                                    TickType_t period_ticks);

void robot_task(void *pvParameters)
{
    TickType_t last_wake_time;
    TickType_t start_tick;
    /* pdMS_TO_TICKS() 是 FreeRTOS 宏：按 configTICK_RATE_HZ 把毫秒换成 tick，
     * 避免任务周期直接写死在某一块板的 tick 频率上。 */
    const TickType_t period_ticks = pdMS_TO_TICKS(ROBOT_TASK_PERIOD_MS);

    (void)pvParameters;
    /* vTaskDelay() 按相对 tick 阻塞当前任务，不占 CPU；用于启动错峰。 */
    vTaskDelay(pdMS_TO_TICKS(ROBOT_TASK_INIT_TIME_MS));
    /* xTaskGetTickCount() 返回当前 RTOS tick，作为周期基准和耗时采样点。 */
    last_wake_time = xTaskGetTickCount();

    for (;;) {
        start_tick = xTaskGetTickCount();
        robot_task_statistics.last_start_tick = (uint32_t)start_tick;

        /* 翎控 ID 查询流程封装在 application，task 只负责固定周期调度。 */
        robot_application_step(HAL_GetTick());

        robot_task_debug_update(start_tick, period_ticks);

        /* vTaskDelayUntil() 以绝对唤醒时刻等待，能避免普通 delay 的周期漂移。 */
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

// ============================= debug ============================ //
/* 与 H7 参考 task 一样，统计代码集中在文件末，仅记录调度观测量。 */
static void robot_task_debug_update(TickType_t start_tick,
                                    TickType_t period_ticks)
{
    TickType_t run_ticks = xTaskGetTickCount() - start_tick;
    robot_task_statistics.last_run_ticks = (uint32_t)run_ticks;
    if ((uint32_t)run_ticks > robot_task_statistics.max_run_ticks)
        robot_task_statistics.max_run_ticks = (uint32_t)run_ticks;
    if (run_ticks >= period_ticks)
        robot_task_statistics.overrun_count++;
    robot_task_statistics.cycle_count++;
}
