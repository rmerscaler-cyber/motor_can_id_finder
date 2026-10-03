#include "dm_id_application.h"
#include "dm_id_application_internal.h"
#include "motor.h"
#include <string.h>

volatile robot_dm_scan_control_t robot_dm_scan;
volatile dm_id_scan_debug_t dm_id_scan_debug;

static void dm_id_debug_clear_slot(uint8_t slot);
static void dm_id_debug_confirm(uint8_t slot, const motor_probe_result_t *result,
                                 uint32_t now_ms);
static void dm_id_debug_summary(void);

void dm_id_application_init(void)
{
    memset((void *)&robot_dm_scan, 0, sizeof(robot_dm_scan));
    memset((void *)&dm_id_scan_debug, 0, sizeof(dm_id_scan_debug));
    robot_dm_scan.probe[0] = motor_probe_register(MOTOR_A);
    robot_dm_scan.probe[1] = motor_probe_register(MOTOR_B);
    robot_dm_scan.probe[2] = motor_probe_register(MOTOR_C);
    robot_dm_scan.bus_mask = 0x07u;
    robot_dm_scan.last_id = 0x7ffu;
    robot_dm_scan.interval_ms = 20u;
    robot_dm_scan.response_timeout_ms = 50u;
    robot_dm_scan.settle_ms = 200u;
    robot_dm_scan.start = 0u; /* 三台 ID 已确认；需要重新扫描时在 Watch 写 start=1。 */
}

static uint8_t dm_id_begin(uint32_t now_ms)
{
    uint8_t slot = robot_dm_scan.slot;
    uint8_t mask = robot_dm_scan.bus_mask & 0x07u;
    uint16_t first = robot_dm_scan.first_id;
    uint16_t last = robot_dm_scan.last_id;
    motor_probe_handle_t probe;

    robot_dm_scan.start = 0u;
    motor_probe_cancel(NULL);
    robot_dm_scan.waiting = 0u;
    if (slot >= DM_ID_SLOT_COUNT || mask == 0u || first > last) goto bad_config;
    probe = robot_dm_scan.probe[slot];
    if (probe == NULL || first < probe->first_can_id || last > probe->last_can_id)
        goto bad_config;

    robot_dm_scan.active_slot = slot;
    robot_dm_scan.active_bus_mask = mask;
    robot_dm_scan.active_first_id = first;
    robot_dm_scan.active_last_id = last;
    robot_dm_scan.next_id = first;
    for (slot = 0u; (mask & (1u << slot)) == 0u; slot++) {}
    robot_dm_scan.next_bus = slot;
    robot_dm_scan.retry_count = 0u;
    robot_dm_scan.query_count = 0u;
    robot_dm_scan.enqueue_fail_count = 0u;
    robot_dm_scan.timeout_count = 0u;
    robot_dm_scan.sweep_count = 0u;
    robot_dm_scan.session++;
    robot_dm_scan.session_start_ms = now_ms;
    robot_dm_scan.last_query_ms = now_ms;
    robot_dm_scan.status = DM_ID_SETTLING;
    robot_dm_scan.enabled = 1u;
    dm_id_debug_clear_slot(robot_dm_scan.active_slot);
    return 1u;

bad_config:
    robot_dm_scan.status = DM_ID_CONFIG_ERROR;
    robot_dm_scan.enabled = 0u;
    return 0u;
}

/* 候选地址逐个递增，每个地址先查询所有选中的总线；小 ID 能较快找到。 */
static void dm_id_advance(void)
{
    uint8_t bus;
    uint8_t mask = robot_dm_scan.active_bus_mask;
    for (bus = robot_dm_scan.next_bus + 1u; bus < MOTOR_BUS_COUNT; bus++) {
        if ((mask & (1u << bus)) != 0u) {
            robot_dm_scan.next_bus = bus;
            return;
        }
    }
    if (robot_dm_scan.next_id < robot_dm_scan.active_last_id)
        robot_dm_scan.next_id++;
    else {
        robot_dm_scan.next_id = robot_dm_scan.active_first_id;
        robot_dm_scan.sweep_count++;
    }
    for (bus = 0u; (mask & (1u << bus)) == 0u; bus++) {}
    robot_dm_scan.next_bus = bus;
}

void dm_id_application_step(uint32_t now_ms)
{
    motor_probe_handle_t probe;
    motor_probe_result_t result = {0};
    uint8_t have_result;
    uint32_t interval, timeout;

    if (robot_dm_scan.start && !dm_id_begin(now_ms)) return;
    if (robot_dm_scan.active_slot >= DM_ID_SLOT_COUNT) return;
    probe = robot_dm_scan.probe[robot_dm_scan.active_slot];
    if (!robot_dm_scan.enabled || probe == NULL) {
        motor_probe_cancel(probe);
        robot_dm_scan.waiting = 0u;
        return;
    }
    if (robot_dm_scan.status == DM_ID_SETTLING) {
        if ((uint32_t)(now_ms - robot_dm_scan.session_start_ms) < robot_dm_scan.settle_ms)
            return;
        robot_dm_scan.status = DM_ID_SCANNING;
    }
    if (robot_dm_scan.status != DM_ID_SCANNING) return;

    if (robot_dm_scan.waiting) {
        have_result = motor_probe_result_read(probe, &result);
        if (have_result) {
            dm_id_scan_debug.current = result; /* 只读诊断镜像，不用于控制判定。 */
            if (result.conflict) {
                robot_dm_scan.status = DM_ID_REPLY_CONFLICT;
                robot_dm_scan.enabled = 0u;
                robot_dm_scan.waiting = 0u;
                motor_probe_cancel(probe);
                return;
            }
            if (result.fields == MOTOR_PROBE_IDS_COMPLETE) {
                dm_id_debug_confirm(robot_dm_scan.active_slot, &result, now_ms);
                robot_dm_scan.status = DM_ID_FOUND;
                robot_dm_scan.enabled = 0u; /* 当前结果锁定；换电机后再 start。 */
                robot_dm_scan.waiting = 0u;
                motor_probe_cancel(probe);
                return;
            }
        }
        timeout = robot_dm_scan.response_timeout_ms;
        if (timeout < 5u) timeout = 5u;
        if ((uint32_t)(now_ms - robot_dm_scan.last_query_ms) < timeout) return;
        robot_dm_scan.timeout_count++;
        motor_probe_cancel(probe);
        robot_dm_scan.waiting = 0u;
        /* 只收到一个寄存器或入队失败时，原地址最多额外重试两次。 */
        if ((have_result || !robot_dm_scan.last_enqueue_ok) && robot_dm_scan.retry_count < 2u)
            robot_dm_scan.retry_count++;
        else {
            robot_dm_scan.retry_count = 0u;
            dm_id_advance();
        }
    }

    interval = robot_dm_scan.interval_ms;
    if (interval < 5u) interval = 5u;
    if ((uint32_t)(now_ms - robot_dm_scan.last_query_ms) < interval) return;
    memset((void *)&dm_id_scan_debug.current, 0, sizeof(dm_id_scan_debug.current));
    robot_dm_scan.last_query_ms = now_ms;
    robot_dm_scan.query_count++;
    robot_dm_scan.last_enqueue_ok = motor_probe_request(probe,
        (motor_bus_e)robot_dm_scan.next_bus, robot_dm_scan.next_id);
    if (!robot_dm_scan.last_enqueue_ok) robot_dm_scan.enqueue_fail_count++;
    robot_dm_scan.waiting = 1u;
}

// ============================= debug ============================ //
/* 调试结果集中在应用文件末；决策使用统一 motor 诊断结果，不读取 debug 镜像。 */
static void dm_id_debug_summary(void)
{
    uint8_t i, j;
    dm_id_scan_debug.found_count = 0u;
    dm_id_scan_debug.duplicate_command_mask = 0u;
    dm_id_scan_debug.duplicate_feedback_mask = 0u;
    for (i = 0u; i < DM_ID_SLOT_COUNT; i++) {
        if (!dm_id_scan_debug.motor[i].found) continue;
        dm_id_scan_debug.found_count++;
        for (j = 0u; j < i; j++) {
            if (!dm_id_scan_debug.motor[j].found ||
                dm_id_scan_debug.motor[i].bus != dm_id_scan_debug.motor[j].bus) continue;
            if (dm_id_scan_debug.motor[i].cmd_id == dm_id_scan_debug.motor[j].cmd_id)
                dm_id_scan_debug.duplicate_command_mask |= (1u << i) | (1u << j);
            if (dm_id_scan_debug.motor[i].can_id == dm_id_scan_debug.motor[j].can_id)
                dm_id_scan_debug.duplicate_feedback_mask |= (1u << i) | (1u << j);
        }
    }
}

static void dm_id_debug_clear_slot(uint8_t slot)
{
    memset((void *)&dm_id_scan_debug.motor[slot], 0, sizeof(dm_id_scan_debug.motor[slot]));
    memset((void *)&dm_id_scan_debug.current, 0, sizeof(dm_id_scan_debug.current));
    dm_id_debug_summary();
}

static void dm_id_debug_confirm(uint8_t slot, const motor_probe_result_t *result,
                                 uint32_t now_ms)
{
    uint8_t i;
    volatile dm_id_motor_debug_t *target = &dm_id_scan_debug.motor[slot];
    target->bus = (uint8_t)result->bus;
    target->cmd_id = result->cmd_id;
    target->can_id = result->feedback_id;
    target->reply_can_id = result->reply_can_id;
    target->session = robot_dm_scan.session;
    target->confirmed_ms = now_ms;
    target->command_reply_count = result->command_reply_count;
    target->feedback_reply_count = result->feedback_reply_count;
    for (i = 0u; i < 8u; i++) {
        target->command_reply[i] = result->command_reply[i];
        target->feedback_reply[i] = result->feedback_reply[i];
    }
    target->found = 1u;
    dm_id_debug_summary();
}
