#include "robot_application.h"
#include "robot_application_internal.h"
#include "motor.h"
#include "dm_id_application.h"
#include <string.h>

volatile robot_lk_scan_control_t robot_lk_scan;

void robot_application_init(void)
{
    memset((void *)&robot_lk_scan, 0, sizeof(robot_lk_scan));
    dm_id_application_init();
    robot_lk_scan.probe = motor_probe_register(MOTOR_YAW);
    if (robot_lk_scan.probe == NULL) return;
    robot_lk_scan.enabled = 0u; /* 翎控已经确认 FDCAN1 / 0x142，本轮默认查询 DM。 */
    robot_lk_scan.automatic = 1u;
    robot_lk_scan.bus_mask = 0x07u;
    robot_lk_scan.manual_can_id = robot_lk_scan.probe->first_can_id;
    robot_lk_scan.next_can_id = robot_lk_scan.probe->first_can_id;
    robot_lk_scan.interval_ms = 50u;
}

static void robot_lk_scan_step(uint32_t now_ms)
{
    uint8_t bus, mask, i;
    uint16_t can_id;
    uint32_t interval;
    motor_probe_handle_t probe = robot_lk_scan.probe;

    if (!robot_lk_scan.enabled || probe == NULL)
        return;
    interval = robot_lk_scan.interval_ms;
    if (interval == 0u) interval = 1u;
    if ((uint32_t)(now_ms - robot_lk_scan.last_query_ms) < interval)
        return;
    robot_lk_scan.last_query_ms = now_ms;

    mask = robot_lk_scan.bus_mask & 0x07u;
    if (robot_lk_scan.automatic) {
        if (mask == 0u) {
            robot_lk_scan.invalid_config_count++;
            return;
        }
        bus = robot_lk_scan.next_bus;
        can_id = robot_lk_scan.next_can_id;
        if (bus >= MOTOR_BUS_COUNT || (mask & (1u << bus)) == 0u) {
            for (bus = 0u; (mask & (1u << bus)) == 0u; bus++) {}
            can_id = probe->first_can_id;
        }
    } else {
        bus = robot_lk_scan.manual_bus;
        can_id = robot_lk_scan.manual_can_id;
    }

    if (bus >= MOTOR_BUS_COUNT || can_id < probe->first_can_id ||
        can_id > probe->last_can_id) {
        robot_lk_scan.invalid_config_count++;
        return;
    }
    robot_lk_scan.last_bus = bus;
    robot_lk_scan.last_can_id = can_id;
    robot_lk_scan.query_count++;
    robot_lk_scan.last_enqueue_ok =
        motor_probe_request(probe, (motor_bus_e)bus, can_id);
    if (!robot_lk_scan.last_enqueue_ok)
        robot_lk_scan.enqueue_fail_count++;

    if (!robot_lk_scan.automatic)
        return;
    /* 每次只发送一帧；入队失败也继续遍历，避免无电机的总线阻塞其他总线。 */
    if (can_id < probe->last_can_id) {
        robot_lk_scan.next_bus = bus;
        robot_lk_scan.next_can_id = can_id + 1u;
    } else {
        for (i = 1u; i <= MOTOR_BUS_COUNT; i++) {
            uint8_t next = (bus + i) % MOTOR_BUS_COUNT;
            if ((mask & (1u << next)) == 0u) continue;
            if (next <= bus) robot_lk_scan.sweep_count++;
            robot_lk_scan.next_bus = next;
            robot_lk_scan.next_can_id = probe->first_can_id;
            break;
        }
    }
}

void robot_application_step(uint32_t now_ms)
{
    dm_id_application_step(now_ms);
    robot_lk_scan_step(now_ms);
}
