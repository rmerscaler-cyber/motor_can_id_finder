#ifndef ROBOT_APPLICATION_INTERNAL_H
#define ROBOT_APPLICATION_INTERNAL_H

#include <stdint.h>
#include "motor.h"

/* 与 F4 application 的私有持久状态写法一致；调试器可修改控制字段。
 * bus 的数值 0/1/2 对应 FDCAN1/2/3。 */
typedef struct {
    motor_probe_handle_t probe; /* 初始化通过 motor.c 获取；应用不引用协议驱动。 */
    uint8_t enabled;          /* 0 暂停；1 查询 */
    uint8_t automatic;        /* 1 遍历；0 重复查询 manual_bus/manual_can_id */
    uint8_t bus_mask;         /* 自动模式：bit0/1/2 选择 FDCAN1/2/3 */
    uint8_t manual_bus;
    uint16_t manual_can_id;   /* 标准 CAN ID；有效范围来自注册配置。 */
    uint32_t interval_ms;

    /* 以下是应用状态，供 Watch 读取。 */
    uint8_t next_bus;
    uint16_t next_can_id;
    uint8_t last_bus;
    uint16_t last_can_id;
    uint8_t last_enqueue_ok;
    uint32_t last_query_ms;
    uint32_t query_count;
    uint32_t enqueue_fail_count;
    uint32_t invalid_config_count;
    uint32_t sweep_count;
} robot_lk_scan_control_t;

extern volatile robot_lk_scan_control_t robot_lk_scan;

#endif
