#include "motor_reg.h"
#include "lk_motor.h"
#include "DM_motor.h"

/* 注册表顺序：A、B、C、yaw。四台均已实测连接 FDCAN1。
 * DM：cmd_id = ESC_ID，can_id = MST_ID；LK 只使用共用收发 can_id。
 * PID 整个配置置零，output_max 暂为 0，待确定控制/补偿参数后再配置。
 * direction=1、offset_ecd=0、current_scale=1 是待标定的初始值；
 * current_scale=1 不表示已经确认翎控电流量化比例。
 */
const motor_reg_t motor_regs[] = {
    {
        .id = MOTOR_A, .type = MOTOR_TYPE_DM, .driver = &dm_motor_driver_ops,
        .can_bus = MOTOR_BUS_CAN1, .cmd_id = 0x001u, .can_id = 0x007u,
        .mode = MOTOR_MODE_POSITION, .output_max = 0.0f,
        .direction = 1, .offset_ecd = 0u, .current_scale = 1.0f,
        .reduction = 1.0f, .wheel_radius = 0.0f,
        .angle_pid = {0}, .speed_pid = {0},
        .speed_feedback = MOTOR_SPEED_FEEDBACK_ANGULAR,
    },
    {
        .id = MOTOR_B, .type = MOTOR_TYPE_DM, .driver = &dm_motor_driver_ops,
        .can_bus = MOTOR_BUS_CAN1, .cmd_id = 0x002u, .can_id = 0x008u,
        .mode = MOTOR_MODE_POSITION, .output_max = 0.0f,
        .direction = 1, .offset_ecd = 0u, .current_scale = 1.0f,
        .reduction = 1.0f, .wheel_radius = 0.0f,
        .angle_pid = {0}, .speed_pid = {0},
        .speed_feedback = MOTOR_SPEED_FEEDBACK_ANGULAR,
    },
    {
        .id = MOTOR_C, .type = MOTOR_TYPE_DM, .driver = &dm_motor_driver_ops,
        .can_bus = MOTOR_BUS_CAN1, .cmd_id = 0x003u, .can_id = 0x009u,
        .mode = MOTOR_MODE_POSITION, .output_max = 0.0f,
        .direction = 1, .offset_ecd = 0u, .current_scale = 1.0f,
        .reduction = 1.0f, .wheel_radius = 0.0f,
        .angle_pid = {0}, .speed_pid = {0},
        .speed_feedback = MOTOR_SPEED_FEEDBACK_ANGULAR,
    },
    {
        .id = MOTOR_YAW, .type = MOTOR_TYPE_LK, .driver = &lk_motor_driver_ops,
        .can_bus = MOTOR_BUS_CAN1, .can_id = 0x142u,
        .mode = MOTOR_MODE_POSITION, .output_max = 0.0f,
        .direction = 1, .offset_ecd = 0u, .current_scale = 1.0f,
        .reduction = LK_REDUCTION, .wheel_radius = 0.0f,
        .angle_pid = {0}, .speed_pid = {0},
        .speed_feedback = MOTOR_SPEED_FEEDBACK_ANGULAR,
    },
};
const uint8_t motor_reg_count = (uint8_t)(sizeof(motor_regs) / sizeof(motor_regs[0]));

/* A/B/C 的 DM 参数只读查询与 yaw 的 LK 查询均由这里绑定协议。
 * DM 范围表示数据域目标 ESC_ID，查询帧本身发往固定 0x7FF。 */
const motor_probe_config_t motor_probe_regs[] = {
    { .id = MOTOR_A, .driver = &dm_motor_driver_ops,
      .first_can_id = 0u, .last_can_id = 0x7ffu },
    { .id = MOTOR_B, .driver = &dm_motor_driver_ops,
      .first_can_id = 0u, .last_can_id = 0x7ffu },
    { .id = MOTOR_C, .driver = &dm_motor_driver_ops,
      .first_can_id = 0u, .last_can_id = 0x7ffu },
    { .id = MOTOR_YAW, .driver = &lk_motor_driver_ops,
      .first_can_id = LK_CAN_ID_BASE + LK_MOTOR_ID_MIN,
      .last_can_id = LK_CAN_ID_BASE + LK_MOTOR_ID_MAX },
};
const uint8_t motor_probe_reg_count =
    (uint8_t)(sizeof(motor_probe_regs) / sizeof(motor_probe_regs[0]));
