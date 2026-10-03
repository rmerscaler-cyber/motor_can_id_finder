#ifndef DM_ID_APPLICATION_INTERNAL_H
#define DM_ID_APPLICATION_INTERNAL_H

#include "motor.h"
#define DM_ID_SLOT_COUNT 3u

typedef enum {
    DM_ID_IDLE = 0,
    DM_ID_SETTLING = 1,
    DM_ID_SCANNING = 2,
    DM_ID_FOUND = 3,
    DM_ID_CONFIG_ERROR = 4,
    DM_ID_REPLY_CONFLICT = 5,
} dm_id_scan_status_e;

typedef struct {
    /* Watch 控制：换上单台电机后设置 slot=0/1/2、start=1。 */
    uint8_t slot;
    uint8_t start;
    uint8_t enabled;
    uint8_t bus_mask;        /* bit0/1/2 = FDCAN1/2/3 */
    uint16_t first_id;
    uint16_t last_id;
    uint32_t interval_ms;
    uint32_t response_timeout_ms;
    uint32_t settle_ms;

    /* 初始化取得的统一诊断句柄，以及当前会话状态。 */
    motor_probe_handle_t probe[DM_ID_SLOT_COUNT];
    uint8_t active_slot;
    uint8_t active_bus_mask;
    uint8_t next_bus;
    uint8_t waiting;
    uint8_t retry_count;
    uint8_t last_enqueue_ok;
    uint16_t next_id;
    uint16_t active_first_id;
    uint16_t active_last_id;
    dm_id_scan_status_e status;
    uint32_t session;
    uint32_t session_start_ms;
    uint32_t last_query_ms;
    uint32_t query_count;
    uint32_t enqueue_fail_count;
    uint32_t timeout_count;
    uint32_t sweep_count;
} robot_dm_scan_control_t;

typedef struct {
    uint8_t found;          /* 两个 ID 寄存器均收到且无冲突后置 1 */
    uint8_t bus;            /* 0/1/2 = FDCAN1/2/3 */
    uint16_t cmd_id;        /* ESC_ID -> 正常注册表 .cmd_id */
    uint16_t can_id;        /* MST_ID -> 正常注册表 .can_id */
    uint16_t reply_can_id;  /* 实际参数应答帧 ID，保留用于对照 */
    uint32_t session;
    uint32_t confirmed_ms;
    uint32_t command_reply_count;
    uint32_t feedback_reply_count;
    uint8_t command_reply[8];
    uint8_t feedback_reply[8];
} dm_id_motor_debug_t;

typedef struct {
    dm_id_motor_debug_t motor[DM_ID_SLOT_COUNT]; /* 0=A，1=B，2=C */
    motor_probe_result_t current;               /* 当前候选的部分/完整回复 */
    uint32_t found_count;
    uint8_t duplicate_command_mask;
    uint8_t duplicate_feedback_mask;
} dm_id_scan_debug_t;

extern volatile robot_dm_scan_control_t robot_dm_scan;
extern volatile dm_id_scan_debug_t dm_id_scan_debug;

#endif
