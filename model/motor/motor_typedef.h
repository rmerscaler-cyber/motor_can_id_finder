#ifndef MOTOR_TYPEDEF_H
#define MOTOR_TYPEDEF_H

#include "struct_typedef.h"
#include "pid.h"            /* pid_type_def (中间层闭环用) */
#include "motor_port.h"     /* motor_bus_e: 设备层逻辑 CAN 总线 */

typedef struct motor_driver_ops motor_driver_ops_t;

/*
 * 结构体分层：motor_reg_t 是只读拓扑/标定表，motor_base_t 是运行期实例，
 * motor_control_t 是中间层控制状态，具体驱动结构体把 motor_base_t 作为第一
 * 成员嵌入。这样既能用统一 handle 调试，又能在协议边界还原具体型号。
 */

/* 电机型号: 保留配置和调试语义；运行时协议实现由 driver ops 绑定 */
typedef enum {
    MOTOR_TYPE_NONE = 0,
    MOTOR_TYPE_DM,       /* 达妙 DM4310 */
    MOTOR_TYPE_LK,       /* 翎控 4010 v3；复用 LK 协议驱动 */
} motor_type_e;

/* 控制器的四个主动关节编号，与 MuJoCo 的 A/B/C/yaw 对应。 */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B,
    MOTOR_C,
    MOTOR_YAW,
    MOTOR_COUNT,
} motor_id_e;

/* 统一控制模式: 只描述 motor.c 内的 PID 拓扑，不表达具体电机协议模式。 */
typedef enum {
    MOTOR_MODE_SPEED,     /* 单速度环 */
    MOTOR_MODE_POSITION,  /* 角度→速度串级 */
    MOTOR_MODE_RELAY,     /* 协议命令透传；不在 motor.c 内运行 PID */
} motor_mode_e;

/* 速度环反馈选择。默认输出端角速度，特殊机构可选线速或转子速度。 */
typedef enum {
    MOTOR_SPEED_FEEDBACK_ANGULAR = 0, /* 输出端 speed_angle，rad/s */
    MOTOR_SPEED_FEEDBACK_LINEAR,      /* 线速度 speed_ms，m/s */
    MOTOR_SPEED_FEEDBACK_ROTOR,       /* 转子 speed_rotor，rad/s */
} motor_speed_feedback_e;

/* 电机配置: 一行一电机 = 硬件拓扑(驱动读) + 控制标定(中间层读), 扁平合并 (原 motor_cfgs 并入) */
typedef struct {
    motor_id_e   id;       /* 关节编号 */
    motor_type_e type;     /* 电机型号 (仅配置/调试语义, 运行时驱动由 driver 绑定) */
    const motor_driver_ops_t *driver; /* 具体协议 ops, 在 motor_reg.c 装配 */
    motor_bus_e  can_bus;  /* 逻辑总线，物理映射在各板 motor_port.c */
    uint16_t     can_id;   /* 反馈帧 ID */
    uint16_t     cmd_id;   /* 命令帧 ID（按电机协议解释） */
    uint8_t      cmd_slot; /* 命令帧槽位（按电机协议解释） */

    /* ---- 控制/标定配置 (原 motor_ctrl_cfg_t 扁平展开, 中间层读) ---- */
    motor_mode_e mode;             /* motor.c 控制模式 */
    fp32 output_max;               /* 最终输出绝对值上限: 单位由协议定义；DM 为输出扭矩 */

    int16_t  direction;            /* 正反 +1/-1 */
    uint16_t offset_ecd;           /* 编码器零点 */
    fp32     current_scale;        /* 电流 LSB→mA (上电标定) */
    fp32     reduction;            /* 转子→输出端减速比；直驱协议实例填 1.0f */
    fp32     wheel_radius;         /* 轮半径 m (无轮=0) */

    pid_config_t angle_pid;        /* 外角度环；舵向机构使用 PID_ANGLE。 */
    pid_config_t speed_pid;        /* 内速度环；统一使用 PID_LINEAR。 */

    motor_speed_feedback_e speed_feedback; /* 速度环反馈来源；默认输出端角速度。 */
} motor_reg_t;


/* ============ 唯一反馈接口体 ============
 * 承载最终物理量和供标定观察的原始位置码 ecd。其他协议整数仍由各驱动保存在
 * 私有结构体中；store_protocol_feedback 在 CAN 中断里写入，
 * read_unfiltered_feedback 填写本结构，再由 application 发布最终区。
 * 同一 application 任务可直接消费最终区；跨任务使用 motor_feedback_read()。 */
typedef struct
{
    /* 最终区：只允许 application feedback_update 通过 publish 覆盖。 */
    fp32 angle;             /* 最终单圈角度，rad，范围 [-pi, pi]。 */
    fp32 total_angle;       /* 最终累计角度，rad。 */
    fp32 speed_angle;       /* 最终输出端角速度，rad/s；协议层已处理减速比。 */
    fp32 speed_rotor;       /* 转子角速度，rad/s；直驱电机等于 speed_angle。 */
    fp32 speed_ms;          /* 最终线速度，m/s；无轮机构为 0。 */
    int16_t current;        /* 最终电流，mA。 */
    fp32 torque;            /* 最终输出扭矩，N·m；无估算能力时为 0。 */
    int8_t temperature;     /* 电机温度，摄氏度。 */
    uint8_t valid;          /* 已至少发布过一帧完整有效反馈。 */
    uint32_t timestamp_ms;  /* 当前快照对应的接收时间戳。 */
    uint32_t sequence;      /* 完整有效反馈帧序号。 */
    uint16_t ecd;           /* 原始位置码：DM 为 16 位量化位置。 */
} motor_feedback_t;

/* 统一控制: 上层设 ref, 中间层按 loops 跑闭环, 输出交给驱动发送 */
typedef struct
{
    motor_mode_e mode;        /* 控制模式 (init 从 cfg 拷入) */
    pid_type_def angle_pid;   /* 外环: 角度环 (ANGLE_SPEED 串级用) */
    pid_type_def speed_pid;   /* 内环: 速度环 */
    fp32 ref;                 /* 目标 (角度/速度/电流, 依 loops 最外层) */
    fp32 output;              /* 控制输出 (电流/扭矩/速度命令, 依 loops) */
} motor_control_t;

/* 统一基类 (注册层/应用唯一接口) */
typedef struct
{
    motor_id_e        id;
    motor_type_e      type;
    const motor_driver_ops_t *driver; /* 初始化期绑定，运行期只读，便于调试器沿实例查看 */
    const motor_reg_t *config;        /* 本实例配置，初始化期绑定，运行期只读 */
    motor_feedback_t  fb;             /* 唯一反馈接口体：application 写最终区。 */
    motor_control_t   ctrl;
    volatile uint32_t raw_feedback_version; /* 驱动私有原始值的帧序号：奇数写入中，偶数完整。 */
    volatile uint32_t feedback_version;     /* 最终 fb：奇数发布中，偶数完整。 */
    volatile uint32_t last_feedback_tick;   /* 最后一帧原始反馈的 HAL 毫秒时间戳。 */
    volatile uint32_t feedback_count;       /* 驱动收到的有效原始反馈帧计数。 */
    uint8_t           online;            /* motor.c 按最后反馈时间维护，便于 Watch */
    uint8_t           enabled;           /* 统一使能状态；失能时 motor_calc 保持 PID/output 清零 */
} motor_base_t;

/* Application 运行期唯一使用的电机引用；motor_id_e 只用于初始化注册和调试标识。 */
typedef motor_base_t *motor_handle_t;

/* 上电诊断配置：尚未知电机地址时使用，不创建/使能控制实例。
 * 与正常注册一样，只在 motor_reg.c 绑定具体协议。 */
typedef struct {
    motor_id_e id;
    const motor_driver_ops_t *driver;
    uint16_t first_can_id;
    uint16_t last_can_id;
} motor_probe_config_t;
typedef const motor_probe_config_t *motor_probe_handle_t;

/* 只读 ID 诊断的协议无关结果，独立于 motor->fb 物理量。 */
#define MOTOR_PROBE_COMMAND_ID   0x01u
#define MOTOR_PROBE_FEEDBACK_ID  0x02u
#define MOTOR_PROBE_IDS_COMPLETE (MOTOR_PROBE_COMMAND_ID | MOTOR_PROBE_FEEDBACK_ID)
typedef struct {
    uint16_t request_id;
    uint16_t value;
    uint8_t field;  /* COMMAND_ID / FEEDBACK_ID，由具体驱动解释寄存器。 */
} motor_probe_reply_t;

typedef struct {
    motor_bus_e bus;
    uint16_t request_id;
    uint16_t cmd_id;
    uint16_t feedback_id;
    uint16_t reply_can_id;
    uint8_t fields;
    uint8_t conflict;
    uint32_t command_reply_count;
    uint32_t feedback_reply_count;
    uint32_t last_rx_ms;
    uint8_t command_reply[8];
    uint8_t feedback_reply[8];
} motor_probe_result_t;

#endif /* MOTOR_TYPEDEF_H */
