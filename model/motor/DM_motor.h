#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "motor_driver.h"
#define MIT_MODE 0x000
#define POS_MODE 0x100
#define SPEED_MODE 0x200

/* 官方 motor-sdk 的只读寄存器查询协议；不是 MIT 使能/控制帧。 */
#define DM_PARAMETER_CAN_ID 0x7ffu
#define DM_READ_PARAMETER   0x33u
#define DM_RID_MST_ID       7u
#define DM_RID_ESC_ID       8u

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#ifndef PI
#define PI 3.14159265358979f
#endif

/* DM MIT 协议原始反馈: CAN 中断写入, 仅本驱动 read_unfiltered_feedback 换算读取。 */
typedef struct {
    uint16_t position;     /* MIT 位置协议整数 */
    int16_t  speed;        /* MIT 速度协议整数 */
    int16_t  torque;       /* MIT 扭矩协议整数 */
    int8_t   temperature;  /* 温度协议值 */
} dm_motor_raw_t;

/* DM MIT 内环参数: 上电全零, 运行期由 Watch 修改 dm_motor_debug->assist,
 * DM 驱动每周期组帧时读取, 不进入统一基类。 */
typedef struct {
    fp32 pos;   /* MIT 位置指令 rad */
    fp32 vel;   /* MIT 速度指令 rad/s */
    fp32 kp;    /* MIT 位置增益 (0~500) */
    fp32 kd;    /* MIT 速度增益 (0~5) */
} dm_motor_assist_t;

typedef struct {
    motor_base_t base;        /* 第一成员: id/type/fb/ctrl/online */
    motor_bus_e can_bus;      /* registry 拷入 */
    uint16_t    can_id;       /* 反馈 ID（由注册表配置） */
    uint16_t    cmd_id;       /* 命令基准 ID，发送 ID = cmd_id + mode */
    dm_motor_raw_t raw;       /* 协议原始值: CAN 中断写, 驱动内部换算 */
    dm_motor_assist_t assist; /* MIT 内环参数: Watch 调, 组帧每周期读 */
    uint8_t     enable;       /* 使能命令成功进入 CAN 邮箱后置 1 */
    uint8_t     command_delay_cycles; /* 使能后延迟控制帧周期数 */
    uint32_t    last_enable_tx_tick;  /* 掉线时限频重发使能 */
    volatile uint32_t command_tx_count;      /* MIT 控制帧成功进入 CAN 邮箱次数 */
    volatile uint32_t command_tx_fail_count; /* MIT 控制帧进入 CAN 邮箱失败次数 */
    volatile uint32_t enable_tx_count;       /* 使能帧成功进入 CAN 邮箱次数 */
    volatile uint32_t enable_tx_fail_count;  /* 使能帧进入 CAN 邮箱失败次数 */
    volatile uint32_t disable_tx_count;      /* 失能帧成功进入 CAN 邮箱次数 */
    volatile uint32_t disable_tx_fail_count; /* 失能帧进入 CAN 邮箱失败次数 */
    fp32 reduction;           /* 转子→输出端减速比；直驱为 1 */
    fp32 wheel_radius;        /* 轮半径 m (fb_parse 线速度用, 无轮=0) */
} dm_motor_t;

/* 顶层 Watch 诊断量：不要求调试器理解 motor_base_t -> dm_motor_t 的强转。
 * 只用于观察 DM 驱动调用链，不参与任何控制判定。 */
typedef struct {
    volatile uint32_t pack_match_count;
    volatile uint32_t pack_skip_disabled_count;
    volatile uint32_t pack_skip_delay_count;
    volatile uint32_t command_attempt_count;
    volatile uint32_t command_enqueue_ok_count;
    volatile uint32_t command_enqueue_fail_count;
    volatile uint32_t enable_call_count;
    volatile uint32_t enable_enqueue_ok_count;
    volatile uint32_t enable_enqueue_fail_count;
    volatile uint32_t disable_call_count;
    volatile uint32_t disable_enqueue_ok_count;
    volatile uint32_t disable_enqueue_fail_count;
    volatile uint32_t last_pack_tick;
    volatile uint32_t last_command_tick;
    volatile uint16_t last_command_id;
    volatile uint8_t last_command_enqueue_ok;
    volatile uint8_t base_enabled;
    volatile uint8_t driver_enabled;
    volatile uint8_t online;
    volatile uint8_t command_delay_cycles;
    volatile uint32_t feedback_count;
    volatile uint32_t last_feedback_tick;
    volatile float last_output;
    volatile float last_position_assist_pos;
    volatile float last_position_assist_vel;
    volatile float last_position_assist_kp;
    volatile float last_position_assist_kd;
    volatile uint8_t last_command_data[8];
} dm_motor_trace_t;

/* Keil Watch 调试入口：指向首个注册成功的 DM4310 实例。
 * motor_handle_t 只暴露 base，调试时通过本指针可直接查看 DM 发送计数，
 * 以及 base.feedback_count / base.last_feedback_tick。 */
extern dm_motor_t * volatile dm_motor_debug;
extern volatile dm_motor_trace_t dm_motor_trace;


/* motor_reg.c 装配到配置行；具体协议函数保持驱动私有。 */
extern const motor_driver_ops_t dm_motor_driver_ops;

#endif
