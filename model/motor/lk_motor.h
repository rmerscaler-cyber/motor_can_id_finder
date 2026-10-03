#ifndef LK_MOTOR_H
#define LK_MOTOR_H


#include "motor_driver.h"
#define LK_CAN_ID_BASE      0x140u
#define LK_MOTOR_ID_MIN     1u
#define LK_MOTOR_ID_MAX     32u

/* 用户确认 4010 v3 与 4005 通信协议相同，沿用原报文和反馈换算。 */
#define LK_ECD_TO_RAD    0.000383495f            /* 2π/16384 (14bit) */
#define LK_DPS_TO_RADPS  0.017453292519943295f   /* π/180 */
#define LK_TWO_PI       6.283185307179586f
#define LK_REDUCTION    1.0f                    /* 4010 v3 按用户资料采用 1:1；注册表 reduction 填本值 */

#define LK_RAD_TO_DEG           57.2957795f   /* 180/π */
/* 驱动换算: 命令值 = 统一单位 × LK_RAD_TO_DEG × 100 × reduction (reduction 取自 registry) */
#define LK_RAD_TO_QUOT          (LK_RAD_TO_DEG * 100.0f * LK_REDUCTION)


/* ============ LK 命令类型 = 统一单位喂 ref, 驱动内换算成协议值 ============ */
typedef enum {
    LK_CTRL_CURRENT       = 0xA1,   /* 电流/扭矩 (ref: mA)            */
    LK_CTRL_SPEED         = 0xA2,   /* 速度     (ref: 输出轴 rad/s)   */
    LK_CTRL_SPEED_LIM_TOR = 0xAD,   /* 限扭矩速度(ref: 输出轴 rad/s)  */
    LK_CTRL_POS_MULTI     = 0xA3,   /* 多圈绝对位置(ref: 输出轴 rad)  */
    LK_CTRL_POS_MULTI_LIM = 0xA4,   /* 限速多圈位置                  */
    LK_CTRL_POS_SINGLE    = 0xA5,   /* 单圈绝对位置                  */
    LK_CTRL_POS_SINGLE_LIM= 0xA6,   /* 限速单圈位置                  */
    LK_CTRL_POS_INC       = 0xA7,   /* 增量位置 (单发 36°=0.628 rad) */
    LK_CTRL_POS_INC_LIM   = 0xA8,   /* 限速增量位置                  */
} lk_ctrl_e;

/* LK 协议原始反馈: CAN 中断写入, 仅本驱动 read_unfiltered_feedback 换算读取。 */
typedef struct {
    uint16_t ecd;          /* 14bit 编码器值 */
    int16_t  speed;        /* deg/s 协议整数 */
    int16_t  current;      /* 电流协议整数 */
    int8_t   temperature;  /* 温度协议值 */
} lk_motor_raw_t;

/* ============ LK 电机实例 = 内嵌统一基类 + LK 私有 ============ */
/* 协议参数在注册表，注册时由 LK ops 从配置填入。 */
typedef struct {
    motor_base_t base;        /* 统一: id/type/fb/ctrl/online —— 第一成员, 中间层只认它 */

    /* ---- LK 私有: 协议参数 (lk_motor_init 从注册表 motor_regs 填入) ---- */
    motor_bus_e  can_bus;     /* 实际 MOTOR_BUS_CAN1/2/3 由注册表选择 */
    uint16_t     can_id;      /* 反馈帧 ID (id + 0x140, 与命令同 ID) */

    /* ---- 反馈标定 (init 从 registry 拷入, fb_parse 换算统一物理量用) ---- */
    int16_t  direction;       /* 正反 +1/-1 */
    uint16_t offset_ecd;      /* 编码器零点 (14bit) */
    fp32     current_scale;   /* 电流 LSB→mA */
    fp32     reduction;       /* 减速比 (速度/位置命令换算用, init 从 registry 拷入) */

    /* ---- LK 私有原始值与跨帧算法状态 ---- */
    lk_motor_raw_t raw;       /* 协议原始值: CAN 中断写, 驱动内部换算 */
    uint16_t last_ecd;
    int32_t  total_round;
    uint8_t  feedback_initialized;

    /* ---- RELAY 命令选择 (lk_motor_set_ctrl 设置, 默认 LK_CTRL_SPEED) ---- */
    lk_ctrl_e ctrl_type;

    /* ---- 输出 (闭环在中间层, 驱动按 ctrl.mode + ctrl_type 组帧) ---- */
    uint8_t  enable;          /* 使能门: 1=发命令帧 0=不发 */
} lk_motor_t;

/* motor_reg.c 装配到 LK 配置行；协议收发由统一 driver ops 接入。 */
extern const motor_driver_ops_t lk_motor_driver_ops;

/* LK 独有的 RELAY 命令选择仍保留具名接口，应用传入已注册的 motor handle。 */
void lk_motor_set_ctrl(motor_handle_t motor, lk_ctrl_e ctrl);

#endif /* LK_MOTOR_H */
