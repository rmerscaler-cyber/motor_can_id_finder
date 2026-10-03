#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "motor_typedef.h"

/*
 * 具体型号驱动的最小协议接口。
 *
 * 函数指针只用于 motor 中间层与 DM/LK 等协议实现之间的必要边界：
 * - register_instance: 创建/取得具体驱动实例；
 * - store_protocol_feedback: 把 CAN 中断收到的协议原始值写入驱动私有结构体；
 * - read_unfiltered_feedback:  从驱动私有原始值换算未滤波物理量，填入统一 motor_feedback_t；
 * - pack_commands:      按具体协议组帧并交给设备层发送；
 * - enable/disable:    执行具体协议的使能门或使能/失能帧。
 *
 * PID、反馈源选择、在线看门狗、限幅、安全覆盖和 ref/output 管理仍由
 * motor.c 直接执行，不得加入本 ops。
 */
struct motor_driver_ops
{
    const char *name; /* 仅供调试器/日志辨识，运行逻辑不得依赖字符串 */

    /* 这些函数指针只存在于“中间层 ↔ 具体协议驱动”的必要边界；PID 和
     * 安全逻辑仍是 motor.c 的具名代码，便于单步调试和查看控制流。 */
    motor_base_t *(*register_instance)(const motor_reg_t *config);
    uint8_t (*store_protocol_feedback)(motor_base_t *motor, const uint8_t *data);
    uint8_t (*read_unfiltered_feedback)(motor_base_t *motor,
                                        motor_feedback_t *feedback);
    void (*pack_commands)(motor_handle_t const *motors, uint8_t count);
    void (*enable)(motor_base_t *motor);
    void (*disable)(motor_base_t *motor);

    /* 可选诊断能力：只读探测地址，不创建实例、不发送控制/使能帧。
     * 仅由 motor.c 调用，应用层不引用具体驱动。 */
    uint8_t (*probe_request)(motor_bus_e bus, uint16_t can_id);
    /* 可选：把只读寄存器回复解释为统一 ID 字段；只在协议层解码字节。 */
    uint8_t (*probe_decode)(const uint8_t data[8], motor_probe_reply_t *reply);
};

#endif /* MOTOR_DRIVER_H */
