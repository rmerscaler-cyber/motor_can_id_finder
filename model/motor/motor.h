#ifndef MOTOR_H
#define MOTOR_H

#include "motor_typedef.h"   /* motor_id_e / motor_base_t / motor_feedback_t / motor_control_t */

/*
 * 电机中间层的核心约束：motor_id_e 只允许出现在初始化注册和调试标识处。
 * 控制器应用运行期保存 motor_handle_t，所有反馈、设定值、在线状态和输出
 * 都从这个实例进入，避免一个全局 ID 把不同任务重新耦合在一起。
 */

/* 中间层实例 API：ID 只在注册时使用；注册完成后应用只保存 motor_handle_t。 */
motor_handle_t motor_register(motor_id_e id);                   /* 按配置键注册单台电机，幂等，失败返回 NULL */
/* 未知 CAN 地址的上电诊断：初始化取得句柄，运行期只使用句柄。
 * 发送结果仅表示入队；DM 寄存器回复走统一诊断收件箱，LK 回复由 BSP debug 记录。
 * 正常电机控制/反馈仍使用 motor_register() 返回的 motor_handle_t。 */
motor_probe_handle_t motor_probe_register(motor_id_e id);
uint8_t motor_probe_request(motor_probe_handle_t probe, motor_bus_e bus,
                            uint16_t can_id);
uint8_t motor_probe_result_read(motor_probe_handle_t probe, motor_probe_result_t *result);
/* 结束等待；传 NULL 取消当前会话。结果读取与 ISR 写入通过短临界区隔离。 */
void motor_probe_cancel(motor_probe_handle_t probe);
void  motor_calc(motor_handle_t motor);                          /* 读取最终反馈快照 → 统一闭环 + 限幅 */
void  motor_cmd_pack(motor_handle_t const *motors, uint8_t count); /* 对传入实例按协议分组并发送 */
uint8_t motor_feedback_unfiltered_read(motor_handle_t motor,
                                       motor_feedback_t *feedback); /* feedback_update 读取未滤波物理量 */
uint8_t motor_feedback_publish(motor_handle_t motor,
                               const motor_feedback_t *feedback); /* feedback_update 覆盖发布最终反馈 */
uint8_t motor_feedback_read(motor_handle_t motor,
                            motor_feedback_t *feedback);        /* 跨任务读取最终区的一致快照 */
const motor_base_t *motor_state(motor_handle_t motor);          /* 只读实例状态，便于调试 */
void  motor_set_ref(motor_handle_t motor, fp32 ref);            /* 设目标 (角度/速度, 依 mode 最外层) */
void  motor_set_output(motor_handle_t motor, fp32 output);      /* PID 后输出回写 */
void  motor_add_output(motor_handle_t motor, fp32 output_delta);/* PID 后叠加前馈并做最终硬限幅 */
void  motor_reset_control(motor_handle_t motor);                /* 清 PID 历史并将输出置零 */
uint8_t motor_is_online(motor_handle_t motor);                  /* 在线判定 */
void  motor_enable(motor_handle_t motor);
void  motor_disable(motor_handle_t motor);
const motor_reg_t *motor_get_config(motor_handle_t motor);      /* 读实例配置 */
/* Called by the device CAN adapter when a registered motor receives a frame. */
void  motor_receive_feedback(motor_handle_t motor, const uint8_t data[8],
                             uint32_t timestamp_ms);

#endif /* MOTOR_H */
