#ifndef MOTOR_PORT_H
#define MOTOR_PORT_H


#include "struct_typedef.h"
/*
 * 电机模型与板级 CAN 之间的物理端口。
 *
 * model 只使用逻辑总线名；本端口负责把逻辑总线映射到 H7 的具体 CAN
 * 外设，并通过 owner + callback 将接收帧交回模型层解码。
 */

/* 控制器端口按物理 FDCAN 编号命名，避免继承底盘功能分配。 */
typedef enum {
    MOTOR_BUS_CAN1 = 0,
    MOTOR_BUS_CAN2,
    MOTOR_BUS_CAN3,
    MOTOR_BUS_COUNT,
} motor_bus_e;

/*
 * 电机接收回调。
 * owner：注册时传入的模型实例指针。
 * data：长度为 8 字节的 CAN 负载，仅在回调期间有效。
 * timestamp_ms：该帧到达时的毫秒时间戳。
 *
 * 当前回溯链：
 * - 赋值：model/motor/motor.c::motor_register() 把 motor_port_feedback()
 *   传给 device/driver/motor/motor_port.c::motor_port_register()，后者存入
 *   motor_routes[endpoint_index].callback。
 * - 调用：device/driver/motor/motor_port.c::motor_port_rx() 收到 BSP CAN
 *   分发帧后，通过 route->callback 调用此指针。
 * - 后续：motor_port_feedback() -> motor_receive_feedback() -> 具体协议
 *   store_protocol_feedback。
 */
typedef void (*motor_port_rx_callback_t)(void *owner,
                                         const uint8_t data[8],
                                         uint32_t timestamp_ms);

/* 端口路由表容量；endpoint_index 的有效范围为 [0, 15]。 */
#define MOTOR_PORT_MAX_ENDPOINTS  16u

/*
 * 注册一条电机接收路由。
 * endpoint_index：由模型层分配的唯一路由槽位。
 * owner：回调时原样传回的模型实例。
 * bus：逻辑电机总线。rx_id：需要订阅的标准 CAN ID。
 * rx_callback：帧到达时的模型层回调。
 * 返回 1 表示注册成功，0 表示参数无效或 BSP endpoint 注册失败。
 */
uint8_t motor_port_register(uint8_t endpoint_index,
                            void *owner,
                            motor_bus_e bus,
                            uint16_t rx_id,
                            motor_port_rx_callback_t rx_callback);

/*
 * 向指定逻辑总线发送一帧标准 CAN 数据。
 * data 指向 length 字节负载；返回 1 表示 BSP 接受发送，0 表示失败。
 */
uint8_t motor_port_send(motor_bus_e bus, uint16_t tx_id,
                        const uint8_t *data, uint8_t length);

/* 返回 HAL 毫秒时基，供电机模型记录反馈和离线时间。 */
uint32_t motor_port_tick_ms(void);

/* 诊断原始订阅只透传标准 8 字节帧；device 不识别寄存器/电机型号。 */
typedef void (*motor_port_probe_rx_callback_t)(void *owner, motor_bus_e bus,
    uint16_t rx_id, const uint8_t data[8], uint32_t timestamp_ms);
uint8_t motor_port_probe_observe(void *owner, motor_port_probe_rx_callback_t callback);

#endif
