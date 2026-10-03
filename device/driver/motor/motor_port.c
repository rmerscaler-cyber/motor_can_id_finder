#include "motor_port.h"
#include "stm32h7xx_hal.h"
#include <stddef.h>
#include "bsp_can.h"

/*
 * 电机 CAN 物理端口实现。本文件独占逻辑总线到板级 CAN 的映射和 BSP
 * endpoint；它不识别电机模型类型，也不解码电机协议。
 */

/* 每个路由槽位与一个 BSP CAN endpoint 一一对应。 */
static bsp_can_endpoint_t motor_endpoints[MOTOR_PORT_MAX_ENDPOINTS];

/* device 不依赖 model 类型，只透传不透明 owner 和回调。 */
typedef struct {
    void *owner;
    motor_port_rx_callback_t callback;
} motor_port_route_t;

static motor_port_route_t motor_routes[MOTOR_PORT_MAX_ENDPOINTS];
static struct {
    void *owner;
    motor_port_probe_rx_callback_t callback;
} probe_route;

static void motor_port_probe_rx(void *owner, const can_raw_msg_t *msg)
{
    motor_bus_e bus;
    (void)owner;
    if (msg == NULL || msg->is_extend || msg->dlc != 8u ||
        msg->id > 0x7ffu || probe_route.callback == NULL) return;
    switch (msg->bus) {
    case BSP_CAN1: bus = MOTOR_BUS_CAN1; break;
    case BSP_CAN2: bus = MOTOR_BUS_CAN2; break;
    case BSP_CAN3: bus = MOTOR_BUS_CAN3; break;
    default: return;
    }
    probe_route.callback(probe_route.owner, bus, (uint16_t)msg->id,
                         msg->data, msg->timestamp_ms);
}

uint8_t motor_port_probe_observe(void *owner, motor_port_probe_rx_callback_t callback)
{
    if (owner == NULL || callback == NULL) return 0u;
    if (probe_route.callback != NULL &&
        (probe_route.owner != owner || probe_route.callback != callback)) return 0u;
    probe_route.owner = owner;
    probe_route.callback = callback;
    return bsp_can_observer_register(&probe_route, motor_port_probe_rx);
}

/* 允许三路 H7 FDCAN，实际电机接线由注册表选择。 */
static uint8_t motor_bus_valid(motor_bus_e bus)
{
    return (unsigned int)bus < MOTOR_BUS_COUNT;
}

static uint8_t motor_physical_bus(motor_bus_e bus)
{
    /* 物理 CAN 编号是板级事实，更换引脚/外设时只修改此处。 */
    switch (bus) {
    case MOTOR_BUS_CAN1: return BSP_CAN1;
    case MOTOR_BUS_CAN2: return BSP_CAN2;
    default:              return BSP_CAN3;
    }
}

static void motor_port_rx(void *owner, const can_raw_msg_t *msg)
{
    motor_port_route_t *route = (motor_port_route_t *)owner;

    /* BSP 帧转发为与硬件无关的负载 + 时间戳，交由 model 解码。 */
    if (route != NULL && route->callback != NULL && msg != NULL)
        route->callback(route->owner, msg->data, msg->timestamp_ms);
}

uint8_t motor_port_register(uint8_t endpoint_index,
                            void *owner,
                            motor_bus_e bus,
                            uint16_t rx_id,
                            motor_port_rx_callback_t rx_callback)
{
    bsp_can_endpoint_config_t config;
    motor_port_route_t *route;

    if (endpoint_index >= MOTOR_PORT_MAX_ENDPOINTS || owner == NULL ||
        rx_callback == NULL || !motor_bus_valid(bus) || rx_id > 0x7ffu)
        return 0u;

    route = &motor_routes[endpoint_index];
    route->owner = owner;
    route->callback = rx_callback;

    /* 路由表作为 BSP endpoint 的 owner，屏蔽 model 实例的具体类型。 */
    config.bus = motor_physical_bus(bus);
    config.rx_id = rx_id;
    config.expected_dlc = 8u;
    config.is_extend = 0u;
    config.owner = route;
    config.rx_callback = motor_port_rx;
    return bsp_can_endpoint_register(&motor_endpoints[endpoint_index], &config) ==
           BSP_CAN_ENDPOINT_OK;
}

uint8_t motor_port_send(motor_bus_e bus, uint16_t tx_id,
                        const uint8_t *data, uint8_t length)
{
    if (!motor_bus_valid(bus) || data == NULL)
        return 0u;
    return bsp_can_send(motor_physical_bus(bus), tx_id, data, length);
}

uint32_t motor_port_tick_ms(void)
{
    return HAL_GetTick();
}
