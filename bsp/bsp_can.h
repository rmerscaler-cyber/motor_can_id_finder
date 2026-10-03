#ifndef BSP_CAN_H
#define BSP_CAN_H


#include "struct_typedef.h"
/* H7 三路物理 FDCAN；四台电机的实际分配由注册表决定。 */
typedef enum {
    BSP_CAN1 = 0,   /* hfdcan1 */
    BSP_CAN2,       /* hfdcan2 */
    BSP_CAN3,       /* hfdcan3 */
    BSP_CAN_NUM,
} bsp_can_id_e;

#define BSP_CAN_ENDPOINT_MAX  16u

uint8_t bsp_can_send(bsp_can_id_e can_bus,
                     uint16_t id,
                     const uint8_t *data,
                     uint8_t len);
uint8_t can_filter_init(void);

typedef struct {
    bsp_can_id_e bus;
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    uint8_t is_extend;
    uint32_t timestamp_ms;
} can_raw_msg_t;

/* FDCAN 接收 -> endpoint 分发 -> motor_port_rx -> 电机协议解码。
 * 回调在中断上下文执行，msg 只在本次回调内有效。
 */
typedef void (*bsp_can_endpoint_rx_cb_t)(void *owner,
                                         const can_raw_msg_t *msg);

/* 未知回复 ID 的诊断订阅：在 endpoint 路由前观察原始帧，不解析协议。
 * 当前仅一个诊断订阅者；注册发生在 CAN 启动前。 */
uint8_t bsp_can_observer_register(void *owner, bsp_can_endpoint_rx_cb_t callback);

typedef struct {
    bsp_can_id_e bus;
    uint32_t rx_id;
    uint8_t expected_dlc;
    uint8_t is_extend;
    void *owner;
    bsp_can_endpoint_rx_cb_t rx_callback;
} bsp_can_endpoint_config_t;

typedef struct {
    bsp_can_id_e bus;
    uint32_t rx_id;
    uint8_t expected_dlc;
    uint8_t is_extend;
    void *owner;
    bsp_can_endpoint_rx_cb_t rx_callback;
    volatile uint32_t rx_count;
    volatile uint32_t last_rx_ms;
    uint8_t registered;
} bsp_can_endpoint_t;

typedef enum {
    BSP_CAN_ENDPOINT_OK = 0,
    BSP_CAN_ENDPOINT_ERR_ARGUMENT,
    BSP_CAN_ENDPOINT_ERR_BUS,
    BSP_CAN_ENDPOINT_ERR_DLC,
    BSP_CAN_ENDPOINT_ERR_ALREADY_REGISTERED,
    BSP_CAN_ENDPOINT_ERR_DUPLICATE,
    BSP_CAN_ENDPOINT_ERR_FULL,
} bsp_can_endpoint_status_e;

bsp_can_endpoint_status_e bsp_can_endpoint_register(
    bsp_can_endpoint_t *endpoint,
    const bsp_can_endpoint_config_t *config);

uint8_t bsp_can_endpoint_send(const bsp_can_endpoint_t *endpoint,
                              uint16_t tx_id,
                              uint8_t *data,
                              uint8_t len);

#endif /* BSP_CAN_H */
