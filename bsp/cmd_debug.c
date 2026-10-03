#include "cmd_debug.h"

volatile bsp_can_bus_trace_t bsp_can_trace[BSP_CAN_NUM];
volatile lk_id_scan_debug_t lk_id_scan_debug;

static void cmd_debug_hal_state(uint8_t bus, FDCAN_HandleTypeDef *hfdcan)
{
    bsp_can_trace[bus].last_hal_error = HAL_FDCAN_GetError(hfdcan);
    bsp_can_trace[bus].last_psr = hfdcan->Instance->PSR;
    bsp_can_trace[bus].last_txfqs = hfdcan->Instance->TXFQS;
}

void cmd_debug_init_result(uint8_t bus, FDCAN_HandleTypeDef *hfdcan, uint8_t ok)
{
    if (bus >= BSP_CAN_NUM || hfdcan == NULL) return;
    bsp_can_trace[bus].init_ok = ok;
    cmd_debug_hal_state(bus, hfdcan);
}

void cmd_debug_rx_id(uint8_t bus, uint32_t id)
{
    if (bus >= BSP_CAN_NUM) return;
    bsp_can_trace[bus].rx_count++;
    bsp_can_trace[bus].last_rx_id = id;
}

void cmd_debug_rx(const can_raw_msg_t *msg)
{
    uint8_t i, motor_id, bus;
    if (msg == NULL || (unsigned int)msg->bus >= BSP_CAN_NUM) return;
    bus = (uint8_t)msg->bus;
    bsp_can_trace[bus].last_rx_ms = msg->timestamp_ms;
    bsp_can_trace[bus].last_rx_dlc = msg->dlc;
    bsp_can_trace[bus].last_rx_is_extend = msg->is_extend;
    for (i = 0u; i < 8u; i++)
        bsp_can_trace[bus].last_rx_data[i] = msg->data[i];

    /* 协议见 lk_can.md：只记录标准 8 字节 0x9A 回复。
     * 在 endpoint 分发前观察，因此无需预先知道/注册电机 ID。 */
    if (msg->is_extend || msg->dlc != 8u || msg->data[0] != 0x9Au ||
        msg->id < 0x141u || msg->id > 0x160u)
        return;
    motor_id = (uint8_t)(msg->id - 0x140u);
    lk_id_scan_debug.reply_count++;
    lk_id_scan_debug.found_mask[bus] |= 1UL << (motor_id - 1u);
    lk_id_scan_debug.hit_count[bus][motor_id - 1u]++;
    lk_id_scan_debug.last_bus = bus;
    lk_id_scan_debug.last_motor_id = motor_id;
    lk_id_scan_debug.last_can_id = (uint16_t)msg->id;
    lk_id_scan_debug.last_rx_ms = msg->timestamp_ms;
    for (i = 0u; i < 8u; i++) lk_id_scan_debug.last_data[i] = msg->data[i];
    if (!lk_id_scan_debug.found) {
        lk_id_scan_debug.first_bus = bus;
        lk_id_scan_debug.first_motor_id = motor_id;
        lk_id_scan_debug.first_can_id = (uint16_t)msg->id;
        for (i = 0u; i < 8u; i++) lk_id_scan_debug.first_data[i] = msg->data[i];
        __DMB();
        lk_id_scan_debug.found = 1u;
    }
}

void cmd_debug_tx_attempt(uint8_t bus, uint16_t id, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    if (bus >= BSP_CAN_NUM || data == NULL || len > 8u) return;
    bsp_can_trace[bus].tx_attempt_count++;
    bsp_can_trace[bus].last_tx_ms = HAL_GetTick();
    bsp_can_trace[bus].last_tx_id = id;
    bsp_can_trace[bus].last_tx_dlc = len;
    for (i = 0u; i < 8u; i++)
        bsp_can_trace[bus].last_tx_data[i] = i < len ? data[i] : 0u;
}

void cmd_debug_tx_result(uint8_t bus, FDCAN_HandleTypeDef *hfdcan, HAL_StatusTypeDef status)
{
    if (bus >= BSP_CAN_NUM || hfdcan == NULL) return;
    if (status == HAL_OK) bsp_can_trace[bus].tx_enqueue_ok_count++;
    else bsp_can_trace[bus].tx_enqueue_fail_count++;
    cmd_debug_hal_state(bus, hfdcan);
}
