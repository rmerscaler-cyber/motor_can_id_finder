#ifndef CMD_DEBUG_H
#define CMD_DEBUG_H

#include "stm32h7xx_hal.h"
#include "bsp_can.h"

/* 对齐 F4 BSP 的 cmd_debug RX/TX 钩子；仅记录观测，不参与应用决策。 */
typedef struct {
    uint32_t tx_attempt_count;
    uint32_t tx_enqueue_ok_count;
    uint32_t tx_enqueue_fail_count;
    uint32_t rx_count;
    uint32_t last_tx_ms;
    uint32_t last_rx_ms;
    uint32_t last_tx_id;
    uint32_t last_rx_id;
    uint32_t last_hal_error;
    uint32_t last_psr;       /* H7 的协议状态寄存器 */
    uint32_t last_txfqs;     /* H7 的发送 FIFO 状态寄存器 */
    uint32_t init_ok;
    uint8_t last_tx_dlc;
    uint8_t last_rx_dlc;
    uint8_t last_rx_is_extend;
    uint8_t last_tx_data[8];
    uint8_t last_rx_data[8];
} bsp_can_bus_trace_t;

typedef struct {
    uint32_t reply_count;
    uint32_t found_mask[BSP_CAN_NUM]; /* bit0 对应 Motor ID 1，bit31 对应 32 */
    uint32_t hit_count[BSP_CAN_NUM][32];
    uint32_t last_rx_ms;
    uint16_t first_can_id;
    uint16_t last_can_id;
    uint8_t found;
    uint8_t first_bus;      /* 0/1/2 = FDCAN1/2/3 */
    uint8_t first_motor_id;
    uint8_t last_bus;
    uint8_t last_motor_id;
    uint8_t first_data[8];
    uint8_t last_data[8];
} lk_id_scan_debug_t;

extern volatile bsp_can_bus_trace_t bsp_can_trace[BSP_CAN_NUM];
extern volatile lk_id_scan_debug_t lk_id_scan_debug;

void cmd_debug_rx_id(uint8_t bus, uint32_t id);
void cmd_debug_rx(const can_raw_msg_t *msg);
void cmd_debug_tx_attempt(uint8_t bus, uint16_t id, const uint8_t *data, uint8_t len);
void cmd_debug_tx_result(uint8_t bus, FDCAN_HandleTypeDef *hfdcan, HAL_StatusTypeDef status);
void cmd_debug_init_result(uint8_t bus, FDCAN_HandleTypeDef *hfdcan, uint8_t ok);

#endif
