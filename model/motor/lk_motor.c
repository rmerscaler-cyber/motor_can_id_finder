#include "lk_motor.h"
#include "motor_utils.h"   /* motor_angle_wrap */
#include "motor_port.h"    /* motor_port_send: CAN 发送由设备层适配 */
#include <string.h>       /* memset */
#include <stddef.h>

/* 复用原 H7 LK 通信协议；本控制器使用一台翎控 4010 v3。 */
#define LK_MOTOR_STATE_MAX  1
static lk_motor_t lk_motors[LK_MOTOR_STATE_MAX];
static uint8_t lk_motor_cnt = 0;   /* 实际注册数 */

/* 原协议 0x9A 读取状态 1：收发共用 0x140 + Motor ID。
 * 此入口不注册电机、不使能电机、不调用控制发送链。 */
static uint8_t lk_motor_probe_request(motor_bus_e bus, uint16_t can_id)
{
    static const uint8_t request[8] = {0x9Au, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    if ((unsigned int)bus >= MOTOR_BUS_COUNT ||
        can_id < LK_CAN_ID_BASE + LK_MOTOR_ID_MIN ||
        can_id > LK_CAN_ID_BASE + LK_MOTOR_ID_MAX)
        return 0u;
    return motor_port_send(bus, can_id, request, sizeof(request));
}

/* 沿用原 LK 驱动的编码器多圈计数。 */
static void lk_ecd_round_update(uint16_t ecd, uint16_t *last_ecd,
                                int32_t *total_round)
{
    int16_t d = (int16_t)(ecd - *last_ecd);
    if (d > 8192) {
        if (ecd >= 15000 && *last_ecd <= 1000) (*total_round)++;
    } else if (d < -8192) {
        if (ecd <= 1000 && *last_ecd >= 15000) (*total_round)--;
    }
    *last_ecd = ecd;
}

/* 小端字节打包 (LK 组帧私有)。 */
static void lk_put_i16(uint8_t *buf, uint8_t off, int16_t val)
{
    buf[off]     = (uint8_t)(val & 0xFF);
    buf[off + 1] = (uint8_t)((val >> 8) & 0xFF);
}

static void lk_put_i32(uint8_t *buf, uint8_t off, int32_t val)
{
    buf[off]     = (uint8_t)(val & 0xFF);
    buf[off + 1] = (uint8_t)((val >> 8) & 0xFF);
    buf[off + 2] = (uint8_t)((val >> 16) & 0xFF);
    buf[off + 3] = (uint8_t)((val >> 24) & 0xFF);
}

static motor_base_t *lk_motor_register(const motor_reg_t *cfg)
{
    uint8_t i;
    lk_motor_t *m;

    if (cfg == NULL || cfg->reduction <= 0.0f) return NULL;

    /* 幂等: 同总线和 CAN ID 已注册则返回原实例。 */
    for (i = 0; i < lk_motor_cnt; i++) {
        if (lk_motors[i].can_bus == cfg->can_bus &&
            lk_motors[i].can_id == cfg->can_id)
            return &lk_motors[i].base;
    }
    if (lk_motor_cnt >= LK_MOTOR_STATE_MAX) return NULL;

    m = &lk_motors[lk_motor_cnt];
    memset(m, 0, sizeof(*m));
    m->base.id = cfg->id;
    m->base.type = cfg->type;
    m->can_bus = cfg->can_bus;
    m->can_id = cfg->can_id;
    m->enable = 0;
    m->ctrl_type = LK_CTRL_SPEED;
    /* 反馈标定从 registry 拷入，协议换算和旧实现保持一致。 */
    m->direction = cfg->direction;
    m->offset_ecd = cfg->offset_ecd;
    m->current_scale = cfg->current_scale;
    m->reduction = cfg->reduction;
    lk_motor_cnt++;
    return &m->base;
}

static uint8_t lk_motor_store_protocol_feedback(
    motor_base_t *motor, const uint8_t *data)
{
    lk_motor_t *m;

    if (motor == NULL || data == NULL) return 0;
    m = (lk_motor_t *)motor;

    m->raw.temperature = (int8_t)data[1];
    m->raw.current = (int16_t)(data[2] | (data[3] << 8));
    m->raw.speed = (int16_t)(data[4] | (data[5] << 8));
    m->raw.ecd = (uint16_t)(data[6] | (data[7] << 6));

    if (!m->feedback_initialized) {
        m->last_ecd = m->raw.ecd;
        m->feedback_initialized = 1u;
    } else {
        lk_ecd_round_update(m->raw.ecd,
                            &m->last_ecd,
                            &m->total_round);
    }

    return 1;
}

static uint8_t lk_motor_read_unfiltered_feedback(
    motor_base_t *motor, motor_feedback_t *feedback)
{
    lk_motor_t *m;

    if (motor == NULL || feedback == NULL) return 0;
    m = (lk_motor_t *)motor;

    feedback->ecd = m->raw.ecd; /* 14 位原始输出轴编码器计数。 */
    feedback->angle = motor_angle_wrap(
        (fp32)(int16_t)(m->raw.ecd - m->offset_ecd) * LK_ECD_TO_RAD);
    feedback->total_angle =
        (fp32)m->total_round * LK_TWO_PI + feedback->angle;
    feedback->speed_angle =
        (fp32)m->raw.speed * LK_DPS_TO_RADPS * (fp32)m->direction;
    feedback->speed_rotor = feedback->speed_angle * m->reduction;
    feedback->current =
        (int16_t)((fp32)m->raw.current * m->current_scale);
    feedback->speed_ms = 0.0f;
    feedback->temperature = m->raw.temperature;

    return 1;
}

static void lk_motor_pack_commands(motor_handle_t const *motors,
                                   uint8_t count)
{
    uint8_t buf[8];
    uint8_t i;

    if (motors == NULL) return;
    for (i = 0; i < count; i++) {
        lk_motor_t *m;

        if (motors[i] == NULL ||
            motors[i]->driver != &lk_motor_driver_ops)
            continue;
        m = (lk_motor_t *)motors[i];
        if (!m->enable) continue;

		/* 组帧: 只填 mask 内且已使能的电机槽位, 有实际填槽才发帧 */
        memset(buf, 0, sizeof(buf));
        switch (m->base.ctrl.mode) {
        case MOTOR_MODE_SPEED:
        case MOTOR_MODE_POSITION: {  /* 中间层 PID → 电流 mA → 0xA1 */
            buf[0] = LK_CTRL_CURRENT;
                lk_put_i16(buf, 4, (int16_t)(m->base.ctrl.output / m->current_scale));
                break;
        }
        case MOTOR_MODE_RELAY:      /* 透传 → 按 ctrl_type 组帧 */
            switch (m->ctrl_type) {
            case LK_CTRL_CURRENT:
                buf[0] = 0xA1;
                lk_put_i16(buf, 4, (int16_t)(m->base.ctrl.output / m->current_scale));
                break;
            case LK_CTRL_SPEED:
                /* 速度 0xA2: 输出轴 rad/s → LK 速度值 = rad/s × (180/π) × 100 × reduction */
                buf[0] = 0xA2;
                lk_put_i32(buf, 4, (int32_t)(m->base.ctrl.output * LK_RAD_TO_DEG * 100.0f * m->reduction));
                break;
            case LK_CTRL_POS_INC:
                /* 位置 0xA7: 输出轴 rad → LK 位置值 = rad × (180/π) × 100 × reduction */
                buf[0] = 0xA7;
                lk_put_i32(buf, 4, (int32_t)(m->base.ctrl.output * LK_RAD_TO_DEG * 100.0f * m->reduction));
                break;
            /* 0xA3-A6/A8/0xAD 同理, 限值字段预留 0 */
            default: continue;
            }
            break;
        default: continue;
        }
        (void)motor_port_send(m->can_bus, m->can_id, buf, 8);
    }
}

static void lk_motor_enable_instance(motor_base_t *motor)
{
    lk_motor_t *m;
    if (motor == NULL) return;
    m = (lk_motor_t *)motor;
    m->enable = 1u;
}

static void lk_motor_disable_instance(motor_base_t *motor)
{
    lk_motor_t *m;
    if (motor == NULL) return;
    m = (lk_motor_t *)motor;
    m->enable = 0u;
}

void lk_motor_set_ctrl(motor_handle_t motor, lk_ctrl_e ctrl)
{
    lk_motor_t *m;
    if (motor == NULL || motor->driver != &lk_motor_driver_ops) return;
    m = (lk_motor_t *)motor;
    m->ctrl_type = ctrl;
}

const motor_driver_ops_t lk_motor_driver_ops = {
    "LK",
    lk_motor_register,
    lk_motor_store_protocol_feedback,
    lk_motor_read_unfiltered_feedback,
    lk_motor_pack_commands,
    lk_motor_enable_instance,
    lk_motor_disable_instance,
    lk_motor_probe_request,
    NULL, /* LK 的已知地址扫描回复仍由 BSP debug 记录。 */
};
