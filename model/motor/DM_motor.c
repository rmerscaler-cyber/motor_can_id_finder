#include "DM_motor.h"
#include "motor_port.h"
#include "user_lib.h"      /* rad_format */
#include <stddef.h>

/*
 * DM4310 驱动保留 MIT 协议细节：CAN 中断只保存位置/速度/扭矩原始整数，
 * application 反馈阶段再读取未滤波物理量。tor 是 motor.c 的最终输出。
 */


#define DM_MOTOR_STATE_MAX  4u
#define DM_ENABLE_RETRY_MS 20u  /* 未收到反馈时限频补发使能命令 */
static dm_motor_t dm_motors[DM_MOTOR_STATE_MAX];
static uint8_t dm_motor_cnt = 0;   /* 实际注册数 */
dm_motor_t * volatile dm_motor_debug = NULL;
volatile dm_motor_trace_t dm_motor_trace;

/* 扫描地址是数据域里的 ESC_ID，参数查询帧的 CAN ID 固定 0x7FF。
 * 一次候选查询读取 ESC_ID 和 MST_ID，均不使能、不改参数。 */
static uint8_t dm_motor_probe_request(motor_bus_e bus, uint16_t address)
{
    uint8_t data[8] = {(uint8_t)address, (uint8_t)(address >> 8),
                       DM_READ_PARAMETER, DM_RID_ESC_ID, 0u, 0u, 0u, 0u};
    uint8_t esc_ok, mst_ok;
    if ((unsigned int)bus >= MOTOR_BUS_COUNT || address > 0x7ffu) return 0u;
    esc_ok = motor_port_send(bus, DM_PARAMETER_CAN_ID, data, 8u);
    data[3] = DM_RID_MST_ID;
    mst_ok = motor_port_send(bus, DM_PARAMETER_CAN_ID, data, 8u);
    return esc_ok && mst_ok;
}

static uint8_t dm_motor_probe_decode(const uint8_t data[8], motor_probe_reply_t *reply)
{
    uint32_t value;
    uint16_t address;
    if (data == NULL || reply == NULL || data[2] != DM_READ_PARAMETER ||
        (data[3] != DM_RID_ESC_ID && data[3] != DM_RID_MST_ID)) return 0u;
    address = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    value = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
            ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
    if (address > 0x7ffu || value > 0x7ffu) return 0u;
    /* ESC_ID 的读回值必须等于应答携带的地址。 */
    if (data[3] == DM_RID_ESC_ID && value != address) return 0u;
    reply->request_id = address;
    reply->value = (uint16_t)value;
    reply->field = data[3] == DM_RID_ESC_ID ?
                   MOTOR_PROBE_COMMAND_ID : MOTOR_PROBE_FEEDBACK_ID;
    return 1u;
}

/* DM4310 协议量化: x 在 [x_min, x_max] 内 → 无符号 uint (bits 位) */
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* 将物理量线性量化到协议指定 bits 位无符号整数。 */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/* DM4310 协议反量化: uint (bits 位) → float, 范围 [x_min, x_max] */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* 将协议整数反量化回统一物理量；闭环只使用反量化后的字段。 */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static motor_base_t *dm_motor_register(const motor_reg_t *cfg)
{
	uint8_t i;
	dm_motor_t *m;

	if (cfg == NULL || cfg->reduction <= 0.0f) return NULL;

	/* 同总线和反馈 ID 才是同一实例，允许不同 CAN 总线使用相同 ID。 */
	for (i = 0; i < dm_motor_cnt; i++) {
        if (dm_motors[i].can_bus == cfg->can_bus &&
            dm_motors[i].can_id == cfg->can_id)
            return &dm_motors[i].base;
    }

	if (dm_motor_cnt >= DM_MOTOR_STATE_MAX) return NULL;

	m = &dm_motors[dm_motor_cnt];
	/* 发布首个 DM4310 实例的 Watch 调试入口。 */
	if (dm_motor_debug == NULL) dm_motor_debug = m;
    m->base.id    = cfg->id;
    m->base.type  = cfg->type;
    m->can_bus    = cfg->can_bus;
    m->can_id     = cfg->can_id;
    m->cmd_id     = cfg->cmd_id;
    m->enable     = 0;
    m->command_delay_cycles = 0u;
    m->last_enable_tx_tick = 0u;
    m->command_tx_count = 0u;
    m->command_tx_fail_count = 0u;
    m->enable_tx_count = 0u;
    m->enable_tx_fail_count = 0u;
    m->disable_tx_count = 0u;
    m->disable_tx_fail_count = 0u;
	m->reduction    = cfg->reduction;
	m->wheel_radius = cfg->wheel_radius;
	m->assist.pos = 0.0f;
	m->assist.vel = 0.0f;
	m->assist.kp  = 0.0f;
	m->assist.kd  = 0.0f;

	dm_motor_cnt++;
	return &m->base;
}

static uint8_t dm_motor_store_protocol_feedback(
	motor_base_t *motor, const uint8_t *data)
{
	dm_motor_t *m;

	if (motor == NULL || data == NULL) return 0;
	m = (dm_motor_t *)motor;

	m->raw.position = (uint16_t)((data[1] << 8) | data[2]);
	m->raw.speed = (int16_t)((data[3] << 4) | (data[4] >> 4));
	m->raw.torque = (int16_t)(((data[4] & 0x0F) << 8) | data[5]);
	m->raw.temperature = (int8_t)data[6];

	return 1;
}

static uint8_t dm_motor_read_unfiltered_feedback(
	motor_base_t *motor, motor_feedback_t *feedback)
{
	dm_motor_t *m;

    if (motor == NULL || feedback == NULL) return 0;
    m = (dm_motor_t *)motor;

    feedback->ecd = m->raw.position; /* MIT 反馈的 16 位量化位置值，不是物理编码器计数。 */
	feedback->total_angle =
		uint_to_float((int)m->raw.position, P_MIN, P_MAX, 16);
	feedback->angle = rad_format(feedback->total_angle);
	feedback->speed_rotor =
		uint_to_float((int)m->raw.speed, V_MIN, V_MAX, 12);
	feedback->speed_angle = feedback->speed_rotor / m->reduction;
	feedback->torque =
		uint_to_float((int)m->raw.torque, T_MIN, T_MAX, 12);
	feedback->temperature = m->raw.temperature;
	feedback->speed_ms = feedback->speed_angle * m->wheel_radius;

	return 1;
}

static uint8_t dm_motor_send_mit_command(dm_motor_t *m)
{
	uint8_t buf[8];
	uint8_t i;
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

	if (m == NULL) return 0u;
	/* MIT 内部 pos/vel/kp/kd 是驱动私有运行期字段 (dm_motor_t.assist)；
	 * 选择纯外部 PID 时用 Watch 将 dm_motor_debug->assist 清零，驱动不擅自改动。 */
	pos_tmp = float_to_uint(m->assist.pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(m->assist.vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(m->assist.kp, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(m->assist.kd, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(m->base.ctrl.output, T_MIN, T_MAX, 12);

	buf[0] = (uint8_t)(pos_tmp >> 8);
	buf[1] = (uint8_t)pos_tmp;
	buf[2] = (uint8_t)(vel_tmp >> 4);
	buf[3] = (uint8_t)(((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
	buf[4] = (uint8_t)kp_tmp;
	buf[5] = (uint8_t)(kd_tmp >> 4);
	buf[6] = (uint8_t)(((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
	buf[7] = (uint8_t)tor_tmp;
	dm_motor_trace.command_attempt_count++;
	dm_motor_trace.last_command_tick = motor_port_tick_ms();
	dm_motor_trace.last_command_id = m->cmd_id + MIT_MODE;
	dm_motor_trace.last_output = m->base.ctrl.output;
	dm_motor_trace.last_position_assist_pos = m->assist.pos;
	dm_motor_trace.last_position_assist_vel = m->assist.vel;
	dm_motor_trace.last_position_assist_kp = m->assist.kp;
	dm_motor_trace.last_position_assist_kd = m->assist.kd;
	for (i = 0u; i < 8u; i++)
		dm_motor_trace.last_command_data[i] = buf[i];

	if (motor_port_send(m->can_bus,
	                          m->cmd_id + MIT_MODE,
	                          buf,
	                          8)) {
		m->command_tx_count++;
		dm_motor_trace.command_enqueue_ok_count++;
		dm_motor_trace.last_command_enqueue_ok = 1u;
		return 1u;
	}
	m->command_tx_fail_count++;
	dm_motor_trace.command_enqueue_fail_count++;
	dm_motor_trace.last_command_enqueue_ok = 0u;
	return 0u;
}

static uint8_t dm_motor_send_enable_command(dm_motor_t *m)
{
	uint8_t data[8];

	if (m == NULL) return 0u;
	data[0]=0xFF; data[1]=0xFF; data[2]=0xFF; data[3]=0xFF;
	data[4]=0xFF; data[5]=0xFF; data[6]=0xFF; data[7]=0xFC;
	return motor_port_send(m->can_bus,
	                             m->cmd_id + MIT_MODE,
	                             data,
	                             8);
}

static uint8_t dm_motor_send_disable_command(dm_motor_t *m)
{
	uint8_t data[8];

	if (m == NULL) return 0u;
	data[0]=0xFF; data[1]=0xFF; data[2]=0xFF; data[3]=0xFF;
	data[4]=0xFF; data[5]=0xFF; data[6]=0xFF; data[7]=0xFD;
	return motor_port_send(m->can_bus,
	                             m->cmd_id + MIT_MODE,
	                             data,
	                             8);
}

static void dm_motor_trace_state(const dm_motor_t *m)
{
	if (m == NULL) return;
	dm_motor_trace.base_enabled = m->base.enabled;
	dm_motor_trace.driver_enabled = m->enable;
	dm_motor_trace.online = m->base.online;
	dm_motor_trace.command_delay_cycles = m->command_delay_cycles;
	dm_motor_trace.feedback_count = m->base.feedback_count;
	dm_motor_trace.last_feedback_tick = m->base.last_feedback_tick;
}

static void dm_motor_pack_commands(motor_handle_t const *motors, uint8_t count)
{
	uint8_t j;

	/* DM 一电机一帧，各自发送 */
	if (motors == NULL) return;
	for (j = 0; j < count; j++) {
		dm_motor_t *m;
		if (motors[j] == NULL || motors[j]->driver != &dm_motor_driver_ops) continue;
		m = (dm_motor_t *)motors[j];
		dm_motor_trace.pack_match_count++;
		dm_motor_trace.last_pack_tick = motor_port_tick_ms();
		dm_motor_trace_state(m);

		/* 与改动前一致：应用使能后每个应用周期发送一次 MIT 命令。
		 * 反馈用于控制和在线判断，不反过来阻断下一条控制命令。 */
		if (!m->base.enabled) {
			dm_motor_trace.pack_skip_disabled_count++;
			continue;
		}

		/* 保留原代码使能后等待 1 tick 再发首条 MIT 请求的时序。 */
		if (m->command_delay_cycles > 0u) {
			m->command_delay_cycles--;
			dm_motor_trace.pack_skip_delay_count++;
			continue;
		}

		(void)dm_motor_send_mit_command(m);
	}
}

static void dm_motor_enable_instance(motor_base_t *motor)
{
	dm_motor_t *m;
	uint32_t now;

	if (motor == NULL) return;
	m = (dm_motor_t *)motor;
	now = motor_port_tick_ms();
	dm_motor_trace.enable_call_count++;
	/* 收到反馈后不重复使能；无反馈时每 20 ms 补发一次，期间 MIT
	 * 控制帧仍按 1 ms 连续发送，恢复改动前已经验证过的节奏。 */
	if (m->enable &&
	    (motor->online ||
	     (uint32_t)(now - m->last_enable_tx_tick) < DM_ENABLE_RETRY_MS)) {
		dm_motor_trace_state(m);
		return;
	}

	/* 使能帧: 0xFF×7 + 0xFC, 发送 ID = 命令ID + MIT_MODE
	   (统一框架下 DM 恒为 MIT 扭矩执行器, 模式单一来源=中间层 ctrl.mode, 驱动不分模式) */
	if (dm_motor_send_enable_command(m)) {
		m->enable = 1u;
		m->last_enable_tx_tick = now;
		m->command_delay_cycles = 1u;
		m->enable_tx_count++;
		dm_motor_trace.enable_enqueue_ok_count++;
	} else {
		/* 发送失败不能提前记作已使能；下一控制周期继续重试。 */
		m->enable_tx_fail_count++;
		dm_motor_trace.enable_enqueue_fail_count++;
	}
	dm_motor_trace_state(m);
}

static void dm_motor_disable_instance(motor_base_t *motor)
{
	dm_motor_t *m;
	if (motor == NULL) return;
	m = (dm_motor_t *)motor;
	dm_motor_trace.disable_call_count++;
	if (!m->enable) {
		dm_motor_trace_state(m);
		return;
	}

	/* 直接发送协议失能帧。MIT pos/kp/kd 可能被保留为非零调试值，
	 * 因此不能先借普通 MIT 帧冒充“零力”，否则仍会产生位置保持力，
	 * 还会额外占用一个 CAN 发送邮箱。 */
	if (dm_motor_send_disable_command(m)) {
		m->enable = 0u;
		m->command_delay_cycles = 0u;
		m->disable_tx_count++;
		dm_motor_trace.disable_enqueue_ok_count++;
	} else {
		m->disable_tx_fail_count++;
		dm_motor_trace.disable_enqueue_fail_count++;
	}
	dm_motor_trace_state(m);
}

const motor_driver_ops_t dm_motor_driver_ops = {
	"DM",
	dm_motor_register,
	dm_motor_store_protocol_feedback,
	dm_motor_read_unfiltered_feedback,
	dm_motor_pack_commands,
	dm_motor_enable_instance,
	dm_motor_disable_instance,
	dm_motor_probe_request,
	dm_motor_probe_decode,
};
