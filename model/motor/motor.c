#include "motor.h"
#include "motor_driver.h"
#include "motor_reg.h"    /* motor_regs: 配置行已绑定具体协议 ops */
#include "motor_utils.h"   /* motor_output_clamp (反馈换算已在各驱动 fb_parse) */
#include "pid.h"           /* 线性/周期角度 PID 及具名配置。 */
#include "motor_port.h"    /* CAN endpoint 注册/发送与毫秒时基由设备层实现 */
#include "cmsis_gcc.h"     /* __DMB: 发布顺序屏障 */
#include <stddef.h>        /* NULL */
#include <string.h>

/*
 * motor.c 是协议无关的控制中间层。它拥有统一 motor_base_t 的控制部分，
 * 负责 PID、在线检测、输出限幅和“每组协议只组一次帧”；DM/LK
 * 只负责协议反馈换算与字节组帧。CAN 回调只更新驱动私有原始收件箱；
 * application 的 feedback_update 完成任务层处理并发布唯一最终反馈。
 */

/* 冷路径注册索引：保证 motor_register(id) 幂等，并供 CAN ID 路由找到已注册实例。 */
static motor_base_t *registry[MOTOR_COUNT];

/* 诊断一次只有一个正在等待的地址，三个顺序会话复用同一原始收件箱。
 * ISR 写入，任务在短临界区读取/重置；与电机控制实例/最终反馈区无关。 */
static motor_probe_handle_t volatile pending_probe;
static volatile motor_probe_result_t pending_probe_result;

static void motor_probe_irq_restore(uint32_t primask)
{
    if (primask == 0u) __enable_irq();
}

static void motor_probe_receive(void *owner, motor_bus_e bus, uint16_t rx_id,
                                 const uint8_t data[8], uint32_t timestamp_ms)
{
    motor_probe_reply_t reply;
    motor_probe_handle_t probe = pending_probe;
    uint8_t i;
    (void)owner;
    if (probe == NULL || bus != pending_probe_result.bus ||
        probe->driver->probe_decode == NULL ||
        !probe->driver->probe_decode(data, &reply) ||
        reply.request_id != pending_probe_result.request_id ||
        (reply.field != MOTOR_PROBE_COMMAND_ID && reply.field != MOTOR_PROBE_FEEDBACK_ID))
        return;

    /* 不把不同回复 CAN ID 的两帧拼成同一电机结果。 */
    if (pending_probe_result.fields != 0u && pending_probe_result.reply_can_id != rx_id) {
        pending_probe_result.conflict = 1u;
        return;
    }
    pending_probe_result.reply_can_id = rx_id;
    pending_probe_result.last_rx_ms = timestamp_ms;
    if (reply.field == MOTOR_PROBE_COMMAND_ID) {
        if ((pending_probe_result.fields & reply.field) != 0u &&
            pending_probe_result.cmd_id != reply.value) pending_probe_result.conflict = 1u;
        pending_probe_result.cmd_id = reply.value;
        pending_probe_result.command_reply_count++;
        for (i = 0u; i < 8u; i++) pending_probe_result.command_reply[i] = data[i];
    } else {
        if ((pending_probe_result.fields & reply.field) != 0u &&
            pending_probe_result.feedback_id != reply.value) pending_probe_result.conflict = 1u;
        pending_probe_result.feedback_id = reply.value;
        pending_probe_result.feedback_reply_count++;
        for (i = 0u; i < 8u; i++) pending_probe_result.feedback_reply[i] = data[i];
    }
    __DMB();
    pending_probe_result.fields |= reply.field;
}

/* 探测是未知地址时的诊断冷路径，协议绑定仍统一来自 motor_reg.c。 */
motor_probe_handle_t motor_probe_register(motor_id_e id)
{
    uint8_t i;
    if ((unsigned int)id >= MOTOR_COUNT) return NULL;
    for (i = 0u; i < motor_probe_reg_count; i++) {
        const motor_probe_config_t *config = &motor_probe_regs[i];
        if (config->id != id) continue;
        if (config->driver == NULL || config->driver->probe_request == NULL ||
            config->first_can_id > config->last_can_id ||
            config->last_can_id > 0x7ffu)
            return NULL;
        if (config->driver->probe_decode != NULL &&
            !motor_port_probe_observe((void *)&pending_probe_result, motor_probe_receive))
            return NULL;
        return config;
    }
    return NULL;
}

uint8_t motor_probe_request(motor_probe_handle_t probe, motor_bus_e bus,
                            uint16_t can_id)
{
    if (probe == NULL || probe->driver == NULL ||
        probe->driver->probe_request == NULL ||
        (unsigned int)bus >= MOTOR_BUS_COUNT ||
        can_id < probe->first_can_id || can_id > probe->last_can_id)
        return 0u;
    if (probe->driver->probe_decode != NULL) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        pending_probe = NULL;
        memset((void *)&pending_probe_result, 0, sizeof(pending_probe_result));
        pending_probe_result.bus = bus;
        pending_probe_result.request_id = can_id;
        __DMB();
        pending_probe = probe; /* 在发送前发布：第一帧回复可能先于第二次发送到达。 */
        motor_probe_irq_restore(primask);
    }
    return probe->driver->probe_request(bus, can_id);
}

uint8_t motor_probe_result_read(motor_probe_handle_t probe, motor_probe_result_t *result)
{
    uint8_t ok = 0u;
    uint32_t primask;
    if (probe == NULL || result == NULL) return 0u;
    primask = __get_PRIMASK();
    __disable_irq();
    if (pending_probe == probe) {
        *result = pending_probe_result;
        ok = result->fields != 0u || result->conflict;
    }
    motor_probe_irq_restore(primask);
    return ok;
}

void motor_probe_cancel(motor_probe_handle_t probe)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (probe == NULL || pending_probe == probe) pending_probe = NULL;
    __DMB();
    motor_probe_irq_restore(primask);
}

/* 约 100 ms 未收到有效反馈判离线；毫秒时基由 device port 提供。 */
#define MOTOR_ONLINE_TIMEOUT_MS  100u
#define MOTOR_FEEDBACK_READ_RETRY  3u

/* 配置表按 id 查找，仅在注册冷路径使用。 */
static const motor_reg_t *motor_reg_find(motor_id_e id)
{
    uint8_t i;
    for (i = 0; i < motor_reg_count; i++) {
        if (motor_regs[i].id == id) return &motor_regs[i];
    }
    return NULL;
}

/* 所有 ops 都是必选协议能力，避免热路径到处判断单个函数指针，确保每个函数指针都非空防上层炸掉。 */
static uint8_t motor_driver_valid(const motor_driver_ops_t *driver)
{
    /* 注册期一次性检查所有 ops，热路径不再到处判断函数指针是否为空。 */
    return driver != NULL &&
           driver->register_instance != NULL &&
           driver->store_protocol_feedback != NULL &&
           driver->read_unfiltered_feedback != NULL &&
           driver->pack_commands != NULL &&
           driver->enable != NULL &&
           driver->disable != NULL;
}

/* 单实例协议反馈入口：由设备层的 CAN 接收回调进入。 */
static uint8_t motor_store_protocol_feedback_instance(
    motor_handle_t motor, const uint8_t *data, uint32_t timestamp_ms)
{
    uint8_t valid;

    if (motor == NULL || data == NULL || motor->driver == NULL) return 0;

    /* 协议原始值存在驱动私有结构体；版本号只标记“原始帧写入中/完整”，供
     * motor_feedback_unfiltered_read 检测任务读取期间是否被 CAN 中断打断。 */
    motor->raw_feedback_version++;
    __DMB();
    valid = motor->driver->store_protocol_feedback(motor, data);
    if (valid) {
        motor->last_feedback_tick = timestamp_ms;
        motor->feedback_count++;
    }
    __DMB();
    motor->raw_feedback_version++;

    if (valid) motor->online = 1u;
    return valid;
}

/* Device CAN routing calls back through an opaque owner; device never sees model types. */
static void motor_port_feedback(void *owner, const uint8_t data[8],
                                uint32_t timestamp_ms)
{
    motor_receive_feedback((motor_handle_t)owner, data, timestamp_ms);
}

void motor_receive_feedback(motor_handle_t motor, const uint8_t data[8],
                            uint32_t timestamp_ms)
{
    (void)motor_store_protocol_feedback_instance(motor, data, timestamp_ms);
}

/* PID 状态、目标和输出属于统一控制层；反馈不在控制器中保存副本。 */
static void motor_control_clear(motor_handle_t motor)
{
    if (motor == NULL) return;

    PID_clear(&motor->ctrl.angle_pid);
    PID_clear(&motor->ctrl.speed_pid);
    motor->ctrl.output = 0.0f;
}

motor_handle_t motor_register(motor_id_e id)
{
    const motor_reg_t *reg;
    const motor_driver_ops_t *driver;
    motor_handle_t motor;

    if ((unsigned int)id >= MOTOR_COUNT) return NULL;
    if (registry[id] != NULL) return registry[id];

    reg = motor_reg_find(id);
    if (reg == NULL) return NULL;
    driver = reg->driver;
    if (!motor_driver_valid(driver)) return NULL;

    /* 具体实例内存和协议字段由驱动拥有；中间层只保存统一 handle。
     * 这里是唯一一个正常运行流程需要 motor_id_e 的地方。 */
    motor = driver->register_instance(reg);
    if (motor == NULL) return NULL;

    motor->id = id;
    motor->type = reg->type;
    motor->driver = driver;
    motor->config = reg;
    /* 电机反馈健康状态 */
    motor->raw_feedback_version = 0u;
    motor->feedback_version = 0u;
    motor->last_feedback_tick = 0u;
    motor->feedback_count = 0u;
    motor->online = 0u;
    motor->enabled = 0u;
    /* 电机控制初始状态 */
    motor->ctrl.mode = reg->mode;
    motor->ctrl.ref = 0.0f;
    motor->ctrl.output = 0.0f;
    /* 初始化pid */
    PID_init(&motor->ctrl.angle_pid, &reg->angle_pid);
    PID_init(&motor->ctrl.speed_pid, &reg->speed_pid);

    /* 逻辑总线映射与 CAN endpoint 注册属于设备层。 */
    if (!motor_port_register((uint8_t)motor->id, motor, reg->can_bus,
                             reg->can_id, motor_port_feedback))
        return NULL;

    registry[id] = motor;
    return motor;
}

uint8_t motor_is_online(motor_handle_t motor)
{
    if (motor == NULL || motor->feedback_count == 0u) return 0u;

    motor->online = ((uint32_t)(motor_port_tick_ms() - motor->last_feedback_tick) <
                     MOTOR_ONLINE_TIMEOUT_MS);
    return motor->online;
}

void motor_calc(motor_handle_t motor)
{
    fp32 speed_feedback;

    if (motor == NULL || motor->config == NULL) return;

    /* 失能或掉线时 PID 不在陈旧测量上继续累计，输出保持为零。 */
    if (!motor->enabled || !motor_is_online(motor) || !motor->fb.valid) {
        motor_control_clear(motor);
        return;
    }

    switch (motor->config->speed_feedback) {
    case MOTOR_SPEED_FEEDBACK_LINEAR:
        speed_feedback = motor->fb.speed_ms;
        break;
    case MOTOR_SPEED_FEEDBACK_ROTOR:
        speed_feedback = motor->fb.speed_rotor;
        break;
    default:
        speed_feedback = motor->fb.speed_angle;
        break;
    }

    /* 按统一控制模式跑闭环: SPEED=单速环, POSITION=角→速串级。
     * 这里保留项目原来的 PID 调用和周期语义，不把每周期控制量改写成真实 dt 模型。 */
    switch (motor->ctrl.mode) {
    case MOTOR_MODE_SPEED:
        motor->ctrl.output = PID_calc(&motor->ctrl.speed_pid,
                                      speed_feedback,
                                      motor->ctrl.ref,
                                      0);
        break;
    case MOTOR_MODE_POSITION:
    {
        float speed_ref = PID_calc(&motor->ctrl.angle_pid,
                                   motor->fb.angle,
                                   motor->ctrl.ref,
                                   speed_feedback);
        motor->ctrl.output = PID_calc(&motor->ctrl.speed_pid,
                                      speed_feedback,
                                      speed_ref,
                                      0);
        break;
    }
    case MOTOR_MODE_RELAY:
        /* 协议命令透传模式不运行 PID，ref 由具体驱动解释并打包。 */
        motor->ctrl.output = motor->ctrl.ref;
        break;
    default:
        motor_control_clear(motor);
        break;
    }

    motor->ctrl.output = motor_output_clamp(motor->ctrl.output,
                                            -motor->config->output_max,
                                             motor->config->output_max);
}

void motor_cmd_pack(motor_handle_t const *motors, uint8_t count)
{
    uint8_t i;

    if (motors == NULL || count == 0) return;

    /* 同一协议驱动只调用一次；具体协议只收到应用明确持有的实例列表。
     * 这是协议 ops 的唯一热路径使用点，且不把整个 motor 控制过程虚函数化。 */
    for (i = 0; i < count; i++) {
        uint8_t previous;
        const motor_driver_ops_t *driver;
        uint8_t already_called = 0;

        if (motors[i] == NULL) continue;
        driver = motors[i]->driver;
        if (driver == NULL) continue;

        for (previous = 0; previous < i; previous++) {
            if (motors[previous] != NULL && motors[previous]->driver == driver) {
                already_called = 1;
                break;
            }
        }
        if (!already_called) driver->pack_commands(motors, count);
    }
}

/* feedback_update 通过驱动 ops 把私有原始值换算成一帧一致的未滤波物理量。 */
uint8_t motor_feedback_unfiltered_read(motor_handle_t motor,
                                       motor_feedback_t *feedback)
{
    motor_feedback_t unfiltered = {0};
    uint32_t version_before;
    uint32_t version_after;
    uint8_t retry;

    if (motor == NULL || feedback == NULL || motor->driver == NULL ||
        motor->driver->read_unfiltered_feedback == NULL)
        return 0u;

    for (retry = 0u; retry < MOTOR_FEEDBACK_READ_RETRY; retry++) {
        version_before = motor->raw_feedback_version;
        if (version_before & 1u) continue;
        __DMB();
        if (!motor->driver->read_unfiltered_feedback(motor, &unfiltered))
            return 0u;
        unfiltered.timestamp_ms = motor->last_feedback_tick;
        unfiltered.sequence = motor->feedback_count;
        unfiltered.valid = (motor->feedback_count != 0u);
        __DMB();
        version_after = motor->raw_feedback_version;
        if (version_before == version_after && !(version_after & 1u)) {
            *feedback = unfiltered;
            return unfiltered.valid;
        }
    }

    return 0u;
}

/* application feedback_update 完成滤波/修正后覆盖唯一最终反馈。 */
uint8_t motor_feedback_publish(motor_handle_t motor,
                               const motor_feedback_t *feedback)
{
    if (motor == NULL || feedback == NULL || !feedback->valid) return 0u;

    motor->feedback_version++;
    __DMB();
    motor->fb = *feedback;
    __DMB();
    motor->feedback_version++;
    return 1u;
}

/* 读取最终反馈的一致快照；发布中的奇数版本不得被消费。 */
uint8_t motor_feedback_read(motor_handle_t motor,
                            motor_feedback_t *feedback)
{
    motor_feedback_t snapshot = {0};
    uint32_t version_before;
    uint32_t version_after;
    uint8_t retry;

    if (feedback == NULL) return 0u;
    *feedback = snapshot;
    if (motor == NULL) return 0u;

    /* 前后版本相同且为偶数，才是一帧完整发布。 */
    for (retry = 0u; retry < MOTOR_FEEDBACK_READ_RETRY; retry++) {
        version_before = motor->feedback_version;
        if (version_before & 1u) continue;
        __DMB();
        snapshot = motor->fb;
        __DMB();
        version_after = motor->feedback_version;
        if (version_before == version_after && !(version_after & 1u)) {
            *feedback = snapshot;
            return snapshot.valid;
        }
    }

    return 0u;
}

const motor_base_t *motor_state(motor_handle_t motor)
{
    return motor;
}

void motor_set_ref(motor_handle_t motor, fp32 ref)
{
    if (motor == NULL) return;
    motor->ctrl.ref = ref;
}

/* PID 后输出回写: 供 app 级后处理(离线保护/功率限制等)在 motor_calc 之后、motor_cmd_pack 之前覆盖输出 */
void motor_set_output(motor_handle_t motor, fp32 output)
{
    if (motor == NULL) return;
    motor->ctrl.output = output;
}

void motor_add_output(motor_handle_t motor, fp32 output_delta)
{
    if (motor == NULL || motor->config == NULL) return;

    /* 前馈在 PID 自身限幅之后叠加，但最终命令仍不能越过执行器硬限幅。 */
    motor->ctrl.output = motor_output_clamp(
        motor->ctrl.output + output_delta,
        -motor->config->output_max,
         motor->config->output_max);
}

void motor_reset_control(motor_handle_t motor)
{
    motor_control_clear(motor);
}

void motor_enable(motor_handle_t motor)
{
    if (motor == NULL || motor->driver == NULL) return;
    /* 先发布统一使能意图，再让协议驱动执行具体使能动作。驱动在本次调用
     * 内观察到的 enabled 与随后 pack_commands 的状态必须一致。 */
    motor->enabled = 1u;
    motor->driver->enable(motor);
}

void motor_disable(motor_handle_t motor)
{
    if (motor == NULL) return;

    motor->enabled = 0u;
    motor_control_clear(motor);
    if (motor->driver != NULL) motor->driver->disable(motor);
}

const motor_reg_t *motor_get_config(motor_handle_t motor)
{
    return motor != NULL ? motor->config : NULL;
}
