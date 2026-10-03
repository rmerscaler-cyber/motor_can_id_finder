#ifndef MOTOR_REG_H
#define MOTOR_REG_H

#include "motor_typedef.h"   /* motor_reg_t */

/* 板级拓扑、协议绑定及 PID 标定由本模型模块的 motor_reg.c 提供。 */
extern const motor_reg_t motor_regs[];
extern const uint8_t motor_reg_count;

extern const motor_probe_config_t motor_probe_regs[];
extern const uint8_t motor_probe_reg_count;

#endif /* MOTOR_REG_H */
