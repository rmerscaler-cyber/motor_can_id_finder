#ifndef MOTOR_UTILS_H
#define MOTOR_UTILS_H

#include "user_lib.h"   /* 复用现有: fp32_constrain / rad_format (不重写) */

/* rad_format 内部用到 PI, 而 user_lib.h 未定义, 在此兜底 */
#ifndef PI
#define PI 3.14159265358979f
#endif

/* ============ 输出限幅 (复用 fp32_constrain, min/max 由调用方填) ============ */
#define motor_output_clamp(val, min, max)   fp32_constrain((val), (min), (max))

/* ============ 角度限幅 (复用 rad_format: -π ~ π) ============ */
#define motor_angle_wrap(rad)          rad_format(rad)

#endif /* MOTOR_UTILS_H */
