#ifndef ROBOT_APPLICATION_H
#define ROBOT_APPLICATION_H

#include <stdint.h>

void robot_application_init(void);
/* 唯一周期入口，由 robot_task 调用。 */
void robot_application_step(uint32_t now_ms);

#endif
