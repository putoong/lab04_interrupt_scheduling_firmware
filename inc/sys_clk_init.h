#ifndef __SYS_CLK_INIT_H
#define __SYS_CLK_INIT_H

#include "stm32f10x_conf.h"
#include "scheduling.h"
#include "lab04_tasks.h"

void SetSysClockTo72(void);
void SystemInit_1 (void);

#define SystemCoreClock  72000000

#endif