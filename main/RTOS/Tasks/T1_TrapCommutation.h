#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task1Name;

// Global Variable
extern TaskCpuUtilization UtilTrapCommutation; // for calculating CPU Utilization for task

// Task Handler
extern TaskHandle_t xTask_TrapCommutation;

// Function prototypes for task
void task_trap_commutation(void *pvParameters);