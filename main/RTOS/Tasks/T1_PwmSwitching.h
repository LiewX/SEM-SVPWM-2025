#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task1Name;

// Global class variable for calculating CPU Utilization for task
extern TaskCpuUtilization UtilPwmSwitching;

// Task Handle
extern TaskHandle_t xTask_PwmSwitching;

// Function prototypes for task
void task_pwm_switching(void *pvParameters);