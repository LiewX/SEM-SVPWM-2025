#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task2Name;

// Global Variables
extern TaskCpuUtilization UtilSvpwmCommutation; // for calculating CPU Utilization for task

// Task Handler
extern TaskHandle_t xTask_SvpwmCommutation;

// Function prototypes for task
void task_svpwm_commutation(void *pvParameters);