#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task6Name;

// Global class variable for calculating CPU Utilization for task
extern TaskCpuUtilization UtilMissedCommutationCounter;

// Task Handle
extern TaskHandle_t xTask_MissedCommutationCounter;

// Function prototypes for task
void task_missed_commutation_counter(void *pvParameters);