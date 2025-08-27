#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"
#include "ProgramRunConfigs.h"

extern const char* task7Name;

// Global Variable
extern TaskCpuUtilization UtilSimulateHallOutput; // for calculating CPU Utilization for task

// Task Handler
extern TaskHandle_t xTask_SimulateHallOutput;

// Function prototypes for task
void task_simulate_hall_output(void *pvParameters);