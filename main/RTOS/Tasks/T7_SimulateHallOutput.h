#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task7Name;

// Global class variable for calculating CPU Utilization for task
extern TaskCpuUtilization UtilSimulateHallOutput;

// Task Handle
extern TaskHandle_t xTask_SimulateHallOutput;

// Function prototypes for task
void task_simulate_hall_output(void *pvParameters);