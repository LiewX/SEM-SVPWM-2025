#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task5Name;

// Global Variable
extern TaskCpuUtilization UtilMotorVelocityCalculation; // for calculating CPU Utilization for task

// Task Handler
extern TaskHandle_t xTask_MotorVelocityCalculation;

// Function prototypes for task
void task_motor_velocity_calculation(void *pvParameters);