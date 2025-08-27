#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task4Name;

// Global Variable
extern TaskCpuUtilization UtilSendToWifi; // for calculating CPU Utilization for task

// Task Handler
extern TaskHandle_t xTask_SendToWifi;

// Queue Handles
extern QueueHandle_t xQueue_wifi;

// Function prototypes for task
void task_send_to_wifi(void *pvParameters);