#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task4Name;

// Global class variable for calculating CPU Utilization for task
extern TaskCpuUtilization UtilSendToWifi;

// Task Handle
extern TaskHandle_t xTask_SendToWiFi;

// Queue Handles
extern QueueHandle_t xQueue_wifi;

// Function prototypes for task
void task_send_to_wifi(void *pvParameters);