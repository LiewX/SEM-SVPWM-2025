#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task3Name;

// Global class variable for calculating CPU Utilization for task
extern TaskCpuUtilization UtilWebSocketHandler;

// Task Handle
extern TaskHandle_t xTask_WebsocketHandler;

// Function prototypes for task
void task_websocket_handler(void *pvParameters);