#pragma once
#include <Arduino.h>
#include "CpuUtilization.h"

extern const char* task3Name;

// Global Variable
extern TaskCpuUtilization UtilWebSocketHandler; // for calculating CPU Utilization for task

// Task Handler
extern TaskHandle_t xTask_WebsocketHandler;

// Function prototypes for task
void task_websocket_handler(void *pvParameters);