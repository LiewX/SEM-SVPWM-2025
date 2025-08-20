#pragma once
#include <Arduino.h>

extern const char* task2Name;

// Volatile variable to store interrupt state
extern volatile uint8_t hallPattern;

// Task Handle
extern TaskHandle_t xTask_UpdateHalls;

// Function prototypes for task
void task_update_halls(void *pvParameters);
