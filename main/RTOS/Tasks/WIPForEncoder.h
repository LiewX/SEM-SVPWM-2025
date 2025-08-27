#if 0
#pragma once
#include <Arduino.h>

extern const char* task2Name;

// Volatile variable to store interrupt state
extern volatile uint8_t hallPattern;

// Task Handler
extern TaskHandle_t xTask_SvpwmCommutation;

// Function prototypes for task
void task_svpwm_commutation(void *pvParameters);
#endif