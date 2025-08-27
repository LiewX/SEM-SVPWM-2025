#pragma once
#include <Arduino.h>

bool create_and_check_sem(SemaphoreHandle_t &sem, const char* semName);
bool create_and_check_queue(QueueHandle_t &queue, const char* queueName, size_t size, size_t itemSize);
bool create_and_check_task(bool taskEnable, void (*taskFunc)(void*), const char* taskName, uint32_t stackSize, UBaseType_t priority, TaskHandle_t* taskHandle);
void print_free_stack(TaskHandle_t taskHandle, const char* taskName);
