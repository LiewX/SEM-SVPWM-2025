#include "T1_PwmSwitching.h"
#include <Arduino.h>
#include "Timing.h"
#include "CpuUtilization.h"

const char* task1Name  = "Task - PWM Switching";

// Task Handle
TaskHandle_t xTask_PwmSwitching = nullptr;

// Task Definition
void task_pwm_switching(void *pvParameters) {


}