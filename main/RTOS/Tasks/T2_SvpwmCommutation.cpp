#include "T2_SvpwmCommutation.h"
#include <Arduino.h>
#include "Timing.h"
#include "PinAssignments.h"
#include "RtosUtils.h"
#include "T4_SendToWifi.h"

const char* task2Name  = "Task - Update SVPWM Vector";

// Task Handler
TaskHandle_t xTask_SvpwmCommutation = nullptr;

// Task Definition
void task_svpwm_commutation(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SVPWM_VECTOR_UPDATE_FREQUENCY); // Set task running frequency
    char formattedMessage[BUFFER_SIZE];


    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    for(;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilSvpwmCommutation.set_start_time();

        // Determine vector direction
        // Get orientation from encoder

        // Determine vector magnitude
        // Get throttle input and calculate PID output

        // Calculate UVW duty cycles based on vector direction and magnitude

        // Update MCPWM params

        // Set task end time (to calculate for CPU Utilization)
        UtilSvpwmCommutation.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}