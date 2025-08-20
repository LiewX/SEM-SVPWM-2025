#include "T6_MissedCommutationCounter.h"
#include <Arduino.h>
#include "Timing.h"
#include "CpuUtilization.h"
#include "HallSensor.h"
#include "WebSocket.h"
#include "Tasks/T4_SendToWifi.h"

const char* task6Name  = "Task - Missed Commutation Counter";

// Task Handle
TaskHandle_t xTask_MissedCommutationCounter = nullptr;

// Task Definition
void task_missed_commutation_counter(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(MISSED_COMMUTATION_COUNTER_PERIOD); // Set task running frequency
    char formattedMessage[BUFFER_SIZE];
    uint32_t missedStatesPerPeriod = 0;
    double missedStatesPerMin = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time

    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilMissedCommutationCounter.set_start_time();

        // Todo: double check
        missedStatesPerPeriod = ulTaskNotifyTake(pdTRUE, 0);
        missedStatesPerMin = (double) missedStatesPerPeriod / MISSED_COMMUTATION_COUNTER_PERIOD;

        #if PRINT_MOTOR_MISSED_STATE
            // Create formatted message
            snprintf(
                formattedMessage,
                sizeof(formattedMessage),
                "Missed States Per Min %.5f", missedStatesPerMin
            );
            // Send the formatted message to the queue
            xQueueSend(xQueue_wifi, &formattedMessage, 0);
        #endif

        // Set task end time (to calculate for CPU Utilization)
        UtilMissedCommutationCounter.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}