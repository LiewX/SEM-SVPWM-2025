#include "T5_MotorVelocityCalculation.h"
#include <Arduino.h>
#include "Timing.h"
#include "CpuUtilization.h"
#include "HallSensor.h"
#include "Websocket.h"
#include "T4_SendToWifi.h"

const char* task5Name  = "Task - Motor Velocity Calculation";

// Task Handler
TaskHandle_t xTask_MotorVelocityCalculation = nullptr;

// Task Definition
void task_motor_velocity_calculation(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(VELOCITY_CALCULATION_PERIOD); // Set task running frequency
    char formattedMessage[BUFFER_SIZE];
    uint32_t countsPerPeriod = 0;
    double countsPerMinute = 0.0;
    double rpm = 0.0;
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time

    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilMotorVelocityCalculation.set_start_time();

        // Todo: double check
        countsPerPeriod = ulTaskNotifyTake(pdTRUE, 0);
        countsPerMinute = (double) countsPerPeriod * (60000.0 / VELOCITY_CALCULATION_PERIOD);
        rpm = countsPerMinute / 6.0;

        #if PRINT_MOTOR_RPM
            // Create formatted message
            snprintf(
                formattedMessage,
                sizeof(formattedMessage),
                "CPP: %lu, CPM: %.2f, RPM: %.2f",countsPerPeriod, countsPerMinute, rpm
            );
            // Send the formatted message to the queue
            xQueueSend(xQueue_wifi, &formattedMessage, 0);
        #endif

        // Set task end time (to calculate for CPU Utilization)
        UtilMotorVelocityCalculation.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}