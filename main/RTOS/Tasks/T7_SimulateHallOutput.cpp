#include "T7_SimulateHallOutput.h"
#include <Arduino.h>
#include "Timing.h"
#include "CpuUtilization.h"
#include "HallSensor.h"
#include "WebSocket.h"
#include "Tasks/T4_SendToWifi.h"
#include "PinAssignments.h"
#include "driver/mcpwm.h"

const char* task7Name  = "Task - Simulate Hall Output";

// Task Handler
TaskHandle_t xTask_SimulateHallOutput = nullptr;

// Hall sensor state table (6-step commutation)
const uint8_t hallTable[6][3] = {
    {1, 0, 1},
    {1, 0, 0},
    {1, 1, 0},
    {0, 1, 0},
    {0, 1, 1},
    {0, 0, 1}
};

// Task Definition
void task_simulate_hall_output(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SIMULATE_HALL_OUTPUT_STATE_PERIOD); // Set task running frequency

    char formattedMessage[BUFFER_SIZE];
    int state = 0;
    
    // Setup PWM channels
    #if ENABLE_T7_SIMUALTE_HALL_OUTPUT

    pinMode(HALL_1_SIMULATE_PIN, OUTPUT);
    pinMode(HALL_2_SIMULATE_PIN, OUTPUT);
    pinMode(HALL_3_SIMULATE_PIN, OUTPUT);
    
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time

    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilSimulateHallOutput.set_start_time();

        // Output digital signals simulating Hall states
        digitalWrite(HALL_1_SIMULATE_PIN, hallTable[state][0]);
        digitalWrite(HALL_2_SIMULATE_PIN, hallTable[state][1]);
        digitalWrite(HALL_3_SIMULATE_PIN, hallTable[state][2]);


        // Move to next state
        state = (state + 1) % 6;

        // Set task end time (to calculate for CPU Utilization)
        UtilSimulateHallOutput.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    #endif
}