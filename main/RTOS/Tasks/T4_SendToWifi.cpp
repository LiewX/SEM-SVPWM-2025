#include "T4_SendToWifi.h"
#include <Arduino.h>
#include "Timing.h"
#include "CpuUtilization.h"
#include "Websocket.h"

const char* task4Name  = "Task - Send WiFi Data";

// Task Handler
TaskHandle_t xTask_SendToWifi = nullptr;

// Queue Handles
QueueHandle_t xQueue_wifi;

// Task Definition
void task_send_to_wifi(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(SEND_TO_WIFI_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    char message [128];
    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilSendToWifi.set_start_time();

        // Wait until there is data in the WiFi queue
        if (xQueueReceive(xQueue_wifi, &message, portMAX_DELAY)) {
            // If client is connected to WebSocket server
            if (clientConnected && client.available())
                client.send(message);  // Send the received message to WebSocket server (Important: make sure 'message' is null-terminated)
        }

        // Set task end time (to calculate for CPU Utilization)
        UtilSendToWifi.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}