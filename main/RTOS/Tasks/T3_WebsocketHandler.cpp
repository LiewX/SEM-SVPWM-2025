#include "T3_WebsocketHandler.h"
#include <Arduino.h>
#include "Timing.h"
#include "Websocket.h"
#include "CpuUtilization.h"

const char* task3Name  = "Task - WebSocket Handler";

// Task Handler
TaskHandle_t xTask_WebsocketHandler = nullptr;

// Task Definition
void task_websocket_handler(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(WEBSOCKET_HANDLING_PERIOD); // Set task running frequency
    TickType_t xLastWakeTime = xTaskGetTickCount();   // Initialize last wake time
    for (;;) {
        // Set task start time (to calculate for CPU Utilization)
        UtilWebSocketHandler.set_start_time();

        // Accept new WebSocket client connections
        if (!clientConnected) {
            auto newClient = server.accept();
            if (newClient.available()) {
                Serial.println("New WebSocket client connected!");
                client = newClient;
                clientConnected = true;
            }
        }
        // Handle client disconnection
        if (clientConnected && !client.available()) {
            Serial.println("Client disconnected!");
            client.close();
            clientConnected = false;
        }

        // Set task end time (to calculate for CPU Utilization)
        UtilWebSocketHandler.set_end_time();
        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}