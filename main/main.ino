#include <Arduino.h>
#include "Websocket.h"
#include "PinAssignments.h"
#include "HallSensor.h"
#include "Timing.h"
#include "CpuUtilization.h"
#include "RtosUtils.h"
#include "./Tasks/TasksHeaders.h"
#include "ProgramRunConfigs.h"

// Global class variable for calculating CPU Utilization for each task
TaskCpuUtilization UtilPwmSwitching;
TaskCpuUtilization UtilWebSocketHandler;
TaskCpuUtilization UtilSendToWifi;
TaskCpuUtilization UtilMotorVelocityCalculation;
TaskCpuUtilization UtilMissedCommutationCounter;
TaskCpuUtilization UtilSimulateHallOutput;

// Main Setup
void setup() {
    Serial.begin(115200);
    hall_sensors_pin_init();
    websocket_setup();

    // Initialization of Global class variable for calculating CPU Utilization for each task
    UtilPwmSwitching            .init(PWM_SWITCHING_PERIOD,             task1Name, &xTask_PwmSwitching);
    UtilWebSocketHandler        .init(WEBSOCKET_HANDLING_PERIOD,        task3Name, &xTask_WebsocketHandler);
    UtilSendToWifi              .init(SEND_TO_WIFI_PERIOD,              task4Name, &xTask_SendToWiFi);
    UtilMotorVelocityCalculation.init(VELOCITY_CALCULATION_PERIOD,      task5Name, &xTask_MotorVelocityCalculation);
    UtilMissedCommutationCounter.init(MISSED_COMMUTATION_COUNTER_PERIOD,task6Name, &xTask_MissedCommutationCounter);
    UtilSimulateHallOutput      .init(SIMULATE_HALL_OUTPUT_STATE_PERIOD,task7Name, &xTask_SimulateHallOutput);

    // Creation status flag for all FreeRTOS kernel objects
    bool creationStatus = 1; 
    // Queue creation
    creationStatus &= create_and_check_queue(xQueue_wifi, "Queue - Send to WiFi", 10, BUFFER_SIZE);
    // Task creation
    // creationStatus &= create_and_check_task(task_pwm_switching,              task1Name, 4096, 4, &xTask_PwmSwitching);
    creationStatus &= create_and_check_task(task_update_halls,                  task2Name, 4096, 5, &xTask_UpdateHalls);
    creationStatus &= create_and_check_task(task_websocket_handler,             task3Name, 3072, 1, &xTask_WebsocketHandler);
    creationStatus &= create_and_check_task(task_send_to_wifi,                  task4Name, 4096, 2, &xTask_SendToWiFi);
    creationStatus &= create_and_check_task(task_motor_velocity_calculation,    task5Name, 4096, 3, &xTask_MotorVelocityCalculation);
    creationStatus &= create_and_check_task(task_missed_commutation_counter,    task6Name, 4096, 3, &xTask_MissedCommutationCounter);
    #if SIMULATE_HALL_OUTPUT
        creationStatus &= create_and_check_task(task_simulate_hall_output,      task7Name, 4096, 4, &xTask_SimulateHallOutput);
    #endif
    // If any of the semaphore/mutex and queue has failed to create, exit
    if (creationStatus == 0) {
        Serial.printf("Exiting program.\n");
        // Stop Program
        for(;;){}
    }

    // Display blinking LED to indicate start of program.
    #if ESP32_S3 == 0
        pinMode(LED_STATUS_PIN, OUTPUT);
        digitalWrite(LED_STATUS_PIN, HIGH);
        Serial.println("Starting program."); 
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LED_STATUS_PIN, LOW);
    #else
        Serial.println("Starting program."); 
    #endif

    hall_sensors_attatch_isr();

    // Check free stack of each tasks
    #if PRINT_FREE_STACK_ON_EACH_TASKS
        vTaskDelay(pdMS_TO_TICKS(4000)); // delay to let tasks run before printing free stack on each tasks
        // print_free_stack(xTask_PwmSwitching, task1Name);
        print_free_stack(xTask_UpdateHalls, task2Name);
        print_free_stack(xTask_WebsocketHandler, task3Name);
        print_free_stack(xTask_SendToWiFi, task4Name);
        print_free_stack(xTask_MotorVelocityCalculation, task5Name);
        print_free_stack(xTask_MissedCommutationCounter, task6Name);
        // print_free_stack(xTask_SimulateHallOutput, task7Name);
        Serial.printf("Free heap size: %d bytes\n", esp_get_free_heap_size());  
        Serial.printf("Minimum free heap ever: %d bytes\n", esp_get_minimum_free_heap_size()); 
    #endif
}

void loop() {
    #if PRINT_CPU_UTILIZATION
        // UtilPwmSwitching.send_util_to_wifi();
        UtilWebSocketHandler        .send_util_to_wifi();
        UtilSendToWifi              .send_util_to_wifi();
        UtilMotorVelocityCalculation.send_util_to_wifi();
        UtilMissedCommutationCounter.send_util_to_wifi();
        UtilSimulateHallOutput      .send_util_to_wifi();
    #endif
    vTaskDelay(pdMS_TO_TICKS(CPU_UTIL_CALCULATION_PERIOD));
}