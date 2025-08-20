#include "CpuUtilization.h"
#include "Timing.h"
#include "RtosUtils.h"
#include "ProgramRunConfigs.h"
#include "./Tasks/T4_SendToWifi.h"

// Constructor
TaskCpuUtilization::TaskCpuUtilization()
    : startTime(0), endTime(0), taskDuration(0.0), taskExecutionCount(0), taskUtilization(0.0), taskToMonitor(nullptr), pTaskHandle(nullptr) {}

void TaskCpuUtilization::init(uint32_t taskPeriod, const char* taskName, TaskHandle_t* pHandle) {
    this->taskToMonitor = taskName;
    this->pTaskHandle = pHandle;

    taskExecutionCount = ((double) CPU_UTIL_CALCULATION_PERIOD / taskPeriod) + 0.5;

    // Create semaphore for synchronizing calculation of task utilization after task execution
    xSemaphore_ExecutedTask = xSemaphoreCreateBinary();
    // Check semaphore creation status
    if (xSemaphore_ExecutedTask == NULL) {
        Serial.printf("Failed to create semaphore for CPU Utilization for '%s'\n", taskName);
    }
}

void TaskCpuUtilization::set_start_time() {
    #if PRINT_CPU_UTILIZATION
    // Set task start time
    startTime = micros();
    #endif
}

void TaskCpuUtilization::set_end_time() {
    #if PRINT_CPU_UTILIZATION
    // Set task end time
    endTime = micros();
    // Give semaphore and allow cpu utilization to be calculated and sent to WiFi
    xSemaphoreGive(xSemaphore_ExecutedTask);
    #endif
}

void TaskCpuUtilization::send_util_to_wifi(){
    #if PRINT_CPU_UTILIZATION
    if (this->pTaskHandle != nullptr && *(this->pTaskHandle) != nullptr) {
        // Check if task is suspended
        if (eTaskGetState(*(this->pTaskHandle)) != eSuspended) {
            // Take semaphore
            if (xSemaphoreTake(xSemaphore_ExecutedTask, pdMS_TO_TICKS(200))) {
                // Calculate task duration
                taskDuration = (double) endTime - startTime;
                if (taskDuration > 0.0)
                    // Calculate total time spent in task during CPU_UTIL_CALCULATION_PERIOD
                    taskUtilization = taskExecutionCount * taskDuration / (CPU_UTIL_CALCULATION_PERIOD*1000) * 100;
                else taskUtilization = 0.0;

                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    // Detailed
                    // "%s:\t\t%7lu\t%3lu\t%1.2f", taskToMonitor, endTime, startTime, taskUtilization
                    "%s:  \t\t\t%1.2f", taskToMonitor, taskUtilization
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            }
            else {
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "%s:\t\tCould not retrieve CPU Utilization", taskToMonitor
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            }
        }
        else {
            snprintf(
                formattedMessage, 
                sizeof(formattedMessage), 
                "%s:\t\tTask is suspended", taskToMonitor
            );
            // Send the formatted message to the queue
            xQueueSend(xQueue_wifi, &formattedMessage, 0);
        }
    }
    else {
        snprintf(
            formattedMessage, 
            sizeof(formattedMessage), 
            "%s:\t\tTask handle is not found", taskToMonitor
        );
        // Send the formatted message to the queue
        xQueueSend(xQueue_wifi, &formattedMessage, 0);
        Serial.print("Test");
    }
    #endif
}