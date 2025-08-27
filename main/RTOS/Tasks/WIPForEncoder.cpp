#if 0
#include "T2_SvpwmCommutation.h"
#include <Arduino.h>
#include "Timing.h"
#include "T5_MotorVelocityCalculation.h"
#include "ProgramRunConfigs.h"
#if PRINT_INDIVIDUAL_STATES
    #include "Tasks/T4_SendToWifi.h"
#endif

const char* task2Name  = "Task - Get Throttle Input";

// Task Handle
TaskHandle_t xTask_SvpwmCommutation = nullptr;

// Task Definition
void task_svpwm_commutation(void *pvParameters) {
    uint8_t invalidStateCount = 0;
    char formattedMessage[BUFFER_SIZE];
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for(;;) {
        // State - 1
        if (hallPattern == 0b101) {
            // Todo: Perform switching
            
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount = 0;
            #if PRINT_INDIVIDUAL_STATES
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "1"
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // Wait for ISR notification for next state change
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        else {
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            // Increment notification count for recording missed state
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount++;
        }

        // State - 2
        if (hallPattern == 0b100) {
            // Todo: Perform switching
            
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount = 0;
            #if PRINT_INDIVIDUAL_STATES
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "2"
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // Wait for ISR notification for next state change
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        else {
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            // Increment notification count for recording missed state
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount++;
        }

        // State - 3
        if (hallPattern == 0b110) {
            // Todo: Perform switching
            
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount = 0;
            #if PRINT_INDIVIDUAL_STATES
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "3"
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // Wait for ISR notification for next state change
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        else {
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            // Increment notification count for recording missed state
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount++;
        }

        // State - 4
        if (hallPattern == 0b010) {
            // Todo: Perform switching
            
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount = 0;
            #if PRINT_INDIVIDUAL_STATES
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "4"
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // Wait for ISR notification for next state change
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        else {
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            // Increment notification count for recording missed state
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount++;
        }

        // State - 5
        if (hallPattern == 0b011) {
            // Todo: Perform switching
            
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount = 0;
            #if PRINT_INDIVIDUAL_STATES
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "5"
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // Wait for ISR notification for next state change
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        else {
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            // Increment notification count for recording missed state
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount++;
        }

        // State - 6
        if (hallPattern == 0b001) {
            // Todo: Perform switching
            
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount = 0;
            #if PRINT_INDIVIDUAL_STATES
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "6"
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // Wait for ISR notification for next state change
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        else {
            // Increment notification count for rpm calculation
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            // Increment notification count for recording missed state
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
            invalidStateCount++;
        }

        // State - Invalid
        // This snippet is necessary to avoid infinite loop in case there is an invalid state
        // Todo: record invalid state and mention to user. currently, it is transmitted through missed state counts through else statements
        if (invalidStateCount == 6) {
            invalidStateCount = 0;
            #if PRINT_INDIVIDUAL_STATES
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "Invalid"
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // Wait for ISR notification for next state change
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

    }
}
#endif