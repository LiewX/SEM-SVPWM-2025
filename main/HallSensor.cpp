// Deprecated
/*
#include "HallSensor.h"
#include "PinAssignments.h"
#include "Tasks/TasksHeaders.h"

// Volatile variable to store interrupt states
volatile uint8_t hallPattern = 0;

// ISR Handlers
void IRAM_ATTR handle_interrupt() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    #if USE_DIGITAL_READ
        hallPattern = (digitalRead(HALL_1_PIN) << 2) |
                      (digitalRead(HALL_2_PIN) << 1) |
                       digitalRead(HALL_3_PIN);
    #else
    hallPattern = (gpio_get_level(HALL_1_PIN) << 2) |
    (gpio_get_level(HALL_2_PIN) << 1) |
    gpio_get_level(HALL_3_PIN);
    #endif
    
    vTaskNotifyGiveFromISR(xTask_UpdateHalls, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

// Initialization Process
void hall_sensors_pin_init() {
    pinMode(HALL_1_PIN, INPUT);
    pinMode(HALL_2_PIN, INPUT);
    pinMode(HALL_3_PIN, INPUT);
    
    // Todo: set internal pull up for hall sensor pins
    
    
}

void hall_sensors_attatch_isr() {
    // Attach interrupts with CHANGE mode (detects both rising and falling edges)
    attachInterrupt(digitalPinToInterrupt(HALL_1_PIN), handle_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HALL_2_PIN), handle_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HALL_3_PIN), handle_interrupt, CHANGE);
}
*/