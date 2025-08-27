#include "T1_TrapCommutation.h"
#include <Arduino.h>
#include "Timing.h"
#include "CpuUtilization.h"
#include "ProgramRunConfigs.h"
#include "PinAssignments.h"
#include "driver/mcpwm.h"
#include "Tasks/T4_SendToWifi.h"
#include "Tasks/T5_MotorVelocityCalculation.h"
#include "Tasks/T6_MissedCommutationCounter.h"
#include "esp_timer.h"


// This file has been copied and modified from https://github.com/espressif/esp-idf/blob/v4.4.7/examples/peripherals/mcpwm/mcpwm_bldc_hall_control/main/mcpwm_bldc_hall_control_example_main.c

const char* task1Name  = "Task - PWM Switching";

// Task Handler
TaskHandle_t xTask_TrapCommutation = nullptr;

// Function Prototypes
static inline uint32_t bldc_get_hall_pattern(bool ccw);
static bool IRAM_ATTR bldc_hall_updated(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata, void *user_data);
static inline float map(float x, float in_min, float in_max, float out_min, float out_max);
static inline uint32_t check_deadzone(uint32_t value);
static void update_bldc_speed(void *arg);
static void bldc_set_phase_up_vm(void);
static void bldc_set_phase_wp_um(void);
static void bldc_set_phase_wp_vm(void);
static void bldc_set_phase_vp_um(void);
static void bldc_set_phase_vp_wm(void);
static void bldc_set_phase_up_wm(void);
static int perform_commutation(uint32_t &hallPattern, byte expectedPattern);

typedef void (*bldc_hall_phase_action_t)(void);

static const bldc_hall_phase_action_t s_hall_actions[] = {
    nullptr,
    bldc_set_phase_vp_wm,
    bldc_set_phase_up_vm,
    bldc_set_phase_up_wm,
    bldc_set_phase_wp_um,
    bldc_set_phase_vp_um,
    bldc_set_phase_wp_vm
};

// Task Definition
void task_trap_commutation(void *pvParameters) {
    uint32_t hallPattern = 0;
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    char formattedMessage[BUFFER_SIZE];

    // ESP_LOGI(TAG, "Setup PWM and Hall GPIO (pull up internally)");
    mcpwm_pin_config_t mcpwm_gpio_config = {
        .mcpwm0a_out_num = BLDC_PWM_UH_GPIO,
        .mcpwm0b_out_num = BLDC_PWM_UL_GPIO,
        .mcpwm1a_out_num = BLDC_PWM_VH_GPIO,
        .mcpwm1b_out_num = BLDC_PWM_VL_GPIO,
        .mcpwm2a_out_num = BLDC_PWM_WH_GPIO,
        .mcpwm2b_out_num = BLDC_PWM_WL_GPIO,
        .mcpwm_sync0_in_num  = -1,  //Not used
        .mcpwm_sync1_in_num  = -1,  //Not used
        .mcpwm_sync2_in_num  = -1,  //Not used
        .mcpwm_fault0_in_num = -1,  //Not used
        .mcpwm_fault1_in_num = -1,  //Not used
        .mcpwm_fault2_in_num = -1,  //Not used
        .mcpwm_cap0_in_num   = HALL_CAP_U_GPIO,
        .mcpwm_cap1_in_num   = HALL_CAP_V_GPIO,
        .mcpwm_cap2_in_num   = HALL_CAP_W_GPIO
    };
    ESP_ERROR_CHECK(mcpwm_set_pin(BLDC_MCPWM_GROUP, &mcpwm_gpio_config));
    // In case there's no pull-up resister for hall sensor on board
    #if HALL_INTERNAL_PULLUP
        gpio_pullup_en(HALL_CAP_U_GPIO);
        gpio_pullup_en(HALL_CAP_V_GPIO);
        gpio_pullup_en(HALL_CAP_W_GPIO);
    #endif

    // ESP_LOGI(TAG, "Initialize PWM (default to turn off all MOSFET)");
    mcpwm_config_t pwm_config = {
        .frequency = PWM_DEFAULT_FREQ,
        .cmpr_a = PWM_MIN_DUTY,
        .cmpr_b = PWM_MIN_DUTY,
        .duty_mode = MCPWM_HAL_GENERATOR_MODE_FORCE_LOW,
        .counter_mode = MCPWM_UP_COUNTER
    };
    ESP_ERROR_CHECK(mcpwm_init(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, &pwm_config));

    // ESP_LOGI(TAG, "Initialize Hall sensor capture");
    mcpwm_capture_config_t cap_config = {
        .cap_edge = MCPWM_BOTH_EDGE,
        .cap_prescale = 1,
        .capture_cb = bldc_hall_updated,
        .user_data = cur_task,
    };
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP0, &cap_config));
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP1, &cap_config));
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP2, &cap_config));
    // ESP_LOGI(TAG, "Please turn on the motor power");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ESP_LOGI(TAG, "Changing speed at background");
    const esp_timer_create_args_t bldc_timer_args = {
        .callback = update_bldc_speed,
        .name = "bldc_speed"
    };
    esp_timer_handle_t bldc_speed_timer;
    ESP_ERROR_CHECK(esp_timer_create(&bldc_timer_args, &bldc_speed_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(bldc_speed_timer, SAMPLE_THROTTLE_AND_UPDATE_DUTY_FREQUENCY));

    // --- Initial rotor alignment ---
    // hallPattern = bldc_get_hall_pattern(false);
    // if (hallPattern >= 1 && hallPattern <= 6) {
    //     s_hall_actions[hallPattern]();  // set initial phase
    // }
    // vTaskDelay(pdMS_TO_TICKS(2500));

    for (;;) {
        // The rotation direction is controlled by inverting the hall sensor value
        hallPattern = bldc_get_hall_pattern(false);
        if (hallPattern >= 1 && hallPattern <= 6) {
            s_hall_actions[hallPattern]();
            #if PRINT_INDIVIDUAL_STATES_FROM_ISR
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "Hall sensor value: %d%d%d", (hallPattern >> 2) & 1, (hallPattern >> 1) & 1, hallPattern & 1
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            #if ENABLE_T5_MOTOR_VELOCITY_CALCULATION
                xTaskNotifyGive(xTask_MotorVelocityCalculation);
            #endif
        } else {
            #if PRINT_INDIVIDUAL_MISSED_STATES
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "Invalid bldc phase, wrong hall sensor value: %d%d%d", (hallPattern >> 2) & 1, (hallPattern >> 1) & 1, hallPattern & 1
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // ESP_LOGE(TAG, "invalid bldc phase, wrong hall sensor value:%d", hallPattern);
            #if ENABLE_T6_MISSED_COMMUTATION_COUNTER
                xTaskNotifyGive(xTask_MissedCommutationCounter);
            #endif
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }


    ///////////////
    // To be implemented
    /*
    int invalidStateCount = 0;
    for (;;) {
        invalidStateCount = perform_commutation(hallPattern, 0b101);
        invalidStateCount = perform_commutation(hallPattern, 0b100);
        invalidStateCount = perform_commutation(hallPattern, 0b110);
        invalidStateCount = perform_commutation(hallPattern, 0b010);
        invalidStateCount = perform_commutation(hallPattern, 0b011);
        invalidStateCount = perform_commutation(hallPattern, 0b001);
        
        if (invalidStateCount >= 6) {
            #if PRINT_INDIVIDUAL_STATES_FROM_ISR
                // Create formatted message
                snprintf(
                    formattedMessage,
                    sizeof(formattedMessage),
                    "Hall pattern: Invalid"
                );
                // Send the formatted message to the queue
                xQueueSend(xQueue_wifi, &formattedMessage, 0);
            #endif
            // Wait for ISR notification for next state change
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
    //*/
}

// Default to false for ccw argument
static inline uint32_t bldc_get_hall_pattern(bool ccw) {
    uint32_t hall_val = gpio_get_level(HALL_CAP_U_GPIO) * 4 + gpio_get_level(HALL_CAP_V_GPIO) * 2 + gpio_get_level(HALL_CAP_W_GPIO) * 1;
    return ccw ? hall_val ^ (0x07) : hall_val;
}

static bool IRAM_ATTR bldc_hall_updated(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata, void *user_data) {
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(task_to_notify, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline uint32_t check_deadzone(uint32_t value, uint32_t minValue) {
    return (value > minValue) ? value : 0;
}

static void update_bldc_speed(void *arg) {
    static float duty = 0.0f;

    // Testing without throttle input to test automatic pwm sweep from PWM_MIN_DUTY to PWM_MAX_DUTY to PWM_MIN_DUTY and repeat
    // static float duty_step = PWM_DUTY_STEP;
    // duty += duty_step;
    // if (duty > PWM_MAX_DUTY || duty < PWM_MIN_DUTY) {
    //     duty_step *= -1;
    // }

    // Sample and map adc value
    uint32_t rawThrottleInput = analogReadMilliVolts(THROTTLE_INPUT_PIN);
    duty = check_deadzone(rawThrottleInput, THROTTLE_DEADZONE);
    duty = map(duty, 0.0f, 3300.0f, 0.0f, 95.0f);

    // Print throttle input to WiFi
    #if PRINT_THROTTLE_INPUT
        char formattedMessage[BUFFER_SIZE];
        // Create formatted message
        snprintf(
            formattedMessage,
            sizeof(formattedMessage),
            "Raw Throttle: %4d,\tThrottle Duty: %5.2f", rawThrottleInput, duty
        );
        // Send the formatted message to the queue
        xQueueSend(xQueue_wifi, &formattedMessage, 0);
    #endif

    mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH, duty);
    mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW,  duty);
    mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH, duty);
    mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW,  duty);
    mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH, duty);
    mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW,  duty);
}

// U+V- / A+B-  / 010
static void bldc_set_phase_up_vm(void) {
    mcpwm_set_duty_type   (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // U+ = PWM
    mcpwm_deadtime_enable (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, PWM_DEADTIME, PWM_DEADTIME); // U- = _PWM_
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH); // V+ = 0
    mcpwm_set_signal_high (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW);  // V- = 1
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH); // W+ = 0
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW);  // W- = 0
}

// W+U- / C+A-  / 100
static void bldc_set_phase_wp_um(void) {
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH); // U+ = 0
    mcpwm_set_signal_high (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW);  // U- = 1
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH); // V+ = 0
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW);  // V- = 0
    mcpwm_set_duty_type   (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // W+ = PWM
    mcpwm_deadtime_enable (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, PWM_DEADTIME, PWM_DEADTIME); // W- = _PWM_
}

// W+V- / C+B-  / 110
static void bldc_set_phase_wp_vm(void) {
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH); // U+ = 0
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW);  // U- = 0
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH); // V+ = 0
    mcpwm_set_signal_high (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW);  // V- = 1
    mcpwm_set_duty_type   (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // W+ = PWM
    mcpwm_deadtime_enable (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, PWM_DEADTIME, PWM_DEADTIME); // W- = _PWM_
}

// V+U- / B+A-  / 101
static void bldc_set_phase_vp_um(void) {
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH); // U+ = 0
    mcpwm_set_signal_high (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW);  // U- = 1
    mcpwm_set_duty_type   (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // V+ = PWM
    mcpwm_deadtime_enable (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, PWM_DEADTIME, PWM_DEADTIME); // V- = _PWM_
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH); // W+ = 0
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW);  // W- = 0
}

// V+W- / B+C-  / 001
static void bldc_set_phase_vp_wm(void) {
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH); // U+ = 0
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW);  // U- = 0
    mcpwm_set_duty_type   (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // V+ = PWM
    mcpwm_deadtime_enable (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, PWM_DEADTIME, PWM_DEADTIME); // V- = _PWM_
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH); // W+ = 0
    mcpwm_set_signal_high (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW);  // W- = 1
}

// U+W- / A+C-  / 011
static void bldc_set_phase_up_wm(void) {
    mcpwm_set_duty_type   (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // U+ = PWM
    mcpwm_deadtime_enable (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, PWM_DEADTIME, PWM_DEADTIME); // U- = _PWM_
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH); // V+ = 0
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW);  // V- = 0
    mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W);
    mcpwm_set_signal_low  (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH); // W+ = 0
    mcpwm_set_signal_high (BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW);  // W- = 1
}

static int perform_commutation(uint32_t &hallPattern, byte expectedPattern) {
    char formattedMessage[BUFFER_SIZE];
    static uint32_t invalidStateCount = 0;
    if (hallPattern == expectedPattern) {
        // Perform switching
        s_hall_actions[hallPattern]();
        // Increment notification count for rpm calculation
        #if ENABLE_T5_MOTOR_VELOCITY_CALCULATION
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
        #endif
        invalidStateCount = 0;
        #if PRINT_INDIVIDUAL_STATES_FROM_ISR
            // Create formatted message
            snprintf(
                formattedMessage,
                sizeof(formattedMessage),
                "Hall pattern: %d%d%d (ISR)", (hallPattern >> 2) & 1, (hallPattern >> 1) & 1, hallPattern & 1
            );
            // Send the formatted message to the queue
            xQueueSend(xQueue_wifi, &formattedMessage, 0);
        #endif
        // Wait for ISR notification for next state change
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        hallPattern = bldc_get_hall_pattern(false);
    }
    else {
        // Increment notification count for rpm calculation
        #if ENABLE_T5_MOTOR_VELOCITY_CALCULATION
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
        #endif
        // Increment notification count for recording missed state
        #if ENABLE_T6_MISSED_COMMUTATION_COUNTER
            xTaskNotifyGive(xTask_MotorVelocityCalculation);
        #endif
        invalidStateCount++;
    }
    return invalidStateCount;
}