#pragma once

// ESP32-S3-WROOM-1
#define ESP32_S3   0

#if ESP32_S3
// Input
#define HALL_CAP_U_GPIO         GPIO_NUM_4
#define HALL_CAP_V_GPIO         GPIO_NUM_5
#define HALL_CAP_W_GPIO         GPIO_NUM_6
#define HALL_INTERNAL_PULLUP    0
#define THROTTLE_INPUT_PIN      2  // Analog 

// Output
#define BLDC_PWM_UH_GPIO 7      // Output pin for MCPWM0A // HIGH Side
#define BLDC_PWM_UL_GPIO 15     // Output pin for MCPWM0B // LOW Side
#define BLDC_PWM_VH_GPIO 16     // Output pin for MCPWM1A // HIGH Side
#define BLDC_PWM_VL_GPIO 17     // Output pin for MCPWM1B // LOW Side
#define BLDC_PWM_WH_GPIO 18     // Output pin for MCPWM2A // HIGH Side
#define BLDC_PWM_WL_GPIO 8      // Output pin for MCPWM2B // LOW Side

#define HALL_1_SIMULATE_PIN 10
#define HALL_2_SIMULATE_PIN 11
#define HALL_3_SIMULATE_PIN 12

#else

// ESP32 (Normal Variant)
// Input
#define HALL_INTERNAL_PULLUP    0
// #define HALL_CAP_U_GPIO GPIO_NUM_36
// #define HALL_CAP_V_GPIO GPIO_NUM_39
// #define HALL_CAP_W_GPIO GPIO_NUM_34
#define HALL_CAP_U_GPIO GPIO_NUM_21
#define HALL_CAP_V_GPIO GPIO_NUM_19
#define HALL_CAP_W_GPIO GPIO_NUM_18
#define THROTTLE_INPUT_PIN 35  // Analog 

// Output
#define BLDC_PWM_UH_GPIO 12  // Output pin for MCPWM0A // HIGH Side
#define BLDC_PWM_UL_GPIO 13  // Output pin for MCPWM0B // LOW Side
#define BLDC_PWM_VH_GPIO 27  // Output pin for MCPWM1A // HIGH Side
#define BLDC_PWM_VL_GPIO 14  // Output pin for MCPWM1B // LOW Side
#define BLDC_PWM_WH_GPIO 25  // Output pin for MCPWM2A // HIGH Side
#define BLDC_PWM_WL_GPIO 26  // Output pin for MCPWM2B // LOW Side
#define LED_STATUS_PIN 2

#endif