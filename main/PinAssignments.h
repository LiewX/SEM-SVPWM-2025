#pragma once

// ESP32-S3-WROOM-1
#define ESP32_S3   1
// Input
#define HALL_1_PIN GPIO_NUM_4
#define HALL_2_PIN GPIO_NUM_5
#define HALL_3_PIN GPIO_NUM_6
#define THROTTLE_INPUT_PIN 2  // Analog 

// Output
#define AH_OUTPUT_PIN 7  // Output pin for MCPWM0A // HIGH Side
#define AL_OUTPUT_PIN 15  // Output pin for MCPWM0B // LOW Side
#define BH_OUTPUT_PIN 16  // Output pin for MCPWM1A // HIGH Side
#define BL_OUTPUT_PIN 17  // Output pin for MCPWM1B // LOW Side
#define CH_OUTPUT_PIN 18  // Output pin for MCPWM2A // HIGH Side
#define CL_OUTPUT_PIN 8  // Output pin for MCPWM2B // LOW Side
// #define LED_STATUS_PIN 2

#define HALL_1_SIMULATE_PIN 10
#define HALL_2_SIMULATE_PIN 11
#define HALL_3_SIMULATE_PIN 12

// ESP32 (Normal Variant)
// // Input
// #define HALL_1_PIN 36
// #define HALL_2_PIN 39
// #define HALL_3_PIN 34
// #define THROTTLE_INPUT_PIN 35  // Analog 

// // Output
// #define AH_OUTPUT_PIN 12  // Output pin for MCPWM0A // HIGH Side
// #define AL_OUTPUT_PIN 13  // Output pin for MCPWM0B // LOW Side
// #define BH_OUTPUT_PIN 27  // Output pin for MCPWM1A // HIGH Side
// #define BL_OUTPUT_PIN 14  // Output pin for MCPWM1B // LOW Side
// #define CH_OUTPUT_PIN 25  // Output pin for MCPWM2A // HIGH Side
// #define CL_OUTPUT_PIN 26  // Output pin for MCPWM2B // LOW Side
// #define LED_STATUS_PIN 2
