#pragma once
// Program Run Config

// WiFi
#define BUFFER_SIZE             128                 // Size of char array that stores formatted message
#define FORCE_WIFI_CONNECTION   1
#define WIFI_SSID               "POCOPHONE F1"      // Replace with your Wi-Fi SSID
#define WIFI_PASSWORD           "verynicepassword"  // Replace with your Wi-Fi password

// Testing
#define USE_DIGITAL_READ        1
#define PRINT_PERCEIVED_STATE   0
#define SIMULATE_HALL_OUTPUT    1

// Runtime Prints
// WiFi
#define PRINT_CPU_UTILIZATION               0
#define PRINT_MOTOR_RPM                     1
#define PRINT_MOTOR_MISSED_STATE            0
// Serial
#define PRINT_FREE_STACK_ON_EACH_TASKS      0