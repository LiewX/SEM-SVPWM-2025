#pragma once
// Program Run Config

// Run Tasks
#define ENABLE_T1_TRAP_COMMUTATION             1
#define ENABLE_T2_SVPWM_COMMUTATION            0
#define ENABLE_T3_WEBSOCKET_HANDLER            1
#define ENABLE_T4_SEND_TO_WIFI                 1
#define ENABLE_T5_MOTOR_VELOCITY_CALCULATION   0
#define ENABLE_T6_MISSED_COMMUTATION_COUNTER   0
#define ENABLE_T7_SIMUALTE_HALL_OUTPUT         0

// WiFi Settings
#define BUFFER_SIZE             128                 // Size of char array that stores formatted message
#if (ENABLE_T3_WEBSOCKET_HANDLER && ENABLE_T4_SEND_TO_WIFI)
#define FORCE_WIFI_CONNECTION   1
#define WIFI_SSID               "POCOPHONE F1"      // Replace with your Wi-Fi SSID
#define WIFI_PASSWORD           "verynicepassword"  // Replace with your Wi-Fi password
#endif

// Runtime Prints
// WiFi
#if (ENABLE_T3_WEBSOCKET_HANDLER && ENABLE_T4_SEND_TO_WIFI)
#define PRINT_CPU_UTILIZATION                           0
#define PRINT_MOTOR_RPM                                 0
#define PRINT_INDIVIDUAL_MISSED_STATES                  0   // Trapezoidal; only use for hall sensor mode
#define PRINT_INDIVIDUAL_STATES_FROM_ISR                1   // Trapezoidal; only use for hall sensor mode
#define PRINT_INDIVIDUAL_STATES_FROM_PERIODIC_SAMPLING  1
#define PRINT_THROTTLE_INPUT                            1
#endif
// Serial
#define PRINT_FREE_STACK_ON_EACH_TASKS      1

// Throttle
#define THROTTLE_DEADZONE       0 // in mV (data type: int)

// Trapezoidal Configs
#define PWM_DEFAULT_FREQ   14400
#define PWM_MIN_DUTY       0.0
#define PWM_MAX_DUTY       95.0
#define PWM_DUTY_STEP      0.5              // For pwm sweep testing without throttle input
#define PWM_DEADTIME       30               // *100ns
#define SAMPLE_THROTTLE_AND_UPDATE_DUTY_FREQUENCY   200000  // microseconds
#define BLDC_MCPWM_GROUP   MCPWM_UNIT_0
#define BLDC_MCPWM_TIMER_U MCPWM_TIMER_0
#define BLDC_MCPWM_TIMER_V MCPWM_TIMER_1
#define BLDC_MCPWM_TIMER_W MCPWM_TIMER_2
#define BLDC_MCPWM_GEN_HIGH MCPWM_GEN_A
#define BLDC_MCPWM_GEN_LOW  MCPWM_GEN_B