## Todo:
- Get throttle input analog deadzone and update macro
- Trap:  (1) Motor initialization to check for correct hall readings, (2) get open-loop function for speed control
- SVPWM: (1) Apply centre-aligned PWM, (2) get RPM from encoder readings
- Revamp Task 5 as it's not accurate and requires low pass

## Test
- !ADC2 reading accuracy with WiFi task on
- (Not important) Check ESP_ERROR_CHECK in T1_TrapCommutation.cpp

## Improvements
- Update PWM duty cycle less frequent? Integration of duty update with control loop
- Some processes could be improved through the use of indexed notification, particularly on T1_TrapCommutation, T5_MotorVelocityCalculation, & T6_MissedCommutationCounter:
    - For T1_TrapCommutation, throttle input is currently being received via semaphore mutex, which is slightly longer than sending 4 bytes through notification. Notification can't be used because it is already in use by the hall sensor ISR.
    - For T5_MotorVelocityCalculation and T6_MissedCommutationCounter, it is possible to combine both the tasks and send both the values to WiFi.
Unfortunately, the Arduino framework doesn't enable indexed notification. Changing to ESP-IDF could solve these.