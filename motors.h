#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

#define STEP_A_PIN 18
#define STEP_B_PIN 17

#define STEP_A_DIR 19
#define STEP_B_DIR 23

// #define STEP_A_ENABLE 17
// #define STEP_B_ENABLE 16

#define MIN_STEP_SIZE_US 200 // 50µs min step size

 // Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
#define SERVO_A_PIN 21
#define SERVO_B_PIN 23

namespace MOTORS {
 enum STEPPERS {
  STEP_A,
  STEP_B
};

enum SERVOS {
  SERVO_A,
  SERVO_B
};

  void begin();  
  void update();
  void setStepperSpeed(uint8_t stepper_idx, int spd);
  void setServoPosition(uint8_t servo_idx, int pos);
}  // namespace MOTORS