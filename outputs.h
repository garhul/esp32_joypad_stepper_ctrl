#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

#define BT_LED 32

#define PWM_A_PIN 22
#define PWM_B_PIN 21

#define STEP_A_PIN 25
#define STEP_B_PIN 26
#define STEP_A_DIR 27
#define STEP_B_DIR 14

#define SERVO_A_PIN 12
#define SERVO_B_PIN 13

#define SREG_DATA 23
#define SREG_CLK 19
#define SREG_LATCH 18


#define MIN_STEP_SIZE_US 200  // 50µs min step size

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
#define SERVO_A_PIN 21
#define SERVO_B_PIN 23

namespace OUTPUTS {

enum STEPPERS {
  STEP_A,
  STEP_B
};

enum PWMS {
  PWM_A,
  PWM_B
};

enum SERVOS {
  SERVO_A,
  SERVO_B
};

enum STEPPING {
  FULL_STEP,
  HALF_STEP,
  QUARTER_STEP,
  EIGHTH_STEP,
  SIXTEENTH_STEP
};

void begin();
void update();
void setBTLed(bool state);
void updateStepSizes(uint8_t stepper_idx);
void updateRegister();
void setMicroStepping(uint8_t mode);
void setStepperSpeed(uint8_t stepper_idx, int spd);
void setServoPosition(uint8_t servo_idx, int pos);
void setDigitalOutput(uint8_t pin, bool level);
void setDigitalOutputs(uint8_t val);
void setPWMOutput(uint8_t chan, uint16_t val);

void setMinStepSize(uint32_t s);

// TODO:: implement methods to set pwm frequency, servo frequency, servo min and servo max and stepper accel factor

}  // namespace OUTPUTS