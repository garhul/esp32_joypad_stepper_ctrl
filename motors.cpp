#include "esp32-hal-gpio.h"
#include "motors.h"

namespace MOTORS {
bool stepper_dir[] = {0, 0};
uint8_t stepper_step_pins[] = {STEP_A_PIN, STEP_B_PIN};
uint8_t stepper_dir_pins[] = {STEP_A_DIR, STEP_B_DIR};

bool stepper_dir_flag[] = {0,0};
uint32_t step_size[] = {0, 0};
bool servo_pos[] = {0, 0};
Servo servos[2];

void begin() {
  Serial.println("init");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servos[0].setPeriodHertz(50); 
  servos[0].attach(SERVO_A_PIN, 800, 2200);

  servos[1].setPeriodHertz(50); 
  servos[1].attach(SERVO_B_PIN, 800, 2200);

  pinMode(stepper_step_pins[0],OUTPUT);
  pinMode(stepper_step_pins[1],OUTPUT);

  pinMode(stepper_dir_pins[0],OUTPUT);
  pinMode(stepper_dir_pins[1],OUTPUT);

  // using default min/max of 1000us and 2000us  
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}

inline void sendStepPulse(uint8_t mtr) {
  static uint64_t __last_step[] = {0, 0};

  if (step_size[mtr] != 0 && (micros() - __last_step[mtr]) > step_size[mtr]) {
    //change pulse
    digitalWrite(stepper_step_pins[mtr], !digitalRead(stepper_step_pins[mtr]));    
    //set direction
    digitalWrite(stepper_dir_pins[mtr], stepper_dir[mtr]);
    __last_step[mtr] = micros();   
  }
}


void update() {  
  sendStepPulse(STEPPERS::STEP_A);
  sendStepPulse(STEPPERS::STEP_B);
}

void setServoPosition(uint8_t servo, int position) {
  int p = map(position, -512, 512, 0, 180);
  // Serial.printf("servo %d set to pos %d \n", servo, p);  
  //TODO:: debounce servo setting position 
  //TODO:: make servo range configurable
  servos[servo].write(p);
}

/**
  @param uint8_t stepper == Stepper index
  @param int speed  == speed, (-1024 to 1024) direction is determined by int > 0

*/
void setStepperSpeed(uint8_t stepper, int speed) {
  if (speed == 0) {
    step_size[stepper] = 0;
  } else {
    stepper_dir[stepper] = (speed > 0) ^ stepper_dir_flag[stepper];  
    step_size[stepper] = (512 / abs(speed)) * MIN_STEP_SIZE_US;    
  }  
}

// TODO:: implement stepper direction modifier led
void setStepperDirection(uint8_t stepper, bool direction) {
  stepper_dir_flag[stepper] = direction;
}

// TODO:: implement microstepping control via 74hc595 or similar
void setMicroStepping(uint8_t cfg) {}

}  // namespace MOTORS
