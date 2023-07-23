
#include <Arduino.h>

#include "motors.h"
#include "pad.h"


// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  PAD::begin();
  MOTORS::begin();
}

int32_t trim(int32_t data, uint32_t deadZone) {
  if (abs(data) < deadZone) return 0;   
  return data;
}

// Arduino loop function. Runs in CPU 1
void loop() {
  static uint32_t joyPoll = 0;
  MOTORS::update();
  
  if ((millis() - joyPoll) > 25) { // keep polling at 40hz (for now)
    joyPoll = millis();

    PAD::update();
    GamepadPtr pad = PAD::getGamepad();
    if (pad != NULL) {

      MOTORS::setStepperSpeed(MOTORS::STEPPERS::STEP_A, trim(pad->axisX(), 50));
      MOTORS::setStepperSpeed(MOTORS::STEPPERS::STEP_B, trim(pad->axisY(), 50));
      
      MOTORS::setServoPosition(MOTORS::SERVOS::SERVO_A, trim(pad->axisRX(), 25));
      MOTORS::setServoPosition(MOTORS::SERVOS::SERVO_B, trim(pad->axisRY(), 25));
    
    
    } else {
      Serial.println("No pad connected");
      delay(2000);
    }
  }
  // MOTORS::setStepperSpeed(MOTORS::STEPPERS::STEP_A, 23);
  // vTaskDelay(1);
  // delay_us();
}
