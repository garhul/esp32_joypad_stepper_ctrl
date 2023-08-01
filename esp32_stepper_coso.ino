
#include <Arduino.h>

#include "outputs.h"
#include "pad.h"

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");
  PAD::begin();
  Serial.println("Pad started");
  // OUTPUTS::begin();
  Serial.println("Setup ready");
}

int32_t trim(int32_t data, uint32_t deadZone) {
  if (abs(data) < deadZone) return 0;
  return data;
}

// Arduino loop function. Runs in CPU 1
void loop() {
  static uint32_t joyPoll = 0;
  // OUTPUTS::update();
  PAD::update();
  
  if ((millis() - joyPoll) > 25) {  // keep polling at 40hz (for now)
    joyPoll = millis();
    GamepadPtr pad = PAD::getGamepad();
    if (pad != NULL) {
      OUTPUTS::setBTLed(HIGH);
      // UPDATE AXES    
      OUTPUTS::setStepperSpeed(OUTPUTS::STEPPERS::STEP_A, trim(pad->axisX(), 50));
      OUTPUTS::setStepperSpeed(OUTPUTS::STEPPERS::STEP_B, trim(pad->axisY(), 50));

      OUTPUTS::setServoPosition(OUTPUTS::SERVOS::SERVO_A, trim(pad->axisRX(), 25));
      OUTPUTS::setServoPosition(OUTPUTS::SERVOS::SERVO_B, trim(pad->axisRY(), 25));

      // UPDATE PWMS
      OUTPUTS::setPWMOutput(OUTPUTS::PWMS::PWM_A, pad->brake());
      OUTPUTS::setPWMOutput(OUTPUTS::PWMS::PWM_B, pad->throttle());

      // UPDATE BUTTONS
      OUTPUTS::setDigitalOutputs((pad->buttons() << 4) + pad->dpad());
    } else {
      // Serial.println("No pad connected");
     
      OUTPUTS::setBTLed(HIGH);
      delay(500);
      OUTPUTS::setBTLed(LOW);
      delay(500);
    }
  }  
  vTaskDelay(1);
  // delay_us();
}
