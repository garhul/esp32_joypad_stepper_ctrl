#include "outputs.h"

namespace OUTPUTS {
const uint8_t stepper_step_pins[] = {STEP_A_PIN, STEP_B_PIN};
const uint8_t stepper_dir_pins[] = {STEP_A_DIR, STEP_B_DIR};
const uint8_t pwm_outputs[] = {PWM_A_PIN, PWM_B_PIN};

struct motorCtrl {
  uint8_t A_MS : 3;
  uint8_t A_EN : 1;
  uint8_t B_MS : 3;
  uint8_t B_EN : 1;

  uint8_t get() const {
    return (A_MS << 5) + (A_EN << 4) + (B_MS << 1) + B_EN;
  }
};

struct {
  uint8_t first_reg;
  motorCtrl second_reg;

  uint16_t get() const {
    return (second_reg.get() << 8) + first_reg;
  }

} register_state;

bool stepper_dir[] = {0, 0};
bool stepper_dir_flag[] = {0, 0};
int stepper_speed[] = {0, 0};
uint16_t stepper_target_step_size[] = {0, 0};

uint32_t step_size[] = {0, 0};
bool servo_pos[] = {0, 0};
uint16_t min_step_sizes[] = {MIN_STEP_SIZE_US, MIN_STEP_SIZE_US};

Servo servos[2];

void begin() {
  Serial.println("init");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servos[0].setPeriodHertz(50);
  servos[0].attach(SERVO_A_PIN, 800, 2200);

  servos[1].setPeriodHertz(50);
  servos[1].attach(SERVO_B_PIN, 800, 2200);

  pinMode(stepper_step_pins[0], OUTPUT);
  pinMode(stepper_step_pins[1], OUTPUT);

  pinMode(stepper_dir_pins[0], OUTPUT);
  pinMode(stepper_dir_pins[1], OUTPUT);

  pinMode(BT_LED, OUTPUT);
  pinMode(PWM_A_PIN, OUTPUT);
  pinMode(PWM_B_PIN, OUTPUT);

  pinMode(SREG_DATA, OUTPUT);
  pinMode(SREG_LATCH, OUTPUT);
  pinMode(SREG_CLK, OUTPUT);

  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
}

inline void sendStepPulse(uint8_t mtr) {
  static uint64_t __last_step[] = {0, 0};

  if (step_size[mtr] != 0 && (micros() - __last_step[mtr]) > step_size[mtr]) {
    // change pulse
    digitalWrite(stepper_step_pins[mtr], !digitalRead(stepper_step_pins[mtr]));
    // set direction
    digitalWrite(stepper_dir_pins[mtr], stepper_dir[mtr]);
    __last_step[mtr] = micros();
  }
}

void update() {
  // update steppers
  uint32_t last_step_update = millis();

  if ((millis() - last_step_update) > 25) {
    updateStepSizes(STEPPERS::STEP_A);
    updateStepSizes(STEPPERS::STEP_B);
    last_step_update = millis();
  }

  sendStepPulse(STEPPERS::STEP_A);
  sendStepPulse(STEPPERS::STEP_B);
}

void updateStepSizes(uint8_t stepper) {
  // simple ramping of speeds

  if (stepper_target_step_size[stepper] == 0) {
    step_size[stepper] = 0;
  }

  if (step_size[stepper] < stepper_target_step_size[stepper]) {
    step_size[stepper] = step_size[stepper] * 1.02;
  } else if (step_size[stepper] > stepper_target_step_size[stepper]) {
    step_size[stepper] = step_size[stepper] * .98;
  }
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
    stepper_target_step_size[stepper] = (512 / abs(speed)) * min_step_sizes[stepper];
  }
}

// TODO:: implement stepper direction modifier led
void setStepperDirection(uint8_t stepper, bool direction) {
  stepper_dir_flag[stepper] = direction;
}

// TODO:: implement microstepping control via 74hc595 or similar
void setMicroStepping(uint8_t channel, uint8_t mode) {
  // Send output to second shift regitser
  uint8_t word = 0xFF;

  switch (mode) {
    case STEPPING::FULL_STEP:  // 000
      if (channel == STEPPERS::STEP_A) {
        register_state.second_reg.A_MS = 0;
      } else {
        register_state.second_reg.B_MS = 0;
      }
      break;

    case STEPPING::HALF_STEP:  // 000
      if (channel == STEPPERS::STEP_A) {
        register_state.second_reg.A_MS = 0b100;
      } else {
        register_state.second_reg.B_MS = 0b100;
      }
      break;

    case STEPPING::QUARTER_STEP:
      if (channel == STEPPERS::STEP_A) {
        register_state.second_reg.A_MS = 0b010;
      } else {
        register_state.second_reg.B_MS = 0b010;
      }
      break;

    case STEPPING::EIGHTH_STEP:
      if (channel == STEPPERS::STEP_A) {
        register_state.second_reg.A_MS = 0b110;
      } else {
        register_state.second_reg.B_MS = 0b110;
      }
      break;

    case STEPPING::SIXTEENTH_STEP:
      if (channel == STEPPERS::STEP_A) {
        register_state.second_reg.A_MS = 0b111;
      } else {
        register_state.second_reg.B_MS = 0b111;
      }
      break;

    default:
      Serial.println("INVALID STEPPING MODE");
  }

  updateRegister();
}

void updateRegister() {
  // send register state
  shiftOut(SREG_DATA,SREG_CLK, MSBFIRST, register_state.second_reg.get());
  shiftOut(SREG_DATA,SREG_CLK, MSBFIRST, register_state.first_reg);
  
  digitalWrite(SREG_LATCH, HIGH);
  delayMicroseconds(100);
  digitalWrite(SREG_LATCH, LOW);
}

void setServoPosition(uint8_t servo, int position) {
  int p = map(position, -512, 512, 0, 180);
  // Serial.printf("servo %d set to pos %d \n", servo, p);
  // TODO:: debounce servo setting position
  // TODO:: make servo range configurable
  servos[servo].write(p);
}

void setDigitalOutput(uint8_t pin, bool level) {
  register_state.first_reg ^= (-level ^ register_state.first_reg) & (1UL << pin);
  Serial.print("Register state: ");
  Serial.println(register_state.get(), BIN);
  updateRegister();
}

void setDigitalOutputs(uint8_t val) {
  register_state.first_reg = val;
  // Serial.print("Register state changed via setDigitalOutputs: ");
  // Serial.println(register_state.get(), BIN);
  updateRegister();
}

void setPWMOutput(uint8_t chan, uint16_t val) {
  analogWrite(pwm_outputs[chan], val);
};

void setMinStepSize(uint32_t s);

void setBTLed(bool state) {
  digitalWrite(BT_LED, state);
}

}  // namespace OUTPUTS
