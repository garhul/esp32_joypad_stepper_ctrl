#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

namespace PAD {

void begin();
GamepadPtr getGamepad();
void update();

}  // namespace PAD
