#include "pad.h"

#define BP32_MAX_GAMEPADS 4

namespace PAD {
GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      GamepadProperties properties = gp->getProperties();
      Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                    gp->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myGamepads[i] = gp;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

// Arduino setup function. Runs in CPU 1
void begin() {
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  
  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();
}

GamepadPtr getGamepad() {
  GamepadPtr myGamepad = myGamepads[0];

  if (myGamepad && myGamepad->isConnected()) return myGamepad;

  return NULL;
}

// Arduino loop function. Runs in CPU 1
void update() {
  // This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepads pointer (the ones received in the callbacks) gets updated
  // automatically.

  BP32.update();

  // // It is safe to always do this before using the gamepad API.
  // // This guarantees that the gamepad is valid and connected.
  // for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    // GamepadPtr myGamepad = myGamepads[0];

    // if (myGamepad && myGamepad->isConnected()) {

  //     // // There are different ways to query whether a button is pressed.
  //     // // By query each button individually:
  //     // //  a(), b(), x(), y(), l1(), etc...
  //     // if (myGamepad->a()) {
  //     //   static int colorIdx = 0;
  //     //   // Some gamepads like DS4 and DualSense support changing the color LED.
  //     //   // It is possible to change it by calling:
  //     //   switch (colorIdx % 3) {
  //     //     case 0:
  //     //       // Red
  //     //       myGamepad->setColorLED(255, 0, 0);
  //     //       break;
  //     //     case 1:
  //     //       // Green
  //     //       myGamepad->setColorLED(0, 255, 0);
  //     //       break;
  //     //     case 2:
  //     //       // Blue
  //     //       myGamepad->setColorLED(0, 0, 255);
  //     //       break;
  //     //   }
  //     //   colorIdx++;
  //     // }

  //     // if (myGamepad->b()) {
  //     //   // Turn on the 4 LED. Each bit represents one LED.
  //     //   static int led = 0;
  //     //   led++;
  //     //   // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
  //     //   // support changing the "Player LEDs": those 4 LEDs that usually
  //     //   // indicate the "gamepad seat". It is possible to change them by
  //     //   // calling:
  //     //   myGamepad->setPlayerLEDs(led & 0x0f);
  //     // }

  //     // if (myGamepad->x()) {
  //     //   // Duration: 255 is ~2 seconds
  //     //   // force: intensity
  //     //   // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
  //     //   // rumble.
  //     //   // It is possible to set it by calling:
  //     //   myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
  //     // }

  //     // Another way to query the buttons, is by calling buttons(), or
  //     // miscButtons() which return a bitmask.
  //     // Some gamepads also have DPAD, axis and more.
      // Serial.printf(
      //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: "
      //     "%4d, %4d, brake: %4d, throttle: %4d, misc: 0x%02x, gyro x:%6d y:%6d "
      //     "z:%6d, accel x:%6d y:%6d z:%6d\n",
      //     0,                         // Gamepad Index
      //     myGamepad->dpad(),         // DPAD
      //     myGamepad->buttons(),      // bitmask of pressed buttons
      //     myGamepad->axisX(),        // (-511 - 512) left X Axis
      //     myGamepad->axisY(),        // (-511 - 512) left Y axis
      //     myGamepad->axisRX(),       // (-511 - 512) right X axis
      //     myGamepad->axisRY(),       // (-511 - 512) right Y axis
      //     myGamepad->brake(),        // (0 - 1023): brake button
      //     myGamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button
      //     myGamepad->miscButtons(),  // bitmak of pressed "misc" buttons
      //     myGamepad->gyroX(),        // Gyro X
      //     myGamepad->gyroY(),        // Gyro Y
      //     myGamepad->gyroZ(),        // Gyro Z
      //     myGamepad->accelX(),       // Accelerometer X
      //     myGamepad->accelY(),       // Accelerometer Y
      //     myGamepad->accelZ()        // Accelerometer Z
      // );

  //     // You can query the axis and other properties as well. See Gamepad.h
  //     // For all the available functions.
  //   } else {
  //     // Stop all movement!
  //   }
  // }

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
  
  // vTaskDelay(1);
  
}
}  // namespace PAD