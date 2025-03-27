#ifndef DRONE_LEDS_CONTROLLER_H
#define DRONE_LEDS_CONTROLLER_H

#include <pico/stdlib.h>

class DroneLedsController
{
public:
  static const int FRONT_LEFT_PIN = 3;
  static const int FRONT_RIGHT_PIN = 25;
  static const int BACK_LEFT_PIN = 11;
  static const int BACK_RIGHT_PIN = 15;
  static const bool ON_VALUE = false; // HIGH=true for common cathode, LOW=true for common anode

  DroneLedsController();

  void setup();

  // Control individual LEDs
  void setFrontLeft(bool on);
  void setFrontRight(bool on);
  void setBackLeft(bool on);
  void setBackRight(bool on);

  // Control groups of LEDs
  void setFront(bool on);
  void setBack(bool on);
  void setLeft(bool on);
  void setRight(bool on);

  void allOn();
  void allOff();
  void toggleAll();

  // Patterns useful for drone status indication
  //void blinkAll(int time_on, int time_off);
  //void rotatePattern(int times);

  // Blink pattern control
  bool updateBlinkPattern(uint32_t time_on, uint32_t time_off);
  bool updateRotatePattern();
  void stopRotatePattern();

  bool enabled;

private:

  // Timer and state tracking for blink pattern
  bool blinkState = false;
  uint32_t blinkStartTime = 0;
  uint32_t blinkOnTime = 0;
  uint32_t blinkOffTime = 0;

  // Timer and state tracking for rotate pattern
  int rotateStep = 0;
  uint32_t rotateStartTime = 0; 
};

// Global instance declaration
extern DroneLedsController droneLeds;

#endif // DRONE_LEDS_CONTROLLER_H