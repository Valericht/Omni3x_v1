#include <Arduino.h>
#include "config.h"
#include "gripper.h"

static bool gripOn = false;
void gripperBegin() { pinMode(PIN_GRIPPER, OUTPUT); gripperSet(false); }
void gripperSet(bool on) {
  gripOn = on;
  digitalWrite(PIN_GRIPPER, (on == GRIPPER_ACTIVE_HIGH) ? HIGH : LOW);
}
bool gripperOn() { return gripOn; }
