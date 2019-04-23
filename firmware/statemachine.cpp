#include <Arduino.h> // for uint32_t
#include "statemachine.h"

void mode_off() {
  if (millis() - last_mode_change > 1000) {
    mode++;
    mode = MODE_PRECHARGE;
    last_mode_change = millis();
  }
}

void mode_precharge() {
  if (millis() - last_mode_change > 1000) {
    mode++;
    mode = MODE_CLOSING;
    last_mode_change = millis();
  }
}

void mode_closing() {
  if (millis() - last_mode_change > 1000) {
    mode++;
    mode = MODE_ON;
    last_mode_change = millis();
  }
}

void mode_on() {
  if (millis() - last_mode_change > 1000) {
    mode++;
    mode = MODE_OFF;
    last_mode_change = millis();
  }
}
