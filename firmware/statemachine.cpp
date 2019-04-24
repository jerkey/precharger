#include <Arduino.h> // for uint32_t
#include "statemachine.h"

int mode = MODE_OFF;

int get_mode() {
  return mode;
}

void set_mode(int mode_to_set) {
  mode = mode_to_set;
  last_mode_change = millis();
}

void state_machine() {
  if (mode == MODE_OFF) mode_off();
  if (mode == MODE_PRECHARGE) mode_precharge();
  if (mode == MODE_CLOSING) mode_closing();
  if (mode == MODE_ON) mode_on();
}

void mode_off() {
  if (millis() - last_mode_change > 1000) {
    set_mode(MODE_PRECHARGE);
  }
}

void mode_precharge() {
  if (millis() - last_mode_change > 1000) {
    set_mode(MODE_CLOSING);
  }
}

void mode_closing() {
  if (millis() - last_mode_change > 1000) {
    set_mode(MODE_ON);
  }
}

void mode_on() {
  if (millis() - last_mode_change > 1000) {
    set_mode(MODE_OFF);
  }
}
