/*
 * *****************************************************************************
 * pin_monitor.cpp
 * *****************************************************************************
 */

#include "pin_monitor.h"
#include "Arduino.h"

Pin_monitor::Pin_monitor(const byte BUTTON_PIN) { _BUTTON_PIN = BUTTON_PIN; }
//***************************************************************************
//LIBRARY FUNCTIONS:
//***************************************************************************

bool Pin_monitor::request_button_state() {

  _current_button_state = digitalRead(_BUTTON_PIN);

  if (_current_button_state == !_previous_button_state) {

    // A SWITCH TO HIGH HAS HAPPEND:
    if (_current_button_state == HIGH) {
      _button_switched_high = true;
      _button_switched_low = false;
    }

    // A SWICH TO LOW HAS HAPPEND:
    if (_current_button_state == LOW) {
      _button_switched_low = true;
      _button_switched_high = false;
    }

    _previous_button_state = _current_button_state;
  }
  return _current_button_state;
}

bool Pin_monitor::switched_high() {
  Pin_monitor::request_button_state();

  bool switchedHigh = _button_switched_high;
  _button_switched_high = false;
  return switchedHigh;
}

bool Pin_monitor::switched_low() {
  Pin_monitor::request_button_state();

  bool switchedLow = _button_switched_low;
  _button_switched_low = false;
  return switchedLow;
}
