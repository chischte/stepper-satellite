/*
 * *****************************************************************************
 * pin_monitor.cpp
 * *****************************************************************************
 */

#include "Arduino.h"
#include "pin_monitor.h"

Pin_monitor::Pin_monitor(const byte BUTTON_PIN) {
  _BUTTON_PIN = BUTTON_PIN;
}
//***************************************************************************
//LIBRARY FUNCTIONS:
//***************************************************************************

void Pin_monitor::setDebounceTime(int debounceTime) {
  _debounceTime = debounceTime;
}

bool Pin_monitor::requestButtonState() {

  _currentButtonState = digitalRead(_BUTTON_PIN);

  // DETECT IF THE BUTTON STATE HAS CHANGED:
  if (_currentButtonState != _debouncedButtonState && _debounceTimerSet == false) {
    // IN THE FIRST RUN,SET THE DEBOUNCE TIMER:
    _prevTime = millis();
    _debounceTimerSet = true;
  }
  // IF THE DEBOUNCE TIME'S UP AND THE SWITCH HAS STILL THE SAME (CHANGED) BUTTON STATE,
  // THEN THE CHANGED BUTTON STATE IS VALID:
  if (millis() - _prevTime > _debounceTime) {
    _debounceTimerSet = false;
    if (_currentButtonState != _debouncedButtonState) {
      _debouncedButtonState = _currentButtonState;

      // IF THE VALID NEW BUTTON STATE IS HIGH, THEN A SWITCH TO HIGH HAS HAPPEND:
      if (_debouncedButtonState == HIGH) {
        _buttonSwitchedHigh = true;
        _buttonSwitchedLow = false;
      }

      // IF THE VALID NEW BUTTON STATE IS LOW; THEN A SWICH TO LOW HAS HAPPEND:
      if (_debouncedButtonState == LOW) {
        _buttonSwitchedLow = true;
        _buttonSwitchedHigh = false;
      }

    }
  }
  return _currentButtonState;
}

bool Pin_monitor::switchedHigh() {
  Pin_monitor::requestButtonState();

  //RETURN THE INFORMATION IF THE BUTTON HAS SWITCHED HIGH, AND RESET IT:
  bool switchedHigh = _buttonSwitchedHigh;
  _buttonSwitchedHigh = false;
  return switchedHigh;
}

bool Pin_monitor::switchedLow() {
  Pin_monitor::requestButtonState();

  //RETURN THE INFORMATION IF THE BUTTON HAS SWITCHED LOW, AND RESET IT:
  bool switchedLow = _buttonSwitchedLow;
  _buttonSwitchedLow = false;
  return switchedLow;
}
