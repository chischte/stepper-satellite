/*
 * *****************************************************************************
 * pin_monitor.h
 * *****************************************************************************
 */

#ifndef PIN_MONITOR_H
#define PIN_MONITOR_H

#include "Arduino.h"

class Pin_monitor {
public:
  // FUNCTIONS:
  Pin_monitor(const byte BUTTON_PIN);
  bool requestButtonState();
  bool switchedHigh();
  bool switchedLow();
  void setDebounceTime(int debounce_time);

  // VARIABLES:
  // n.a.

private:
  // FUNCTIONS:
  // n.a.

  // VARIABLES:
  byte _BUTTON_PIN;
  bool _buttonSwitchedHigh = false;
  bool _buttonSwitchedLow = false;
  bool _debounceTimerSet = false;
  bool _debouncedButtonState = 0;
  bool _currentButtonState = 0;
  unsigned int _debounceTime = 10;
  unsigned long _prevTime = 0;

};

#endif
