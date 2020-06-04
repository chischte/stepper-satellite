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
  bool request_button_state();
  bool switched_high();
  bool switched_low();

  // VARIABLES:
  // n.a.

private:
  // FUNCTIONS:
  // n.a.

  // VARIABLES:
  byte _BUTTON_PIN;
  bool _button_switched_high = false;
  bool _button_switched_low = false;
  bool _previous_button_state = 0;
  bool _current_button_state = 0;

};

#endif
