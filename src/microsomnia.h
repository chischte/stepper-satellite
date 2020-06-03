/*
 * *****************************************************************************
 * microsomnia.h
 * *****************************************************************************
 */

#ifndef MICROSOMNIA_H
#define MICROSOMNIA_H

#include "Arduino.h"

class Microsomnia {
public:
  // FUNTIONS:
  Microsomnia(unsigned long timeoutTime = 5000);
  void reset_time();
  void set_flag_activated(bool setActive);
  bool is_marked_activated(); // returns true if timeout is active
  bool has_timed_out(); // returns true if timeout time has been reached
  void set_time(unsigned long);
  bool delay_time_is_up(unsigned long delayTime);
  unsigned long get_remaining_delay_time();
  unsigned long get_remaining_timeout_time();

  // VARIABLES:
  // n.a.

private:
  // FUNCTIONS:
  // n.a.

  // VARIABLES:
  unsigned long _previous_time = 0;
  unsigned long _timeout_time = 0;
  unsigned long _delay_time = 0;
  bool _timeout_is_marked_activated = false;
  bool _delay_is_active = false;
};
#endif
