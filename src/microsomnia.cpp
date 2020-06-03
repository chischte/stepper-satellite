/*
 * *****************************************************************************
 * microsomnia.cpp
 * *****************************************************************************
 */

#include "microsomnia.h"
#include "Arduino.h"

Microsomnia::Microsomnia(unsigned long timeoutTime /*= 5000*/) {
  _timeout_time = timeoutTime;
  _previous_time = micros();
}

void Microsomnia::set_time(unsigned long timeoutTime) {
  _timeout_time = timeoutTime;
  _previous_time = micros();
}

void Microsomnia::reset_time() { _previous_time = micros(); }

void Microsomnia::set_flag_activated(bool setActive) { _timeout_is_marked_activated = setActive; }

bool Microsomnia::is_marked_activated() // returns true if timeout is active
{
  return _timeout_is_marked_activated;
}

bool Microsomnia::has_timed_out() // returns true if timeout time has been reached
{
  bool timeoutTimedOut;
  if (micros() - _previous_time > _timeout_time) {
    timeoutTimedOut = true;
  } else {
    timeoutTimedOut = false;
  }
  return timeoutTimedOut;
}

bool Microsomnia::delay_time_is_up(unsigned long delayTime) {
  _delay_time = delayTime;
  if (!_delay_is_active) {
    _previous_time = micros();
    _delay_is_active = true;
  } else if (micros() - _previous_time > _delay_time) {
    _delay_is_active = false;
    return (1);
  }
  return (0);
}

unsigned long Microsomnia::get_remaining_delay_time() {
  unsigned long timePassed = micros() - _previous_time;
  unsigned long timeRemaining;
  if (_delay_time > timePassed) {
    timeRemaining = _delay_time - timePassed;
  } else {
    timeRemaining = 0;
  }
  return timeRemaining;
}

unsigned long Microsomnia::get_remaining_timeout_time() {
  unsigned long timePassed = micros() - _previous_time;
  unsigned long timeRemaining;
  if (_timeout_time > timePassed) {
    timeRemaining = _timeout_time - timePassed;
  } else {
    timeRemaining = 0;
  }
  return timeRemaining;
}
