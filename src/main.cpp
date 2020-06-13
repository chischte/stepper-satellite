/*
 * *****************************************************************************
 * STEPPER SATELLITE
 * *****************************************************************************
 * Generates stepper signals for two motors using a Teensy 4.0 
 * *****************************************************************************
 * Michael Wettstein
 * June 2020, Zürich
 * *****************************************************************************
 * 
 * TODO:
 * Update comments, description and runtime measurement for teensy
 * 
 * *****************************************************************************
 * RUNTIME:
 * Measured max runtime: 28us
 * Resulting max rpm: 2678
 * RPM = 75000/runtime
 * 75000 = (10^6 micros*60 seconds / 2 Switches / 2 Microsteps / 200Steps)
 * 
 * RUNTIME COSTS:
 * 12us for a debounce (removed)
 * 10us for a pin monitoring (removed)
 * 6us for a digital read (removed)
 * 5us for a digitalWrite (removed)
 * 5us for a insomnia-delay (partly removed)
 * *****************************************************************************
 * --> SET print_debug_information false WHEN OPERATING <--
 * *****************************************************************************
 */

#include <Arduino.h>
#include <Insomnia.h>
#include <microsomnia.h> // micros() version of insomnia-library
#include <pin_monitor.h> // not in use because of digitalReads (slow)

// GLOBAL VARIABLES ------------------------------------------------------------

// --!-!-!--> DEACTIVATE DEBUG WHEN OPERATING <---!-!-!-!-!-!
bool print_debug_info = false;
//bool print_debug_info = true;

// RUNTIME MEASUREMENT:
bool measure_runtime_enabled;

// SPEED AND TIME SETUP: (ADJUST TO FIT MOTOR SETUP)
const int min_motor_rpm = 100; 
const int max_motor_rpm = 1750; // Motor max = 1750 (specification)
unsigned int acceleration_time = 10200; // [ms] from min to max rpm
const int calculation_resolution = 200; // bigger = smoother

// MOTOR PARAMETERS:
const int micro_step_factor = 2;
const int switches_per_step = 2; // on and off
const int full_steps_per_turn = 200; // 360/1.8°
const int switches_per_turn = full_steps_per_turn * micro_step_factor * switches_per_step;

// VALUES FOR IN LOOP CALCULATIONS:
int time_per_speedlevel;
int rpm_shift_per_speedlevel;
int upper_motor_rpm;
unsigned long upper_motor_microdelay;
unsigned long lower_motor_microdelay;
unsigned long rpm_times_microdelay_constant;
int lower_motor_rpm;

// STATE FLAGS UPPER MOTOR:
bool upper_motor_is_running = false;
bool upper_motor_is_ramping_up = false;
bool upper_motor_is_ramping_down = false;

// STATE FLAGS LOWER MOTOR:
bool lower_motor_is_running = false;
bool lower_motor_is_ramping_up = false;
bool lower_motor_is_ramping_down = false;

// I/O-PINS:
const byte UPPER_MOTOR_INPUT_PIN = 2; // PB2
const byte LOWER_MOTOR_INPUT_PIN = 3; //  PB1

const byte UPPER_MOTOR_STEP_PIN = 9; //PD5
const byte LOWER_MOTOR_STEP_PIN = 10; //PD6

// DELAYS ----------------------------------------------------------------------
Insomnia print_delay;
Insomnia change_values_delay;
Microsomnia upper_motor_switching_delay;
Microsomnia lower_motor_switching_delay;

// FUNCTION DECLARARIONS IF NEEDED FOR THE COMPILER ----------------------------
void stepper_loop();

// FUNCTIONS *******************************************************************

// CALCULATE UPPER MOTOR SPEED -------------------------------------------------
void upper_motor_manage_ramp_up() {
  upper_motor_rpm += rpm_shift_per_speedlevel;

  // REACHED TOPSPEED:
  if (upper_motor_rpm > max_motor_rpm) {
    upper_motor_rpm = max_motor_rpm;
  }
}

void upper_motor_manage_ramp_down() {
  upper_motor_rpm -= rpm_shift_per_speedlevel;

  // REACHED MINIMUM SPEED:
  if (upper_motor_rpm < min_motor_rpm) {
    upper_motor_rpm = min_motor_rpm;
    upper_motor_is_running = false;
  }
}

// CALCULATE LOWER MOTOR SPEED -------------------------------------------------
void lower_motor_manage_ramp_up() {

  lower_motor_rpm += rpm_shift_per_speedlevel;

  // REACHED TOPSPEED:
  if (lower_motor_rpm > max_motor_rpm) {
    lower_motor_rpm = max_motor_rpm;
  }
}

void lower_motor_manage_ramp_down() {

  lower_motor_rpm -= rpm_shift_per_speedlevel;

  // REACHED MINIMUM SPEED:
  if (lower_motor_rpm > min_motor_rpm) {
    lower_motor_rpm = min_motor_rpm;
    lower_motor_is_running = false;
  }
}

// INITIAL CALCULATIONS --------------------------------------------------------
void make_initial_calculations() {

  Serial.println("INITIAL CALCULATIONS:");

  float float_time_per_speedlevel = float(acceleration_time) / (calculation_resolution - 1);
  time_per_speedlevel = int(float_time_per_speedlevel);
  Serial.print("TIME PER SPEEDLEVEL [ms]: ");
  Serial.println(float_time_per_speedlevel,1);

  float float_rpm_shift_per_speedlevel = (max_motor_rpm - min_motor_rpm) / calculation_resolution;
  rpm_shift_per_speedlevel = int(float_rpm_shift_per_speedlevel);
  Serial.print("RPM SHIFT PER SPEEDLEVEL:");
  Serial.println(float_rpm_shift_per_speedlevel,1);

  rpm_times_microdelay_constant = pow(10, 6) * 60 / switches_per_turn;
  Serial.print("RPM TIMES MICRODELAY CONSTANT: ");
  Serial.println(rpm_times_microdelay_constant);
}

unsigned long calculate_microdelay_using_float(int rpm) {
  float float_microdelay = float(rpm_times_microdelay_constant) / rpm;
  unsigned long microdelay = round(float_microdelay);
  return microdelay;
}

// MONITOR PINS ----------------------------------------------------------------
void monitor_input_pin_upper_motor() {

  if (digitalRead(UPPER_MOTOR_INPUT_PIN)) {
    upper_motor_is_ramping_up = true;
    upper_motor_is_ramping_down = false;
    upper_motor_is_running = true;
  } else {
    upper_motor_is_ramping_up = false;
    upper_motor_is_ramping_down = true;
  }
  if (print_debug_info) {
    upper_motor_is_ramping_up = true;
    upper_motor_is_ramping_down = false;
    upper_motor_is_running = true;
  }
}

void monitor_input_pin_lower_motor() {

  if (digitalRead(LOWER_MOTOR_INPUT_PIN)) {
    lower_motor_is_ramping_up = true;
    lower_motor_is_ramping_down = false;
    lower_motor_is_running = true;
  } else {
    lower_motor_is_ramping_up = false;
    lower_motor_is_ramping_down = true;
  }

  if (measure_runtime_enabled) {
    upper_motor_is_ramping_up = true;
    upper_motor_is_running = true;
    lower_motor_is_ramping_up = true;
    lower_motor_is_running = true;
  }
}

// OTHER FUNCTIONS--------------------------------------------------------------
void measure_runtime() {
  measure_runtime_enabled = true; // simulates running motors
  long number_of_cycles = 100000;
  unsigned long time_elapsed = 0;
  unsigned long time_before_loop = 0;
  unsigned long max_runtime = 0;
  unsigned long time_for_all_loops = 0;

  for (long i = number_of_cycles; i > 0; i--) {

    time_before_loop = micros();
    stepper_loop();
    time_elapsed = micros() - time_before_loop;

    time_for_all_loops = time_for_all_loops + time_elapsed;

    if (time_elapsed > max_runtime) {
      max_runtime = time_elapsed;
    }
  }
  unsigned long avg_runtime = time_for_all_loops / number_of_cycles;

  Serial.print("TOTAL RUNTIME [ms]: ");
  Serial.println(time_for_all_loops / 1000);

  Serial.print("AVG RUNTIME [us]: ");
  Serial.println(avg_runtime);

  Serial.print("MAX RUNTIME [us]: ");
  Serial.println(max_runtime);
  measure_runtime_enabled = false;
}

void update_upper_motorspeed() {

  if (upper_motor_is_ramping_up) {
    upper_motor_manage_ramp_up();
  }
  if (upper_motor_is_ramping_down) {
    upper_motor_manage_ramp_down();
  }
  upper_motor_microdelay = calculate_microdelay_using_float(upper_motor_rpm);
}

void update_lower_motorspeed() {
  if (lower_motor_is_ramping_up) {
    lower_motor_manage_ramp_up();
  }
  if (lower_motor_is_ramping_down) {
    lower_motor_manage_ramp_down();
  }
  lower_motor_microdelay = calculate_microdelay_using_float(lower_motor_rpm);
}

void switch_outputs() {
  if (upper_motor_is_running) {
    if (upper_motor_switching_delay.delay_time_is_up(upper_motor_microdelay)) {
      digitalWrite(UPPER_MOTOR_STEP_PIN, !digitalRead(UPPER_MOTOR_STEP_PIN)); // NANO PIN 10
    }
  }

  if (lower_motor_is_running) {
    if (lower_motor_switching_delay.delay_time_is_up(lower_motor_microdelay)) {
      digitalWrite(LOWER_MOTOR_STEP_PIN, !digitalRead(LOWER_MOTOR_STEP_PIN));
    }
  }
}

void stepper_loop() {

  monitor_input_pin_upper_motor();
  monitor_input_pin_lower_motor();

  if (change_values_delay.delay_time_is_up(time_per_speedlevel)) {
    update_upper_motorspeed();
    update_lower_motorspeed();
  }

  switch_outputs();
}

// SETUP ***********************************************************************
void setup() {

  Serial.begin(115200);
  while (!Serial && millis() <= 3000) {
    // wait for serial connection
  }

  measure_runtime();

  make_initial_calculations();

  pinMode(UPPER_MOTOR_STEP_PIN, OUTPUT);
  pinMode(LOWER_MOTOR_STEP_PIN, OUTPUT);

  // SET INITIAL SPEED:
  upper_motor_rpm = min_motor_rpm;
  lower_motor_rpm = min_motor_rpm;

  Serial.println("EXIT SETUP");
}

// LOOP ************************************************************************
void loop() {

  stepper_loop(); // separated for runtime measurement

  if (print_debug_info) {
    if (print_delay.delay_time_is_up(800)) {

      Serial.print("  MOTOR RUNNING: ");
      Serial.print(upper_motor_is_running);

      Serial.print("  RPM: ");
      Serial.print(upper_motor_rpm);

      Serial.print("  DELAY: ");
      Serial.println(upper_motor_microdelay);
    }
  }
}
// ********************************************************** END OF PROGRAM ***