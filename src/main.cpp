/*
 * *****************************************************************************
 * STEPPER SATELLITE
 * *****************************************************************************
 * Generates stepper signals for two motors using an Arduino 
 * *****************************************************************************
 * Michael Wettstein
 * June 2020, Zürich
 * *****************************************************************************
 * OPTIMIZATION POTENTIAL:
 * Use a cycle counter instead of a switch delay to avoid stuttering when
 * motor is running fast and runtime does not fit to delay time.
 * Limit minimum delay to exactly a multiple of runtime (probably min. 2 times).
 * If necessary make runtime more constant or even a bit longer when finetuning
 * to reach max motor speed.
 * Analyze and optimze acceleration curve, e.g. decrease time per speedlevel
 * at the beginning, when acceleration is slower.
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
int speedlevel_cycle_counter = 0;

// --!-!-!--> DEACTIVATE DEBUG WHEN OPERATING <---!-!-!-!-!-!
//bool print_debug_info = false;
bool print_debug_info = true;

// RUNTIME MEASUREMENT:
int avg_runtime_us = 0;
bool measure_runtime_enabled;

// SPEED AND TIME SETUP: (ADJUST TO FIT MOTOR SETUP)
const int min_motor_rpm = 100; // min = 10 (calculation algorithm)
const int max_motor_rpm = 1300; // Motor max = 1750 (specification)
unsigned int acceleration_time = 200; // [ms] from min to max rpm

// MOTOR PARAMETERS:
const int micro_step_factor = 2;
const int switches_per_step = 2; // on and off
const int calculation_resolution = 200; // bigger = smoother
const int full_steps_per_turn = 200; // 360/1.8°

// VALUES FOR IN LOOP CALCULATIONS:
int int_time_per_speedlevel;
unsigned long long_cycles_per_speedlevel;
int startspeed_microdelay;
int topspeed_microdelay;
int microdelay_difference_per_speedlevel;
int upper_motor_microdelay;
int lower_motor_microdelay;

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
float calculate_microdelay(float rpm);
void stepper_loop();

// FUNCTIONS *******************************************************************

// CALCULATE UPPER MOTOR SPEED -------------------------------------------------
void upper_motor_manage_ramp_up() {
  upper_motor_microdelay -= microdelay_difference_per_speedlevel;

  // REACHED TOPSPEED:
  if (upper_motor_microdelay < topspeed_microdelay) {
    upper_motor_microdelay = topspeed_microdelay;
  }
}

void upper_motor_manage_ramp_down() {
  upper_motor_microdelay += microdelay_difference_per_speedlevel;

  // REACHED MINIMUM SPEED:
  if (upper_motor_microdelay > startspeed_microdelay) {
    upper_motor_microdelay = startspeed_microdelay;
    upper_motor_is_running = false;
  }
}

// CALCULATE LOWER MOTOR SPEED -------------------------------------------------
void lower_motor_manage_ramp_up() {

  lower_motor_microdelay -= microdelay_difference_per_speedlevel;

  // REACHED TOPSPEED:
  if (lower_motor_microdelay < topspeed_microdelay) {
    lower_motor_microdelay = topspeed_microdelay;
  }
}

void lower_motor_manage_ramp_down() {

  lower_motor_microdelay += microdelay_difference_per_speedlevel;

  // REACHED MINIMUM SPEED:
  if (lower_motor_microdelay > startspeed_microdelay) {
    lower_motor_microdelay = startspeed_microdelay;
    lower_motor_is_running = false;
  }
}

// INITIAL CALCULATIONS --------------------------------------------------------
void make_initial_calculations() {

  Serial.println("INITIAL CALCULATIONS:");

  float float_time_per_speedlevel = float(acceleration_time) / (calculation_resolution - 1);
  int_time_per_speedlevel = int(float_time_per_speedlevel);
  Serial.print("TIME PER SPEEDLEVEL [ms]: ");
  Serial.println(float_time_per_speedlevel);

  float cycles_per_speedlevel = (float_time_per_speedlevel * 1000) / avg_runtime_us;
  long_cycles_per_speedlevel = int(cycles_per_speedlevel);
  Serial.print("CYCLES PER SPEEDLEVEL: ");
  Serial.println(long_cycles_per_speedlevel);

  startspeed_microdelay = calculate_microdelay(min_motor_rpm);
  Serial.print("INITIAL MICRO-DELAY: ");
  Serial.println(startspeed_microdelay);

  topspeed_microdelay = calculate_microdelay(max_motor_rpm);
  Serial.print("TOPSPEED MICRO-DELAY:");
  Serial.println(topspeed_microdelay);

  float delay_difference = startspeed_microdelay - topspeed_microdelay;
  float float_delay_difference_per_speedlevel = delay_difference / calculation_resolution;
  microdelay_difference_per_speedlevel = int(float_delay_difference_per_speedlevel);
}

float calculate_microdelay(float rpm) {

  float turns_per_second = rpm / 60;
  float switches_per_turn = full_steps_per_turn * micro_step_factor * switches_per_step;
  float steps_per_second = turns_per_second * switches_per_turn;
  float float_microdelay = 1000000 / steps_per_second;
  return float_microdelay;
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
int measure_runtime() {
  measure_runtime_enabled = true; // simulates running motors
  long number_of_cycles = 10000;
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
  return avg_runtime;
}

void update_motor_frequencies() {
  //if (speedlevel_cycle_counter == long_cycles_per_speedlevel) {
  if (change_values_delay.delay_time_is_up(100)) {
    speedlevel_cycle_counter = 0;
    // UPPER MOTOR:
    if (upper_motor_is_ramping_up) {
      upper_motor_manage_ramp_up();
    }
    if (upper_motor_is_ramping_down) {
      upper_motor_manage_ramp_down();
    }
    // LOWER MOTOR:
    if (lower_motor_is_ramping_up) {
      lower_motor_manage_ramp_up();
    }
    if (lower_motor_is_ramping_down) {
      lower_motor_manage_ramp_down();
    }
  }
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

  speedlevel_cycle_counter++;
  monitor_input_pin_upper_motor();
  monitor_input_pin_lower_motor();
  update_motor_frequencies();
  switch_outputs();
}

// SETUP ***********************************************************************
void setup() {

  Serial.begin(115200);
  while (!Serial) {
  }
  avg_runtime_us = measure_runtime();

  make_initial_calculations();

  pinMode(UPPER_MOTOR_STEP_PIN, OUTPUT);
  pinMode(LOWER_MOTOR_STEP_PIN, OUTPUT);

  // SET INITIAL SPEED:
  upper_motor_microdelay = startspeed_microdelay;
  lower_motor_microdelay = startspeed_microdelay;

  Serial.println("EXIT SETUP");
}

// LOOP ************************************************************************
void loop() {

  stepper_loop(); // separated for runtime measurement

  if (print_debug_info) {
    if (print_delay.delay_time_is_up(1000)) {

      Serial.print("  MOTOR RUNNING: ");
      Serial.print(upper_motor_is_running);

      Serial.print("  DELAY: ");
      Serial.println(upper_motor_microdelay);
    }
  }
}
// ********************************************************** END OF PROGRAM ***