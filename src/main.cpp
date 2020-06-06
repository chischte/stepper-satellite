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
 * Analize acceleration curve graphically (spread sheet) and find optimization
 * potential, e.g. decrease time per speedlevel at the beginning, when
 * acceleration is slower.
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
bool print_debug_info = false;
//bool print_debug_info = true;

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
int int_cycles_per_speedlevel;
int startspeed_step_counter_delay;
int topspeed_step_counter_delay;
int microdelay_difference_per_speedlevel;
int upper_motor_step_counter_delay;
int lower_motor_step_counter_delay;

int upper_motor_counter = 0;
int lower_motor_counter = 0;

// STATE FLAGS UPPER MOTOR:
bool upper_motor_is_running = false;
bool upper_motor_is_ramping_up = false;
bool upper_motor_is_ramping_down = false;

// STATE FLAGS LOWER MOTOR:
bool lower_motor_is_running = false;
bool lower_motor_is_ramping_up = false;
bool lower_motor_is_ramping_down = false;

// I/O-PINS:
const byte UPPER_MOTOR_INPUT_PIN = 10; // PB2
const byte LOWER_MOTOR_INPUT_PIN = 9; //  PB1

const byte UPPER_MOTOR_STEP_PIN = 5; //PD5
const byte LOWER_MOTOR_STEP_PIN = 6; //PD6

// DELAYS ----------------------------------------------------------------------
Insomnia print_delay;
Microsomnia upper_motor_switching_delay;
Microsomnia lower_motor_switching_delay;
Microsomnia runtime_equalizer_timeout(50);

// FUNCTION DECLARARIONS IF NEEDED FOR THE COMPILER ----------------------------
float calculate_microdelay(float rpm);
void stepper_loop();

// FUNCTIONS *******************************************************************

// CALCULATE UPPER MOTOR SPEED -------------------------------------------------
void upper_motor_manage_ramp_up() {
  upper_motor_step_counter_delay -= microdelay_difference_per_speedlevel;

  // REACHED TOPSPEED:
  if (upper_motor_step_counter_delay < topspeed_step_counter_delay) {
    upper_motor_step_counter_delay = topspeed_step_counter_delay;
  }
}

void upper_motor_manage_ramp_down() {
  upper_motor_step_counter_delay += microdelay_difference_per_speedlevel;

  // REACHED MINIMUM SPEED:
  if (upper_motor_step_counter_delay > startspeed_step_counter_delay) {
    upper_motor_step_counter_delay = startspeed_step_counter_delay;
    upper_motor_is_running = false;
  }
}

// CALCULATE LOWER MOTOR SPEED -------------------------------------------------
void lower_motor_manage_ramp_up() {

  lower_motor_step_counter_delay -= microdelay_difference_per_speedlevel;

  // REACHED TOPSPEED:
  if (lower_motor_step_counter_delay < topspeed_step_counter_delay) {
    lower_motor_step_counter_delay = topspeed_step_counter_delay;
  }
}

void lower_motor_manage_ramp_down() {

  lower_motor_step_counter_delay += microdelay_difference_per_speedlevel;

  // REACHED MINIMUM SPEED:
  if (lower_motor_step_counter_delay > startspeed_step_counter_delay) {
    lower_motor_step_counter_delay = startspeed_step_counter_delay;
    lower_motor_is_running = false;
  }
}

// INITIAL CALCULATIONS --------------------------------------------------------
void make_initial_calculations() {

  Serial.println("INITIAL CALCULATIONS:");


  float startspeed_micro_delay = calculate_microdelay(min_motor_rpm);
  Serial.print("INITIAL DELAY [us]: ");
  Serial.println(startspeed_micro_delay);

  startspeed_step_counter_delay = startspeed_micro_delay / avg_runtime_us;
  Serial.print("INITIAL DELAY [cycles]: ");
  Serial.println(startspeed_step_counter_delay);

  float topspeed_micro_delay = calculate_microdelay(max_motor_rpm);
  Serial.print("TOPSPEED DELAY [us]:");
  Serial.println(topspeed_micro_delay);

  topspeed_step_counter_delay = round(topspeed_micro_delay / avg_runtime_us);
  Serial.print("TOPSPEED DELAY [cylces]:");
  Serial.println(topspeed_step_counter_delay);

  topspeed_micro_delay = topspeed_step_counter_delay * avg_runtime_us;
  Serial.print("RESULTING TOPSPEED DELAY [us]:");
  Serial.println(topspeed_micro_delay);

  float resulting_topspeed =
      60.0 * 1000000.0 /
      (topspeed_micro_delay * micro_step_factor * switches_per_step * full_steps_per_turn);
  Serial.print("RESULTING TOPSEED [rpm]:");
  Serial.println(resulting_topspeed);

  float next_slower_possible_speed = 60.0 * 1000000.0 /
                                     ((topspeed_micro_delay + avg_runtime_us) * micro_step_factor *
                                      switches_per_step * full_steps_per_turn);
  Serial.print("NEXT SLOWER SPEED [rpm]:");
  Serial.println(next_slower_possible_speed);

  float next_faster_possible_speed = 60.0 * 1000000.0 /
                                     ((topspeed_micro_delay - avg_runtime_us) * micro_step_factor *
                                      switches_per_step * full_steps_per_turn);
  Serial.print("NEXT FASTER SPEED [rpm]:");
  Serial.println(next_faster_possible_speed);

  // float delay_difference = startspeed_step_counter_delay - topspeed_step_counter_delay;
  // float float_delay_difference_per_speedlevel = delay_difference / calculation_resolution;
  // microdelay_difference_per_speedlevel = int(float_delay_difference_per_speedlevel);
  float float_time_per_speedlevel = float(acceleration_time) / (calculation_resolution - 1);
  int_time_per_speedlevel = int(float_time_per_speedlevel);
  Serial.print("TIME PER SPEEDLEVEL [ms]: ");
  Serial.println(float_time_per_speedlevel);

  float cycles_per_speedlevel = (float_time_per_speedlevel * 1000) / avg_runtime_us;
  int_cycles_per_speedlevel = int(cycles_per_speedlevel);
  Serial.print("CYCLES PER SPEEDLEVEL: ");
  Serial.println(int_cycles_per_speedlevel);
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

  if (PINB & _BV(PINB2)) {
    upper_motor_is_ramping_up = true;
    upper_motor_is_ramping_down = false;
    upper_motor_is_running = true;
  } else {
    upper_motor_is_ramping_up = false;
    upper_motor_is_ramping_down = true;
  }
}

void monitor_input_pin_lower_motor() {

  if (PINB & _BV(PINB1)) {
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
  Serial.println("MEASURE RUNTIME:");

  measure_runtime_enabled = true; // simulates running motors
  long number_of_cycles = 100000;
  unsigned long time_elapsed = 0;
  unsigned long time_before_loop = 0;
  unsigned long max_runtime = 0;
  unsigned long min_runtime = 666;
  unsigned long time_for_all_loops = 0;
  unsigned long count_max_values = 0;

  for (long i = number_of_cycles; i > 0; i--) {

    //delay(50);
    time_before_loop = micros();

    stepper_loop();

    time_elapsed = micros() - time_before_loop;

    time_for_all_loops = time_for_all_loops + time_elapsed;

    //delay(50);
    //Serial.println(time_elapsed);

    if (time_elapsed > max_runtime) {
      max_runtime = time_elapsed;
    }
    if (time_elapsed < min_runtime) {
      min_runtime = time_elapsed;
    }
  }
  float avg_runtime = float(time_for_all_loops) / number_of_cycles;

  Serial.print("TOTAL RUNTIME [ms]: ");
  Serial.println(time_for_all_loops / 1000);

  Serial.print("MIN RUNTIME [us]: ");
  Serial.println(min_runtime);

  Serial.print("MAX RUNTIME [us]: ");
  Serial.println(max_runtime);

  Serial.print("AVG RUNTIME [us]: ");
  Serial.println(avg_runtime);
  Serial.println("-----");

  measure_runtime_enabled = false;
  return int(avg_runtime);
}

void update_motor_frequencies() {
  if (speedlevel_cycle_counter == int_cycles_per_speedlevel) {
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

  upper_motor_counter++;

  if (upper_motor_is_running) {
    if (upper_motor_counter > upper_motor_step_counter_delay) {
      upper_motor_counter = 0;
      PORTD ^= _BV(PD5); // NANO PIN 10
    }
  }

  lower_motor_counter++;

  if (lower_motor_is_running) {
    if (lower_motor_counter > lower_motor_step_counter_delay) {
      lower_motor_counter = 0;
      PORTD ^= _BV(PD6); // NANO PIN 9
    }
  }
}

void stepper_loop() {
  //runtime_equalizer_timeout.reset_time();

  speedlevel_cycle_counter++;
  monitor_input_pin_upper_motor();
  monitor_input_pin_lower_motor();
  update_motor_frequencies();
  switch_outputs();

  //while (!runtime_equalizer_timeout.has_timed_out()) {
  // WAIT
  //}
}

// SETUP ***********************************************************************
void setup() {

  Serial.begin(115200);
  avg_runtime_us = measure_runtime();

  make_initial_calculations();

  pinMode(UPPER_MOTOR_STEP_PIN, OUTPUT);
  pinMode(LOWER_MOTOR_STEP_PIN, OUTPUT);

  // SET INITIAL SPEED:
  upper_motor_step_counter_delay = startspeed_step_counter_delay;
  lower_motor_step_counter_delay = startspeed_step_counter_delay;

  Serial.println("EXIT SETUP");
}

// LOOP ************************************************************************
void loop() {

  stepper_loop(); // seprated for runtime measurement

  if (print_debug_info) {
    if (print_delay.delay_time_is_up(1000)) {

      Serial.print("  MOTOR RUNNING: ");
      Serial.print(upper_motor_is_running);

      Serial.print("  DELAY: ");
      Serial.println(upper_motor_step_counter_delay);
    }
  }
}
// ********************************************************** END OF PROGRAM ***