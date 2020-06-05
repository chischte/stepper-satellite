/*
 * *****************************************************************************
 * STEPPER SATELLITE
 * *****************************************************************************
 * Generates stepper signals using an Arduino 
 * *****************************************************************************
 * TODO:
 * Measure effective speed
 * https://baremetalmicro.com/tutorial_avr_digital_io/index.html
 * https://github.com/NicksonYap/digitalWriteFast
 * *****************************************************************************
 * -->PRESS MOTOR BUTTONS WHEN MEASURING RUNTIME
 * RUNTIME:
 * Measured max runtime: 28us**
 * **  --> substracted 5us for an insomnia-delay
 * Resulting max rpm: 2678
 * RPM = 75000/runtime
 * 75000 = (10^6 micros*60 seconds / 2 Switches / 2 Microsteps / 200Steps)
 * COSTS:
 * 12us for a debounce (removed)
 * 10us for a pin monitoring
 * 6us for a digital read
 * 5us for a digitalWrite
 * 5us for a insomnia-delay
 * *****************************************************************************
 * --> SET print_debug_information false WHEN OPERATING
 * *****************************************************************************
 */

#include <Arduino.h>
#include <Insomnia.h>
#include <microsomnia.h>
#include <pin_monitor.h>

// GLOBAL VARIABLES ------------------------------------------------------------

// --> DEACTIVATE DEBUG WHEN OPERATING <---!-!-!-!-!-!

bool print_debug_info = false;
//bool print_debug_info = true;

// --> DEACTIVATE DEBUG WHEN OPERATING <---!-!-!-!-!-!

// RUNTIME MEASUREMENT:
int avg_runtime_us = 0;
bool measure_runtime;

// INITIAL CALCULATIONS:
int int_cycles_per_speedlevel;

// SPEED AND TIME SETUP:
const int min_motor_rpm = 100; // min = 10 (calculation algorithm)
const int max_motor_rpm = 1300; // Motor max = 1750 (specification)
unsigned int acceleration_time = 200; // microseconds from min to max rpm

// MOTOR PARAMETERS:
const int micro_step_factor = 2;
const int switches_per_step = 2; // on and off
const int calculation_resolution = 200;
const int full_steps_per_turn = 200; // 360/1.8°

// VALUES FOR IN LOOP CALCULATIONS:
int int_time_per_speedlevel;

int startspeed_microdelay;
int topspeed_microdelay;
int microdelay_difference_per_speedlevel;
int upper_motor_microdelay;
int lower_motor_microdelay;

int current_step;

// STATE FLAGS UPPER MOTOR:
bool upper_motor_is_running = false;
bool upper_motor_is_ramping_up = false;
bool upper_motor_is_ramping_down = false;

// STATE FLAGS LOWER MOTOR:
bool lower_motor_is_running = false;
bool lower_motor_is_ramping_up = false;
bool lower_motor_is_ramping_down = false;

// PINS:
const byte UPPER_MOTOR_INPUT_PIN = 10; // PB2
const byte LOWER_MOTOR_INPUT_PIN = 9; //  PB1
const byte TEST_SWITCH_PIN = 11; // PB3

const byte UPPER_MOTOR_STEP_PIN = 5; //PD5
const byte LOWER_MOTOR_STEP_PIN = 6; //PD6

// DELAYS ----------------------------------------------------------------------
Insomnia print_delay;
Insomnia update_values_delay;
Microsomnia upper_motor_switching_delay;
Microsomnia lower_motor_switching_delay;

// FUNCTION DECLARARION IF NEEDED FOR THE COMPILER -----------------------------

float calculate_microdelay(float rpm);
int calculate_current_step_number(unsigned long time_elapsed);
void stepper_loop();

// FUNCTIONS *******************************************************************

// CALCULATE UPPER MOTOR SPEED -------------------------------------------------
void upper_motor_manage_ramp_up() {
  upper_motor_microdelay -= microdelay_difference_per_speedlevel;
  //Serial.println("RAMP UP");
  current_step++;

  // REACHED TOPSPEED:
  if (upper_motor_microdelay < topspeed_microdelay) {
    upper_motor_microdelay = topspeed_microdelay;
  }
}

void upper_motor_manage_ramp_down() {
  //Serial.println("RAMP DOWN");
  upper_motor_microdelay += microdelay_difference_per_speedlevel;
  current_step++;

  // REACHED MINIMUM SPEED:
  if (upper_motor_microdelay > startspeed_microdelay) {
    upper_motor_microdelay = startspeed_microdelay;
    upper_motor_is_running = false;
  }
}

// CALCULATE LOWER MOTOR SPEED -------------------------------------------------
void lower_motor_manage_ramp_up() {

  lower_motor_microdelay -= microdelay_difference_per_speedlevel;
  current_step++;

  // REACHED TOPSPEED:
  if (lower_motor_microdelay < topspeed_microdelay) {
    lower_motor_microdelay = topspeed_microdelay;
  }
}

void lower_motor_manage_ramp_down() {

  lower_motor_microdelay += microdelay_difference_per_speedlevel;
  current_step++;

  // REACHED MINIMUM SPEED:
  if (lower_motor_microdelay > startspeed_microdelay) {
    lower_motor_microdelay = startspeed_microdelay;
    lower_motor_is_running = false;
  }
}

// INITIAL CALCULATIONS --------------------------------------------------------
void make_initial_calculations() {

  float float_time_per_speedlevel = float(acceleration_time) / (calculation_resolution - 1);
  int_time_per_speedlevel = int(float_time_per_speedlevel);
  Serial.print("TIME PER SPEEDLEVEL [ms]: ");
  Serial.println(float_time_per_speedlevel);

  float cycles_per_speedlevel = (float_time_per_speedlevel * 1000) / avg_runtime_us;
  int_cycles_per_speedlevel = int(cycles_per_speedlevel);
  Serial.print("CYCLES PER SPEEDLEVEL: ");
  Serial.println(int_cycles_per_speedlevel);

  startspeed_microdelay = calculate_microdelay(min_motor_rpm);
  Serial.print("INITIAL MICRO-DELAY: ");
  Serial.println(startspeed_microdelay);
  topspeed_microdelay = calculate_microdelay(max_motor_rpm);

  Serial.print("TOPSPEED MICRO-DELAY:");
  Serial.println(topspeed_microdelay);

  float delay_difference = startspeed_microdelay - topspeed_microdelay;
  float float_delay_difference_per_speedlevel = delay_difference / calculation_resolution;
  microdelay_difference_per_speedlevel = int(float_delay_difference_per_speedlevel);

  float rpm_shift_per_speedlevel;
  rpm_shift_per_speedlevel = float(max_motor_rpm - min_motor_rpm) / calculation_resolution;
  Serial.print("RPM SHIFT PER SPEEDLEVEL: ");
  Serial.println(rpm_shift_per_speedlevel);
}

float calculate_microdelay(float rpm) {

  float turns_per_second = rpm / 60;
  float switches_per_turn = full_steps_per_turn * micro_step_factor * switches_per_step;
  float steps_per_second = turns_per_second * switches_per_turn;
  float float_microdelay = 1000000 / steps_per_second;
  return float_microdelay;
}

// SETUP ***********************************************************************
int measure_setup_runtime() {
  measure_runtime = true;
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
  measure_runtime = false;
  return avg_runtime;
}

void setup() {

  Serial.begin(115200);
  avg_runtime_us = measure_setup_runtime();
  Serial.println("START CALCULATIONS:");
  make_initial_calculations();
  pinMode(UPPER_MOTOR_STEP_PIN, OUTPUT);
  pinMode(LOWER_MOTOR_STEP_PIN, OUTPUT);
  pinMode(TEST_SWITCH_PIN, INPUT_PULLUP); // for mega only
  pinMode(50, INPUT_PULLUP); // for mega only
  // SET INITIAL SPEED:
  upper_motor_microdelay = startspeed_microdelay;
  lower_motor_microdelay = startspeed_microdelay;

  Serial.println("EXIT SETUP");
}

void stepper_loop() {

  static int speedlevel_cycle_counter = 0;
  speedlevel_cycle_counter++;

  // REACT TO INPUT PIN STATES -------------------------------------------------

  // UPPER MOTOR:
  if (PINB & _BV(PINB2)) {
    upper_motor_is_ramping_up = true;
    upper_motor_is_ramping_down = false;
    upper_motor_is_running = true;
  } else {
    upper_motor_is_ramping_up = false;
    upper_motor_is_ramping_down = true;
  }
  // LOWER MOTOR:

  if (PINB & _BV(PINB1)) {
    lower_motor_is_ramping_up = true;
    lower_motor_is_ramping_down = false;
    lower_motor_is_running = true;
  } else {
    lower_motor_is_ramping_up = false;
    lower_motor_is_ramping_down = true;
  }

  if (measure_runtime) {
    upper_motor_is_ramping_up = true;
    upper_motor_is_running = true;
    lower_motor_is_ramping_up = true;
    lower_motor_is_running = true;
  }
  // UPDATE MOTOR FREQUENCIES ----------------------------------------------------

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
  // SWITCH OUTPUTS -----------------------------------------------------------

  if (upper_motor_is_running) {
    if (upper_motor_switching_delay.delay_time_is_up(upper_motor_microdelay)) {
      PORTD ^= _BV(PD5); // NANO PIN 10
    }
  }

  if (lower_motor_is_running) {
    if (lower_motor_switching_delay.delay_time_is_up(lower_motor_microdelay)) {
      PORTD ^= _BV(PD6); // NANO PIN 9
    }
  }
  // GET INFORMATION -----------------------------------------------------------
}

// LOOP ************************************************************************
void loop() {
  stepper_loop();

  if (print_debug_info) {
    if (print_delay.delay_time_is_up(1000)) {

      Serial.print("  MOTOR RUNNING: ");
      Serial.print(upper_motor_is_running);

      Serial.print(" CURRENT STEP : ");
      Serial.print(current_step);

      Serial.print("  DELAY: ");
      Serial.println(upper_motor_microdelay);
    }
  }
}