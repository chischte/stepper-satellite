/*
 * *****************************************************************************
 * STEPPER SATELLITE
 * *****************************************************************************
 * Generates stepper signals using an Arduino
 * *****************************************************************************
 * RUNTIME:
 * Measured runtime:
 * Resulting max stepper frequency:
 *
 * TODO:
 * Make calculations only once at beginnig if possible
 * Create Microsomnia-delay library
 * Measure runtime! Should not exceed 30 micros
 * ...currently 68 micros!!!!
 */

#include <Arduino.h>
#include <Debounce.h>
#include <Insomnia.h>

// GLOBAL VARIABLES:
const int min_motor_rpm = 100;
const int max_motor_rpm = 2500;
const int micro_step_factor = 2;
const int switches_per_step = 2; // on and off
const int acceleration_time = 2000; // microseconds from 0 to max rpm
const int full_steps_per_turn = 200; // 360/1.8°

unsigned long upper_motor_starting_time;
unsigned long upper_motor_stopping_time;

bool upper_motor_is_running = false;
int upper_motor_rpm = 0;
unsigned long upper_motor_microdelay;

// PINS:
const byte UPPER_MOTOR_INPUT_PIN = 2;

Insomnia print_delay;

// GENERATE OBJECTS:
Debounce upper_motor_input_pin(UPPER_MOTOR_INPUT_PIN);

// FUNCTION DECLARARION IF NEEDED FOR THE COMPILER:
int calculate_rpm(unsigned long starting_time);
unsigned long calculate_microdelay(int rpm);

// FUNCTIONS:

long upper_motor_calculate_delay_time() {
  long delay_time = 0;
  return delay_time;
}

void start_upper_motor() {
  if (!upper_motor_is_running) {
    upper_motor_is_running = true;
    upper_motor_starting_time = millis();
  }
  upper_motor_rpm = calculate_rpm(upper_motor_starting_time);
  upper_motor_microdelay = calculate_microdelay(upper_motor_rpm);
}

int calculate_rpm(unsigned long starting_time) {
  unsigned long time_elapsed = millis() - starting_time;
  float speed_factor = float(time_elapsed) / float(acceleration_time);
  if (speed_factor > 1) {
    speed_factor = 1;
  }
  float float_rpm = speed_factor * max_motor_rpm;
  int rpm = float_rpm;
  return rpm;
}

unsigned long calculate_microdelay(int rpm) {
  unsigned long microdelay;
  float turns_per_second = float(rpm) / 60;
  float steps_per_second =
      turns_per_second * steps_per_second * micro_step_factor * switches_per_step;
  float float_microdelay = 1000000 / steps_per_second;
  microdelay = float_microdelay;
  return microdelay;
}

long measure_runtime() {
  static long previous_micros = micros();
  long time_elapsed = micros() - previous_micros;
  previous_micros = micros();
  return time_elapsed;
}

void stop_upper_motor() {}

void make_initial_calculations() {
  unsigned long switches_per_turn;
  switches_per_turn = full_steps_per_turn * micro_step_factor * switches_per_step;
  Serial.print("SWITCHES PER TURN: ");
  Serial.println(switches_per_turn);
}

void setup() {

  Serial.begin(115200);
  make_initial_calculations();
  Serial.println("EXIT SETUP");
}

void loop() {

  start_upper_motor();

  long runtime = measure_runtime();
  if (print_delay.delay_time_is_up(500)) {
    Serial.print("RPM: ");
    Serial.println(upper_motor_rpm);
    Serial.print("DELAY: ");
    Serial.println(upper_motor_microdelay);
    Serial.print("CODE RUNTIME: ");
    Serial.println(runtime);
  }
}