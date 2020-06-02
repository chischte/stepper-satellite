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
 * ...currently 68 micros, 128 2nd measurement!!!!
 */

#include <Arduino.h>
#include <Debounce.h>
#include <Insomnia.h>

// GLOBAL VARIABLES:
const int min_motor_rpm = 200;
const int max_motor_rpm = 2500;
const int calculation_resolution = 2;
const int micro_step_factor = 2;
const int switches_per_step = 2; // on and off
const float acceleration_time = 10000; // microseconds from min to max rpm
const int full_steps_per_turn = 200; // 360/1.8°

int switchdelay_micros_array[calculation_resolution];
int switchdelay_micros;

unsigned long upper_motor_starting_time;
unsigned long upper_motor_stopping_time;
unsigned long upper_start_time_elapsed;

bool upper_motor_is_running = false;
int upper_motor_rpm = 0;
unsigned long upper_motor_microdelay;

// PINS:
const byte UPPER_MOTOR_INPUT_PIN = 2;

Insomnia print_delay;

// GENERATE OBJECTS:
Debounce upper_motor_input_pin(UPPER_MOTOR_INPUT_PIN);

// FUNCTION DECLARARION IF NEEDED FOR THE COMPILER:
float calculate_rpm(float time_elapsed);
unsigned long calculate_microdelay(float rpm);

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
  upper_start_time_elapsed = millis() - upper_motor_starting_time;
  unsigned long current_step = calculation_resolution * acceleration_time / upper_start_time_elapsed;
  switchdelay_micros = switchdelay_micros_array[int(current_step)];
}

float calculate_rpm(float time_elapsed) {
  float speed_factor = time_elapsed / acceleration_time;

  if (speed_factor > 1) {
    speed_factor = 1;
  }

  float float_rpm = speed_factor * (max_motor_rpm - min_motor_rpm) + min_motor_rpm;
  return float_rpm;
}

unsigned long calculate_microdelay(float rpm) {

  float turns_per_second = rpm / 60;
  float switches_per_turn = full_steps_per_turn * micro_step_factor * switches_per_step;
  float steps_per_second = turns_per_second * switches_per_turn;
  float float_microdelay = 1000000 / steps_per_second;

  return int(float_microdelay);
}

long measure_runtime() {
  static long previous_micros = micros();
  long runtime_elapsed = micros() - previous_micros;
  previous_micros = micros();
  return runtime_elapsed;
}

void stop_upper_motor() {}

void make_initial_calculations() {
  unsigned long switches_per_turn;
  switches_per_turn = full_steps_per_turn * micro_step_factor * switches_per_step;
  Serial.print("SWITCHES PER TURN: ");
  Serial.println(switches_per_turn);

  float time_per_speedlevel;
  time_per_speedlevel = acceleration_time / (calculation_resolution - 1);
  Serial.print("TIME PER SPEEDLEVEL: ");
  Serial.println(time_per_speedlevel);

  float rpm_shift_per_speedlevel;
  rpm_shift_per_speedlevel = float(max_motor_rpm - min_motor_rpm) / calculation_resolution;
  Serial.print("RPM SHIFT PER SPEEDLEVEL: ");
  Serial.println(rpm_shift_per_speedlevel);

  for (int i; i < calculation_resolution; i++) {
    Serial.print("STEP NO: ");
    Serial.print(i);

    float time_elapsed = time_per_speedlevel * (i); // +1 to start at min pwm
    Serial.print(" TIME ELAPSED: ");
    Serial.print(time_elapsed);
    float rpm = calculate_rpm(time_elapsed);
    Serial.print(" RPM: ");
    Serial.print(rpm);
    switchdelay_micros_array[i] = calculate_microdelay(rpm);
    Serial.print(" DELAY MICROS: ");
    Serial.println(switchdelay_micros_array[i]);
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("START CALCULATIONS");
  make_initial_calculations();
  Serial.println("EXIT SETUP");
}

void loop() {

  start_upper_motor();

  long runtime = measure_runtime();
  if (print_delay.delay_time_is_up(1500)) {
    Serial.print("START TIME ELAPSED: ");
    Serial.print(upper_start_time_elapsed);
    Serial.print("  RPM: ");
    Serial.print(upper_motor_rpm);
    Serial.print("  DELAY: ");
    Serial.print(upper_motor_microdelay);
    Serial.print("  CODE RUNTIME: ");
    Serial.println(runtime);
  }
}