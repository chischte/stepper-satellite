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
 * ...currently 60 micros!!!
 */

#include <Arduino.h>
#include <Debounce.h>
#include <Insomnia.h>

// GLOBAL VARIABLES:
const int min_motor_rpm = 200;
const int max_motor_rpm = 2500;
const int calculation_resolution = 20;
const int micro_step_factor = 2;
const int switches_per_step = 2; // on and off
int startspeed_microdelay;
int topspeed_microdelay;
int int_delay_difference_per_speedlevel;
unsigned int acceleration_time = 20000; // microseconds from min to max rpm
const int full_steps_per_turn = 200; // 360/1.8°

int switchdelay_micros_array[calculation_resolution];
int switchdelay_micros;
int current_step;

unsigned long upper_motor_starting_time;
unsigned long upper_motor_stopping_time;
unsigned long upper_start_time_elapsed;

bool upper_motor_is_running = false;
int upper_motor_rpm = 0;
int upper_motor_microdelay;

float float_time_per_speedlevel;
int int_time_per_speedlevel;

// PINS:
const byte UPPER_MOTOR_INPUT_PIN = 2;
const byte UPPER_MOTOR_STEP_PIN=3;

Insomnia print_delay;
Insomnia upper_motor_ramp_up_delay;

// GENERATE OBJECTS:
Debounce upper_motor_input_pin(UPPER_MOTOR_INPUT_PIN);

// FUNCTION DECLARARION IF NEEDED FOR THE COMPILER:
float calculate_microdelay(float rpm);
int calculate_current_step_number(unsigned long time_elapsed);

// FUNCTIONS -------------------------------------------------------------------

void start_upper_motor() {
  if (!upper_motor_is_running) {
    upper_motor_is_running = true;
    upper_motor_starting_time = millis();
    upper_motor_microdelay = startspeed_microdelay;
    current_step=0;
  }

  if (upper_motor_ramp_up_delay.delay_time_is_up(int_time_per_speedlevel)) {
    upper_motor_microdelay -= int_delay_difference_per_speedlevel;
    current_step++;
  }
  if (upper_motor_microdelay<topspeed_microdelay){
    upper_motor_microdelay=topspeed_microdelay;
  }
 }

unsigned long measure_runtime() {
  static long previous_micros = micros();
  unsigned long runtime_elapsed = micros() - previous_micros;
  previous_micros = micros();
  return runtime_elapsed;
}

void stop_upper_motor() {}

void make_initial_calculations() {
  
  float_time_per_speedlevel = float(acceleration_time) / (calculation_resolution - 1);
  int_time_per_speedlevel = int(float_time_per_speedlevel);
  Serial.print("TIME PER SPEEDLEVEL: ");
  Serial.println(float_time_per_speedlevel);

  startspeed_microdelay = calculate_microdelay(min_motor_rpm);
  Serial.print("INITIAL DELAY: ");
  Serial.println(startspeed_microdelay);
  topspeed_microdelay = calculate_microdelay(max_motor_rpm);

  Serial.print("TOPSPEED DELAY:");
  Serial.println(topspeed_microdelay);

  float delay_difference = startspeed_microdelay - topspeed_microdelay;
  float delay_difference_per_speedlevel = delay_difference / calculation_resolution;
  int_delay_difference_per_speedlevel = int(delay_difference_per_speedlevel);

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

void setup() {

  Serial.begin(115200);
  Serial.println("START CALCULATIONS");
  make_initial_calculations();
  Serial.println("EXIT SETUP");
}

void loop() {

  start_upper_motor();

  unsigned long runtime = measure_runtime();

  if (print_delay.delay_time_is_up(1500)) {

    Serial.print("START TIME ELAPSED: ");
    Serial.print(upper_start_time_elapsed);

    Serial.print(" CURRENT STEP : ");
    Serial.print(current_step);

    Serial.print("  DELAY: ");
    Serial.print(upper_motor_microdelay);

    Serial.print("  CODE RUNTIME: ");
    Serial.println(runtime);
  }
}