/*
 * *****************************************************************************
 * STEPPER SATELLITE
 * *****************************************************************************
 * Generates stepper signals using an Arduino 
 * *****************************************************************************
 * RUNTIME:
 * Measured runtime: 16micros 
 * Resulting max rpm: 4687RPM
 * RPM = 75000/runtime
 * 75000 = (10^6 micros*60 seconds / 2 Switches / 2 Microsteps / 200Steps)
 * *****************************************************************************
 */

#include <Arduino.h>
#include <Debounce.h>
#include <Insomnia.h>
#include <microsomnia.h>

// GLOBAL VARIABLES ------------------------------------------------------------

// SPEED AND TIME SETUP:
const int min_motor_rpm = 100;
const int max_motor_rpm = 2500;
unsigned int acceleration_time = 1500; // microseconds from min to max rpm

// MOTOR PARAMETERS:
const int micro_step_factor = 2;
const int switches_per_step = 2; // on and off
const int calculation_resolution = 20;
const int full_steps_per_turn = 200; // 360/1.8°

// VALUES FOR IN LOOP CALCULATIONS:
int int_time_per_speedlevel;

int startspeed_microdelay;
int topspeed_microdelay;
int microdelay_difference_per_speedlevel;
int upper_motor_microdelay;

int current_step;

// STATE FLAGS:
bool upper_motor_is_ramping_up = true;
bool upper_motor_is_ramping_down = false;
bool upper_motor_ramp_up_started = false;
bool upper_motor_ramp_down_started = false;
bool motor_has_stopped = false;

// PINS:
const byte UPPER_MOTOR_INPUT_PIN = 10;
const byte LOWER_MOTOR_INPUT_PIN = 9;

const byte UPPER_MOTOR_STEP_PIN = 5;
const byte LOWER_MOTOR_STEP_PIN = 6;

// GENERATE OBJECTS ------------------------------------------------------------
Insomnia print_delay;
Insomnia testing_switch_delay;
Insomnia update_values_delay;
Microsomnia upper_motor_switching_delay;
Debounce upper_motor_input_pin(UPPER_MOTOR_INPUT_PIN);
Debounce lower_motor_input_pin(LOWER_MOTOR_INPUT_PIN);

// FUNCTION DECLARARION IF NEEDED FOR THE COMPILER -----------------------------
float calculate_microdelay(float rpm);
int calculate_current_step_number(unsigned long time_elapsed);

// FUNCTIONS -------------------------------------------------------------------

void upper_motor_manage_ramp_up() {
  if (!upper_motor_ramp_up_started) {
    upper_motor_ramp_up_started = true;
    upper_motor_ramp_down_started = false;
    upper_motor_microdelay = startspeed_microdelay;
    current_step = 0;
  }
  upper_motor_microdelay -= microdelay_difference_per_speedlevel;
  current_step++;

  // REACHED TOPSPEED:
  if (upper_motor_microdelay < topspeed_microdelay) {
    upper_motor_microdelay = topspeed_microdelay;
    // Serial.println("REACHED TOPSPEED");
      }
  if (upper_motor_switching_delay.delay_time_is_up(upper_motor_microdelay)) {
    digitalWrite(UPPER_MOTOR_STEP_PIN, !digitalRead(UPPER_MOTOR_STEP_PIN));
    digitalWrite(LOWER_MOTOR_STEP_PIN, !digitalRead(LOWER_MOTOR_STEP_PIN));
  }
}

void upper_motor_manage_ramp_down() {
  if (!upper_motor_ramp_down_started) {
    upper_motor_ramp_down_started = true;
    upper_motor_ramp_up_started = false;
    current_step = 0;
  }
  upper_motor_microdelay += microdelay_difference_per_speedlevel;
  current_step++;

  //REACHED MINIMUM SPEED
  if (upper_motor_microdelay > startspeed_microdelay) {
    upper_motor_microdelay = startspeed_microdelay;
    upper_motor_is_ramping_up = false;
    upper_motor_is_ramping_down = false;
    //Serial.println("MOTOR HAS STOPPED");
  }
  if (upper_motor_switching_delay.delay_time_is_up(upper_motor_microdelay)) {
    digitalWrite(UPPER_MOTOR_STEP_PIN, !digitalRead(UPPER_MOTOR_STEP_PIN));
    digitalWrite(LOWER_MOTOR_STEP_PIN, !digitalRead(LOWER_MOTOR_STEP_PIN));
  }
}

unsigned long measure_runtime() {
  static long previous_micros = micros();
  unsigned long runtime_elapsed = micros() - previous_micros;
  previous_micros = micros();
  return runtime_elapsed;
}

// INITIAL CALCULATIONS --------------------------------------------------------
void make_initial_calculations() {

  float float_time_per_speedlevel = float(acceleration_time) / (calculation_resolution - 1);
  int_time_per_speedlevel = int(float_time_per_speedlevel);
  Serial.print("TIME PER SPEEDLEVEL: ");
  Serial.println(float_time_per_speedlevel);

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

// SETUP -----------------------------------------------------------------------
void setup() {

  Serial.begin(115200);
  Serial.println("START CALCULATIONS:");
  make_initial_calculations();
  Serial.println("EXIT SETUP");
  pinMode(UPPER_MOTOR_STEP_PIN, OUTPUT);
  pinMode(LOWER_MOTOR_STEP_PIN, OUTPUT);
  upper_motor_microdelay = startspeed_microdelay;
}

// LOOP ------------------------------------------------------------------------
void loop() {

  if (upper_motor_input_pin.switchedHigh()){
upper_motor_is_ramping_up=true;
upper_motor_is_ramping_down=false;
  }

  if (upper_motor_input_pin.switchedLow()){
upper_motor_is_ramping_up=false;
upper_motor_is_ramping_down=true;
  }

    if (lower_motor_input_pin.switchedHigh()){
upper_motor_is_ramping_up=true;
upper_motor_is_ramping_down=false;
  }

  if (lower_motor_input_pin.switchedLow()){
upper_motor_is_ramping_up=false;
upper_motor_is_ramping_down=true;
  }

  if (update_values_delay.delay_time_is_up(int_time_per_speedlevel)) {
    if (upper_motor_is_ramping_up) {
      upper_motor_manage_ramp_up();
    }
    if (upper_motor_is_ramping_down) {
      upper_motor_manage_ramp_down();
    }
  }

  unsigned long runtime = measure_runtime();

  if (print_delay.delay_time_is_up(1500)) {

    Serial.print(" CURRENT STEP : ");
    Serial.print(current_step);

    Serial.print("  DELAY: ");
    Serial.print(upper_motor_microdelay);

    Serial.print("  CODE RUNTIME: ");
    Serial.println(runtime);
  }
}