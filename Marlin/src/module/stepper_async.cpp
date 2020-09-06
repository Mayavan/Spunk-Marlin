#include "stepper_async.h"
#include "../core/serial.h"
#include "../pins/lpc1768/pins_BTT_SKR_V1_3.h"
#include "PololuDriver.h"

#define getTimeMicroseconds() micros()
#define getTimeMilliseconds() millis()

PololuStepper motor_x = PololuStepper(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
PololuStepper motor_y = PololuStepper(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);

int i=0; 
int stepped = 0;

void stepper_async::init() {
  current_x_state = INITIAL_X;
  current_y_state = INITIAL_Y;
  current_x_velocity = 0;
  current_y_velocity = 0;
  target_x_state = INITIAL_X;
  target_y_state = INITIAL_Y;
  interval = 0;
  max_velocity_x = MAX_VELOCITY_X;
  max_velocity_y = MAX_VELOCITY_Y;
  acceleration_x = ACCELERATION_X;
  acceleration_y = ACCELERATION_Y;
}

void stepper_async::controller_thread(){ 
  // Handle X-axis
  if(abs(target_x_state-current_x_state) > TOLERENCE)
  {
    i = i+1;
    // Set direction
    amount = target_x_state - current_x_state;
    direction = (amount>0)?1:-1;
    motor_x.setDir(direction);

    float decelerate_tolerence_estimate = (target_x_state - (current_x_state+current_x_velocity+(acceleration_x * (INTEGRAL_FREQUENCY+1)/(2*INTEGRAL_FREQUENCY))));
    
    // Update velocity
    if(abs(decelerate_tolerence_estimate)<TOLERENCE) // Deccelerate
    {
      interval = getTimeMicroseconds()-previous_step_time_us;
      current_x_velocity = current_x_velocity - (acceleration_x*interval/1.0e6);
      if(current_x_velocity<0) current_x_velocity = 0;
    }
    else if(current_x_velocity<max_velocity_x)
    {
      interval = getTimeMicroseconds()-previous_step_time_us;
      current_x_velocity = current_x_velocity + (acceleration_x*interval/1.0e6);
      if(current_x_velocity>max_velocity_x) current_x_velocity = max_velocity_x;
    }
    
    step_time_period = 1/(current_x_velocity*MICROSTEPS_PER_STEP);
    if(interval > step_time_period)
    {
      stepped++;
      motor_x.smartStep();
      // Update Current position
      current_x_state = current_x_state + ((DEGREE_PER_MICROSTEP/2.0) * direction)/GEAR_RATIO_X;
      previous_step_time_us = getTimeMicroseconds();
    }

  }
  else
  {
    current_x_velocity = 0;
  }

  // Handle Y-axis
  if(abs(target_y_state-current_y_state) > TOLERENCE)
  {
    i = i+1;
    // Set direction
    amount = target_y_state - current_y_state;
    direction = (amount>0)?1:-1;
    motor_y.setDir(direction);

    float decelerate_tolerence_estimate = (target_y_state - (current_y_state+current_y_velocity+(acceleration_y * (INTEGRAL_FREQUENCY+1)/(2*INTEGRAL_FREQUENCY))));
    
    // Update velocity
    if(abs(decelerate_tolerence_estimate)<TOLERENCE) // Deccelerate
    {
      interval = getTimeMicroseconds()-previous_step_time_us;
      current_y_velocity = current_y_velocity - (acceleration_y*interval/1.0e6);
      if(current_y_velocity<0) current_y_velocity = 0;
    }
    else if(current_y_velocity<max_velocity_y)
    {
      interval = getTimeMicroseconds()-previous_step_time_us;
      current_y_velocity = current_y_velocity + (acceleration_y*interval/1.0e6);
      if(current_y_velocity>max_velocity_y) current_y_velocity = max_velocity_y;
    }

    step_time_period = 1/(current_y_velocity*MICROSTEPS_PER_STEP);
    if(interval > step_time_period)
    {
      motor_y.smartStep();
      // Update Current position
      current_y_state = current_y_state + ((DEGREE_PER_MICROSTEP/2.0) * direction)/GEAR_RATIO_Y;
      previous_step_time_us = getTimeMicroseconds();
    }

  }
  else
  {
    current_y_velocity = 0;
  }
}

void stepper_async::set_async_target_x(float x) {
  motor_x.enable();
  motor_y.enable();
  target_x_state = x;

  SERIAL_ECHOLNPGM("Values are set");
  SERIAL_ECHOLNPAIR_F("x:", target_x_state);
}

void stepper_async::set_async_target_y(float y) {
  motor_x.enable();
  motor_y.enable();
  target_y_state = y;

  SERIAL_ECHOLNPGM("Values are set");
  SERIAL_ECHOLNPAIR_F("y:", target_y_state);
}

void stepper_async::print_log(float z){
  if(z==1)
    SERIAL_ECHOLNPAIR_F("i:", i);
  if(z==2){
    SERIAL_ECHOLNPAIR_F("Current X Velocity:", current_x_velocity);
    SERIAL_ECHOLNPAIR_F("Current Y Velocity:", current_y_velocity);
  }
  if(z==3)
    SERIAL_ECHOLNPAIR_F("Interval:", interval);
  if(z==4){
    SERIAL_ECHOLNPAIR_F("Current X Position:", current_x_state);
    SERIAL_ECHOLNPAIR_F("Current Y Position:", current_y_state);
  }
  if(z==5)
    SERIAL_ECHOLNPAIR_F("Target X Position:", target_x_state);
    SERIAL_ECHOLNPAIR_F("Target Y Position:", target_y_state);
  if(z==6)
    SERIAL_ECHOLNPAIR_F("Stepped:", stepped);
  if(z==7)
    SERIAL_ECHOLNPAIR_F("direction:", direction);
  if(z==8)
    SERIAL_ECHOLNPAIR_F("DEGREE_PER_MICROSTEP:", DEGREE_PER_MICROSTEP);
}

void stepper_async::set_velocity_x(float x) {
  max_velocity_x = x * GEAR_RATIO_X;

  SERIAL_ECHOLNPGM("Values are set");
  SERIAL_ECHOLNPAIR_F("x:", max_velocity_x);
}

void stepper_async::set_velocity_y(float y) {
  max_velocity_y = y * GEAR_RATIO_Y;

  SERIAL_ECHOLNPGM("Values are set");
  SERIAL_ECHOLNPAIR_F("y:", max_velocity_y);
}

void stepper_async::set_acceleration_x(float x) {
  acceleration_x = x * GEAR_RATIO_X;

  SERIAL_ECHOLNPGM("Values are set");
  SERIAL_ECHOLNPAIR_F("x:", acceleration_x);
}

void stepper_async::set_acceleration_y(float y) {
  acceleration_y = y * GEAR_RATIO_Y;

  SERIAL_ECHOLNPGM("Values are set");
  SERIAL_ECHOLNPAIR_F("y:", acceleration_y);
}

void stepper_async::set_current_x(float x) {
  current_x_state = x;
  target_x_state = x;
}

void stepper_async::set_current_y(float y) {
  current_y_state = y;
  target_y_state = y;
}
