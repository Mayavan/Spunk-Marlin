#include "../gcode.h"
#include "../parser.h"
#include "../../module/stepper_async.h"

extern xyze_pos_t current_position,  // High-level current tool position
                  destination;
/**
 * G100: Asynchronous movement of X Y Z E axes
 */
void GcodeSuite::G100() {
  SERIAL_ECHOPGM("G100 called \n");

  if ( (parser.seenval(axis_codes[0])) ) {
    stepper_async::getInstance().set_async_target_x(parser.value_axis_units(X_AXIS));
  }
  if (parser.seenval(axis_codes[1])) {
    stepper_async::getInstance().set_async_target_y(parser.value_axis_units(Y_AXIS));
  }
  if (parser.seenval(axis_codes[2])) {
    stepper_async::getInstance().print_log(parser.value_axis_units(Z_AXIS));
  }
}

/**
 * G101: Set velocity of X Y axes in in degrees/second
 */
void GcodeSuite::G101() {
  SERIAL_ECHOPGM("G101 called \n");

  if ( (parser.seenval(axis_codes[0])) ) {
    stepper_async::getInstance().set_velocity_x(parser.value_axis_units(X_AXIS));
  }
  if (parser.seenval(axis_codes[1])) {
    stepper_async::getInstance().set_velocity_y(parser.value_axis_units(Y_AXIS));
  }

}

/**
 * G102: Set acceleration of X Y axes in in degrees/sq. second
 */
void GcodeSuite::G102() {
  SERIAL_ECHOPGM("G102 called \n");

  if ( (parser.seenval(axis_codes[0])) ) {
    stepper_async::getInstance().set_acceleration_x(parser.value_axis_units(X_AXIS));
  }
  if (parser.seenval(axis_codes[1])) {
    stepper_async::getInstance().set_acceleration_y(parser.value_axis_units(Y_AXIS));
  }

}

/**
 * G103: Set current position of X Y axes
 */
void GcodeSuite::G103() {
  SERIAL_ECHOPGM("G103 called \n");

  if ( (parser.seenval(axis_codes[0])) ) {
    stepper_async::getInstance().set_current_x(parser.value_axis_units(X_AXIS));
  }
  if (parser.seenval(axis_codes[1])) {
    stepper_async::getInstance().set_current_y(parser.value_axis_units(Y_AXIS));
  }

}

/**
 * G104: TODO: For future use
 */
void GcodeSuite::G104() {
  SERIAL_ECHOPGM("G104 called \n");
}

/**
 * G105: TODO: For future use
 */
void GcodeSuite::G105() {
  SERIAL_ECHOPGM("G105 called \n");
}