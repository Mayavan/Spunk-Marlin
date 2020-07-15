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