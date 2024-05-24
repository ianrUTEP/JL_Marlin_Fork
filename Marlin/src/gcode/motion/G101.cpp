/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"

#include "../../MarlinCore.h"

#if ALL(FWRETRACT, FWRETRACT_AUTORETRACT)
  #include "../../feature/fwretract.h"
#endif

#include "../../sd/cardreader.h"

#if ENABLED(NANODLP_Z_SYNC)
  #include "../../module/planner.h"
#endif

extern xyze_pos_t destination;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  feedRate_t fast_move_feedrate = MMM_TO_MMS(G0_FEEDRATE);
#endif

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
void GcodeSuite::G101(TERN_(HAS_FAST_MOVES, const bool fast_move/*=false*/)) {
  static const char z_axis_codes[] = { LIST_N(NUM_Z_STEPPERS, 'U', 'V', 'W') };
  static float change = 0; // A variable to store changes of each z stepper
  static float last_z = 0; // A variable to track the last known z before performing the adjustments
  static feedRate_t tilt_fr_mm_s = 9;  // Feedrate of the tilting. Stored and evaluated in mm/s

  if (!MOTION_CONDITIONS) return;

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_RUNNING));

  #ifdef G0_FEEDRATE
    feedRate_t old_feedrate;
    #if ENABLED(VARIABLE_G0_FEEDRATE)
      if (fast_move) {
        old_feedrate = feedrate_mm_s;             // Back up the (old) motion mode feedrate
        feedrate_mm_s = fast_move_feedrate;       // Get G0 feedrate from last usage
      }
    #endif
  #endif

  // Include the O flag to first reset the current displacements to 0.
  // Primarily for testing purposes only. 
  if (parser.seen('O')) {
    for(int i = 0; i < NUM_Z_STEPPERS; i++) {
      z_deltas[i] = 0;
    }
  }

  // Include a Q value to set the feedrate of the tilting moves. 
  // If not included, the last known will be used
  // The Q value should be provided in GCODE as mm/min, the value_feedrate() automatically converts to mm/s
  if (parser.floatval('Q') > 0) {
    tilt_fr_mm_s = parser.value_feedrate();
  }

  stepper.set_separate_multi_axis(true);  //Separate Z axes
  for (int i = 0; i < NUM_Z_STEPPERS; i++) {
    // Check for pressence of current stepper
    if (parser.seenval(z_axis_codes[i])) {
      last_z = current_position.z;
      // Change is the difference between the provided value and the stored value
      // In this way, only the required motion for each axis will be done
      // The provided value is then stored to the array holding the difference positions
      // Values are signed floats - not sure if necessary for resolution
      // 
      // The movement target for each axis is the current z position of the entire axis
      // plus the necessary change to get to the delta location
      stepper.set_all_z_lock(true, i);  // Lock all except current Z stepper
      change = z_deltas[i] - parser.floatval(z_axis_codes[i]);
      do_blocking_move_to_z(current_position.z - change, tilt_fr_mm_s);
      stepper.set_all_z_lock(false);  // Unlock all axes
      z_deltas[i] = parser.floatval(z_axis_codes[i]); // Save the new delta position
      // Reset the current position to the last known
      // Also must set the current position in the planner or else it will execute a move
      current_position.z = last_z;
      planner.set_position_mm(current_position);
    }
  }
  stepper.set_separate_multi_axis(false); // Relock all the steppers

  get_destination_from_command();                 // Get X Y [Z[I[J[K]]]] [E] F (and set cutter power)

  #ifdef G0_FEEDRATE
    if (fast_move) {
      #if ENABLED(VARIABLE_G0_FEEDRATE)
        fast_move_feedrate = feedrate_mm_s;       // Save feedrate for the next G0
      #else
        old_feedrate = feedrate_mm_s;             // Back up the (new) motion mode feedrate
        feedrate_mm_s = MMM_TO_MMS(G0_FEEDRATE);  // Get the fixed G0 feedrate
      #endif
    }
  #endif

  #if ALL(FWRETRACT, FWRETRACT_AUTORETRACT)

    if (MIN_AUTORETRACT <= MAX_AUTORETRACT) {
      // When M209 Autoretract is enabled, convert E-only moves to firmware retract/recover moves
      if (fwretract.autoretract_enabled && parser.seen_test('E')
        && !parser.seen(STR_AXES_MAIN)
      ) {
        const float echange = destination.e - current_position.e;
        // Is this a retract or recover move?
        if (WITHIN(ABS(echange), MIN_AUTORETRACT, MAX_AUTORETRACT) && fwretract.retracted[active_extruder] == (echange > 0.0)) {
          current_position.e = destination.e;       // Hide a G1-based retract/recover from calculations
          sync_plan_position_e();                   // AND from the planner
          return fwretract.retract(echange < 0.0);  // Firmware-based retract/recover (double-retract ignored)
        }
      }
    }

  #endif // FWRETRACT

  #if ANY(IS_SCARA, POLAR)
    fast_move ? prepare_fast_move_to_destination() : prepare_line_to_destination();
  #else
    prepare_line_to_destination();
  #endif

  #ifdef G0_FEEDRATE
    // Restore the motion mode feedrate
    if (fast_move) feedrate_mm_s = old_feedrate;
  #endif

  #if ENABLED(NANODLP_Z_SYNC)
    #if ENABLED(NANODLP_ALL_AXIS)
      #define _MOVE_SYNC parser.seenval('X') || parser.seenval('Y') || parser.seenval('Z')  // For any move wait and output sync message
    #else
      #define _MOVE_SYNC parser.seenval('Z')  // Only for Z move
    #endif
    if (_MOVE_SYNC) {
      planner.synchronize();
      SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);
    }
    TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_IDLE));
  #else
    TERN_(FULL_REPORT_TO_HOST_FEATURE, report_current_grblstate_moving());
  #endif
}
