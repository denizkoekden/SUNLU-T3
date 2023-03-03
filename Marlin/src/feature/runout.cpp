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

/**
 * feature/runout.cpp - Runout sensor support
 */

#include "../inc/MarlinConfigPre.h"
#include "../lcd/marlinui.h"

#if HAS_FILAMENT_SENSOR

#include "runout.h"

FilamentMonitor runout;

static uint16_t interrupt_timer_pre,interrupt_timer;
bool FilamentMonitorBase::enabled = true,
     FilamentMonitorBase::filament_ran_out;  // = false

volatile bool FilamentMonitorBase::test_filament_ran_out=false;  // = false

#if ENABLED(HOST_ACTION_COMMANDS)
  bool FilamentMonitorBase::host_handling; // = false
#endif

#if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)
  #include "../module/tool_change.h"
  #define DEBUG_OUT ENABLED(DEBUG_TOOLCHANGE_MIGRATION_FEATURE)
  #include "../core/debug_out.h"
#endif

#if HAS_FILAMENT_RUNOUT_DISTANCE
  float RunoutResponseDelayed::runout_distance_mm = FILAMENT_RUNOUT_DISTANCE_MM;
  volatile float FilamentSensorEncoder::runout_mm_countdown[NUM_RUNOUT_SENSORS]={0};
  #if ENABLED(FILAMENT_MOTION_SENSOR)
    uint8_t FilamentSensorEncoder::motion_detected=0;
  volatile	int8_t FilamentSensorEncoder::counter_change=0;
	uint8_t FilamentSensorEncoder::direction_bits=0;
	uint8_t FilamentSensorEncoder::filament_err_num=0;
	uint8_t FilamentSensorEncoder::Has_Filament_Err=0;
	uint8_t FilamentSensorEncoder::Get_Filament_Err=0;
  uint8_t FilamentSensorEncoder::LCD_Filament_Err=0;
	uint8_t FilamentSensorEncoder::enter_filament_Num=0;
  	uint8_t FilamentSensorEncoder::enter_filament_Flag=0;
    	uint8_t FilamentSensorEncoder::back_fila_Num=0;
      	uint8_t FilamentSensorEncoder::back_fila_Flag=0;
        uint8_t FilamentSensorEncoder::fila_error=0;

	
  #endif
#else
  int8_t RunoutResponseDebounced::runout_count[NUM_RUNOUT_SENSORS]; // = 0
#endif


//
// Filament Runout event handler
//
#include "../MarlinCore.h"
#include "../feature/pause.h"
#include "../gcode/queue.h"

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "host_actions.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

void event_filament_runout(const uint8_t extruder,uint8_t err_code) {

  if (did_pause_print) return;  // Action already in progress. Purge triggered repeated runout.

  #if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)
    if (migration.in_progress) {
      DEBUG_ECHOLNPGM("Migration Already In Progress");
      return;  // Action already in progress. Purge triggered repeated runout.
    }
    if (migration.automode) {
      DEBUG_ECHOLNPGM("Migration Starting");
      if (extruder_migration()) return;
    }
  #endif

  TERN_(EXTENSIBLE_UI, ExtUI::onFilamentRunout(ExtUI::getTool(extruder)));

  #if ANY(HOST_PROMPT_SUPPORT, HOST_ACTION_COMMANDS, MULTI_FILAMENT_SENSOR)
    const char tool = '0' + TERN0(MULTI_FILAMENT_SENSOR, extruder);
  #endif

  //action:out_of_filament
  #if ENABLED(HOST_PROMPT_SUPPORT)
    host_action_prompt_begin(PROMPT_FILAMENT_RUNOUT, PSTR("FilamentRunout T"), tool);
    host_action_prompt_show();
  #endif

  const bool run_runout_script = !runout.host_handling;

  #if ENABLED(HOST_ACTION_COMMANDS)
    if (run_runout_script
      && ( strstr(FILAMENT_RUNOUT_SCRIPT, "M600")
        || strstr(FILAMENT_RUNOUT_SCRIPT, "M125")
        || TERN0(ADVANCED_PAUSE_FEATURE, strstr(FILAMENT_RUNOUT_SCRIPT, "M25"))
      )
    ) {
      host_action_paused(false);
    }
    else {
      // Legacy Repetier command for use until newer version supports standard dialog
      // To be removed later when pause command also triggers dialog
      #ifdef ACTION_ON_FILAMENT_RUNOUT
        host_action(PSTR(ACTION_ON_FILAMENT_RUNOUT " T"), false);
        SERIAL_CHAR(tool);
        SERIAL_EOL();
      #endif

      host_action_pause(false);
    }
    SERIAL_ECHOPGM(" " ACTION_REASON_ON_FILAMENT_RUNOUT " ");
    SERIAL_CHAR(tool);
    SERIAL_EOL();
  #endif // HOST_ACTION_COMMANDS

  if (run_runout_script) {
    #if MULTI_FILAMENT_SENSOR
      char script[strlen(FILAMENT_RUNOUT_SCRIPT) + 1];
      sprintf_P(script, PSTR(FILAMENT_RUNOUT_SCRIPT), tool);
      #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
        SERIAL_ECHOLNPAIR("Runout Command: ", script);
      #endif
      queue.inject(script);
	  
	  
    #else
      #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
        SERIAL_ECHOPGM("Runout Command: ");
        SERIAL_ECHOLNPGM(FILAMENT_RUNOUT_SCRIPT);
      #endif
	  runout.setAutoRecovery_State(1);
	  runout.addRecoveryCnt_State();
	  
	  ui.pause_print();
    runout.clear_new_error_out();

    #endif
  }
}
void poll_runout_pins() {

  runout.add_change();

}

#endif // HAS_FILAMENT_SENSOR
