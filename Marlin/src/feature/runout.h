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
#pragma once

/**
 * feature/runout.h - Runout sensor support
 */

#include "../sd/cardreader.h"
#include "../module/printcounter.h"
#include "../module/planner.h"
#include "../module/stepper.h" // for block_t
#include "../gcode/queue.h"
#include "../feature/pause.h"

#include "../inc/MarlinConfig.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

//#define FILAMENT_RUNOUT_SENSOR_DEBUG
#ifndef FILAMENT_RUNOUT_THRESHOLD
  #define FILAMENT_RUNOUT_THRESHOLD 5
#endif


static volatile uint16_t counter_change_all;
void event_filament_runout(const uint8_t extruder,uint8_t err_code);
void poll_runout_pins();
static void poll_runout_pins_enter(){poll_runout_pins();}
extern bool Filament_Runout_Check_Flag;


template<class RESPONSE_T, class SENSOR_T>
class TFilamentMonitor;
class FilamentSensorEncoder;
class FilamentSensorSwitch;
class RunoutResponseDelayed;
class RunoutResponseDebounced;

/********************************* TEMPLATE SPECIALIZATION *********************************/

typedef TFilamentMonitor<
          TERN(HAS_FILAMENT_RUNOUT_DISTANCE, RunoutResponseDelayed, RunoutResponseDebounced),
          TERN(FILAMENT_MOTION_SENSOR, FilamentSensorEncoder, FilamentSensorSwitch)
        > FilamentMonitor;

extern FilamentMonitor runout;

/*******************************************************************************************/

class FilamentMonitorBase {
  public:
	static bool enabled, filament_ran_out;
	static volatile  bool test_filament_ran_out;
	
	uint8_t Filament_Runout_AutoRecovery_State =0;
	uint8_t Filament_Runout_AutoRecovery_Cnt =0;
    #if ENABLED(HOST_ACTION_COMMANDS)
      static bool host_handling;
    #else
      static constexpr bool host_handling = false;
    #endif
};

template<class RESPONSE_T, class SENSOR_T>
class TFilamentMonitor : public FilamentMonitorBase {
  private:
    typedef RESPONSE_T response_t;
    typedef SENSOR_T   sensor_t;
    static  response_t response;
    static  sensor_t   sensor;

  public:
    static inline void setup() {
      sensor.setup();
      reset();
    }

	uint8_t get_has_run_out()	{return sensor.has_run_out();}
	uint8_t get_has_error_out()	{return sensor.get_error_out();}
	uint8_t clear_has_error_out()	{return sensor.clear_error_out();}
	uint8_t clear_new_error_out()	{return sensor.clear_Has_error_out();}

  uint8_t get_LCD_error_out()	{return sensor.get_LCD_error_out();}
	uint8_t clear_LCD_error_out()	{return sensor.clear_LCD_error_out();}
	
	void add_change()	{return sensor.add_change();}
    static inline void reset() {
      filament_ran_out = false;
      response.reset();
	  sensor.reset();
    }
	void setAutoRecovery_State(int8_t en) 	{ Filament_Runout_AutoRecovery_State =en;}
	uint8_t getAutoRecovery_State() 	{ return Filament_Runout_AutoRecovery_State;}
	
	void addRecoveryCnt_State()	{ Filament_Runout_AutoRecovery_Cnt++;}
	uint8_t getRecoveryCnt_State()		{ return Filament_Runout_AutoRecovery_Cnt;}
	void clearRecoveryCnt_State() 	{  Filament_Runout_AutoRecovery_Cnt =0;}
	void setFilamentRunout_enabled(uint8_t en)
	{
		#if 1 //modify by zb
		if(((enabled&0x01)==0)&&((en&0x01)!=0))
		{
			attachInterrupt(FIL_RUNOUT1_PIN,poll_runout_pins_enter,CHANGE);
		}
		enabled=en;
		#endif
	}

    // Call this method when filament is present,
    // so the response can reset its counter.
    static inline void filament_present(const uint8_t extruder) {
      response.filament_present(extruder);
    }

    #if HAS_FILAMENT_RUNOUT_DISTANCE
      static inline float& runout_distance() { return response.runout_distance_mm; }
      static inline void set_runout_distance(const_float_t mm) { response.runout_distance_mm = mm; }
    #endif

    // Handle a block completion. RunoutResponseDelayed uses this to
    // add up the length of filament moved while the filament is out.
    static inline void block_completed(const block_t * const b) {

      if (getFilamentEnableState()!=0) {
        response.block_completed(b);
        sensor.block_completed(b);
      }
    }
	static inline uint8_t getFilamentEnableState() {
		uint8_t FialmentEnable =0;
		FialmentEnable = ((test_filament_enable)|((test_jams_enable)<<1));
		return FialmentEnable;
	}

    // Give the response a chance to update its counter.
    static inline void run() {

	if(getFilamentEnableState()!=0 && !filament_ran_out && (printingIsActive() || did_pause_print) ){//|| test_filament_ran_out)
        TERN_(HAS_FILAMENT_RUNOUT_DISTANCE, cli()); // Prevent RunoutResponseDelayed::block_completed from accumulating here
        response.run();
        sensor.run();
        uint8_t runout_flags ;//= sensor.has_run_out();
		if((sensor.has_run_out() & 0x03)!=0)
			runout_flags=0xff;
		else
			runout_flags=0;
		  
        TERN_(HAS_FILAMENT_RUNOUT_DISTANCE, sei());
        #if MULTI_FILAMENT_SENSOR
          #if ENABLED(WATCH_ALL_RUNOUT_SENSORS)
            const bool ran_out = !!runout_flags;  // any sensor triggers
            uint8_t extruder = 0;
            if (ran_out) {
              uint8_t bitmask = runout_flags;
              while (!(bitmask & 1)) {
                bitmask >>= 1;
                extruder++;
              }
            }
          #else
            const bool ran_out = TEST(runout_flags, active_extruder);  // suppress non active extruders
            uint8_t extruder = active_extruder;
          #endif
        #else
          const bool ran_out = !!runout_flags;
          uint8_t extruder = active_extruder;
        #endif

        #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
          if (runout_flags) {
            SERIAL_ECHOPGM("Runout Sensors: ");
            LOOP_L_N(i, 8) SERIAL_ECHO('0' + TEST(runout_flags, i));
            SERIAL_ECHOPAIR(" -> ", extruder);
            if (ran_out) SERIAL_ECHOPGM(" RUN OUT");
            SERIAL_EOL();
          }
        #endif


        if (ran_out) {
					
          filament_ran_out = true;
          event_filament_runout(extruder,runout_flags);
          planner.synchronize();
        }
      }
    }
};

/*************************** FILAMENT PRESENCE SENSORS ***************************/

class FilamentSensorBase {
  protected:
    /**
     * Called by FilamentSensorSwitch::run when filament is detected.
     * Called by FilamentSensorEncoder::block_completed when motion is detected.
     */
    static inline void filament_present(const uint8_t extruder) {
      runout.filament_present(extruder); // ...which calls response.filament_present(extruder)
    }

  public:
    static inline void setup() {
      #define _INIT_RUNOUT_PIN(P,S,U,D) do{ if (ENABLED(U)) SET_INPUT_PULLUP(P); else if (ENABLED(D)) SET_INPUT_PULLDOWN(P); else SET_INPUT(P); }while(0)
      #define  INIT_RUNOUT_PIN(N) _INIT_RUNOUT_PIN(FIL_RUNOUT##N##_PIN, FIL_RUNOUT##N##_STATE, FIL_RUNOUT##N##_PULLUP, FIL_RUNOUT##N##_PULLDOWN)
      #if NUM_RUNOUT_SENSORS >= 1
        INIT_RUNOUT_PIN(1);
		#if 1
	    pinMode(FIL_RUNOUT1_PIN,INPUT_PULLUP);
		attachInterrupt(FIL_RUNOUT1_PIN,poll_runout_pins_enter,CHANGE);
		#endif
      #endif
      #if NUM_RUNOUT_SENSORS >= 2
        INIT_RUNOUT_PIN(2);
      #endif
      #if NUM_RUNOUT_SENSORS >= 3
        INIT_RUNOUT_PIN(3);
      #endif
      #if NUM_RUNOUT_SENSORS >= 4
        INIT_RUNOUT_PIN(4);
      #endif
      #if NUM_RUNOUT_SENSORS >= 5
        INIT_RUNOUT_PIN(5);
      #endif
      #if NUM_RUNOUT_SENSORS >= 6
        INIT_RUNOUT_PIN(6);
      #endif
      #if NUM_RUNOUT_SENSORS >= 7
        INIT_RUNOUT_PIN(7);
      #endif
      #if NUM_RUNOUT_SENSORS >= 8
        INIT_RUNOUT_PIN(8);
      #endif
      #undef _INIT_RUNOUT_PIN
      #undef  INIT_RUNOUT_PIN
    }
#if 0
    // Return a bitmask of runout pin states
    static inline void poll_runout_pins() {
    #if 0
      #define _OR_RUNOUT(N) | (READ(FIL_RUNOUT##N##_PIN) ? _BV((N) - 1) : 0)
      return (0 REPEAT_1(NUM_RUNOUT_SENSORS, _OR_RUNOUT));
      #undef _OR_RUNOUT
	  #endif
	  interrupt_timer=millis();
	  if(interrupt_timer-interrupt_timer_pre>10)
	  {
		  uint16_t aa=&counter_change_all;
	  	counter_change_all++;	 
		
	  
	  }
	  interrupt_timer_pre=interrupt_timer;
	 // return;
    }
#endif
    // Return a bitmask of runout flag states (1 bits always indicates runout)
    static inline uint8_t poll_runout_states() {
    #if 0
      return poll_runout_pins() ^ uint8_t(0
        #if NUM_RUNOUT_SENSORS >= 1
          | (FIL_RUNOUT1_STATE ? 0 : _BV(1 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 2
          | (FIL_RUNOUT2_STATE ? 0 : _BV(2 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 3
          | (FIL_RUNOUT3_STATE ? 0 : _BV(3 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 4
          | (FIL_RUNOUT4_STATE ? 0 : _BV(4 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 5
          | (FIL_RUNOUT5_STATE ? 0 : _BV(5 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 6
          | (FIL_RUNOUT6_STATE ? 0 : _BV(6 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 7
          | (FIL_RUNOUT7_STATE ? 0 : _BV(7 - 1))
        #endif
        #if NUM_RUNOUT_SENSORS >= 8
          | (FIL_RUNOUT8_STATE ? 0 : _BV(8 - 1))
        #endif
      );
		#endif
		return 0;
    }
};

#if ENABLED(FILAMENT_MOTION_SENSOR)

  /**
   * This sensor uses a magnetic encoder disc and a Hall effect
   * sensor (or a slotted disc and optical sensor). The state
   * will toggle between 0 and 1 on filament movement. It can detect
   * filament runout and stripouts or jams.
   */
  class FilamentSensorEncoder : public FilamentSensorBase {
    private:
      static uint8_t motion_detected;
      static inline uint8_t poll_motion_sensor() {
	  	#if 0
        static uint8_t old_state;
		static uint16_t tknum=0;;
	//	if(!TEST(direction_bits, E_AXIS))
			{
	//	if(tknum++==10000)		do{ Serial.print("11counter_change="); Serial.println(counter_change); }while(0);
        const uint8_t new_state = poll_runout_pins();	
		const uint8_t change    = old_state ^ new_state;
        old_state = new_state;
		if(change)
		{
			counter_change++;
		}
	    #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
	        if (change) {
	          SERIAL_ECHOPGM("Motion detected:");
	          LOOP_L_N(e, NUM_RUNOUT_SENSORS)
	          if (TEST(change, e)) SERIAL_CHAR(' ', '0' + e);
	            SERIAL_EOL();
	        }
	     #endif
	     motion_detected |= change;
		}
	#endif
		;
		return counter_change;
      }

    public:
	  static volatile int8_t counter_change;
	  static uint8_t direction_bits;
	  static uint8_t filament_err_num;
	  static uint8_t Has_Filament_Err;
	  static uint8_t Get_Filament_Err;
    static uint8_t LCD_Filament_Err;
    
	  static uint8_t enter_filament_Num;    
	  static uint8_t enter_filament_Flag;
  	static uint8_t back_fila_Num;	
	  static uint8_t back_fila_Flag;
	  static uint8_t fila_error;

      static volatile float runout_mm_countdown[NUM_RUNOUT_SENSORS];
      static inline uint8_t has_run_out() { //yes
				
        return Has_Filament_Err;
      }
      static inline uint8_t get_error_out() { //yes
        return Get_Filament_Err;
      }			
      static inline uint8_t clear_error_out() { //yes
        Get_Filament_Err =0;
      }		

      static inline uint8_t get_LCD_error_out() { //yes
        return LCD_Filament_Err;
      }			
      static inline uint8_t clear_LCD_error_out() { //yes
        LCD_Filament_Err =0;
      }		
       static inline uint8_t clear_Has_error_out() { //yes
       
        Has_Filament_Err =0;
        Get_Filament_Err =0;
        enter_filament_Num =0;
        enter_filament_Flag =0;
      }	     		
	  static void add_change() {
        counter_change++;
      }
      static inline void block_completed(const block_t * const b) {
        
      static millis_t get_cur_time=0,get_last_time=0;  
        
			
	  	direction_bits=b->direction_bits;

        if (did_pause_print||b->steps.e) { // Allow pause purge move to re-trigger runout state//b->steps.x || b->steps.y || b->steps.z || 
          // Only trigger on extrusion with XYZ movement to allow filament change and retract/recover.
          const uint8_t e_axis = b->extruder;
          const int32_t steps = b->steps.e;
           
		  if(TEST(b->direction_bits, 3U))  {//2 
			  
		  }
		  else  //进料
		  {			  	
			 
	      runout_mm_countdown[e_axis] += abs(steps * planner.steps_to_mm[E_AXIS_N(e_axis)]);
             
			  if(runout_mm_countdown[e_axis] >= FILAMENT_RUNOUT_DISTANCE_MM )
			  {			
          int8_t tmp=(uint8_t)(runout_mm_countdown[e_axis]/FILAMENT_RUNOUT_DISTANCE_MM);
          tmp++;
          runout_mm_countdown[e_axis]=0;
			  	if((counter_change == 0 )/*(counter_change >=0)&& (counter_change<=2)*/&& (test_filament_enable) )
			  	{

            filament_err_num++;
            counter_change=0;
            if((filament_err_num>(RUNOUT_FAST_PRINT_CNT))||(Filament_Runout_Check_Flag))//DAVID 
            {
                Filament_Runout_Check_Flag =false;
                filament_err_num=0;
                Has_Filament_Err=1;//return 1;//no fialment
                Get_Filament_Err =Has_Filament_Err;
                LCD_Filament_Err = Has_Filament_Err;
            }
			  	}else{
         
            if(abs(counter_change) <= abs(tmp))
            {
              if(enter_filament_Flag)
              {
                enter_filament_Flag =0;
                enter_filament_Num++;
                if(enter_filament_Num>=(RUNOUT_FAST_PRINT_CNT+1)) //zb
                {
                  Has_Filament_Err=1;
				          enter_filament_Num =0 ; //
                  Get_Filament_Err =Has_Filament_Err;
                  LCD_Filament_Err = Has_Filament_Err;
                  goto FIAMENTPROC;
                
                }
              }
            }
            else  
            {
              enter_filament_Num =0;
              enter_filament_Flag =0;
            }
           


            if(counter_change>tmp*5)
            {

              filament_err_num=0;
              Has_Filament_Err=0;           
              counter_change=0;           
              
            }
            else if(/*(counter_change>tmp+2|| counter_change<tmp)*/(abs(counter_change-tmp)>2)&& (test_jams_enable ) )
            {
        
              enter_filament_Flag =1;
              filament_err_num++;
              enter_filament_Flag =1;             
              counter_change=0;
              
              if(filament_err_num>=RUNOUT_FAST_PRINT_CNT /*||(ScreenHandler.Filament_Runout_Check_Flag)*/)
              {
               
                filament_err_num=0;
                //Has_Filament_Err=2;//2;   //
                //LCD_Filament_Err = Has_Filament_Err;
                //Has_Filament_Err=0;
              }
            }
            else
            {

              filament_err_num=0;
              Has_Filament_Err=0;
              counter_change=0;
            }


			  	}

				
			  }



		  }  
    }
		    FIAMENTPROC:
		  
			  if(enter_filament_Num>0)
			  {
          get_cur_time=millis();
          if(get_cur_time-get_last_time<30000)//30S
          {
            get_cur_time =get_cur_time;
            
          }
          else 
          {
            get_last_time=get_cur_time;
            enter_filament_Num =0;
            enter_filament_Flag =0;
            
          }  
			  }
			  else
			  {
		  
				  get_last_time=get_cur_time;
			  }  
			  // If the sensor wheel has moved since the last call to
			  // this method reset the runout counter for the extruder.
			  if (TEST(motion_detected, b->extruder))
				filament_present(b->extruder);
			  
			  // Clear motion triggers for next block
			  motion_detected = 0;
			  


		  
        }

 


		
      
	 // static void clean_counter_change(){counter_change=0;}
	  void reset() {
	  
      filament_err_num=0;
      //counter_change=0;
      counter_change=0;
      runout_mm_countdown[0]=0;//modify by zb
      Has_Filament_Err=0;
      enter_filament_Flag =0;
      enter_filament_Num =0;
		
      }
      static inline uint8_t run() {return counter_change;}//{ return poll_motion_sensor(); }
  };

#else

  /**
   * This is a simple endstop switch in the path of the filament.
   * It can detect filament runout, but not stripouts or jams.
   */
  class FilamentSensorSwitch : public FilamentSensorBase {
    private:
      static inline bool poll_runout_state(const uint8_t extruder) {
        const uint8_t runout_states = poll_runout_states();
        #if MULTI_FILAMENT_SENSOR
          if ( !TERN0(DUAL_X_CARRIAGE, idex_is_duplicating())
            && !TERN0(MULTI_NOZZLE_DUPLICATION, extruder_duplication_enabled)
          ) return TEST(runout_states, extruder); // A specific extruder ran out
        #else
          UNUSED(extruder);
        #endif
        return !!runout_states;                   // Any extruder ran out
      }

    public:
      static inline void block_completed(const block_t * const) {}

      static inline void run() {
        LOOP_L_N(s, NUM_RUNOUT_SENSORS) {
          const bool out = poll_runout_state(s);
          if (!out) filament_present(s);
          #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
            static uint8_t was_out; // = 0
            if (out != TEST(was_out, s)) {
              TBI(was_out, s);
              SERIAL_ECHOLNPAIR_P(PSTR("Filament Sensor "), '0' + s, out ? PSTR(" OUT") : PSTR(" IN"));
            }
          #endif
        }
      }
  };


#endif // !FILAMENT_MOTION_SENSOR

/********************************* RESPONSE TYPE *********************************/

#if HAS_FILAMENT_RUNOUT_DISTANCE

  // RunoutResponseDelayed triggers a runout event only if the length
  // of filament specified by FILAMENT_RUNOUT_DISTANCE_MM has been fed
  // during a runout condition.
  class RunoutResponseDelayed {
    private:
      //static volatile float runout_mm_countdown[NUM_RUNOUT_SENSORS];

    public:
      static float runout_distance_mm;

      static inline void reset() {
        LOOP_L_N(i, NUM_RUNOUT_SENSORS) filament_present(i);
      }

      static inline void run() {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR_DEBUG)
          static millis_t t = 0;
          const millis_t ms = millis();
          if (ELAPSED(ms, t)) {
            t = millis() + 1000UL;
//            LOOP_L_N(i, NUM_RUNOUT_SENSORS)
//              SERIAL_ECHOPAIR_P(i ? PSTR(", ") : PSTR("Remaining mm: "), runout_mm_countdown[i]);
//            SERIAL_EOL();
          }
        #endif
      }

      static inline uint8_t has_run_out() {
				
        uint8_t runout_flags = 0;
        //LOOP_L_N(i, NUM_RUNOUT_SENSORS) if (runout_mm_countdown[i] < 0) SBI(runout_flags, i);
        return runout_flags;
      }

      static inline void filament_present(const uint8_t extruder) {
        //runout_mm_countdown[extruder] = runout_distance_mm;
      }

      static inline void block_completed(const block_t * const b) {
        if (b->steps.x || b->steps.y || b->steps.z || did_pause_print) { // Allow pause purge move to re-trigger runout state
          // Only trigger on extrusion with XYZ movement to allow filament change and retract/recover.
          const uint8_t e = b->extruder;
          const int32_t steps = b->steps.e;
          //runout_mm_countdown[e] -= (TEST(b->direction_bits, E_AXIS) ? -steps : steps) * planner.steps_to_mm[E_AXIS_N(e)];
        }
      }
  };

#else // !HAS_FILAMENT_RUNOUT_DISTANCE

  // RunoutResponseDebounced triggers a runout event after a runout
  // condition has been detected runout_threshold times in a row.

  class RunoutResponseDebounced {
    private:
      static constexpr int8_t runout_threshold = FILAMENT_RUNOUT_THRESHOLD;
      static int8_t runout_count[NUM_RUNOUT_SENSORS];

    public:
      static inline void reset() {
        LOOP_L_N(i, NUM_RUNOUT_SENSORS) filament_present(i);
      }

      static inline void run() {
        LOOP_L_N(i, NUM_RUNOUT_SENSORS) if (runout_count[i] >= 0) runout_count[i]--;
      }

      static inline uint8_t has_run_out() {
				
        uint8_t runout_flags = 0;
        LOOP_L_N(i, NUM_RUNOUT_SENSORS) if (runout_count[i] < 0) SBI(runout_flags, i);
        return runout_flags;
      }

      static inline void block_completed(const block_t * const) { }

      static inline void filament_present(const uint8_t extruder) {
        runout_count[extruder] = runout_threshold;
      }
  };

#endif // !HAS_FILAMENT_RUNOUT_DISTANCE
