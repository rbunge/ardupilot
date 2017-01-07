
#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA_Custom_ControlLaw.h"
#include "AA_Flight_Sensors_Actuators.h"

// DEFINES

# define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)

#define  MANUAL_FLIGHT 1
#define  MANUAL_SPIN_APPROACH 2
#define  ARREST 3
#define  PULLOUT 4
#define  AUTO_LEVEL_FLIGHT 5
#define  MANUAL_AUTO_LEVEL_FLIGHT 6


// These functions are executed when control mode is in AUTO
// Please read AA241X_aux.h for all necessary definitions and interfaces

// *****   AA241X Fast Loop - @ ~50Hz   ***** //
static void AA241X_AUTO_FastLoop(void) {
  
  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if ((CPU_time_ms - Last_AUTO_stampTime_ms) > 100) {
    // Then we've just switched to AUTO, and need to save the flight data as the trim condition we want to control around. Used for PS2
    local_state = MANUAL_FLIGHT;
  }
  
  // Running state machine and appropriate controller
  run_state_machine();
  run_controller();
};


///////////////////////////////////////////////////////////////////////////////
///////////////////////// STATE MACHINE ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
static int8_t run_state_machine(void){

  // ARREST, PULLOUT, AUTO_LEVEL 
  if (aa_sm_mode == 1){
    switch (local_state) {
      case MANUAL_FLIGHT:
        local_state = ARREST;
        break;
      case ARREST:
        if (Arrest_Detected()){ local_state = PULLOUT;}
        break;
      case PULLOUT:
        if (Level_Flight_Condition()){ local_state = AUTO_LEVEL_FLIGHT; }
        break;
      case AUTO_LEVEL_FLIGHT:
        break;
      default:
        local_state = MANUAL_FLIGHT;
        break;
        
     }
  }
     
     // MANUAL_SPIN_APPROACH, ARREST, PULLOUT, AUTO_LEVEL
  if (aa_sm_mode == 2){
    switch (local_state) {
      case MANUAL_FLIGHT:
        local_state = MANUAL_SPIN_APPROACH;
        break;
      case MANUAL_SPIN_APPROACH:
        if (Spin_Detected()){ local_state = ARREST;}
        break;
      case ARREST:
        if (Arrest_Detected()){ local_state = PULLOUT;}
        break;
      case PULLOUT:
        if (Level_Flight_Condition()){ local_state = AUTO_LEVEL_FLIGHT; }
        break;
      case AUTO_LEVEL_FLIGHT:
        break;
      default:
        local_state = MANUAL_FLIGHT;
        break;
        
      }
     }
  
     // MANUAL_AUTO_LEVEL
     if (aa_sm_mode == 3){
        local_state = MANUAL_AUTO_LEVEL_FLIGHT;
     }
  
  
  return local_state;   
};


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// CONTTROLLERS ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
// Run controllers
static void run_controller(void){
 switch (local_state) {
  case MANUAL_FLIGHT:
     manual_flight_controller();
     break;
  case MANUAL_SPIN_APPROACH:
     manual_spin_approach_controller();
     break;
  case ARREST:
     spin_arrest_controller();
     break;
  case PULLOUT:
     pullout_controller();
     break;
  case AUTO_LEVEL_FLIGHT:
     auto_level_flight_controller();
     break;
  case MANUAL_AUTO_LEVEL_FLIGHT:
     manual_auto_level_controller();
     break;
  default:
     manual_flight_controller();
     break;
  }
};

//Manual flight controller
static void manual_flight_controller(void){
  
  Servo_Ch1_PWM = RC_In_Ch1_PWM;
  Servo_Ch2_PWM = RC_In_Ch2_PWM;
  Servo_Ch3_PWM = RC_In_Ch3_PWM;
  Servo_Ch4_PWM = RC_In_Ch4_PWM;
  Servo_Ch5_PWM = RC_In_Ch5_PWM;
  Servo_Ch6_PWM = RC_In_Ch6_PWM;
  Servo_Ch7_PWM = RC_In_Ch7_PWM;
     
};

//Manual spin approach flight controller
static void manual_spin_approach_controller(void){
  float elev_angle_max;
  
  // Baseline controls
  Servo_Ch1_PWM = RC_In_Ch1_PWM;
  Servo_Ch2_PWM = RC_In_Ch2_PWM;
  Servo_Ch3_PWM = RC_In_Ch3_PWM;
  Servo_Ch4_PWM = RC_In_Ch4_PWM;
  Servo_Ch5_PWM = RC_In_Ch5_PWM;
  Servo_Ch6_PWM = RC_In_Ch6_PWM;
  Servo_Ch7_PWM = RC_In_Ch7_PWM;
  
//  float elev_angle = pwm2angle_elevator(RC_In_Ch2_PWM);  // passthrough RC elevator
//  float da = aa_k_phi*(roll_att - A_roll_0*3.142/180.0) + aa_k_p*roll_Rate; // autolevel aileron
//  float dr = a_k_r*yaw_Rate;  // yaw     
};


//////////////////////////// SPIN ARREST CONTROLLER ////////////////////////////
static void spin_arrest_controller(void){
  // Set baseline control positions
  float da = 0;
  float da_left = 0;
  float da_right = 0;
  float elev_angle = arrest_elevator_angle;
  float dr =  sign_f(yaw_Rate);  // rudder  full anti-spin
  float dt = 0;
  float flap_angle = 0;
 
  // Set actuators from control variables 
  map_control_vars_to_actuators(float elev_angle, float da, float dr, float dt, float flap_angle)
};



//////////////////////////// PULLOUT CONTOLLER ////////////////////////////
static void pullout_controller(void){
  // Set baseline control positions
  float k_phi = aa_k_phi;
  float da = k_phi*(roll_att - A_roll_0*3.142/180.0);
  float elev_angle = pullout_elevator_angle;
  float dr = 0;
  float dt = 0;
  float flap_angle = 0;  // consider changing to flap_angle if indeed the control variable is an angle rather than a normalized throw 
  
  // Set actuators from control variables 
  map_control_vars_to_actuators(float elev_angle, float da, float dr, float dt, float flap_angle)
};


//////////////////////////// AUTO-LEVEL CONTOLLER ////////////////////////////
static void auto_level_flight_controller(void){
  float da = aa_k_phi*(roll_att - A_roll_0*3.142/180.0) + aa_k_p*roll_Rate;
  float elev_angle = 0;
  float dr = 0;
  float dt = A_dt_level;
  float flap_angle = 0;  // consider changing to flap_angle if indeed the control variable is an angle rather than a normalized throw 
  
  // Set actuators from control variables 
  map_control_vars_to_actuators(float elev_angle, float da, float dr, float dt, float flap_angle)
};


static void manual_auto_level_controller(void){
  float da = aa_k_phi*(roll_att - A_roll_0*3.142/180.0) + aa_k_p*roll_Rate;
  float elev_angle = 0;
  float dr = 0;
  float dt = A_dt_level;
  float flap_angle = 0;  // consider changing to flap_angle if indeed the control variable is an angle rather than a normalized throw 
  
  // Set actuators from control variables 
  map_control_vars_to_actuators(float elev_angle, float da, float dr, float dt, float flap_angle)
  
  // Overwrite all servos with RC PWM values, except for ailerons. This is configuration dependent! The only generic way is to compute the equivalent control variables from the RC_In_PWMs and then input these values in "map_control"
  Servo_Ch2_PWM = RC_In_Ch2_PWM;
  Servo_Ch3_PWM = RC_In_Ch3_PWM;
  Servo_Ch4_PWM = RC_In_Ch4_PWM;
  Servo_Ch5_PWM = RC_In_Ch5_PWM;
  Servo_Ch7_PWM = RC_In_Ch7_PWM;
};


// Elevator Normal Force Coefficient Controller
static float elevator_CN_controller(float CN_desired){
 return 0; 
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////// AUXILIARY FUNCTIONS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// *****   AA241X Medium Loop - @ ~10Hz  *****  //
static void AA241X_AUTO_MediumLoop(void){
    
  // YOUR CODE HERE
};
// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void){
  // YOUR CODE HERE
};

