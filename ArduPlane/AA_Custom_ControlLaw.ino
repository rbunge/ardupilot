
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

////////////// 1/5-SUPER CUB, THROWS AND SERVO PWM VALUES
//// CONTROL SURFACE THROWS
//#define ELEVATOR_FULL_UP_ANGLE 41.0
//#define ELEVATOR_FULL_DOWN_ANGLE 30.0
//#define RUDDER_FULL_LEFT_ANGLE 27.0
//#define RUDDER_FULL_RIGHT_ANGLE 28.0
//#define RIGHT_AIELERON_FULL_LEFT_ANGLE 28.0
//#define RIGHT_AIELERON_FULL_RIGHT_ANGLE 32.0
//#define LEFT_AIELERON_FULL_LEFT_ANGLE 34.0
//#define LEFT_AIELERON_FULL_RIGHT_ANGLE 28.0
//#define FLAP_FULL_DOWN_ANGLE 14.5
//
//// PWM TRIM VALUES
//#define ELEVATOR_TRIM_PWM 1543     // In Apr 3 and 4, this used to be 1528 in the code
//#define RUDDER_TRIM_PWM 1451
//#define RIGHT_AILERON_TRIM_PWM 1509
//#define LEFT_AILERON_TRIM_PWM 1481
//#define FLAP_TRIM_PWM 1735
//
//// PWM THROW VALUES
//#define ELEVATOR_FULL_UP_PWM 1000    // Apr 3 and 4: 984
//#define ELEVATOR_FULL_DOWN_PWM 1942  // Apr 3 and 4: 1926
//#define RUDDER_FULL_LEFT_PWM 1847
//#define RUDDER_FULL_RIGHT_PWM 1067
//#define RIGHT_AIELERON_FULL_LEFT_PWM 1864
//#define RIGHT_AIELERON_FULL_RIGHT_PWM 1120
//#define LEFT_AIELERON_FULL_LEFT_PWM 1871
//#define LEFT_AIELERON_FULL_RIGHT_PWM 1124
//#define FLAP_FULL_DOWN_PWM 1516
//#define THROTTLE_OFF_PWM 1190
//#define THROTTLE_MAX_PWM 1918


//////////// 1/4-SUPER CUB, THROWS AND SERVO PWM VALUES
// CONTROL SURFACE THROWS
#define ELEVATOR_FULL_UP_ANGLE 41.0
#define ELEVATOR_FULL_DOWN_ANGLE 30.0
#define RUDDER_FULL_LEFT_ANGLE 27.0
#define RUDDER_FULL_RIGHT_ANGLE 28.0
#define RIGHT_AIELERON_FULL_LEFT_ANGLE 28.0
#define RIGHT_AIELERON_FULL_RIGHT_ANGLE 32.0
#define LEFT_AIELERON_FULL_LEFT_ANGLE 34.0
#define LEFT_AIELERON_FULL_RIGHT_ANGLE 28.0
#define FLAP_FULL_DOWN_ANGLE 14.5

// PWM TRIM VALUES
#define ELEVATOR_TRIM_PWM 1543     // In Apr 3 and 4, this used to be 1528 in the code
#define RUDDER_TRIM_PWM 1451
#define RIGHT_AILERON_TRIM_PWM 1509
#define LEFT_AILERON_TRIM_PWM 1481
#define FLAP_TRIM_PWM 1735

// PWM THROW VALUES
#define ELEVATOR_FULL_UP_PWM 1000    // Apr 3 and 4: 984
#define ELEVATOR_FULL_DOWN_PWM 1942  // Apr 3 and 4: 1926
#define RUDDER_FULL_LEFT_PWM 1847
#define RUDDER_FULL_RIGHT_PWM 1067
#define RIGHT_AIELERON_FULL_LEFT_PWM 1864
#define RIGHT_AIELERON_FULL_RIGHT_PWM 1120
#define LEFT_AIELERON_FULL_LEFT_PWM 1871
#define LEFT_AIELERON_FULL_RIGHT_PWM 1124
#define FLAP_FULL_DOWN_PWM 1516
#define THROTTLE_OFF_PWM 1190
#define THROTTLE_MAX_PWM 1918

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

/////////////
// RIGHT AILERON
static int16_t angle2pwm_right_aileron(float angle){
  // Positive aileron angle deflection induces a left rolling moment (right aileron trailing edge down)
  return angle2pwm(angle, RIGHT_AILERON_TRIM_PWM, RIGHT_AIELERON_FULL_LEFT_PWM, RIGHT_AIELERON_FULL_RIGHT_PWM, RIGHT_AIELERON_FULL_LEFT_ANGLE, RIGHT_AIELERON_FULL_RIGHT_ANGLE);
};

static int16_t fraction_deflection2pwm_right_aileron(float fraction_deflection){
  // Positive aileron angle deflection induces a left rolling moment (right aileron trailing edge down)
  return fraction_deflection2pwm(fraction_deflection, RIGHT_AILERON_TRIM_PWM, RIGHT_AIELERON_FULL_LEFT_PWM, RIGHT_AIELERON_FULL_RIGHT_PWM, RIGHT_AIELERON_FULL_LEFT_ANGLE, RIGHT_AIELERON_FULL_RIGHT_ANGLE);
};

static float pwm2angle_right_aileron(float ch1_pwm){
  static float trim_pwm = float(RIGHT_AILERON_TRIM_PWM);
  static float positive_throw_pwm = float(RIGHT_AIELERON_FULL_LEFT_PWM);
  static float negative_throw_pwm = float(RIGHT_AIELERON_FULL_RIGHT_PWM);  
  static float positive_throw_angle = RIGHT_AIELERON_FULL_LEFT_ANGLE;
  static float negative_throw_angle = RIGHT_AIELERON_FULL_RIGHT_ANGLE;
  static float angle;
  
  if ( (ch1_pwm - trim_pwm)*(positive_throw_pwm - trim_pwm) >= 0){
    // Same sign, deflection was towards positive throw direction
    angle =  fabsf(((ch1_pwm - trim_pwm)/(positive_throw_pwm - trim_pwm)))*positive_throw_angle;
  } else {
    // Different sign, deflection was towards negative throw direction
    angle = -fabsf(((ch1_pwm - trim_pwm)/(negative_throw_pwm - trim_pwm)))*negative_throw_angle;
  }
  return angle;
};

/////////////
// LEFT AILERON
static int16_t angle2pwm_left_aileron(float angle){
  // Positive aileron angle deflection induces a left rolling moment (left aileron trailing edge up)
  return angle2pwm(angle, LEFT_AILERON_TRIM_PWM, LEFT_AIELERON_FULL_LEFT_PWM, LEFT_AIELERON_FULL_RIGHT_PWM, LEFT_AIELERON_FULL_LEFT_ANGLE, LEFT_AIELERON_FULL_RIGHT_ANGLE);
};

static int16_t fraction_deflection2pwm_left_aileron(float fraction_deflection){
  // Positive aileron angle deflection induces a left rolling moment (left aileron trailing edge up)
  return fraction_deflection2pwm(fraction_deflection, LEFT_AILERON_TRIM_PWM, LEFT_AIELERON_FULL_LEFT_PWM, LEFT_AIELERON_FULL_RIGHT_PWM, LEFT_AIELERON_FULL_LEFT_ANGLE, LEFT_AIELERON_FULL_RIGHT_ANGLE);
};



/////
// ELEVATOR
static int16_t angle2pwm_elevator(float angle){
  // Positive elevator deflection induces a pitch down moment (elevator trailing edge down)
  return angle2pwm(angle, ELEVATOR_TRIM_PWM, ELEVATOR_FULL_DOWN_PWM, ELEVATOR_FULL_UP_PWM, ELEVATOR_FULL_DOWN_ANGLE, ELEVATOR_FULL_UP_ANGLE);
};

static float pwm2angle_elevator(float ch2_pwm){
  static float trim_pwm = float(ELEVATOR_TRIM_PWM);
  static float positive_throw_pwm = float(ELEVATOR_FULL_DOWN_PWM);
  static float negative_throw_pwm = float(ELEVATOR_FULL_UP_PWM);  
  static float positive_throw_angle = ELEVATOR_FULL_DOWN_ANGLE;
  static float negative_throw_angle = ELEVATOR_FULL_UP_ANGLE;
  static float angle;
  
  if ( (ch2_pwm - trim_pwm)*(positive_throw_pwm - trim_pwm) >= 0){
    // Same sign, deflection was towards positive throw direction
    angle =  fabsf(((ch2_pwm - trim_pwm)/(positive_throw_pwm - trim_pwm)))*positive_throw_angle;
  } else {
    // Different sign, deflection was towards negative throw direction
    angle = -fabsf(((ch2_pwm - trim_pwm)/(negative_throw_pwm - trim_pwm)))*negative_throw_angle;
  }
  return angle;
};


////////
// RUDDER
static int16_t angle2pwm_rudder(float angle){
  // Positive rudder angle deflection induces a left yawing moment (rudder trailing edge left)
  return angle2pwm(angle, RUDDER_TRIM_PWM, RUDDER_FULL_LEFT_PWM, RUDDER_FULL_RIGHT_PWM, RUDDER_FULL_LEFT_ANGLE, RUDDER_FULL_RIGHT_ANGLE);
};

static int16_t fraction_deflection2pwm_rudder(float fraction_deflection){
  // Positive aileron angle deflection induces a left rolling moment (left aileron trailing edge up)
  return fraction_deflection2pwm(fraction_deflection, RUDDER_TRIM_PWM, RUDDER_FULL_LEFT_PWM, RUDDER_FULL_RIGHT_PWM, RUDDER_FULL_LEFT_ANGLE, RUDDER_FULL_RIGHT_ANGLE);
};

//////
// FLAP
static int16_t angle2pwm_flap(float angle){
  // Positive flap angle deflection causes a trailing edge down deflection
  return angle2pwm(angle, FLAP_TRIM_PWM, FLAP_FULL_DOWN_PWM, FLAP_TRIM_PWM, FLAP_FULL_DOWN_ANGLE, 1.0);
};

////////
// THROTTLE
static int16_t fraction2pwm_throttle(float throttle_fraction_command){
  int16_t pwm = THROTTLE_OFF_PWM + throttle_fraction_command*(THROTTLE_MAX_PWM - THROTTLE_OFF_PWM);
  return pwm;
};

////////////////////////////
// LOWER LEVEL AUX FUNCTIONS
static int16_t angle2pwm(float angle, int16_t trim_pwm, int16_t positive_throw_pwm, int16_t negative_throw_pwm, float positive_throw_angle, float negative_throw_angle){
  static float fraction_deflection;
  static int16_t pwm;
  if (angle >= 0){
    fraction_deflection = fabsf(angle)/positive_throw_angle;
    pwm =  trim_pwm + (positive_throw_pwm - trim_pwm)*fraction_deflection;
  } else {
    fraction_deflection = fabsf(angle)/negative_throw_angle;
    pwm =  trim_pwm + (negative_throw_pwm -  trim_pwm)*fraction_deflection; 
  }
  
  return pwm;
}

static int16_t fraction_deflection2pwm(float fraction_deflection, int16_t trim_pwm, int16_t positive_throw_pwm, int16_t negative_throw_pwm, float positive_throw_angle, float negative_throw_angle){
  static int16_t pwm;
  if (fraction_deflection >= 0){
    pwm =  trim_pwm + (positive_throw_pwm - trim_pwm)*fabsf(fraction_deflection);
  } else {
    pwm =  trim_pwm + (negative_throw_pwm -  trim_pwm)*fabsf(fraction_deflection); 
  }
  
  return pwm;
}

float sign_f(float f){
  if (f > 0) {return  1;}
  if (f < 0) {return -1;}
  return 1;
}

/////////////////////////////////////////////////////////////
/// MAPPING BETWWEEN CONTROL VARIABLES AND ACTUATOR COMMANDS
/////////////////////////////////////////////////////////////
void map_control_vars_to_actuators(float elev_angle, float da, float dr, float dt, float flap_angle){
  
  // 1/4-Super Cub
  Servo_Ch1_PWM = fraction_deflection2pwm_left_aileron( da );
  Servo_Ch2_PWM = angle2pwm_left_elevator( elev_angle );
  Servo_Ch3_PWM = fraction2pwm_throttle( dt );
  Servo_Ch4_PWM = fraction_deflection2pwm_rudder( dr );
  Servo_Ch5_PWM = angle2pwm_flap( flap_angle );
  Servo_Ch6_PWM = fraction_deflection2pwm_right_aileron( da );
  Servo_Ch7_PWM = angle2pwm_right_elevator( elev_angle );
  
}
