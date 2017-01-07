
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


///// SPIN DETECTION CONDITIONS /////
static int8_t Spin_Detected(void){
  
  // Baseline spin detection parameters
  float p_dot_R;
  float p_dot_L;
  float p       = roll_Rate;
  float p_dot   = roll_Acc;
  float r       = yaw_Rate;
  float r_dot   = yaw_Acc;
  float k_dr    = -8;
  float k_p     = -1.5;

  //Parameters:  A_p_dot_o, A_p_o, A_k_da, A_Eta
  p_dot_R = A_p_dot_o*PI/180.0 - (A_p_dot_o/A_p_o)*p;
  p_dot_L = -A_p_dot_o*PI/180.0 - (A_p_dot_o/A_p_o)*p;
  float da = pwm2angle_right_aileron(RC_In_Ch1_PWM);
  float Eta_spin = 2;
 
  switch (uint16_t(AA_Test_Set)) {
    case 7:
      if (AA_Test_SNbr == 1){ return (fabsf(roll_Rate) > 3.5); };
      if (AA_Test_SNbr == 2){ return (fabsf(yaw_Rate) >  1.5); };
      if (AA_Test_SNbr == 3){ 
        float spin_rate_sqrd = roll_Rate*roll_Rate + yaw_Rate*yaw_Rate;
        return (spin_rate_sqrd > 3.4*3.4); 
      };
      break;
    case 12:
      if (AA_Test_SNbr == 4){ Eta_spin = A_Eta_spin; };
      break;
    case 13:
      if (AA_Test_SNbr == 1){ Eta_spin = -3; };
      if (AA_Test_SNbr == 2){ Eta_spin = -6; };
      if (AA_Test_SNbr == 3){ Eta_spin = -9; };
      if (AA_Test_SNbr == 4){ Eta_spin = A_Eta_spin; };
      break;
    case 15:
    case 16:
    case 17:
    case 18:
      if (RD_exit() && YD_exit() && r > A_r_0_detect*PI/180.0){
          r_spin_detect = yaw_Rate*180/PI;
          return 1;
      } else {
       return 0; 
      }
//      return (( r_dot*r > 0) && (fabs(r) > A_r_0_detect*PI/180.0) );
      break;
   };
   
   if (((p_dot - A_k_da*da) > p_dot_R + Eta_spin) || ((p_dot - A_k_da*da) < p_dot_L - Eta_spin)){ return 1; }
   
   return 0;
}


///// ARREST DETECTION CONDITIONS /////
static int8_t Arrest_Detected(void){
  
  // Baseline arrest detection parameters
  float p_dot_R;
  float p_dot_L;
  float p = roll_Rate;  
  float p_dot = roll_Acc;
  //Parameters:  A_p_dot_o, A_p_o, A_k_da, A_Eta
  p_dot_R = A_p_dot_o*PI/180.0 - (A_p_dot_o/A_p_o)*p;
  p_dot_L = -A_p_dot_o*PI/180.0 - (A_p_dot_o/A_p_o)*p;
  float da = pwm2angle_right_aileron(RC_In_Ch1_PWM);
  float Eta_arrest = -2;
  
  switch (uint16_t(AA_Test_Set)) {
    case 8:
      if (AA_Test_SNbr == 1){ Eta_arrest = -6; };
      if (AA_Test_SNbr == 2){ return (fabsf(roll_Rate) < 0.5); };
      if (AA_Test_SNbr == 3){ return (fabsf(yaw_Rate)  < 0.5); };
      if (AA_Test_SNbr == 4){ 
        float spin_rate_sqrd = roll_Rate*roll_Rate + yaw_Rate*yaw_Rate;
        return (spin_rate_sqrd < 0.5*0.5); 
      };
      break;
    case 12:
      if (AA_Test_SNbr == 4){ Eta_arrest = A_Eta_arrest; };
      break;
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
      return 0;
      Eta_arrest = -12;
      break;
  };  
  
  if (((p_dot - A_k_da*da) < p_dot_R + Eta_arrest) && ((p_dot - A_k_da*da) > p_dot_L - Eta_arrest)){ return 1; }
  
  return 0;
}

//// LEVEL FLIGHT CONDTION //////
static int8_t Level_Flight_Condition(void){
  if (pitch_att - A_pitch_level*3.142/180.0 > 0){
   return 1; 
  }
  return 0;
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
  
  
//  switch (uint16_t(AA_Test_Set)) {
//    case 10:
//      if (AA_Test_SNbr == 1){ elev_angle_max = -30 ;};
//      if (AA_Test_SNbr == 2){ elev_angle_max = -20 ;};
//      if (AA_Test_SNbr == 3){ elev_angle_max = aa_elev_angle_max ;};
//      if ( elev_angle < elev_angle_max ) { elev_angle = elev_angle_max;};
//      Servo_Ch2_PWM = angle2pwm_elevator( elev_angle );
//      break;
//    case 11:
//
//      if (AA_Test_SNbr == 1){ 
//        Servo_Ch1_PWM     = fraction_deflection2pwm_right_aileron(da);
//        Servo_Ch6_PWM    = fraction_deflection2pwm_left_aileron(da); };
//      if (AA_Test_SNbr == 2){ 
//        Servo_Ch4_PWM   = fraction_deflection2pwm_rudder(dr);};
//      if (AA_Test_SNbr == 3){ 
//        Servo_Ch1_PWM     = fraction_deflection2pwm_right_aileron(da);
//        Servo_Ch6_PWM    = fraction_deflection2pwm_left_aileron(da); 
//        Servo_Ch4_PWM   = fraction_deflection2pwm_rudder(dr);};
//      break;
//     case 16:
//     case 17:
//     case 18:
////        Servo_Ch1_PWM     = fraction_deflection2pwm_right_aileron(da);
////        Servo_Ch6_PWM    = fraction_deflection2pwm_left_aileron(da); 
////        Servo_Ch4_PWM   = fraction_deflection2pwm_rudder(dr);
//       break;
       
  };
  
    
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
  
//  // Perturb controls from the baseline according to the test
//  switch (uint16_t(AA_Test_Set)) {
//    case 1:
//      if (AA_Test_SNbr == 1){ elev_angle = -20 ;};
//      if (AA_Test_SNbr == 2){ elev_angle = -15 ;};
//      if (AA_Test_SNbr == 3){ elev_angle = -10 ;};
//      if (AA_Test_SNbr == 4){ elev_angle =  arrest_elevator_angle ;};
//      break;
//   case 2:
//      elev_angle = -10;
//      break;
//   case 3:
//      if (AA_Test_SNbr == 1){elev_angle = 0;};
//      if (AA_Test_SNbr == 2){elev_angle = 10;};
//      if (AA_Test_SNbr == 3){elev_angle = 20;};
//      if (AA_Test_SNbr == 4){elev_angle = arrest_elevator_angle ;};
//      break;
//   case 5:
//      if (AA_Test_SNbr == 1){ da =  sign_f(roll_Rate); dr = 0;}   // aileron full anti-spin
//      if (AA_Test_SNbr == 2){ da = -sign_f(roll_Rate); dr = 0;}   // aileron fill pro-spin
//      if (AA_Test_SNbr == 3){ dr =  sign_f(yaw_Rate);  da = 0;}   // rudder  full anti-spin
//      da_left  = da;
//      da_right = da;
//      break;
//   case 6:
//      if (AA_Test_SNbr == 1){ da =  sign_f(roll_Rate); dr = 0;}   // aileron full anti-spin
//      if (AA_Test_SNbr == 2){ da = -sign_f(roll_Rate); dr = 0;}   // aileron fill pro-spin
//      if (AA_Test_SNbr == 3){ dr =  sign_f(yaw_Rate);  da = 0;}   // rudder  full anti-spin
//      elev_angle = -10;
//      da_left  = da;
//      da_right = da;
//      break;
//   case 8:
//      elev_angle = -10;
//      break; 
//   case 14:
//   case 15:
//   case 16:
//      elev_angle = arrest_elevator_angle;
//      da =  -sign_f(roll_Rate)*A_da_arrest;
//      dr =  -sign_f(yaw_Rate)*A_dr_arrest;
//      da_left  = da;
//      da_right = da;
//      break;
//   case 17:
//      elev_angle = arrest_elevator_angle;
//      dr =  -sign_f(yaw_Rate)*A_dr_arrest;
//      if ( roll_Rate > 0) {da_left = 1; da_right = 0;};
//      if ( roll_Rate < 0) {da_left = 0; da_right = -1;};
//     break;
//   case 18:
//      elev_angle = arrest_elevator_angle;
//      dr =  -sign_f(yaw_Rate)*A_dr_arrest;
//      da_left = 1;   // left aileron full trailing edge up
//      da_right = -1; // right aileron full trailing edged up
//     break;
//   };   
 
  // Set actuators from control variables 
  map_control_vars_to_actuators(float elev_angle, float da, float dr, float dt, float flap_angle)
  
//  Servo_Ch1_PWM     = fraction_deflection2pwm_right_aileron(da_right);
//  Servo_Ch2_PWM    = angle2pwm_elevator( elev_angle );
//  Servo_Ch3_PWM = fraction2pwm_throttle(dt);
//  Servo_Ch4_PWM   = fraction_deflection2pwm_rudder(dr);
//  Servo_Ch5_PWM     = angle2pwm_flap(df);
//  Servo_Ch6_PWM    = fraction_deflection2pwm_left_aileron(da_left);
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
  
//  // Perturb controls from the baseline according to the test
//  switch (uint16_t(AA_Test_Set)) {
//    case 1:
//      elev_angle = 0;
//      da = 0;
//      break;
//    case 2:
//      if (AA_Test_SNbr == 1){ elev_angle = -5 ;};
//      if (AA_Test_SNbr == 2){ elev_angle = -10 ;};
//      if (AA_Test_SNbr == 3){ elev_angle = -15 ;};
//      if (AA_Test_SNbr == 4){ elev_angle = pullout_elevator_angle ;};
//      da = 0;
//      break;
//    case 3:
//      da = 0;
//      break;
//    case 4:
//      if (AA_Test_SNbr == 1){ k_phi = 0.2 ;};
//      if (AA_Test_SNbr == 2){ k_phi = 0.8 ;};
//      if (AA_Test_SNbr == 3){ k_phi = 1.2 ;};
//      if (AA_Test_SNbr == 4){ k_phi = aa_k_phi ;};
//      da = k_phi*(roll_att - A_roll_0*3.142/180.0);
//      break;
//    case 9:
//      elev_angle = elevator_CN_controller(1.0);
//     break; 
//   };   
//
//  Servo_Ch1_PWM     = fraction_deflection2pwm_right_aileron(da);
//  Servo_Ch2_PWM    = angle2pwm_elevator(elev_angle);
//  Servo_Ch3_PWM = fraction2pwm_throttle(dt);
//  Servo_Ch4_PWM   = angle2pwm_rudder(dr);
//  Servo_Ch5_PWM     = angle2pwm_flap(df);
//  Servo_Ch6_PWM    = fraction_deflection2pwm_left_aileron(da);
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
  
//  float da = aa_k_phi*(roll_att - A_roll_0*3.142/180.0) + aa_k_p*roll_Rate;
//  Servo_Ch1_PWM     = fraction_deflection2pwm_right_aileron(da);
//  Servo_Ch2_PWM     = angle2pwm_elevator(0);
//  Servo_Ch3_PWM     = fraction2pwm_throttle(A_dt_level);
//  Servo_Ch4_PWM     = angle2pwm_rudder(0);
//  Servo_Ch5_PWM     = angle2pwm_flap(0);
//  Servo_Ch6_PWM     = fraction_deflection2pwm_left_aileron(da);
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









// *****   AA241X Medium Loop - @ ~10Hz  *****  //
static void AA241X_AUTO_MediumLoop(void){
    
  // YOUR CODE HERE
};
// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void){
  // YOUR CODE HERE
};
///////////////////////////////////////////////////////////////////////////////
////////////////////////// AUXILIARY FUNCTIONS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Roll divergence exit
static int8_t RD_exit(void){
  float p       = roll_Rate;
  float p_dot   = roll_Acc;
  float da      = pwm2angle_right_aileron(RC_In_Ch1_PWM);
  float p_dot_minus_da = p_dot -  A_k_da*da;
  
  return outside_linear_space(p, p_dot_minus_da, A_p_o*PI/180.0, A_p_dot_o*PI/180.0) && p*p_dot_minus_da > 0;
}

// Roll divergence exit
static int8_t YD_exit(void){
  float p       = roll_Rate;
  float r       = yaw_Rate;
  float r_dot   = yaw_Acc;
  float dr      = pwm2angle_right_aileron(RC_In_Ch4_PWM);
  float k_dr    = 0;  // to reduce false triggering, and becuase pwm2angle_rudder() was not implemented  
  float k_p     = 0;  // to reduce false triggering
  float r_dot_minus_dr_p = r_dot -  k_dr*dr - k_p*p;
  
  return outside_linear_space(r, r_dot_minus_dr_p, A_r_o*PI/180.0, A_r_dot_o*PI/180.0) && r*r_dot_minus_dr_p > 0;
}

////// Dimensionless roll divergence exit
//static int8_t DRD_exit(void){
//  float p       = roll_Rate;
//  float p_dot   = roll_Acc;
//  float da = pwm2angle_right_aileron(RC_In_Ch1_PWM);
//  float p_dot_minus_da = p_dot -  A_k_da*da;
//  
//  return outside_linear_space(p, p_dot_minus_da, A_p_o, A_p_dot_o);
//}

static int8_t outside_linear_space(float x,float y,float x_o, float y_o){
  float y_R =  y_o - (y_o/x_o)*x;
  float y_L = -y_o - (y_o/x_o)*x;
  
  if ( y > y_R || y < y_L){ return 1; }
  
 return 0; 
}



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
