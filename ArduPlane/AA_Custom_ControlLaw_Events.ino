

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


//////// ROLL/YAW DIVERGENCE EXIT CONDITIONS /////
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

