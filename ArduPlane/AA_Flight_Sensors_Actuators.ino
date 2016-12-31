
#include <math.h>
#include <AP_Math.h>
#include "defines.h"
//#include "AA_Flight_Sensors_Actuators.h"

# define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)

#define HEAD_BYTE1      0xA3   // Decimal 163
#define HEAD_BYTE2      0x95   // Decimal 149
#define END_BYTE        0xBA   // Decimal 186

#define PACKET_HEADER   0xAA
#define PACKET_FOOTER   0x55

#define LOG_AA241X_MSG  0x0B  
#define NED_to_Body(x)  DCM_Matrix.mul_transpose(x)  // Convert vector3f x from NED frame to Body Frame.  Returns a type Vector3f
#define Body_to_NED(x)  DCM_Matrix*(x)  // Convert vector3f x from Body frame to NED Frame.  Returns a type Vector3f

#define PWM_MIN 800
#define PWM_MAX 2200

#define CENTER_LAGUNITA_LAT   374224444  // Latitutde of Center of Lake Lag in degrees * 1e7
#define CENTER_LAGUNITA_LONG -1221760917 // Latitutde of Center of Lake Lag in degrees * 1e7
#define CENTER_LAGUNITA_ALT 40  // altitude above sea level of center of Lake Lag

#define MIN_AIRSPEED 2.0  // airspeed below which the mission will be ended if the mission has started and the altitude is below ALTITUDE_START

#define MAX_ALTITUDE 121.92 // max altitude in meters above Lake Lag allowed for flight.  Above this altitude, the camera function stops returning snapshots 
#define MIN_ALTITUDE_SNAPSHOT 30.48  // min altitude in meters above Lake Lag allowed for flight.
#define ALTITUDE_START 15.24  // min altitude in meters above Lake Lag allowed for flight.
#define MAX_FOV_DIAM 60.0
#define MIN_FOV_DIAM 30.0
#define MAX_ESTIMATE_DIAM 40.0
#define MIN_ESTIMATE_DIAM 20.0

#define WAIT_TIME_MS 3000  // wait time between pictures

#define ALPHA 200.0  // weight for position error term
#define BETA 5000.0  // weight for time of sight term
#define GAMMA 1.0  // min position error used for score

#define Z_POS_CONST 0.7  // constant for the Z_position_Baro low pass filter
#define Z_VEL_CONST 0.7  // constant for the Z_velocity low pass filter

#define GUMSTIX_ACK B1010000


// *****  AA241X AUXILIARY FUNCTIONS - IF BUGS FOUND LET US KNOW ASAP, DO NOT EDIT!   ***** //

const int Competition_day = 99;

//static int16_t AA241X_roll_servo_PWM = 901;
//static int16_t AA241X_pitch_servo_PWM = 901;
//static int16_t AA241X_throttle_servo_PWM = 901;
//static int16_t AA241X_rudder_servo_PWM = 901;
//static int16_t AA241X_flap_servo_PWM = 901;
//static int16_t AA241X_roll2_servo_PWM = 901;

//static int16_t RC_roll_PWM;
//static int16_t RC_pitch_PWM;
//static int16_t RC_throttle_PWM;
//static int16_t RC_rudder_PWM;
//static int16_t RC_flap_PWM;
//static int16_t RC_roll2_PWM;



static struct Location center_lake_lag;

static char sighted_persons = 0; 
static char sightedPerson[4] = {0, 0, 0};
static float CPU_time_sight_ms = 0.0;

static float CPU_time_start_ms = 0.0;

static float X_person_truth[4];
static float Y_person_truth[4];

// UPDATE SERVO POSITIONS
static void update_servos(void){
//  AA241X_roll_servo_PWM     = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*Roll_servo/100.0); 
//  AA241X_pitch_servo_PWM    = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*Pitch_servo/100.0); 
//  AA241X_throttle_servo_PWM = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*Throttle_servo/100.0); 
//  AA241X_rudder_servo_PWM   = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*Rudder_servo/100.0); 
//  AA241X_flap_servo_PWM   = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*flap_servo/100.0);
//  AA241X_roll2_servo_PWM   = (int16_t)(PWM_MIN  + (PWM_MAX - PWM_MIN)*roll2_servo/100.0);
//  
//  channel_roll->radio_out     = constrain_int16(AA241X_roll_servo_PWM,     PWM_MIN, PWM_MAX);
//  channel_pitch->radio_out    = constrain_int16(AA241X_pitch_servo_PWM,    PWM_MIN, PWM_MAX);
//  channel_throttle->radio_out = constrain_int16(AA241X_throttle_servo_PWM, PWM_MIN, PWM_MAX);
//  channel_rudder->radio_out   = constrain_int16(AA241X_rudder_servo_PWM,   PWM_MIN, PWM_MAX);
//  hal.rcout->write(4, AA241X_flap_servo_PWM);
//  hal.rcout->write(5, AA241X_roll2_servo_PWM);

  channel_roll->radio_out     = Roll_servo_PWM;  // consider creating variable called servo_Ch_Out1_PWM, that replaces Roll_servo_PWM.  
  channel_pitch->radio_out    = Pitch_servo_PWM;  // Likewise for other channels.  This allows the user to define a custom mapping function from control variables to servo channels, using variable naming that is generic (not particular to a given configuration).
  channel_throttle->radio_out = Throttle_servo_PWM;
  channel_rudder->radio_out   = Rudder_servo_PWM;
  hal.rcout->write(4, flap_servo_PWM);
  hal.rcout->write(5, roll2_servo_PWM);

}

static void update_Last_AUTO_time_and_Main_loop_deltaTime(void){
  Last_AUTO_stampTime_ms = CPU_time_ms;
 // Main_loop_deltaTime_ms = (float)delta_ms_main_loop;
}

static void AA241X_AUX_MediumLoop(void){
//    uint8_t countt = 0;
//    while (hal.uartE->available() > 0) {
////        uint8_t data = hal.uartE->read();
//        countt++;
//        //        hal.uartE->write(data+1);
//        //        uint8_t a = hal.uartE->read();
//        //        uint8_t b = hal.uartE->read();
//        //        uint8_t c = hal.uartE->read();
//        //        uint8_t d = hal.uartE->read();
//        //        uint8_t e = hal.uartE->read();
//        float f = readFloat();
//        Debug("Attempting to %f", f);
//        
//	}
//    if (countt != 0) {Debug("Attempting to %d", countt);}
    
    
    
  if (control_mode == AUTO)
  {
      AA241X_AUTO_MediumLoop();
  }
}

static void AA241X_AUX_SlowLoop(void){
  if (control_mode == AUTO)
  {
      AA241X_AUTO_SlowLoop();
  }
}


// UPDATE FLIGHT VARIABLES used for AA241X Control Law
static void update_AA241X_flight_variables(void) {
  Vector3f angular_vel;
  Vector3f accelerometer_reading;
  Matrix3f DCM_Matrix;
  Vector3f NED_Velocity;
  Vector3f Body_Velocity;
  Vector3f NED_Gravity;
  Vector3f Body_Gravity;
  
  static float dist;
  static float bearing;
  static float altitude;
  static char allow_start = 1;
 
  static uint16_t last_millis = 0;
  static uint16_t current_millis;
  static float last_CPU_time_ms = 0.0;
  static float delta_time_sec;
  static float battery_energy_consumed_start = 0.0;
  static float last_CPU_time_Z_filter_ms; 
  static float last_Z_position_Baro = 0.0;
  static float last_Z_velocity = 0.0;
  static float last_roll_Rate;
  static float last_pitch_Rate;
  static float last_yaw_Rate;
  static char Z_filter_flag = 0;
  float Z_position_Baro_raw;
  float Z_velocity_raw;
 
  static struct Location loc;
  
  
  // Update CPU time 
  current_millis = millis();
  if (current_millis < last_millis){
    CPU_time_ms = CPU_time_ms + (65535.0 - (float)last_millis) + (float)current_millis ;
  } else {
    CPU_time_ms = CPU_time_ms + (float)(current_millis - last_millis);
  }
  last_millis = current_millis;
 
  
  // Update total battery energy consumed
  read_battery();
  battery_Current = battery.current_amps();  //current_amps1;
  battery_voltage = battery.voltage();   //battery_voltage1;
  delta_time_sec = (CPU_time_ms - last_CPU_time_ms)/1000.0;
  battery_energy_consumed = battery_energy_consumed + battery_Current*battery_voltage*delta_time_sec;
  mission_energy_consumed = battery_energy_consumed - battery_energy_consumed_start;
  last_CPU_time_ms = CPU_time_ms;
  
  Main_loop_deltaTime_ms = 0; //(float)delta_ms_fast_loop;
  
  roll_att  = ahrs.roll;
  pitch_att = ahrs.pitch;
  yaw_att   = ahrs.yaw;
  
  angular_vel = ahrs.get_gyro();   
  roll_Rate   = angular_vel.x;
  pitch_Rate  = angular_vel.y;
  yaw_Rate    = angular_vel.z;
  
  
  // Z_position_Baro and Z_velocity filters
  Z_position_Baro_raw = -1.0*(relative_altitude() + home.alt/100.0 - CENTER_LAGUNITA_ALT);  // Z_position in meters from center of lake lag
  if (Z_filter_flag == 0) {
    Z_position_Baro = Z_position_Baro_raw;
    Z_velocity = 0.0;
    Z_filter_flag = 1;
  } else {
    // Z_position_Baro low pass filter
    Z_position_Baro = (1-Z_POS_CONST)*Z_position_Baro_raw + Z_POS_CONST*last_Z_position_Baro;
    // Z_velocity low pass filter
    Z_velocity_raw = 1000.0*(Z_position_Baro - last_Z_position_Baro)/((CPU_time_ms - last_CPU_time_Z_filter_ms));
    Z_velocity      =  (1-Z_VEL_CONST)*Z_velocity_raw     + Z_VEL_CONST*last_Z_velocity;
    // Roll acceleration filter
    roll_Acc  = 1000*(roll_Rate  - last_roll_Rate )/(CPU_time_ms - last_CPU_time_Z_filter_ms);
    pitch_Acc = 1000*(pitch_Rate - last_pitch_Rate)/(CPU_time_ms - last_CPU_time_Z_filter_ms);
    yaw_Acc   = 1000*(yaw_Rate   - last_yaw_Rate  )/(CPU_time_ms - last_CPU_time_Z_filter_ms);
  }
  last_Z_position_Baro = Z_position_Baro;
  last_Z_velocity = Z_velocity;
  last_CPU_time_Z_filter_ms = CPU_time_ms; 
  last_roll_Rate = roll_Rate;
  last_pitch_Rate = pitch_Rate;
  last_yaw_Rate = yaw_Rate;
  
  
  Air_speed = airspeed.get_airspeed();
  
  // Variables derived from a GPS reading
  center_lake_lag.lat = CENTER_LAGUNITA_LAT;
  center_lake_lag.lng = CENTER_LAGUNITA_LONG;
  gpsOK  = ahrs.get_position(loc);
  if (gps.status(0) == AP_GPS::GPS_OK_FIX_3D){
    // Update GPS time
    GPS_time_ms = gps.time_week_ms(0);
    
    bearing = (((float)get_bearing_cd(center_lake_lag, loc))/100.0)*PI/180.0;
    dist    = (float)get_distance(center_lake_lag, loc);
    
    X_position     = cos(bearing)*dist;
    Y_position     = sin(bearing)*dist;
    Z_position_GPS = -((float)(gps.location().alt * 10UL)/1000.0- CENTER_LAGUNITA_ALT); // altitude in MSL
  
    ground_speed  = ((float) gps.ground_speed()); 
    ground_course = (((float) gps.ground_course_cd())/100.0)*PI/180;
  
    X_velocity     = cos(ground_course)*ground_speed;
    Y_velocity     = sin(ground_course)*ground_speed;
    NED_Velocity.x = X_velocity;
    NED_Velocity.y = Y_velocity;
    NED_Velocity.z = Z_velocity;
    
    DCM_Matrix.from_euler(roll_att, pitch_att, yaw_att);
    Body_Velocity = NED_to_Body(NED_Velocity);  // see http://gentlenav.googlecode.com/files/EulerAngles.pdf for definition of DCM_Matrix
    u_vel         = Body_Velocity.x;
    v_vel         = Body_Velocity.y;
    w_vel         = Body_Velocity.z; 
  } 
  
  // Compute acceleration
  accelerometer_reading = ins.get_accel();
  NED_Gravity.x = 0;
  NED_Gravity.y = 0;
  NED_Gravity.z = 9.80665;
  Body_Gravity  = NED_to_Body(NED_Gravity);
  accel_x       = accelerometer_reading.x + Body_Gravity.x;
  accel_y       = accelerometer_reading.y + Body_Gravity.y;
  accel_z       = accelerometer_reading.z + Body_Gravity.z;
  
  acclmeter_x  = accelerometer_reading.x;
  acclmeter_y  = accelerometer_reading.y;
  acclmeter_x  = accelerometer_reading.z;
  
  
  // Read RC signals
  RC_roll_PWM      = channel_roll->radio_in;
  RC_pitch_PWM     = channel_pitch->radio_in;
  RC_throttle_PWM  = channel_throttle->radio_in;
  RC_rudder_PWM    = channel_rudder->radio_in;
  RC_flap_PWM      = hal.rcin->read(4);
  RC_roll2_PWM     = hal.rcin->read(5);
    
  RC_roll     = 100*((float)(RC_roll_PWM     - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  RC_pitch    = 100*((float)(RC_pitch_PWM    - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  RC_throttle = 100*((float)(RC_throttle_PWM - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  RC_rudder   = 100*((float)(RC_rudder_PWM   - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  RC_flap     = 100*((float)(RC_flap_PWM - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));
  RC_roll2    = 100*((float)(RC_roll2_PWM - PWM_MIN))/((float)(PWM_MAX - PWM_MIN));

  
  // Updating the "RC_Trim" value for each RC Channel input
  // see trim_control_surfaces() in radio.pde  
  RC_Roll_Trim     = 100.0*((float)(channel_roll->radio_trim     - PWM_MIN)) / ((float)(PWM_MAX - PWM_MIN));  
  RC_Pitch_Trim    = 100.0*((float)(channel_pitch->radio_trim    - PWM_MIN)) / ((float)(PWM_MAX - PWM_MIN));  
  RC_Throttle_Trim = 100.0*((float)(channel_throttle->radio_trim - PWM_MIN)) / ((float)(PWM_MAX - PWM_MIN));  
  RC_Rudder_Trim   = 100.0*((float)(channel_rudder->radio_trim   - PWM_MIN)) / ((float)(PWM_MAX - PWM_MIN));
  
//    uint8_t countt = 0;
//    while (hal.uartE->available() > 0) {
//        uint8_t data = hal.uartE->read();
//        countt++;
//        //        hal.uartE->write(data+1);
//        //        uint8_t a = hal.uartE->read();
//        //        uint8_t b = hal.uartE->read();
//        //        uint8_t c = hal.uartE->read();
//        //        uint8_t d = hal.uartE->read();
//        //        uint8_t e = hal.uartE->read();
//        //        float f = readFloat();
//        //        Debug("Attempting to %d %d %d %d %d", a, b, c, d, e);
//        
//	}
//    if (countt != 0) {Debug("Attempting to %d", countt);}
    //Debug("Hello");
    uint8_t countt = 0;
    if (hal.uartE->available() > 0) {
//        int time_first_check = millis();
        
        hal.scheduler->delay(4);
//        Debug("UARTE has %d before reads", hal.uartE->available());
        
        //        uint8_t data = hal.uartE->read();
        countt++;
        //        hal.uartE->write(data+1);
        //        uint8_t a = hal.uartE->read();
        //        uint8_t b = hal.uartE->read();
        //        uint8_t c = hal.uartE->read();
        //        uint8_t d = hal.uartE->read();
        //        uint8_t e = hal.uartE->read();
        uint8_t header = hal.uartE->read();
        
//        float f0 = readFloat();
//        float f1 = readFloat();
//        float f2 = readFloat();
//        float f3 = readFloat();
//        float f4 = readFloat();
//        float f5 = readFloat();
//        float f6 = readFloat();
//        float f7 = readFloat();
//        float f8 = readFloat();
        

        
        
        float f0 = readFloat();
        
        int16_t Sensor1 = readInt();
        int16_t Sensor2 = readInt();
        int16_t Sensor3 = readInt();
        int16_t Sensor4 = readInt();
        int16_t Sensor5 = readInt();
        int16_t Sensor6 = readInt();
        int16_t Sensor7 = readInt();
        int16_t Sensor8 = readInt();
        
        int16_t Sensor9 = readInt();
        int16_t Sensor10 = readInt();
        int16_t Sensor11 = readInt();
        int16_t Sensor12 = readInt();
        int16_t Sensor13 = readInt();
        int16_t Sensor14 = readInt();
        int16_t Sensor15 = readInt();
        int16_t Sensor16 = readInt();
        
        uint8_t footer = hal.uartE->read();
        //Debug("Got packet");
        
        if (header == PACKET_HEADER && footer == PACKET_FOOTER) {
//            Debug("Attempting to %f %f %f %f %f \n", f0, f1, f2, f3, f4);
//            Debug("Attempting to %f %f %f %f \n", f5, f6, f7, f8);
//            Arduino_CPU_time = f0;
//            P1 = f1;
//            P2 = f2;
//            P3 = f3;
//            P4 = f4;
//            P5 = f5;
//            P6 = f6;
//            P7 = f7;
//            P8 = f8;
//            Log_Write_AA241X_PRESSURE();
            
            
            Debug("Attempting to %f %i %i %i %i \n", f0, Sensor1, Sensor2, Sensor3, Sensor4);
            Debug("Attempting to %i %i %i %i \n", Sensor5, Sensor6, Sensor7, Sensor8);
            Debug("Attempting to %i %i %i %i \n", Sensor9, Sensor10, Sensor11, Sensor12);
            Debug("Attempting to %i %i %i %i \n", Sensor13, Sensor14, Sensor15, Sensor6);
            
            
            P1=(float) 1000*(5*((float)Sensor1)/1023.0-2.5);
            P2=(float) 1000*(5*((float)Sensor2)/1023.0-2.5);
            P3=(float) 1000*(5*((float)Sensor3)/1023.0-2.5);
            P4=(float) 1000*(5*((float)Sensor4)/1023.0-2.5);
            P5=(float) 1000*(5*((float)Sensor5)/1023.0-2.5);
            P6=(float) 1000*(5*((float)Sensor6)/1023.0-2.5);
            P7=(float) 1000*(5*((float)Sensor7)/1023.0-2.5);
            P8=(float) 1000*(5*((float)Sensor8)/1023.0-2.5);
            
            P9 =(float) 1000*(5*((float)Sensor9)/1023.0-2.5);
            P10=(float) 1000*(5*((float)Sensor10)/1023.0-2.5);
            P11=(float) 1000*(5*((float)Sensor11)/1023.0-2.5);
            P12=(float) 1000*(5*((float)Sensor12)/1023.0-2.5);
            P13=(float) 1000*(5*((float)Sensor13)/1023.0-2.5);
            P14=(float) 1000*(5*((float)Sensor14)/1023.0-2.5);
            P15=(float) 1000*(5*((float)Sensor15)/1023.0-2.5);
            P16=(float) 1000*(5*((float)Sensor16)/1023.0-2.5);
            
            Log_Write_AA241X_PRESSURE();
            Log_Write_AA241X_PRESSURE_2();
            
            Debug("Attempting to %f %f %f %f %f \n", f0, P1, P2, P3, P4);
            Debug("Attempting to %f %f %f %f \n", P5, P6, P7, P8);
            Debug("Attempting to %f %f %f %f \n", P9, P10, P11, P12);
            Debug("Attempting to %f %f %f %f \n", P13, P14, P15, P16);
            

            
        }
//        Debug("UARTE has %d after reads", hal.uartE->available());
//        while (hal.uartE->available() > 0){
//            hal.uartE->read();
//        }
//        
//        Debug("UARTE has %d after flush", hal.uartE->available());
//        
	}
    if (countt != 0) {Debug("Attempting to %d", countt);}
    


//  hal.uartE->write('a');
}

static void Log_Write_AA241X_AHF(void){
// if (g.aa241x_attitude_log_frequency == 1 && (-Z_position_Baro - (home.alt/100.0 - CENTER_LAGUNITA_ALT)) > g.aa241x_attitude_log_altitude )
// {
   Log_Write_AA241X_HF1();
   Log_Write_AA241X_HF2();
   Log_Write_AA241X_HF3();
// }
}
//
//static void Log_Write_AA241X_AMF(void){
// if (g.aa241x_attitude_log_frequency != 1)
// {
//   Log_Write_AA241X_HF();
// }
//}

union Data{
    int8_t bytes[4];
    float f;
};
float readFloat(void){
    union Data u;
    u.bytes[0] = hal.uartE->read();
    u.bytes[1] = hal.uartE->read();
    u.bytes[2] = hal.uartE->read();
    u.bytes[3] = hal.uartE->read();
    return u.f;
}

union DataI{
    int8_t bytes[2];
    int16_t I;
};
float readInt(void){
    union DataI u;
    u.bytes[0] = hal.uartE->read();
    u.bytes[1] = hal.uartE->read();
//    u.bytes[2] = hal.uartE->read();
//    u.bytes[3] = hal.uartE->read();
    return u.I;
}








//char sendTestData(void){
//  float data[] = {1.0, 2.0, 1.0, 2.0, 1.0, 2.0};
//  return sendFloatPacket(SEND_TEST_DATA, data);
//}
//
//char sendFloatPacket(byte packet_id, float data[]){
//  byte nFloats = sizeof(data)/4;
//  Serial2.write(PACKET_HEADER);
//  Serial2.write(packet_id);
//  Serial2.write((byte)nFloats);
//  for (int i = 0; i < nFloats; i++){
//  sendFloat(data[i]);
//  }
//  Serial2.write(PACKET_FOOTER);
//  return gotACK();
//}
//
//char sendFloat(float f){
//  byte * p_f = (byte *) &f;
//  Serial2.write(p_f[0]);
//  Serial2.write(p_f[1]);
//  Serial2.write(p_f[2]);
//  Serial2.write(p_f[3]);
//  return 1;
//}
//
//float readFloat(void){
// union Data u;
// u.bytes[0] = Serial2.read();
// u.bytes[1] = Serial2.read();
// u.bytes[2] = Serial2.read();
// u.bytes[3] = Serial2.read();
// return u.f;
//}
//
//char gotACK(){
//  delayMicroseconds(100);
//  char ack = Serial2.read();
//  return ack == GUMSTIX_ACK;
//}
//char readTestData(void){
//  
//    char byte1 = Serial2.read();
//    char byte2 = Serial2.read();
//    if (byte1 == 1 && byte2 == 2){
//      return 1;
//    }
//    return 0;
//}







