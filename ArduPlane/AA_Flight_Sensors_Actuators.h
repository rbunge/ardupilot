
#define ENERGY_LIMIT   0.300*7.4*3600 // max energy in Joules allowed for the mission.  Above this energy consumption, the camera function stops returning snapshots

// ***** Multiplier matrices to convert logged variables into SI, degrees and % of full PWM units  ***** //
// If you multiply each column of the respective log line (MF or HF) by these coefficients, this should convert the data to SI units, except for angles and angular rates, which are in
// degrees.  The Servo positions will be expressed in % of full deflection (i.e. the same as RC_roll)
// MF = [1/1000 1/1000 1/100 1/100 1/100 1/100 1/100 1 1 1/100 1/100 1 1 1/100 1/100] 
// HF = [1/1000 1/100 1/100 1/100 1/100 1 1 1 1/2 1/2 1/2 1/2]


// ***** AA241X FLIGHT VARIABLE DEFINITIONS  *****  //
// Euler angles, in radians from -PI to PI . Computed using attitude filter
// compensating for gyro and dead-reckoning drifts using accelerometer,
// magnetometer, GPS and barometer readings.  See AHRS_DCM.cpp if you want to know more
static float roll_att;
static float pitch_att;
static float yaw_att;

// Angular velocity components in body fixed frame,  in rad/s.  Computed using attitude filter
// compensating for gyro and dead-reckoning drifts using accelerometer,
// magnetometer, GPS and barometer readings. See AHRS_DCM.cpp if you want to know more
static float roll_Rate;
static float pitch_Rate;
static float yaw_Rate;

// Angular acceleration
static float roll_Acc;
static float yaw_Acc;
static float pitch_Acc;

// Linear velocity components in body fixed frame, in m/s
static float u_vel;
static float v_vel;
static float w_vel;

// Linear acceleration in body fixed frame, in m/s^2.
static float accel_x;
static float accel_y;
static float accel_z;

// Accelerometer readings, in m/s^2
static float acclmeter_x;
static float acclmeter_y;
static float acclmeter_z;

static float X_position;        // North position in meters, with respect to Center of Lake Lag
static float Y_position;        // East position in meters, with respect to Center of Lake Lag
static float Z_position_GPS;    // Down position in meters from sea level, reported by the GPS.  Z_position = -altitude above center of lake lagunita
static float Z_position_Baro;   // Down position in meters from sea level, reported by the Barometer.  Z_position = -altitude above center of lake lagunita

static float X_velocity; // Northwards velocity in m/s
static float Y_velocity; // Eastwards velocity in m/s
static float Z_velocity; // Downwards velocity in m/s.  Z_velocity = dZ_pos/dt = -climb rate

static float ground_speed;  // Speed relative to ground, in m/s
static float ground_course; // Direction of velocity relative to North, in radians

static float Air_speed; // Air relative speed in m/s, measured by pitot tube

static bool gpsOK = false;

static float battery_Current;  // Current being consumed by on-board system, in Amps
static float battery_voltage;  // Voltage across battery, in Volts
static float battery_energy_consumed = 0.0;  // Battery energy consumed since last re-boot, in Joules = VoltAmpSec
static float mission_energy_consumed = 0.0;  // Battery energy consumed since start of the mission, in Joules

// Position to which the servos will be udpated, in % of full range (i.e. 0.0 = lower physical limit, 100.0 = upper physical limit)
// IMPORTANT: servos will only be updated when UAV is in AUTO mode
//            and with whatever values ar being set in AA241X_AUTO_FastLoop().
//            They can't be updated from MediumLoop() nor SlowLoop()
static float Roll_servo;     // Servo associated with Channel 1 
static float Pitch_servo;    // Servo associated with Channel 2
static float Throttle_servo; // Servo associated with Channel 3
static float Rudder_servo;   // Servo associated with Channel 4
static float flap_servo;     // Servo associated with Channel 5
static float roll2_servo;    // Servo associated with Channel 6

// Position to which the servos will be udpated, in PWM
// IMPORTANT: servos will only be updated when UAV is in AUTO mode
//            and with whatever values ar being set in AA241X_AUTO_FastLoop().
//            They can't be updated from MediumLoop() nor SlowLoop()
static int16_t Servo_Ch1_PWM;     // Servo associated with Channel 1 
static int16_t Servo_Ch2_PWM;    // Servo associated with Channel 2
static int16_t Servo_Ch3_PWM; // Servo associated with Channel 3
static int16_t Servo_Ch4_PWM;   // Servo associated with Channel 4
static int16_t Servo_Ch5_PWM;     // Servo associated with Channel 5
static int16_t Servo_Ch6_PWM;    // Servo associated with Channel 6
static int16_t Servo_Ch7_PWM;    // Servo associated with Channel 6

// Position to which the servos are commanded by the RC Transmitter, in % of full range (i.e. 0.0 = lower physical limit, 100.0 = upper physical limit)
static float RC_roll;      // Command read on Input 1, coming from the RC Transmitter, through the receiver
static float RC_pitch;     // Command read on Input 2, coming from the RC Transmitter, through the receiver
static float RC_throttle;  // Command read on Input 3, coming from the RC Transmitter, through the receiver
static float RC_rudder;    // Command read on Input 4, coming from the RC Transmitter, through the receiver
static float RC_flap;
static float RC_roll2; 

// Position to which the servos are commanded by the RC Transmitter, in PWM
static int16_t RC_In_Ch1_PWM;
static int16_t RC_In_Ch2_PWM;
static int16_t RC_In_Ch3_PWM;
static int16_t RC_In_Ch4_PWM;
static int16_t RC_In_Ch5_PWM;
static int16_t RC_In_Ch6_PWM;
static int16_t RC_In_Ch7_PWM;

static float RC_Roll_Trim;      // Position of roll stick on RC Transmitter at the instant when control mode switched out of MANUAL, in % of full range. (i.e. 50.0 is halfway, 0.0 all to one side, 100.0 all to the opposite side)
static float RC_Pitch_Trim;     // Position of pitch stick on RC Transmitter at the instant when control mode switched out of MANUAL, in % of full range
static float RC_Throttle_Trim;  // Position of throttle stick on RC Transmitter at the instant when control mode switched out of MANUAL, in % of full range
static float RC_Rudder_Trim;    // Position of rudder stick on RC Transmitter at the instant when control mode switched out of MANUAL, in % of full range

static float CPU_time_ms = 0.0;
static float GPS_time_ms = 0.0;
static float Main_loop_deltaTime_ms; // time in ms that Main loop (i.e. ArduPlane fast + medium + slow loops) took, ideally should be less than 20ms to ensure that the fast loop is executed at 50Hz
static float Last_AUTO_stampTime_ms = 0.0; // holds the last CPU time in ms for which we were in AUTO mode 

static float WayPointX[3];
static float WayPointY[3];
static float WayPointZ[3];
static float WayPointHeading[3];

static float r_spin_detect;

static float Arduino_CPU_time = 0;
static float P1;
static float P2;
static float P3;
static float P4;
static float P5;
static float P6;
static float P7;
static float P8;

static float P9;
static float P10;
static float P11;
static float P12;
static float P13;
static float P14;
static float P15;
static float P16;


