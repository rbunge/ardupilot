// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

#if CLI_ENABLED == ENABLED
// Code to Write and Read packets from DataFlash.log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
    cliSerial->println_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->println_P(PSTR("none"));
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
 #define PLOG(_s) if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf_P(PSTR(" %S"), PSTR(# _s))
        PLOG(ATTITUDE_FAST);
        PLOG(ATTITUDE_MED);
        PLOG(GPS);
        PLOG(PM);
        PLOG(CTUN);
        PLOG(NTUN);
        PLOG(MODE);
        PLOG(IMU);
        PLOG(CMD);
        PLOG(CURRENT);
        PLOG(COMPASS);
        PLOG(TECS);
        PLOG(CAMERA);
        PLOG(RC);
        PLOG(SONAR);
 #undef PLOG
    }

    cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);
    return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2)
               || ((uint16_t)dump_log > last_log_num))
    {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return 0;
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint32_t bits;

    if (argc != 2) {
        cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = 0xFFFFFFFFUL;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(MODE);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(COMPASS);
        TARG(TECS);
        TARG(CAMERA);
        TARG(RC);
        TARG(SONAR);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }
    return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}

#endif // CLI_ENABLED == ENABLED

static void do_erase_logs(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs"));
    DataFlash.EraseAll();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete"));
}


// Write an attitude packet
static void Log_Write_Attitude(void)
{
    Vector3f targets;       // Package up the targets into a vector for commonality with Copter usage of Log_Wrote_Attitude
    targets.x = nav_roll_cd;
    targets.y = nav_pitch_cd;
    targets.z = 0;          //Plane does not have the concept of navyaw. This is a placeholder.

    DataFlash.Log_Write_Attitude(ahrs, targets);

#if AP_AHRS_NAVEKF_AVAILABLE
 #if OPTFLOW == ENABLED
    DataFlash.Log_Write_EKF(ahrs,optflow.enabled());
 #else
    DataFlash.Log_Write_EKF(ahrs,false);
 #endif
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.Log_Write_SIMSTATE(DataFlash);
#endif
}


struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint32_t loop_time;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    uint8_t  i2c_lockup_count;
    uint16_t ins_error_count;
};

// Write a performance monitoring packet. Total length : 19 bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        loop_time       : millis() - perf_mon_timer,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        i2c_lockup_count: hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write a mission command. Total length : 36 bytes
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink(cmd,mav_cmd);
    DataFlash.Log_Write_MavCmd(mission.num_commands(),mav_cmd);
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint8_t startup_type;
    uint16_t command_total;
};

static void Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        startup_type    : type,
        command_total   : mission.num_commands()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // write all commands to the dataflash as well
    AP_Mission::Mission_Command cmd;
    for (uint16_t i = 0; i < mission.num_commands(); i++) {
        if (mission.read_cmd_from_storage(i,cmd)) {
            Log_Write_Cmd(cmd);
        }
    }
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    float   accel_y;
};

// Write a control tuning packet. Total length : 22 bytes
static void Log_Write_Control_Tuning()
{
    //Vector3f accel = ins.get_accel();
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_ms         : hal.scheduler->millis(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)channel_throttle->servo_out,
        rudder_out      : (int16_t)channel_rudder->servo_out,
        accel_y         : roll_Acc
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write a TECS tuning packet
static void Log_Write_TECS_Tuning(void)
{
    SpdHgt_Controller->log_data(DataFlash, LOG_TECS_MSG);
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t yaw;
    float wp_distance;
    int16_t target_bearing_cd;
    int16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    int16_t airspeed_cm;
    float   altitude;
    uint32_t groundspeed_cm;
};

// Write a navigation tuning packe
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_ms             : hal.scheduler->millis(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        wp_distance         : auto_state.wp_distance,
        target_bearing_cd   : (int16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (int16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)altitude_error_cm,
        airspeed_cm         : (int16_t)airspeed.get_airspeed_cm(),
        altitude            : barometer.get_altitude(),
        groundspeed_cm      : (uint32_t)(gps.ground_speed()*100)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Sonar {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float distance;
    float voltage;
    float baro_alt;
    float groundspeed;
    uint8_t throttle;
    uint8_t count;
    float correction;
};

// Write a sonar packet
static void Log_Write_Sonar()
{
    struct log_Sonar pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SONAR_MSG),
        timestamp   : hal.scheduler->millis(),
        distance    : (float)rangefinder.distance_cm(),
        voltage     : rangefinder.voltage_mv()*0.001f,
        baro_alt    : barometer.get_altitude(),
        groundspeed : gps.ground_speed(),
        throttle    : (uint8_t)(100 * channel_throttle->norm_output()),
        count       : rangefinder_state.in_range_count,
        correction  : rangefinder_state.correction
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t surface_quality;
    float flow_x;
    float flow_y;
    float body_x;
    float body_y;
};

#if OPTFLOW == ENABLED
// Write an optical flow packet
static void Log_Write_Optflow()
{
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_ms         : hal.scheduler->millis(),
        surface_quality : optflow.quality(),
        flow_x           : flowRate.x,
        flow_y           : flowRate.y,
        body_x           : bodyRate.x,
        body_y           : bodyRate.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

static void Log_Write_Current()
{
    DataFlash.Log_Write_Current(battery, channel_throttle->control_in);

    // also write power status
    DataFlash.Log_Write_Power();
}

static void Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_ms                 : hal.scheduler->millis(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()      
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Write_GPS(uint8_t instance)
{
    DataFlash.Log_Write_GPS(gps, instance, current_loc.alt - ahrs.get_home().alt);
}

static void Log_Write_IMU() 
{
    DataFlash.Log_Write_IMU(ins);
}

static void Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
}

static void Log_Write_Baro(void)
{
    DataFlash.Log_Write_Baro(barometer);
}

// Write a AIRSPEED packet
static void Log_Write_Airspeed(void)
{
    DataFlash.Log_Write_Airspeed(airspeed);
}

////////////////// STARDUPLANE DATA PACKETS //////////////////
struct PACKED log_AA241X_HF1 {
    LOG_PACKET_HEADER;   
    // High Frequency Variables  "f h hhh fff BBBB"
    float cpu_time_ms;  // f
    
    int16_t Z_Position_Baro; // h
    
    float Roll;  // f
    float Pitch; // f
    float Yaw;   // f
    
    float Roll_rate;     // f
    float Pitch_rate;    // f
    float Yaw_rate;      // f
    
    float Roll_acc; //f
    float Pitch_acc;  //f
    float Yaw_acc; //f
    
    float Acclmeter_x;
    float Acclmeter_y;
    float Acclmeter_z;

};

static void Log_Write_AA241X_HF1(void)
{
    struct log_AA241X_HF1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AA241X_HF1_MSG),
        // High Frequency Variables 
        cpu_time_ms   :  CPU_time_ms,  
        
        Z_Position_Baro   :  int16_t(Z_position_Baro*100.0), 
        
        Roll   :  roll_att,
        Pitch  :  pitch_att,
        Yaw    :  yaw_att,
        
        Roll_rate   : roll_Rate,
        Pitch_rate  : pitch_Rate,
        Yaw_rate    : yaw_Rate,
        
        Roll_acc   : roll_Acc,
        Pitch_acc  : pitch_Acc,
        Yaw_acc    : yaw_Acc,
        
        Acclmeter_x    : acclmeter_x,
        Acclmeter_y    : acclmeter_y,
        Acclmeter_z    : acclmeter_z,

    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));  
}

struct PACKED log_AA241X_HF2 {
    LOG_PACKET_HEADER;   
    // High Frequency Variables  "f HHHHHH HHHHHH"
    float cpu_time_ms;  // f
    
    uint16_t Roll_RC_In_PWM;      // H
    uint16_t Pitch_RC_In_PWM;     // H
    uint16_t Throttle_RC_In_PWM;  // H
    uint16_t Rudder_RC_In_PWM;    // H
    uint16_t Flap_RC_In_PWM;    // H
    uint16_t Roll2_RC_In_PWM;    // H  
    
    uint16_t Roll_Servo_Out_PWM;      // H
    uint16_t Pitch_Servo_Out_PWM;     // H
    uint16_t Throttle_Servo_Out_PWM;  // H
    uint16_t Rudder_Servo_Out_PWM;    // H
    uint16_t Flap_Servo_Out_PWM;    // H
    uint16_t Roll2_Servo_Out_PWM;    // H  
    
};

static void Log_Write_AA241X_HF2(void)
{
    struct log_AA241X_HF2 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AA241X_HF2_MSG),
        // High Frequency Variables 
        cpu_time_ms   :  CPU_time_ms,  
        
        Roll_RC_In_PWM       :   uint16_t(channel_roll->radio_in),      
        Pitch_RC_In_PWM      :   uint16_t(channel_pitch->radio_in),
        Throttle_RC_In_PWM   :   uint16_t(channel_throttle->radio_in),
        Rudder_RC_In_PWM     :   uint16_t(channel_rudder->radio_in), 
        Flap_RC_In_PWM     :     uint16_t(hal.rcin->read(4)), 
        Roll2_RC_In_PWM     :    uint16_t(hal.rcin->read(5)), 
        
        Roll_Servo_Out_PWM       :   uint16_t(channel_roll->radio_out),      
        Pitch_Servo_Out_PWM      :   uint16_t(channel_pitch->radio_out),
        Throttle_Servo_Out_PWM   :   uint16_t(channel_throttle->radio_out),
        Rudder_Servo_Out_PWM     :   uint16_t(channel_rudder->radio_out), 
        Flap_Servo_Out_PWM     :     uint16_t(hal.rcout->read(4)), 
        Roll2_Servo_Out_PWM     :    uint16_t(hal.rcout->read(5)), 
  
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));  
}

struct PACKED log_AA241X_HF3 {
    LOG_PACKET_HEADER;   
    // High Frequency Variables  "f ffBB BB"
   float cpu_time_ms;  // f
   
   float air_Speed;        // f
   float z_velocity_Baro;  // f 
   uint8_t Control_Mode;  // B
   uint8_t Local_State;  // B    
   
   uint8_t Test_Set;   // B
   uint8_t Test_SNmbr; // B 
    
};

static void Log_Write_AA241X_HF3(void)
{
    struct log_AA241X_HF3 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AA241X_HF3_MSG),
        // High Frequency Variables 
        cpu_time_ms   :  CPU_time_ms,  
        
        air_Speed         :    Air_speed,
        z_velocity_Baro   :    Z_velocity, 
        Control_Mode      :    uint8_t(control_mode),
        Local_State       :    uint8_t(local_state),
        Test_Set       :      uint8_t(AA_Test_Set), 
        Test_SNmbr       :    uint8_t(AA_Test_SNbr),     
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));  
}


struct PACKED log_AA241X_PRESSURE {
    LOG_PACKET_HEADER;
    // High Frequency Variables  "ff ffff ffff"
    float cpu_time_ms;  //
    float arduino_CPU_time;
    
    float p1;
    float p2;
    float p3;
    float p4;
    
    float p5;
    float p6;
    float p7;
    float p8;
};

static void Log_Write_AA241X_PRESSURE(void)
{
    struct log_AA241X_PRESSURE pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AA241X_PRESSURE_MSG),
        // High Frequency Variables
        cpu_time_ms   :  CPU_time_ms,
        
        arduino_CPU_time        :    Arduino_CPU_time,
        p1   :    P1,
        p2   :    P2,
        p3   :    P3,
        p4   :    P4,
        p5   :    P5,
        p6   :    P6,
        p7   :    P7,
        p8   :    P8,
        
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AA241X_PRESSURE_2 {
    LOG_PACKET_HEADER;
    // High Frequency Variables  "ff ffff ffff"
    float cpu_time_ms;  //
    float arduino_CPU_time;
    
    float p9;
    float p10;
    float p11;
    float p12;
    
    float p13;
    float p14;
    float p15;
    float p16;
};

static void Log_Write_AA241X_PRESSURE_2(void)
{
    struct log_AA241X_PRESSURE_2 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AA241X_PRESSURE_2_MSG),
        // High Frequency Variables
        cpu_time_ms   :  CPU_time_ms,
        
        arduino_CPU_time        :    Arduino_CPU_time,
        p9   :    P9,
        p10   :    P10,
        p11   :    P11,
        p12   :    P12,
        p13   :    P13,
        p14   :    P14,
        p15   :    P15,
        p16   :    P16,
        
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}



////////////////// STARDUPLANE DATA PACKETS //////////////////

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "IHIhhhBH", "LTime,MLC,gDt,GDx,GDy,GDz,I2CErr,INSErr" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "BH",         "SType,CTot" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Icccchhf",    "TimeMS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,AccY" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "ICfccccfI",   "TimeMS,Yaw,WpDist,TargBrg,NavBrg,AltErr,Arspd,Alt,GSpdCM" },
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "IffffBBf",   "TimeMS,DistCM,Volt,BaroAlt,GSpd,Thr,Cnt,Corr" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "IHB", "TimeMS,ArmState,ArmChecks" },
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "IBBcfff",  "TimeMS,Type,State,Servo,Demanded,Achieved,P" },
    { LOG_AA241X_HF1_MSG, sizeof(log_AA241X_HF1),
      "HF1",  "fhffffffffffff",     "CP_t,Z_pB,fi,theta,psi,p,q,r,pdot,qdot,rdot,accx,accy,accz" },
    { LOG_AA241X_HF2_MSG, sizeof(log_AA241X_HF2),
      "HF2",  "fHHHHHHHHHHHH",     "CP_t,I1,I2,I3,I4,I5,I6,O1,O2,O3,O4,O5,O6" },
    { LOG_AA241X_HF3_MSG, sizeof(log_AA241X_HF3),
      "HF3",  "fffBBBB",     "CP_t,AS,VzB,CoMo,LoSt,TSet,TNbr" },
    { LOG_AA241X_PRESSURE_MSG, sizeof(log_AA241X_PRESSURE),
      "PRS",  "ffffffffff",  "CP_t,A_t,P1,P2,P3,P4,P5,P6,P7,P8" },
    { LOG_AA241X_PRESSURE_2_MSG, sizeof(log_AA241X_PRESSURE_2),
      "PRS2",  "ffffffffff",  "CP_t,A_t,P9,P10,P11,P12,P13,P14,P15,P16" },
#if OPTFLOW == ENABLED
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),
      "OF",   "IBffff",   "TimeMS,Qual,flowX,flowY,bodyX,bodyY" },
#endif
    TECS_LOG_FORMAT(LOG_TECS_MSG)
};

#if CLI_ENABLED == ENABLED
// Read the DataFlash.log memory : Packet Parser
static void Log_Read(uint16_t log_num, int16_t start_page, int16_t end_page)
{
    cliSerial->printf_P(PSTR("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n"),
                        (unsigned)hal.util->available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             print_flight_mode,
                             cliSerial);
}
#endif // CLI_ENABLED

// start a new log
static void start_logging() 
{
    DataFlash.StartNewLog();
    DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));
#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
    DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

    // write system identifier as well if available
    char sysid[40];
    if (hal.util->get_system_id(sysid)) {
        DataFlash.Log_Write_Message(sysid);
    }
}

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Startup(uint8_t type) {}
static void Log_Write_Current() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_TECS_Tuning() {}
static void Log_Write_Performance() {}
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd) {}
static void Log_Write_Attitude() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_GPS(uint8_t instance) {}
static void Log_Write_IMU() {}
static void Log_Write_RC() {}
static void Log_Write_Airspeed(void) {}
static void Log_Write_Baro(void) {}
static void Log_Write_Sonar() {}
#if OPTFLOW == ENABLED
static void Log_Write_Optflow() {}
#endif
static void Log_Arm_Disarm() {}

static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}


#endif // LOGGING_ENABLED
