/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.org/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

#include "Copter.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130),
    SCHED_TASK(throttle_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,  200,    160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(read_aux_switches,     10,     50),
    SCHED_TASK(arm_motors_check,      10,     50),
    SCHED_TASK(auto_disarm_check,     10,     50),
    SCHED_TASK(auto_trim,             10,     75),
    SCHED_TASK(read_rangefinder,      20,    100),
    SCHED_TASK(update_proximity,     100,     50),
    SCHED_TASK(update_beacon,        400,     50),
    SCHED_TASK(update_visual_odom,   400,     50),
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(run_nav_updates,       50,    100),
    SCHED_TASK(update_throttle_hover,100,     90),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(compass_accumulate,   100,    100),
    SCHED_TASK(barometer_accumulate,  50,     90),
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75),
#endif
    SCHED_TASK(fourhundred_hz_logging,400,    50),
    SCHED_TASK(update_notify,         50,     90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(gpsglitch_check,       10,     50),
    SCHED_TASK(landinggear_update,    10,     75),
    SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK(gcs_check_input,      400,    180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK(gcs_send_deferred,     50,    550),
    SCHED_TASK(gcs_data_stream_send,  50,    550),
    SCHED_TASK(update_mount,          50,     75),
    SCHED_TASK(update_trigger,        50,     75),
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK(dataflash_periodic,    400,    300),
    SCHED_TASK(perf_update,           0.1,    75),
    SCHED_TASK(read_receiver_rssi,    10,     75),
    SCHED_TASK(rpm_update,            10,    200),
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
#if ADSB_ENABLED == ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100),
#endif
    SCHED_TASK(terrain_update,        10,    100),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK(gripper_update,        10,     75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
    SCHED_TASK(button_update,          5,    100),
    SCHED_TASK(stats_update,           1,    100),
#if XBEE_TELEM==ENABLED
//  SCHED_TASK(swarm_formation,        10,    100),             //swarm formation
    SCHED_TASK(swarm_test,             10,    100),            //a copter fly with desired velocity
    //SCHED_TASK(swarm_test2, 10,100),
   //SCHED_TASK(swarm_test3, 10,100),
   //SCHED_TASK(swarm_formation, 10,100),
   //SCHED_TASK(swarm_formation_v2, 10,100),
#endif
};


void Copter::setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = AP_HAL::micros();
}

/*
  try to accumulate a baro reading
 */
void Copter::barometer_accumulate(void)
{
    barometer.accumulate();
}

void Copter::perf_update(void)
{
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PERF: %u/%u %lu %lu",
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

/*
  update AP_Stats
 */
void Copter::stats_update(void)
{
    g2.stats.update();
}

void Copter::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available > MAIN_LOOP_MICROS ? 0u : time_available);
}


// Main loop - 400hz
void Copter::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();
    
    // run low level rate controllers that only require IMU data
    attitude_control->rate_controller_run();

    // send outputs to the motors library immediately
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
#endif //HELI_FRAME

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading or position
    check_ekf_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_thr_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
}

// update_mount - update camera mount position
// should be run at 50hz
void Copter::update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif
}

// update camera trigger
void Copter::update_trigger(void)
{
#if CAMERA == ENABLED
    camera.update_trigger();
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// Full rate logging of attitude, rate and pid loops
// should be run at 400hz
void Copter::fourhundred_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
        if (rssi.enabled()) {
            DataFlash.Log_Write_RSSI(rssi);
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration(ins);
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control->control_monitor_log();
        Log_Write_Proximity();
        Log_Write_Beacon();
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs().send_message(MSG_SERVO_OUTPUT_RAW);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif

#if PRECISION_LANDING == ENABLED
    // log output
    Log_Write_Precland();
#endif
}

void Copter::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    // update ch6 in flight tuning
    tuning();
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    arming.update();

    if (!motors->armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // set all throttle channel settings
        motors->set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    check_usb_mux();

    // log terrain data
    terrain_logging();

    adsb.set_is_flying(!ap.land_complete);
    
    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    update_sensor_status_flags();
}

// called at 50hz
void Copter::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
                DataFlash.Log_Write_GPS(gps, i);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

void Copter::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

// read baro and rangefinder altitude at 10hz
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

#if XBEE_TELEM==ENABLED
/*==================copter fly with desired velocity================*/
void Copter::swarm_test()
{
    if(copter.control_mode!=GUIDED)
        return;
    if(copter.control_mode==GUIDED)
    {
        if(!time_start_flag)
        {
            time_start=AP_HAL::millis();
            time_start_flag = true;
        }

        
        uint32_t time_current;
        time_current=AP_HAL::millis();
        uint32_t time_delta=time_current-time_start;
        /*float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;*/
        Vector3f desire_vel;
        if(time_delta>0  && time_delta<=10000)
        {
            desire_vel.x=1.5f;
            desire_vel.y=0.0f;
            desire_vel.z=0.0f;
            copter.guided_set_velocity(Vector3f(desire_vel.x * 100.0f, desire_vel.y * 100.0f, -desire_vel.z * 100.0f), false, 0, false, 0, false);
        }
    }

}
/*==================fly a square================*/

void Copter::swarm_test2()
{
    if(copter.control_mode!=GUIDED)
        return;
    if(copter.control_mode==GUIDED)
    {
        if(!time_start_flag)
        {
            time_start=AP_HAL::millis();
            time_start_flag = true;
        }
        uint32_t time_current;
        time_current=AP_HAL::millis();
        uint32_t time_delta=time_current-time_start;
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        Vector3f desire_vel;
        if(time_delta>0  && time_delta<=10000)
        {
            desire_vel.x=1.5f;
            desire_vel.y=0.0f;
            desire_vel.z=0.0f;
            copter.guided_set_velocity(Vector3f(desire_vel.x * 100.0f, desire_vel.y * 100.0f, -desire_vel.z * 100.0f), false, yaw_cd, false, yaw_rate_cds, yaw_relativeguided_set_velocit);
        }
        else if(time_delta>10000 && time_delta<=20000)
        {
            desire_vel.x=0.0f;
            desire_vel.y=1.5f;
            desire_vel.z=0.0f;
            copter.guided_set_velocity(Vector3f(desire_vel.x * 100.0f, desire_vel.y * 100.0f, -desire_vel.z * 100.0f), false, yaw_cd, false, yaw_rate_cds, yaw_relative);
        }
        else if(time_delta>20000 && time_delta<=30000)
        {
            desire_vel.x=-1.5f;
            desire_vel.y=0.0f;
            desire_vel.z=0.0f;
            copter.guided_set_velocity(Vector3f(desire_vel.x * 100.0f, desire_vel.y * 100.0f, -desire_vel.z * 100.0f), false, yaw_cd, false, yaw_rate_cds, yaw_relative);
        }
        else if(time_delta>30000 && time_delta<=40000)
        {
            desire_vel.x=0.0f;
            desire_vel.y=-1.5f;
            desire_vel.z=0.0f;
            copter.guided_set_velocity(Vector3f(desire_vel.x * 100.0f, desire_vel.y * 100.0f, -desire_vel.z * 100.0f), false, yaw_cd, false, yaw_rate_cds, yaw_relative);
        }

        //land
        if(time_delta>40000)Vector3f desire_vel;
        {
            copter.set_mode(LAVector3f desire_vel;COMMAND);
        }
        
    
    }   

}


/*==================fly a circle and land================*/

#define CIRCLE_RADIUS 2
#define OMEGA 1
void Copter::swarm_test3()
{
    if(copter.control_mode!=GUIDED)
        return;
    if(copter.control_mode==GUIDED)
    {
        if(!time_start_flag)
        {
            time_start=AP_HAL::millis();
            time_start_flag = true;
        }
        uint32_t time_current;
        time_current=AP_HAL::millis();
        uint32_t time_delta=time_current-time_start;
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        Vector3f desire_vel;
        if(time_delta>0  && time_delta<=100000)
        {
            desire_vel.x=OMEGA*CIRCLE_RADIUS*cosf(OMEGA*time_delta);
            desire_vel.y=-OMEGA*CIRCLE_RADIUS*sinf(OMEGA*time_delta);
            desire_vel.z=0.0f;
            copter.guided_set_velocity(Vector3f(desire_vel.x * 100.0f, desire_vel.y * 100.0f, -desire_vel.z * 100.0f), false, yaw_cd, false, yaw_rate_cds, yaw_relative);
        }
        else
        {
            copter.set_mode(LAND, MODE_REASON_GCS_COMMAND);
        }
        
      gcs().send_text(MAV_SEVERITY_INFO,"RUNNNING SWARM CONTROL!");  
    }
    
}

/* ===========================swarm copter control==========================*/
#define DESIRED_X 3
#define DESIRED_Y 0
#define GAMA 2
#define RADIUS 5
#define OMEGA 1
int array[2][2]={{0,1},{1,0}};
void Copter::swarm_formation()
{
    if(copter.control_mode!=GUIDED)
        return;
    if(copter.control_mode==GUIDED)
    {

        if(!time_start_flag)
        {
            time_start=AP_HAL::millis();
            time_start_flag = true;
        }       
         uint32_t time_current=AP_HAL::millis();
        uint32_t time_delta=time_current-time_start;

        static uint32_t control_x=RADIUS*OMEGA*cosf(OMEGA*time_delta);
        static uint32_t control_y=-RADIUS*OMEGA*sinf(OMEGA*time_delta);

        mavlink_global_position_int_t gpos;
        Location neighbor_loc;
        Vector2f loc_diff, neighbor_vel;
        Location self_loc = copter.current_loc;
        int32_t self_hdg = copter.ahrs.yaw_sensor;                  //cetidegrees
        if(self_hdg > 18000)
            self_hdg = self_hdg - 36000;
        int32_t hdg_diff = 0;
        const Vector3f &self_vel = gps.velocity();
    
        Vector2f range = location_diff(self_loc, copter.ahrs.get_home());

        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
         Vector3f desire_vel;
        for(int i=0;i<MAX_NEI;i++)
        {
           for(int j=0;j<MAX_NEI;j++)
           {
                if(array[i][j]==1)
                {
                    if(get_neighbours(j,gpos))
                    {
                        Vector2f dist_ij=location_diff(self_loc,neighbor_loc);
                        neighbor_loc.lat = gpos.lat;
                        neighbor_loc.lng = gpos.lon;
                        neighbor_loc.alt = gpos.alt/10; //cm
                        neighbor_vel = Vector2f(gpos.vx, gpos.vy);
                        control_x=control_x-array[i][j]*dist_ij.x-DESIRED_X+GAMA*(self_vel.x-neighbor_vel.x);
                        control_y=control_y-array[i][j]*dist_ij.y-DESIRED_Y+GAMA*(self_vel.y-neighbor_vel.y);
                    }
                }
           }
           
            desire_vel.x=self_vel.x+0.1*control_x;
            desire_vel.y=self_vel.y+0.1*control_y;
            desire_vel.z=0.0f;                
            copter.guided_set_velocity(Vector3f(desire_vel.x * 100.0f, desire_vel.y * 100.0f, -desire_vel.z * 100.0f), true, yaw_cd, true, yaw_rate_cds, yaw_relative);
        }
    }        
}
//	guided_state.forced_rpy_cd.x = 0;//degrees(q.get_euler_roll()) * 100.0f;
//	guided_state.forced_rpy_cd.y = 0;//degrees(q.get_euler_pitch()) * 100.0f;
//	guided_state.forced_rpy_cd.z = 0;//degrees(q.get_euler_yaw()) * 100.0f;
//	guided_state.forced_throttle = 100.0f;
//	guided_state.last_forced_rpy_ms.x = now;
//	guided_state.last_forced_rpy_ms.y = now;
//	guided_state.last_forced_rpy_ms.z = now;
//	guided_state.last_forced_throttle_ms = now;

#define LEADER 1
#define DESIRED_X -300
#define DESIRED_Y 0
void Copter::swarm_formation_v2(void)
{
    if(copter.control_mode==GUIDED)
    {
        mavlink_global_position_int_t gpos;
        Location neighbor_loc;
        Vector3f neighbor_vel;
        if(get_neighbours(LEADER,gpos))
        {
            neighbor_loc.lat = gpos.lat;
	        neighbor_loc.lng = gpos.lon;
	        neighbor_vel.x = gpos.vx;
	        neighbor_vel.y = gpos.vy;
            neighbor_vel.z = gpos.vz;
	        float neighbor_hdg = gpos.hdg / 100 * 3.14159 / 180;
            Vector3f local_des_g;
	        local_des_g.x = cos(neighbor_hdg)*DESIRED_X - sin(neighbor_hdg)*DESIRED_Y;
	        local_des_g.y = sin(neighbor_hdg)*DESIRED_X + cos(neighbor_hdg)*DESIRED_Y;
            location_offset(neighbor_loc, local_des_g.x, local_des_g.y);
            Vector3f destination;
            destination.x = neighbor_loc.lat;
            destination.y = neighbor_loc.lng;
            destination.z = neighbor_loc.alt;
            copter.guided_set_destination_posvel(destination, Vector3f(neighbor_vel.x, neighbor_vel.y, -neighbor_vel.z), false, 0, false, 0, false);
        }
    }
   

}


#endif
AP_HAL_MAIN_CALLBACKS(&copter);
