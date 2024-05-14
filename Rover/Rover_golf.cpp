#include "Rover.h"

#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include <AP_RangeFinder/AP_RangeFinder_Backend.h> // Josh
#include <AP_Proximity/AP_Proximity_Backend.h>     // Josh
#include <AP_Proximity/AP_Proximity_RangeFinder.h> // Josh
#include <AP_RangeFinder/AP_RangeFinder.h>         // Josh
// #include <AP_RangeFinder/AP_RangeFinder_SR73F.h>  // Josh
// static RangeFinder lidar;
#endif

void Rover::hundred_hz_loop(void)
{
    // test
    // sim_pi_guide();
    // calc_triangle_sidelen(1.414,1,45);//1
    // calc_triangle_angleC(2,1.732,1);//30
    if (pi_ctl != true) // 202310 gps
        return;

    float scale = 0.2f;
    if (yaw_enable)
    {
        //float yaw_get = degrees(ahrs.yaw);
        float yaw_get,accuracy_deg;
        uint32_t time_ms;
        gps.gps_yaw_deg(yaw_get, accuracy_deg, time_ms);
        //gcs().send_text(MAV_SEVERITY_INFO, "GPS_yaw=%.0f.",yaw_get); 
        float yaw_rate_to = 0;
        // 全部转到0-360比较
        if (yaw_get < 0)
            yaw_get += 360.0f;

        // update last time angle
        if (fabsf(old_yaw - yaw_get) < g.steer_error * 0.85)
            pie_ctl_times++;
        else
        {
            old_yaw = yaw_get;
            pie_ctl_times = 0;//pie_ctl_times / 2
        }
        yaw_rate_to = yaw_desire - yaw_get;
        if (yaw_rate_to < 0)
            yaw_rate_to += 360.0f;
        /*       if (yaw_rate_to > 180)
                   //yaw_rate_to = 360-yaw_rate_to;//反向转yaw_rate_to
                   yaw_rate_to = -1;

               //否则正向
               yaw_rate_to = yaw_rate_to > 0 ? g.steer_rate_use : -g.steer_rate_use;
        */
        //--------------update 20211124------------
        if (yaw_rate_to > 180)
            yaw_rate_to = yaw_rate_to - 360; // 反向转
        // yaw_rate_to = yaw_rate_to / 180.f * g.steer_rate_use; // scale +-180 to +-4500
        if (yaw_rate_to > 0)
        {
            yaw_rate_to = g.steer_yaw_min + (yaw_rate_to + pie_ctl_times * scale) * g.golf_yawrate_k;
            if (yaw_rate_to > g.steer_rate_use)
            {
                 yaw_rate_to = g.steer_rate_use;            
            }

        }
        else if (yaw_rate_to < 0)
        {
            yaw_rate_to = -g.steer_yaw_min + (yaw_rate_to - pie_ctl_times * scale) * g.golf_yawrate_k;
            if (yaw_rate_to < -g.steer_rate_use)
            {
                 yaw_rate_to = -g.steer_rate_use;            
            }

        }

        //--------------end------------------------
        if (fabsf(yaw_desire - yaw_get) < g.steer_error)
        {
            rover.mode_gobatt.set_para(); // stop
            yaw_enable = false;
            yaw_complete = true;
            yaw_rate_to = 0;
            pi_ctl_start = AP_HAL::millis();
            one_hz_times = 0;
            pie_ctl_times = 0;
        }
        else
        {
            float throttle = 0;
            if(fabsf(yaw_rate_to) > g.steer_rate_use * 0.9 && pie_ctl_times > 50)// 4050 when rover can't rotate even though the steer is up to the max
                throttle = g.golf_forward + pie_ctl_times * 0.05;   
        
            rover.mode_gobatt.set_para(throttle, yaw_rate_to);
            gcs().send_text(MAV_SEVERITY_INFO, "gps desire=%.0f get=%.0f throttle=%.0f turn=%.0f", yaw_desire, yaw_get,throttle, yaw_rate_to);
        }

    }
}

void Rover::one_hz_loop(void)
{
    // debug info
    int imode = control_mode->mode_number();
    
    if(old_imode != imode &&(imode == 10 || imode == 11))//auto or rtl mode enable fence
    {
        if(!g2.fence.enabled())
        {
            g2.fence.enable(true);
            gcs().send_text(MAV_SEVERITY_NOTICE, "auto or rtl mode enable fence.");
        }
         //enable laider
        enable_rangefinder(-1, true);
            
    }

    if(old_imode != imode || old_isSleep != isSleep)
    {
        old_imode = imode;
        old_isSleep = isSleep;
        gcs().send_text(MAV_SEVERITY_INFO, "Mode= %d, work_enable= %d, isSleep=%d", imode, work_enable, isSleep);
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "golf_work_state = %d ", golf_work_state);
    if (pi_ctl && oldpi_ctl_step != pi_ctl_step)
    {
        oldpi_ctl_step = pi_ctl_step;
        gcs().send_text(MAV_SEVERITY_INFO, "pi_ctl= %d, step=%d", pi_ctl, pi_ctl_step);
    }

    if((one_hz_times > 0 && one_hz_times%60 == 0) || (test_work_s>0 && test_work_s%60 == 0))
    {
        if(pi_ctl)
            gcs().send_text(MAV_SEVERITY_INFO, "Mode= %d,isSleep=%d,step=%d", imode, isSleep,pi_ctl_step);
        else
            gcs().send_text(MAV_SEVERITY_INFO, "Mode= %d,isSleep=%d", imode, isSleep);
    }
       

    float batt_volt = battery.voltage();
    //gcs().send_text(MAV_SEVERITY_INFO, "batt_volt %.2f", batt_volt);
    
    if(g.batt_nd_rtl > 1.5f && g.batt_nd_rtl < 2.5f)
    {
        float pitch_get = degrees(ahrs.get_pitch());
    //test the distance to home   
        float  distance1 = rover.current_loc.get_distance(ahrs.get_home())*100;
        gcs().send_text(MAV_SEVERITY_INFO, "Golf Dis from home=%.0f.",distance1); 
    //test GPS yaw
        float yaw_deg,accuracy_deg;
        uint32_t time_ms;
        gps.gps_yaw_deg(yaw_deg, accuracy_deg, time_ms);
        gcs().send_text(MAV_SEVERITY_INFO, "GPS_yaw=%.0f,pitch=%.0f.",yaw_deg,pitch_get); 
    //status().gps_yaw;
    }
    //TEST DOOR
    //motor_push();
    //gcs().send_text(MAV_SEVERITY_INFO, "motor_push");

    // if (work_enable && (AP_HAL::millis() - rover_golf_start > 5 * 3600 * 1000) && rover_golf_start!=0)
    //{
    //     rover.golf_end_mission();
    // }

    // float test_distance_cm;
    // test_distance_cm = rangefinder.get_data((uint8_t)0);
    // gcs().send_text(MAV_SEVERITY_INFO, "0 %f", test_distance_cm);
    // test_distance_cm = rangefinder.get_data((uint8_t)1);
    // gcs().send_text(MAV_SEVERITY_INFO, "1 %f", test_distance_cm);

    //test UWB
    if(g.batt_nd_rtl > 0.f && g.batt_nd_rtl < 1.5f)
    {
        float dis = 0.0f, angle = 0.0f;
        g2.beacon.get_data(dis, angle);
        dis = dis * 100;//m->cm
        gcs().send_text(MAV_SEVERITY_NOTICE, "uwb dis=%.2f angle=%.2f ",dis, angle);
    }
    if (imode == 0 || golf_work_state == GOLF_NOWORK) // manual
        return;

    // sr73f_can.update();
    // golf: regular start&retur
    uint8_t hour, min, sec;
    uint16_t ms;
    // 获取现在的UTC时间 时 分 秒
    if (!AP::rtc().get_local_time(hour, min, sec, ms))
        gcs().send_text(MAV_SEVERITY_DEBUG, "UTC get time faild!");
    // else
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "H:M:S %d:%d:%d", hour, min, sec);
    batt_nd_charge = false;
    bool batt_is_low = (batt_volt < g.batt_nd_rtl) ? true : false;
    bool batt_is_full = (batt_volt > g.batt_charge_to) ? true : false;
    if (batt_is_low)
    {
        batt_nd_charge = true;
    }
    //sitl 
    static bool door_nd_close = false;
    golf_is_full = false;//!(rover.check_digital_pin(AUX_GOLF_PIN));
    if (golf_is_full)
        gcs().send_text(MAV_SEVERITY_INFO, "golf full %i", golf_is_full);
    nd_collision = false;//!(rover.check_digital_pin(AUX_AVOID_PIN));
    if (nd_collision)
        gcs().send_text(MAV_SEVERITY_INFO, "collision %i", nd_collision);
    // if(golf_is_full) gcs().send_text(MAV_SEVERITY_INFO, "golf full %i", golf_is_full);
    // if(nd_collision) gcs().send_text(MAV_SEVERITY_INFO, "collision %i", nd_collision);
    uint8_t nd_avd = 0;

    // Begin Josh

    if (pi_ctl == false && rover.control_mode->is_autopilot_mode())
    {
        motor_pull();
    }

    // #if HAL_ENABLE_LIBUAVCAN_DRIVERS
    //      float distance_cm;
    // #endif
    //  char            c = 0;
    //	static char   buf[16];
    //	static unsigned int i;
    //      while(hal.uartD->available() > 0 && c != '\n' && i < sizeof(buf)){
    //   if(hal.uartD->available() > 0 && c != '\n' ){
    //		c 		 = hal.uartD->read();
    //		buf[i++] = c;
    //        gcs().send_text(MAV_SEVERITY_INFO, "UART D %d", c);   // Josh for IR on uartD
    //}

    // float last_time;
    // bool hasDate;
    if (golf_work_state != GOLF_PI_CTL && nd_collision) // Josh  collision
    {
        // golf_work_state = GOLF_COLLISION; uwb202207
    }
    // #if HAL_ENABLE_LIBUAVCAN_DRIVERS
    else
    {

        // Josh  2021August12

        //    float distance0=0.0, distance45=0.0, distance315=0.0;
        //   get_proximity_dis(distance0,distance45,distance315);

        //           	hal.console->printf("prx_status=%d\r\n",g2.proximity.get_status() );
        // AP_Proximity::Proximity_Distance_Array dist_array;
        // if (g2.proximity.get_horizontal_distances(dist_array))
        // {
        //     distance0 =dist_array.distance[0];
        //     distance45 =dist_array.distance[1];
        //     distance315 =dist_array.distance[7];

        // }

        /*           if(g2.proximity.get_status() == AP_Proximity::Proximity_Good)
                   {
                      g2.proximity.get_horizontal_distances(0.0, distance0);
                //    g2.proximity.get_horizontal_distance(45.0, distance45);
                    //g2.proximity.get_horizontal_distance(315.0, distance315);
                   }
         */
        /*
              if (distance0 > 0.50 && distance0 < 5.00)
                  gcs().send_text(MAV_SEVERITY_INFO, "0=%lf ", distance0);
              if (distance45 > 0.50 && distance45 < 5.00)
                  gcs().send_text(MAV_SEVERITY_INFO, "45=%lf ", distance45);
              if (distance315 > 0.50 && distance315 < 5.00)
                  gcs().send_text(MAV_SEVERITY_INFO, "315=%lf ", distance315);


              if (((distance0 > 1.00 && distance0 < 5.00) || (distance45 > 1.00 && distance45 < 5.00) || (distance315 > 1.00 && distance315 < 5.00))
                          && pi_ctl == false && rover.control_mode->is_autopilot_mode())
              {
                   nd_avd = 1;
                   golf_work_state = GOLF_WORK;
                   gcs().send_text(MAV_SEVERITY_INFO, "set avoid");
              }
              gcs().send_text(MAV_SEVERITY_INFO, "0=%lf 45=%lf 315=%lf v=%i", distance0,distance45,distance315, nd_avd);
          // end of 2021August12
  */

        /*
                for (uint8_t i = 0; i < rover.rangefinder.num_sensors(); i++)
                {

                    AP_RangeFinder_Backend *s = rover.rangefinder.get_backend(i);
                    if (s == nullptr)
                    {
                        continue;
                    }

                    distance_cm = s->distance_cm();
                    // target_deg = s->target_deg();
                    //last_time = s->last_reading_ms();
                    //hasDate = s->has_data;
        //            gcs().send_text(MAV_SEVERITY_INFO, "lidar #%i  status = %i  count = %i", i,  s->status(), s->range_valid_count());
                    //        gcs().send_text(MAV_SEVERITY_INFO, "#i=%i dist=%lf deg=%f status = %i", i,distance_cm,target_deg, s->status());
                    if ((distance_cm > 100.0) && (distance_cm < 500.0) && (pi_ctl == false) && (rover.control_mode->is_autopilot_mode()) && (s->status() == RangeFinder::Status::Good))
                    //           &&  s->has_data() )
                    {
                        //             gcs().send_text(MAV_SEVERITY_INFO, "lidar dist #%i =  %lf", i, distance_cm);
                         nd_avd = 1;
                         golf_work_state = GOLF_WORK;
                    }
                    else
                    {
        //                nd_avd = 0;
                    }

                }
        */
    }

    //    gcs().send_text(MAV_SEVERITY_INFO, "mode=%i work_enable=%i state=%i colli=%i", rover.control_mode->mode_number(),
    //                    work_enable, golf_work_state, nd_collision);
    // #endif

    // End Josh

    // 定时启动
    if (g.golf_timing_enable == 1)
    {
        bool time1_avaiable = !((g2.start_1_hour == 0) && (g2.start_1_min == 0) && (g2.end_1_hour == 0) && (g2.end_1_min == 0));
        bool time1_to_start = ((hour == (uint8_t)g2.start_1_hour) && (min == (uint8_t)g2.start_1_min) && (sec < 4));
        //    bool time1_to_start = ((hour*24+min >= (uint8_t)g2.start_1_hour*24+(uint8_t)g2.start_1_min)
        //                        && (hour*24+min < (uint8_t)g2.end_1_hour*24 +(uint8_t)g2.end_1_min) && (sec < 4));
        bool time1_to_end = ((hour == (uint8_t)g2.end_1_hour) && (min == (uint8_t)g2.end_1_min) && (sec < 4));
        bool time2_avaiable = !((g2.start_2_hour == 0) && (g2.start_2_min == 0) && (g2.end_2_hour == 0) && (g2.end_2_min == 0));
        bool time2_to_start = ((hour == (uint8_t)g2.start_2_hour) && (min == (uint8_t)g2.start_2_min) && (sec < 4));
        //    bool time2_to_start = ((hour*24+min >= (uint8_t)g2.start_2_hour*24+(uint8_t)g2.start_2_min)
        //                        && (hour*24+min < (uint8_t)g2.end_2_hour*24 +(uint8_t)g2.end_2_min) && (sec < 4));
        bool time2_to_end = ((hour == (uint8_t)g2.end_2_hour) && (min == (uint8_t)g2.end_2_min) && (sec < 4));
        bool time3_avaiable = !((g2.start_3_hour == 0) && (g2.start_3_min == 0) && (g2.end_3_hour == 0) && (g2.end_3_min == 0));
        bool time3_to_start = ((hour == (uint8_t)g2.start_3_hour) && (min == (uint8_t)g2.start_3_min) && (sec < 4));
        //    bool time3_to_start = ((hour*24+min >= (uint8_t)g2.start_3_hour*24+(uint8_t)g2.start_3_min)
        //                        && (hour*24+min < (uint8_t)g2.end_3_hour*24 +(uint8_t)g2.end_3_min) && (sec < 4));
        bool time3_to_end = ((hour == (uint8_t)g2.end_3_hour) && (min == (uint8_t)g2.end_3_min) && (sec < 4));

        //        gcs().send_text(MAV_SEVERITY_INFO, "t1s=%i t1e=%i, slp=%i", time1_to_start, time1_to_end, isSleep);

        if ((time1_to_start && time1_avaiable) ||
            (time2_to_start && time2_avaiable) ||
            (time3_to_start && time3_avaiable))
        {
            isperiod = true;
            triggerhour = hour+1;
            triggermin = min;
            triggerhour = triggerhour % 24;
            gcs().send_text(MAV_SEVERITY_INFO,"Trig hour:%d,min:%d",triggerhour,triggermin);
            test_work_s = 0;
            needsleep = false;
            golf_set_sleepflg(0);
            gcs().send_text(MAV_SEVERITY_INFO, "Time to start.");
        }

        if ((time1_to_end && time1_avaiable) ||
            (time2_to_end && time2_avaiable) ||
            (time3_to_end && time3_avaiable)

        )
        {
            isperiod = false;
            triggerhour = 0;
            triggermin = 0;
            // 结束了之后回去充电
            golf_set_sleepflg(1);
            gcs().send_text(MAV_SEVERITY_INFO, "Time to end.");
        }
    }
    else if(isperiod)
    {
        isperiod = false;
        triggerhour = 0;
        triggermin = 0;
        gcs().send_text(MAV_SEVERITY_INFO, "Timing_enable = 0.");
    }

    if(isperiod)
    {
        bool gosleeping = false;
        //exchange work status
       
        if((!isSleep && pi_ctl) /*&& !needsleep*/)
        {
            int triggertotalmins = triggerhour * 60 + triggermin;
            triggertotalmins = triggertotalmins-5; //reduce 5 mins to prepare pi ctrl 
            if(triggertotalmins < 0)
                triggertotalmins = triggertotalmins + 24 * 60;
            if(hour * 60 + min >= triggertotalmins)
                gosleeping = true;
            
        }

        
        if((triggerhour == hour && triggermin == min) || gosleeping)
        {
            isSleep = !isSleep;
            triggerhour = triggerhour + 1;
            triggerhour = triggerhour % 24;
            gcs().send_text(MAV_SEVERITY_INFO,"Trig hour:%d,min:%d",triggerhour,triggermin);
            if(isSleep)//sleep to charge
            {
                golf_set_sleepflg(1);
                gcs().send_text(MAV_SEVERITY_INFO, "Time to charge.");
            }
            else
            {
                golf_set_sleepflg(0);
                gcs().send_text(MAV_SEVERITY_INFO, "Time to pick ball.");
            }
        }
    }
    // 状态机
    //    gcs().send_text(MAV_SEVERITY_DEBUG, "golf_work_state = %d ", golf_work_state);

    switch (golf_work_state)
    {
    case GOLF_COLLISION:
        if (work_enable)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf collision");
            rover.set_mode(rover.mode_hold, ModeReason::EVERYDAY_END);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf collision backward");
            if (pi_ctl) // control_mode->mode_number() != Mode::Number::HOLD)
            {
                rover.set_mode(rover.mode_hold, ModeReason::EVERYDAY_END);
                // SleepForSeconds(10);
            }
            rover.set_mode(rover.mode_gobatt, ModeReason::EVERYDAY_END);
            pi_ctl_id = 9010;
            //            golf_send_cmd(pi_ctl_id, rover.ahrs.yaw_sensor, target_deg); // Josh added parameters
            //            target_deg = target_deg > 0 ? fabsf(target_deg) - 45.0 : 45.0 - fabsf(target_deg);
            yaw_desire = constrain_deg(constrain_deg(degrees(rover.ahrs.yaw_sensor)) + 0);
            yaw_enable = true;
            yaw_complete = false;
            nd_collision = 0;
            golf_work_state = GOLF_PI_AVOID;
            work_golf_back = false;
            pi_ctl = true;
        }
        break;
    case GOLF_HOLD:
        if (work_enable)
        {
             AP_GPS::GPS_Status gpsst = gps.status();
                
            if (gpsst == AP_GPS::GPS_OK_FIX_3D_RTK_FIXED  || gpsst == AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) // wait for gps rtk
            {
                rover.set_mode(rover.mode_auto, ModeReason::EVERYDAY_START);
                arming.arm(AP_Arming::Method::RUDDER);
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf Time Start");
                golf_work_state = GOLF_WORK;
                work_golf_back = false; // Josh
            }
            else
                gcs().send_text(MAV_SEVERITY_INFO, "Waiting for GPS RTK Fixed.");

        }
        break;
    case GOLF_WORK:
        test_work_s++;
        if (test_work_s >= 5 && door_nd_close)
        {
            ServoRelayEvents.do_set_servo(5, g.pwm_normal);
            door_nd_close = false;
        }
        if ((test_work_s >= g.test_full_sec) || golf_is_full)
        {
            rover.set_mode(rover.mode_rtl, ModeReason::EVERYDAY_END);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf is full");
            test_work_s = 0;
            golf_work_state = GOLF_BACK;
            work_golf_back = true; // Josh

        }
        if (batt_nd_charge)
        {
            rover.set_mode(rover.mode_rtl, ModeReason::EVERYDAY_END);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf batt is low");
            test_work_s = 0;
            golf_work_state = GOLF_BACK;
            work_golf_back = true; // Josh
            golf_set_sleepflg(1.0);
        }
        if (nd_avd)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf avd now");
            //            rover.set_mode(rover.mode_gobatt, ModeReason::EVERYDAY_END);
            golf_work_state = GOLF_PI_AVOID;
            pi_ctl_id = 9020;
            yaw_complete = true;
            pi_ctl_start = AP_HAL::millis();
            target_deg = target_deg > 0 ? fabsf(target_deg) - 45.0 : 45.0 - fabsf(target_deg);
            rover.mode_gobatt.set_para(1.0, target_deg);
            //            golf_send_cmd(pi_ctl_id, rover.ahrs.yaw_sensor, target_deg); // Josh added parameters
            pi_ctl = true;
        }

        if (g2.wp_nav.reached_destination())
        {
            //   gcs().send_text(MAV_SEVERITY_CRITICAL, "after dest");
            golf_work_state = GOLF_BACK;
            work_golf_back = true; // Josh
            rover.set_mode(rover.mode_rtl, ModeReason::EVERYDAY_END);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "after dest, RTL");

            // golf_work_state = GOLF_PREP_PI;
        }
        break;
    case GOLF_BACK:
        if (g2.wp_nav.reached_destination())
        {
            //
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf near home");
            //
            //calculate the distance to home   
            float  dist = rover.current_loc.get_distance(ahrs.get_home())*100;
            gcs().send_text(MAV_SEVERITY_INFO, "GPS from home: %.0f.",dist);
            //CLOSEHOME
            old_gpsdis = dist;
            directionflg = 1;
            //
            if(fabsf(dist) > 200)
            {       
                gcs().send_text(MAV_SEVERITY_INFO, "Golf is too far.");      
                //golf_work_state = GOLF_HOLD;
                //rover.set_mode(rover.mode_manual, ModeReason::EVERYDAY_END);
               // break;

            }
            //  rover.set_mode(rover.mode_rtl, ModeReason::EVERYDAY_END);
            golf_work_state = GOLF_PREP_PI;
        }
        break;
    case GOLF_PREP_PI:
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf prep for pi");
        rover.set_mode(rover.mode_gobatt, ModeReason::EVERYDAY_END);
        golf_work_state = GOLF_PI_CTL;
        pi_ctl_id = 9000;
        // adjust direction
        // yaw_enable = true;
        // yaw_complete = false;
        // yaw_desire = g.golf_yaw;//180
        pi_ctl_start = AP_HAL::millis();
        one_hz_times = 0;
        pie_ctl_times = 0;
        pi_ctl = true;
        lidarvaildflg = false; //check lidar in pictl
        //close to home
        rover.mode_gobatt.set_para(g.golf_forward, 0);//forward
        break;
    case GOLF_PI_CTL:
        // 等树莓派控制完成后pi_ctl会设置成false
        one_hz_times++;
        if (!pi_ctl)
        {
            float  gpsdis = 0;
            gpsdis = rover.current_loc.get_distance(ahrs.get_home())*100;
            gcs().send_text(MAV_SEVERITY_INFO, "Golf Dis from home=%.0f.",gpsdis); 
            if(/*gpsdis < g.golf_gps_dis &&*/ one_hz_times > 3)
            {
                golf_work_state = GOLF_HOLD;
                one_hz_times = 0;
            }
        }
        break;
    case GOLF_LOW_BATT:
        one_hz_times++;
        if (batt_is_full)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf batt is ready");
            // golf_work_state = GOLF_HOLD;
            golf_work_state = GOLF_PI_CTL;
            golf_set_sleepflg(0);
        }
        break;
    case GOLF_PI_AVOID:
        if (!pi_ctl)
        {
            if (work_golf_back) //  Josh during RTL avoidance, after avoid, keep going RTL
            {
                golf_work_state = GOLF_BACK;
                work_golf_back = true; // Josh
                rover.set_mode(rover.mode_rtl, ModeReason::EVERYDAY_END);

            } // Josh END
            else
            {
                rover.set_mode(rover.mode_auto, ModeReason::EVERYDAY_START);
                golf_work_state = GOLF_WORK;
                work_golf_back = false; // Josh
            }
        }
        break;
    default:
        break;
    }
}

void Rover::sim_pi_ctl(void)
{
    //    gcs().send_text(MAV_SEVERITY_DEBUG, "pi_ctl_step = %d",  pi_ctl_step);
    if (pi_ctl)
    {
        int lidardis = get_distance(-1);
        if(lidardis > 0 && lidardis < 10000)
            lidarvaildflg = true;

        switch (pi_ctl_id)
        {
        // collision
        case 9010:
        {
            switch (pi_ctl_step)
            {
            case 0:
                rover.mode_gobatt.set_para(-50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 1:
                if (AP_HAL::millis() - pi_ctl_start > 3000)
                    pi_ctl_step++;
                break;
            case 2:
                rover.mode_gobatt.set_para(-50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 3:
                if (AP_HAL::millis() - pi_ctl_start > 3000)
                {
                    pi_ctl_step = 0;
                    pi_ctl = false;
                    pi_ctl_start = 0;
                }
                break;
            }

            break;
        }
        case 9020: // obscale avoid
        {
            //          gcs().send_text(MAV_SEVERITY_INFO, "9020 obscale aovid step=%i mode=%i", pi_ctl_step, rover.control_mode);

            switch (pi_ctl_step)
            {
            case 0:
                if (yaw_complete)
                    pi_ctl_step++;
                break;
            case 1:
                pi_ctl_start = AP_HAL::millis();
                //                gcs().send_text(MAV_SEVERITY_INFO, "yaw = %f targetDeg = %f ", degrees(rover.ahrs.yaw_sensor) , target_deg);
                yaw_desire = constrain_deg(constrain_deg(degrees(rover.ahrs.yaw_sensor)) + target_deg);
                yaw_enable = true;
                yaw_complete = false;
                pi_ctl_step++;
                break;
            case 2:
                if (yaw_complete)
                    pi_ctl_step++;
                break;
            case 3:
                rover.mode_gobatt.set_para(50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 4:
                if (AP_HAL::millis() - pi_ctl_start > 3000)
                    pi_ctl_step++;
                break;
            case 5:
                rover.mode_gobatt.set_para(50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 6:
                if (AP_HAL::millis() - pi_ctl_start > 3000)
                    pi_ctl_step++;
                break;
            case 7:
                //                gcs().send_text(MAV_SEVERITY_INFO, "yaw = %f targetDeg = %f ", degrees(rover.ahrs.yaw_sensor) , target_deg);
                yaw_desire = constrain_deg(constrain_deg(degrees(rover.ahrs.yaw_sensor)));
                pi_ctl_start = AP_HAL::millis();
                yaw_enable = true;
                yaw_complete = false;
                pi_ctl_step++;
                break;
            case 8:
                if (yaw_complete)
                    pi_ctl_step++;
                break;
            case 9:
                rover.mode_gobatt.set_para(50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 10:
                if (AP_HAL::millis() - pi_ctl_start > 3000)
                    pi_ctl_step++;
                break;
            case 11:
                rover.mode_gobatt.set_para(50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 12:
                if (AP_HAL::millis() - pi_ctl_start > 3000)
                {
                    pi_ctl_step = 0;
                    pi_ctl = false;
                    pi_ctl_start = 0;
                }
                break;
            }
            break;
        }
        case 9000: // reached home then unload ball
        {
            nd_backward = pi_ctl;
            //            gcs().send_text(MAV_SEVERITY_INFO, "9000 step=%i", pi_ctl_step );
            switch (pi_ctl_step)
            {
            case 0:// after return home, check and move rover to home
                //rover.mode_gobatt.stop_rover(); // stop
                if(closetohome(g.uwb_offset))
                {
                    pi_ctl_step++;
                    pi_ctl_start = AP_HAL::millis();
                    one_hz_times = 0;
                }
                break;
            case 1://wait until GPS stable
                {
                    //disable laider
                    enable_rangefinder(-1, false);
                    rover.mode_gobatt.set_para(); // stop
                    float  gdist = rover.current_loc.get_distance(ahrs.get_home())*100;
                    gcs().send_text(MAV_SEVERITY_INFO, "GPS from home: %.0f.",gdist);
                    if (one_hz_times > 3)
                    {
                        pi_ctl_step++;
                        pi_ctl_start = AP_HAL::millis();
                        one_hz_times = 0;
                    }
                }
                break;    
            case 2:                           // start GPS turn to face dock
                rover.mode_gobatt.set_para(); // stop
                // adjust direction
                yaw_desire = g.golf_yaw;
                yaw_enable = true;
                yaw_complete = false;
                pi_ctl_step++;
                pi_ctl_start = AP_HAL::millis();
                one_hz_times = 0;
                pie_ctl_times = 0;
                break;
            case 3: // start uwb
                if (yaw_complete)
                {
                    rover.mode_gobatt.set_para(); // stop
                    uwb_complete = false;
                    sim_pi_guide_state = 0;
                    pi_ctl_step++;
                    pi_ctl_start = AP_HAL::millis();
                    one_hz_times = 0;
                    pie_ctl_times = 0;
                }
                break;
            case 4: // do uwb
                if (uwb_complete)
                {
                    pi_ctl_step++;
                    pi_ctl_start = AP_HAL::millis();
                    one_hz_times = 0;
                    pie_ctl_times = 0;
                    uwb_complete = false;
                    rover.mode_gobatt.set_para(); // stop
                }
                else
                    sim_pi_guide();
                break;
            case 5:// gps again
                rover.mode_gobatt.set_para(); // stop
                // adjust direction
                yaw_desire = g.golf_yaw;
                yaw_enable = true;
                yaw_complete = false;
                pi_ctl_step++;
                pi_ctl_start = AP_HAL::millis();
                one_hz_times = 0;
                pie_ctl_times = 0;           
                break;
            case 6: // check uwb anlge and dis
                if (yaw_complete && one_hz_times > 2)
                {
                    rover.mode_gobatt.set_para(); // stop
                    float dis = 0.0f, angle = 0.0f;
                    g2.beacon.get_data(dis, angle);
                    dis = dis * 100;//m->cm

                    gcs().send_text(MAV_SEVERITY_INFO, "uwb dis=%.2f angle=%.2f ",dis, angle);
                    if(angle > g.uwb_angleL || angle < g.uwb_angleR)//check uwb angle and dis 15,-10 
                    {
                        gcs().send_text(MAV_SEVERITY_NOTICE, "UWB angle is outside, Set rover to backward.");
                        //rover.set_mode(rover.mode_hold, ModeReason::EVERYDAY_END);
                        //golf_end_mission();
                        test_work_s = g.test_full_sec - 60;
                        needsleep = isSleep;//mark sleep flg for auto start massion
                        golf_backward(-1);//back and auto don't change isSleep state
                        return;
                    }
                    else
                    {
                        pi_ctl_start = AP_HAL::millis();
                        one_hz_times = 0;
                        pie_ctl_times = 0;
                        pi_ctl_step++;
                        rover.mode_gobatt.set_para();//stop  
                    }
                }
                break;

            case 7: // forward5
                if (true)
                { 
                    //guide mode to forward
                    //int32_t latitude = g.stage_lat *10000000 + g.stage_latdb;
                    //int32_t longitude = g.stage_long * 10000000 + g.stage_longdb;
                    //gcs().send_text(MAV_SEVERITY_INFO, "fly: lat= %d,long=%d. ",(int)latitude,(int)longitude);
                    //Location target_loc( latitude, longitude,0,Location::AltFrame::ABOVE_HOME);//492253836, -1226664934
                    //calculate the desired location
                    float distance = g.press_up / 100.0f; //cm -> m
                    float angle = g.golf_yaw; //degree (0-360)
                    gcs().send_text(MAV_SEVERITY_INFO, "fly: dis= %.2f,angle=%.0f. ",distance,angle);
                    Location target_loc = calc_desired_location(distance, angle);
                    gcs().send_text(MAV_SEVERITY_INFO, "fly: lat= %d,long=%d. ",(int)target_loc.lat,(int)target_loc.lng);
                    bool rt = fly_to_here(target_loc);
                    if(!rt)
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "Guide mode fly to here error.");
                    rover_reached_stick = false;
                    pi_ctl_start = AP_HAL::millis();
                    one_hz_times = 0;
                    pie_ctl_times = 0;
                    reached_guided = false;
                    pi_ctl_step++;
                    break;
                    //rover.mode_gobatt.set_para(g.golf_throttle); // 50
                    //int disLidar = get_distance(-1);
                    float  dis = rover.current_loc.get_distance(ahrs.get_home())*100;
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS from home: %.0f.",dis);
                    if (dis > g.press_low || one_hz_times > g.golf_time_forward) 
                    {
                        rover_reached_stick = false;
                        pi_ctl_start = AP_HAL::millis();
                        one_hz_times = 0;
                        pie_ctl_times = 0;
                        pi_ctl_step++;
                        rover.mode_gobatt.set_para();//stop
                    }  
                    else
                    {
                        if (fabsf(old_dis - dis) < 20) // cm
                            pie_ctl_times++;
                        else
                        {
                            old_dis = dis;
                            pie_ctl_times = pie_ctl_times / 2;//0
                        }
                        float f = g.golf_throttle + pie_ctl_times * 0.1;
                        rover.mode_gobatt.set_para(f, 0);
                        gcs().send_text(MAV_SEVERITY_INFO, "times= %d, dis= %.1f, forward= %.0f", pie_ctl_times, dis, f);
                    }
                }
                break;
            case 8:
                {   
                    if (!reached_guided && g2.wp_nav.reached_destination())//guide mode reach loc
                    {   
                        reached_stageup = false;
                        reached_guided = true;
                        //enable laider
                        enable_rangefinder(-1, true);
                        gcs().send_text(MAV_SEVERITY_INFO, "guide mode reach loc and open door.");
                        //motor_push(); don't open the door avoid droping ball off
                        pi_ctl_start = AP_HAL::millis();
                        one_hz_times = 0;
                        pie_ctl_times = 0;
                        //pi_ctl_step++;
                        rover.set_mode(rover.mode_gobatt, ModeReason::EVERYDAY_END);
                        gcs().send_text(MAV_SEVERITY_NOTICE, "Working to change mode to gobatt.");
                        //rover.mode_gobatt.set_para(g.golf_throttle);//go ahead
                    }
                    
                    if(!reached_guided)
                        break;
                    // open door and continue forward
                    //int disLidar = get_distance(-1);
                    float  dis = rover.current_loc.get_distance(ahrs.get_home())*100;
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS from home: %.0f.",dis);
                    //if(dis < g.press_up && !reached_stageup)//g.golf_time_forward
                    if((dis < g.press_up && one_hz_times < g.golf_time_forward )&& !reached_stageup)//go forward
                    {
                        if (fabsf(old_dis - dis) < 20) // cm
                            pie_ctl_times++;
                        else
                        {
                            old_dis = dis;
                            pie_ctl_times = pie_ctl_times / 2;//0
                        }
                        float f = g.golf_throttle + pie_ctl_times * 0.1;
                        rover.mode_gobatt.set_para(f, 0);
                        gcs().send_text(MAV_SEVERITY_INFO, "times= %d, dis= %.1f, forward= %.0f", pie_ctl_times, dis, f);
                        //one_hz_times = 0;
                    }
                    //else if(dis > g.press_up && !reached_stageup)//
                    else if((dis > g.press_up || one_hz_times >= g.golf_time_forward)  && !reached_stageup)//stop
                    {
                       // old_dis = dis;
                        pie_ctl_times = 0;
                        if(one_hz_times>g.golf_time_forward+1)//stop 2s and then back
                            reached_stageup = true;
                        rover.mode_gobatt.set_para();  
                        gcs().send_text(MAV_SEVERITY_INFO, "start back times= %d.", pie_ctl_times); 
                    }
                    else if(pie_ctl_times < g.press_low)//back for stage up
                    {
                        pie_ctl_times++;
                        reached_stageup = true;
                        rover.mode_gobatt.set_para(-g.golf_throttleR*0.75);//28-32
                        gcs().send_text(MAV_SEVERITY_INFO, "back times= %d.", pie_ctl_times); 
                    }  
                    else
                    {
                        reached_stageup = true;
                        rover.mode_gobatt.set_para();    
                        gcs().send_text(MAV_SEVERITY_INFO, "back stop.");                   
                    }

                    //motor_push();  drop ball down
                    // if (AP_HAL::millis() - pi_ctl_start > g.golf_time_closedoor)//3000
                    if (one_hz_times > g.golf_time_opendoor) // 5
                    {
                        rover.mode_gobatt.set_para(0); // stop
                        pi_ctl_start = AP_HAL::millis();
                        one_hz_times = 0;
                        pie_ctl_times = 0;
                        pi_ctl_step++;
                        motor_push(); 
                    }
                }
                break;             
            case 9:                           // unload ball
               // switch(one_hz_times % 6)
               // {
                    //case 0://forward
                    //    rover.mode_gobatt.set_para(g.golf_throttle*0.75);
                    //break;
                    //case 4: //back
                    //    rover.mode_gobatt.set_para(-g.golf_throttle); // stop
                    //break;
                //    default: //stop
                //        rover.mode_gobatt.set_para(); // stop
                //    break;
                //}
                rover.mode_gobatt.set_para(); // stop
                motor_push(); //open door

                // if (AP_HAL::millis() - pi_ctl_start > g.golf_time_opendoor)//30000
                if (one_hz_times > g.unload_sec) // 30
                {
                    pi_ctl_start = AP_HAL::millis();
                    one_hz_times = 0;
                    pi_ctl_step++;
                    rover.mode_gobatt.set_para(); // stop
                }
                break;
            case 10:                           // move forward again for charge
                rover.mode_gobatt.set_para(g.golf_throttle); // *1.2
                if (one_hz_times > 3) // 30
                {
                    rover.mode_gobatt.set_para(); // stop
                    pi_ctl_start = AP_HAL::millis();
                    one_hz_times = 0;
                    pi_ctl_step++;
                }
                break;                  
            case 11:
                // close door and wait 3s
                rover.mode_gobatt.set_para(); // stop
                // if (AP_HAL::millis() - pi_ctl_start < g.golf_time_closedoor)//3000
                if (one_hz_times < g.golf_time_closedoor) // 5
                {
                    motor_pull();
                    //    rover.mode_gobatt.set_para(g.golf_throttle/2);
                }
                else if(!lidarvaildflg)//no lidar siginal hold
                {
                    gcs().send_text(MAV_SEVERITY_NOTICE, "No Lidar, Set rover to hold.");
                    rover.set_mode(rover.mode_hold, ModeReason::EVERYDAY_END);
                    golf_end_mission();
                    return;

                }
                else if (!isSleep)
                {
                    int imode = control_mode->mode_number();
                    if(imode != 17)
                    {
                        rover.set_mode(rover.mode_gobatt, ModeReason::EVERYDAY_END);
                        gcs().send_text(MAV_SEVERITY_NOTICE, "Working to change mode to gobatt.");
                    }
                    else
                    {
                        pi_ctl_start = AP_HAL::millis();
                        one_hz_times = 0;
                        pi_ctl_step++;
                    }
                }
                else if(isSleep)
                {
                    int imode = control_mode->mode_number();
                    if(imode != 4)
                    {
                        rover.set_mode(rover.mode_hold, ModeReason::EVERYDAY_END);
                        gcs().send_text(MAV_SEVERITY_NOTICE, "Sleeping to change mode to HOLD.");
                    }    
                }
                break;
            case 12:
                // backward
                {
                    //float pitch_get = degrees(ahrs.get_pitch());
                    //if(pitch_get <  g.press_low)
                    //{
                    //    gcs().send_text(MAV_SEVERITY_NOTICE, "Stage is not down, waitting down to backward.");
                    //    break;
                   // }
                    //rover.mode_gobatt.set_para(-g.golf_throttleR); //-50
                    //int dis = get_distance(-1); //lidar
                    float dis = 0.0f,angle = 0.f;
                    g2.beacon.get_data(dis, angle);
                    dis = dis * 100; // m->cm
                    float  gpsdis = 0;
                    gpsdis = rover.current_loc.get_distance(ahrs.get_home())*100;
                    gcs().send_text(MAV_SEVERITY_INFO, "Golf gpsDis from home=%.0f.",gpsdis); 
                    if( gpsdis > g.press_up + 300 )
                    {
                        rover.mode_gobatt.set_para(); // complate
                        pi_ctl_step = 0;
                        pi_ctl = false;
                        pi_ctl_start = 0;
                        one_hz_times = 0;
                        nd_backward = pi_ctl;
                        pie_ctl_times = 0;
                    }
                    else if (gpsdis < 150 || dis > g.golf_near_distence + 500|| one_hz_times > g.golf_time_backward) // 12
                    {
                        //
                        rover_reached_stick = false;
                        pi_ctl_start = AP_HAL::millis();
                        one_hz_times = 0;
                        pie_ctl_times = 0;
                        pi_ctl_step++;
                        rover.mode_gobatt.set_para();//stop    
                    }
                    else
                    {
                        if (fabsf(old_gpsdis - gpsdis) < 20) // cm
                            pie_ctl_times++;
                        else
                        {
                            old_gpsdis = gpsdis;
                            pie_ctl_times = pie_ctl_times / 2;//0
                        }
                        float f = g.golf_throttleR + pie_ctl_times * 0.1;
                        rover.mode_gobatt.set_para(-f);
                        gcs().send_text(MAV_SEVERITY_INFO, "times= %d, dis= %.f, back= %.0f", pie_ctl_times, dis, -f);
                    }                                      
                }

                break;
            case 13:// gps again to face center
            {
                rover.mode_gobatt.set_para(); // stop
                AP_GPS::GPS_Status gpsst = gps.status();
                
                if ((gpsst == AP_GPS::GPS_OK_FIX_3D_RTK_FIXED || gpsst == AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) && one_hz_times > 5 ) // wait for gps
                {
                    // adjust direction
                    yaw_desire = g.gps_yaw_center;//76
                    yaw_enable = true;
                    yaw_complete = false;
                    pi_ctl_step++;
                    pi_ctl_start = AP_HAL::millis();
                    one_hz_times = 0;
                    pie_ctl_times = 0;
                }
            }           
                break;
            case 14: // forward to center
                if (yaw_complete)
                {
                    //rover.mode_gobatt.set_para(g.golf_throttle); // 50
                    //int disLidar = get_distance(-1);
                    float  dis = rover.current_loc.get_distance(ahrs.get_home())*100;
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "GPS from home: %.0f.",dis);
                    if (dis > g.press_up|| one_hz_times > g.golf_time_closedoor) 
                    {
                        //finish
                        rover.mode_gobatt.set_para(); // stop
                        pi_ctl_step = 0;
                        pi_ctl = false;
                        pi_ctl_start = 0;
                        one_hz_times = 0;
                        //test_work_s = 0; for step 6 back set to 120s
                        nd_backward = pi_ctl;
                        pie_ctl_times = 0;
                        //if(needsleep)
                        //{
                        //    isSleep = true;
                        //    needsleep = false;
                        //}

                    }  
                    else
                    {
                        if (fabsf(old_dis - dis) < 20) // cm
                            pie_ctl_times++;
                        else
                        {
                            old_dis = dis;
                            pie_ctl_times = pie_ctl_times / 2;//0
                        }
                        float f = g.golf_throttle + pie_ctl_times * 0.1;
                        rover.mode_gobatt.set_para(f, 0);
                        gcs().send_text(MAV_SEVERITY_INFO, "times= %d, gdis= %.1f, forward= %.0f", pie_ctl_times, dis, f);
                    }
                }
                break;

            }
            break;
        }
        default:
            break;
        }
    }
}

void Rover::sim_pi_guide(void)
{
    if (pi_ctl != true) // 202207uwb
        return;

    // if (pi_ctl != true && pi_ctl_id != 9000 && pi_ctl_step != 1)
    //     return;
    // float sidelen = 0.f;
    //check GPS distance
    float  gpsdis = 0;
    gpsdis = rover.current_loc.get_distance(ahrs.get_home())*100;
    gcs().send_text(MAV_SEVERITY_INFO, "Gps Dis from home=%.0f.",gpsdis); 
    if(gpsdis > g.golf_gps_dis)
        sim_pi_guide_state = 3;

    float delt = 0.f;
    float dis = 0.0f, angle = 0.0f, turn = 0.0f, scale = 0.2f;
    g2.beacon.get_data(dis, angle);
    // if (fabsf(angle)<g.golf_max_degerr)
    //     angle = 0.0;
    dis = dis * 100; // m->cm
    delt = uwb_admire - angle;
    gcs().send_text(MAV_SEVERITY_INFO, "UWB dis:%.0f,ang:%.0f,delt:%.0f.",dis,angle,delt); 
    // output raw angle and dis
    // float rawdis, rawangle;
    // g2.beacon.get_data_raw(rawdis,rawangle);
    // if(sim_pi_guide_state>0)
    //     gcs().send_text(MAV_SEVERITY_NOTICE, "pi_gd_state, %d, rdis, %.2f, rangle, %.2f",
    //                                                sim_pi_guide_state,rawdis, rawangle);
    // output end
    //    gcs().send_text(MAV_SEVERITY_NOTICE, "pi_gd_state=%d dis=%.2f angle=%.2f uwb_admire=%.2f",
    //                                                    sim_pi_guide_state,dis, angle, uwb_admire);
    //no uwb to hold
    if(dis <= 0.0f)
    {
        gcs().send_text(MAV_SEVERITY_NOTICE, "No UWB, Set rover to hold.");
        rover.set_mode(rover.mode_hold, ModeReason::EVERYDAY_END);
        golf_end_mission();
        return;


    }
    if (true) // if(!nd_collision)
    {
        switch (sim_pi_guide_state)
        {
        // wait AOA data stable
        case 0:
            rover.mode_gobatt.set_para(); // stop
            if (AP_HAL::millis() - pi_ctl_start > 2000)
            {
                // calculate admire offset angle
                // sidelen = calc_triangle_sidelen(dis,g.golf_near_distence,angle);
                // uwb_admire = calc_triangle_angleC(dis,sidelen,g.golf_near_distence);
                // if(angle > 0)
                //     uwb_admire = -uwb_admire;
                uwb_admire = 0;//uwb offset angle 0
                pi_ctl_start = AP_HAL::millis();
                sim_pi_guide_state++;
                //pie_ctl_times = 0;
            }
            break;
        // try to turn till angle in error
        case 1:
            g2.beacon.get_data(dis, angle);
            dis = dis * 100; // m->cm
            delt = uwb_admire - angle;
            if (fabsf(delt) < g.golf_max_degerr || fabsf(delt) < dis/100)
            {
                rover.mode_gobatt.set_para(); // stop to avoid over
                pi_ctl_start = AP_HAL::millis();
                sim_pi_guide_state++;
                pie_ctl_times = 0;
            }
            else
            {
                if (fabsf(old_yaw - angle) < g.golf_max_degerr*0.85)
                    pie_ctl_times++;
                else
                {
                    old_dis = dis;
                    old_yaw = angle;
                    pie_ctl_times = 0;//pie_ctl_times / 2;//0
                }

                if (delt > 0)
                    turn = g.steer_yaw_min + g.golf_yawrate_k * (fabsf(delt) + pie_ctl_times * scale) > g.golf_max_turn ? g.golf_max_turn : (g.steer_yaw_min + g.golf_yawrate_k * (delt + pie_ctl_times * scale));
                else
                    turn = g.steer_yaw_min + g.golf_yawrate_k * (fabsf(delt) + pie_ctl_times * scale) > g.golf_max_turn ? -g.golf_max_turn : (-g.steer_yaw_min + g.golf_yawrate_k * (delt - pie_ctl_times * scale));

                // if(turn > 0 && turn < g.steer_yaw_min)
                //         turn = g.steer_yaw_min;
                // else if(turn < 0 && turn > -g.steer_yaw_min)
                //         turn = -g.steer_yaw_min;

                // gcs().send_text(MAV_SEVERITY_NOTICE, "pi_gd_state=%d dis=%.2f angle=%.2f uwb_admire=%.2f",
                //                                     sim_pi_guide_state,dis, angle, uwb_admire);
                //when rover can't rotate even though the steer is up to the max
                float throttle = 0;
                if( fabsf(turn) > g.golf_max_turn * 0.9 && pie_ctl_times > 50) //4050 move and rotate
                {
                    if (fabsf(old_dis - dis) < 20) // cm
                    {
                        throttle = g.golf_forward + pie_ctl_times * 0.05;
                        rover.mode_gobatt.set_para(throttle, turn);
                    }
                    else
                    {
                        old_dis = dis;  
                        rover.mode_gobatt.set_para(throttle, turn);//0
                    }                 
                    
                }
                else             
                    rover.mode_gobatt.set_para(throttle, turn);
                gcs().send_text(MAV_SEVERITY_INFO, "times= %d angle= %.0f, throttle=%.0f turn= %.0f", pie_ctl_times, angle, throttle,turn);
                //if angle changed, go first to wait uwb stabilizing
                g2.beacon.get_data(dis, angle);
                if (fabsf(old_yaw - angle) > g.golf_max_degerr*0.85)
                {
                    pi_ctl_start = AP_HAL::millis();
                    sim_pi_guide_state = 1;//0
                }
            }

            break;
            // go straight to tag
        case 2:
            // gcs().send_text(MAV_SEVERITY_NOTICE, "pi_gd_state=%d dis=%.2f", sim_pi_guide_state,dis);
            g2.beacon.get_data(dis, angle);
            dis = dis * 100; // m->cm
            delt = uwb_admire - angle;
            if (dis < g.golf_near_distence)
            {
                pi_ctl_start = AP_HAL::millis();
                //rover.mode_gobatt.set_para(0, 0);
                sim_pi_guide_state++;
                pie_ctl_times = 0;
            }
            else if (fabsf(delt) < g.golf_max_degerr || fabsf(delt) < dis/100)
            {
                if (fabsf(old_dis - dis) < 20) // cm
                    pie_ctl_times++;
                else
                {
                    old_dis = dis;
                    pie_ctl_times = pie_ctl_times / 2;//0
                }
                float f = g.golf_forward + pie_ctl_times * 0.1;
                rover.mode_gobatt.set_para(f, 0);
                gcs().send_text(MAV_SEVERITY_INFO, "angle= %.0f, dis= %.0f, forward= %.0f", angle, dis, f);
            }
            else
            // if (AP_HAL::millis() - pi_ctl_start > 3000)
            {
                //rover.mode_gobatt.set_para(0, 0);
                pi_ctl_start = AP_HAL::millis();
                sim_pi_guide_state = 1;//0
                pie_ctl_times = 0;
            }
            break;
            // rotate to face tag.
        case 3:
            //if(fabsf(delt) < g.golf_max_degerr || fabsf(delt) < dis/100)
            //{
                rover.mode_gobatt.set_para(0, 0);
                pi_ctl_start = AP_HAL::millis();
                sim_pi_guide_state = 0;
                uwb_complete = true; // uwb adjust finished.
                pie_ctl_times = 0;

            //}
            //else
            //{
            //    //rover.mode_gobatt.set_para(0,0);
            //    pi_ctl_start = AP_HAL::millis();
            //    sim_pi_guide_state = 0;
            //    pie_ctl_times = 0;
            //}
            break;
        default:
            break;
        }
    }
    else
    {
        rover_reached_stick = true;
    }
}

void Rover::init_golfpin(void)
{
    hal.gpio->pinMode(AUX_ENA_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(AUX_IN1_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(AUX_IN2_PIN, HAL_GPIO_OUTPUT);
}

void Rover::motor_stop(void)
{
    hal.gpio->write(AUX_ENA_PIN, 0);
    hal.gpio->write(AUX_IN1_PIN, 0);
    hal.gpio->write(AUX_IN2_PIN, 0);
}

void Rover::motor_pull(void)
{
    hal.gpio->write(AUX_ENA_PIN, 1);
    hal.gpio->write(AUX_IN1_PIN, 0);
    hal.gpio->write(AUX_IN2_PIN, 1);
}

void Rover::motor_push(void)
{
    hal.gpio->write(AUX_ENA_PIN, 1);
    hal.gpio->write(AUX_IN1_PIN, 1);
    hal.gpio->write(AUX_IN2_PIN, 0);
}

bool Rover::golf_start_mission(void)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "golf_start_mission");
    if (nd_backward)
    {
        golf_backward(-1);
        gcs().send_text(MAV_SEVERITY_DEBUG, "golf_start_mission to gobatt");
        return false;
    }
    gcs().send_text(MAV_SEVERITY_DEBUG, "golf_start_mission to auto");
    rover.rover_golf_start = AP_HAL::millis();
    work_enable = true;
    start_auto = false;
    uwb_complete = false;
    pi_ctl = false; // //202207uwb
    oldpi_ctl_step = 100;
    pi_ctl_step = 0;
    sim_pi_guide_state = 0; // 202207uwb
    golf_work_state = GOLF_WORK;
    if(needsleep)//
    {
        needsleep = false; //for back
        isSleep = true;
    } 
    else
    {
        //golf_work_state = GOLF_HOLD;
        isSleep = false;
    }
    return true;
}

void Rover::golf_end_mission(void)
{
    nd_backward = pi_ctl;

    gcs().send_text(MAV_SEVERITY_DEBUG, "golf_end_mission");
    // rover.set_mode(rover.mode_rtl, ModeReason::EVERYDAY_END);    uwb202207
    rover_golf_start = AP_HAL::millis();
    work_enable = false;
    batt_nd_charge = true;
    pi_ctl = false; // //202207uwb
    pi_ctl_step = 0;
    sim_pi_guide_state = 0; // 202207uwb
    yaw_enable = false; //202310 gps
    golf_work_state = GOLF_NOWORK;//GOLF_BACK ->manual avoid near distination and gohome
    work_golf_back = true; // Josh
    isSleep = true;        // Josh
    start_auto = true;
    rover.mode_gobatt.set_para();
    needsleep = false; //for back
}

void Rover::start_debug(void)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "start gobatt debug");
    golf_work_state = GOLF_PI_CTL;
    pi_ctl_id = 9000;
    pi_ctl = true;
    sim_pi_guide_state = 0;
    yaw_complete = true;
    pi_ctl_start = AP_HAL::millis();
}

void Rover::get_proximity_dis(float &distance0, float &distance45, float &distance315)
{
    AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity == nullptr)
    {
        return;
    }

    if (proximity->get_status() == AP_Proximity::Status::Good)
    {
        AP_Proximity::Proximity_Distance_Array dist_array;
        if (proximity->get_horizontal_distances(dist_array))
        {
            distance0 = dist_array.distance[0];
            distance45 = dist_array.distance[1];
            distance315 = dist_array.distance[7];
        }
    }
}

int Rover::get_distance(int idx)
{
    int mindis = 10000; // cm
    AP_RangeFinder_Backend *s = nullptr;

    if (idx >= 0 && idx < rover.rangefinder.num_sensors())
    {
        s = rover.rangefinder.get_backend(idx);
        if (s != nullptr)
            mindis = s->distance_cm();
       // gcs().send_text(MAV_SEVERITY_INFO, "lidar dist #%i =  %d", idx, mindis);
        return mindis;
    }

    for (uint8_t i = 0; i < rover.rangefinder.num_sensors(); i++)
    {

        s = rover.rangefinder.get_backend(i);
        if (s == nullptr)
        {
            continue;
        }
        int distance = s->distance_cm();
        if (mindis > distance)
            mindis = distance;
      //  gcs().send_text(MAV_SEVERITY_INFO, "lidar dist #%i =  %d", i, distance);
    }

    return mindis;
}

void Rover::enable_rangefinder(int idx,bool enableflg)
{

    AP_RangeFinder_Backend *s = nullptr;

    if (idx >= 0 && idx < rover.rangefinder.num_sensors())
    {
        s = rover.rangefinder.get_backend(idx);
        if (s != nullptr)
        {
            s->enable(enableflg);
            gcs().send_text(MAV_SEVERITY_INFO, "lidar#%i =  %d", idx, enableflg);
        }    
             
       return;
    }

    for (uint8_t i = 0; i < rover.rangefinder.num_sensors(); i++)
    {

        s = rover.rangefinder.get_backend(i);
        if (s == nullptr)
        {
            continue;
        }
        s->enable(enableflg);
        gcs().send_text(MAV_SEVERITY_INFO, "lidar#%i =  %d", idx, enableflg);
    }
}

bool Rover::near_target(int distmax, int distmin)
{
    for (uint8_t i = 0; i < rover.rangefinder.num_sensors(); i++)
    {

        AP_RangeFinder_Backend *s = rover.rangefinder.get_backend(i);
        if (s == nullptr)
        {
            continue;
        }
        int distance = s->distance_cm();
        gcs().send_text(MAV_SEVERITY_INFO, "lidar dist #%i =  %d,distmax=%d,distmin=%d", i, distance, distmax, distmin);
        // gcs().send_text(MAV_SEVERITY_INFO, "#i=%i dist=%lf status = %i", i,distance, s->status());
        if (distance >= distmin && distance < distmax)
        {
            return true;
        }
    }
    return false;
}

float Rover::calc_triangle_sidelen(float a, float b, float angleC)
{
    if (a <= 0 || b <= 0 || angleC <= 0)
        return 0.f;
    float c2 = a * a + b * b - 2 * a * b * cosf(angleC * 3.1415926 / 180.0);
    if (c2 <= 0)
        return 0.f;

    float c = sqrtf(c2);
    gcs().send_text(MAV_SEVERITY_INFO, "triangle_sidelen a:%.2f, b:%.2f,c:%.2f,angleC=%.2f", a, b, c, angleC);
    return c;
}

float Rover::calc_triangle_angleC(float a, float b, float c)
{
    if (a <= 0 || b <= 0 || c <= 0)
        return 0.f;
    float tempdata = (a * a + b * b - c * c) / (2 * a * b);
    float angleC = acosf(tempdata) / 3.1415926 * 180.0;
    gcs().send_text(MAV_SEVERITY_INFO, "triangle_angleC a:%.2f, b:%.2f,c:%.2f,angleC=%.2f", a, b, c, angleC);
    return angleC;
}

void Rover::golf_backward(int sleepflg)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "golf_backward");
    //if(!needsleep)
    //    needsleep = sleepflg;
    work_enable = true;
    golf_work_state = GOLF_PI_CTL;
    if (sleepflg != -1)
        isSleep = (bool)sleepflg;
    start_auto = false;
    uwb_complete = false;
    pi_ctl = true; //
    pi_ctl_step = 12;//back 11 close door
    pi_ctl_start = 0; // AP_HAL::millis();
    one_hz_times = 0;//g.golf_time_closedoor-4;
    sim_pi_guide_state = 0; // 202207uwb
    rover.set_mode(rover.mode_gobatt, ModeReason::EVERYDAY_END);
}

bool Rover::golf_is_athome()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "golf_is_athome");
    if (pi_ctl)
        return true;
    //change state from GOLF_NOWORK(manual and gobatt endmission) to GOLF_BACK
    golf_work_state = GOLF_BACK;

    return false;
}

void Rover::golf_set_sleepflg(float sleepflg)
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "golf_set_sleepflg: %f", sleepflg);
    if (sleepflg > 0.0f)
    {
        work_enable = false;
        isSleep = true;
        batt_nd_charge = true;
        if (golf_work_state != GOLF_PREP_PI && golf_work_state != GOLF_PI_CTL)
        {
            golf_work_state = GOLF_BACK;
            work_golf_back = true; // Josh
            rover.set_mode(rover.mode_rtl, ModeReason::EVERYDAY_END);
        }
    }
    else
    {
        work_enable = true;
        isSleep = false;
        batt_nd_charge = false;
        pi_ctl_start = 0; // AP_HAL::millis();
        one_hz_times = 0;
        test_work_s = 0;
    }

}
bool Rover::closetohome(float dis)
{
    //calculate the distance to home   
    float  gdist = rover.current_loc.get_distance(ahrs.get_home())*100;
    gcs().send_text(MAV_SEVERITY_INFO, "GPS close: %.0f,old:%.0f,dir:%d.",gdist,old_gpsdis,directionflg);
    //
    if(gdist > 450)
    {
        rover.mode_gobatt.set_para();//stop
        gcs().send_text(MAV_SEVERITY_NOTICE, "Golf is too far(%.0f) to hold.",gdist);
        rover.set_mode(rover.mode_hold, ModeReason::EVERYDAY_END);
        golf_end_mission();
        return false;


    }
    else if(fabsf(gdist) > dis)
    {
        
            if (fabsf(old_gpsdis - gdist) < 20) // cm
                pie_ctl_times++;
             else
            {
                pie_ctl_times = 0;
                old_gpsdis = gdist; 
            }
            float f = g.golf_forward + pie_ctl_times * 0.05;
            if(gdist - old_gpsdis > 10)//if dis become short, continue forward
            {
                directionflg = directionflg*(-1);
                pie_ctl_times = 0;
                old_gpsdis = gdist; 
            }

            if(one_hz_times%2 == 0)
                rover.mode_gobatt.set_para(directionflg*f, 0);  
            else
                rover.mode_gobatt.set_para();  

            //old_gpsdis = gdist; 
    }
    else
    {
        one_hz_times = 0;
        rover.mode_gobatt.set_para();//stop
        return true;
    }
 

    return false;

}

bool Rover::fly_to_here(Location target_loc)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Set Guided mode.");
    rover.set_mode(rover.mode_guided, ModeReason::EVERYDAY_END);
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!control_mode->in_guided_mode()) {
        return false;
    }
    float gspeed= 0.8;//m/s
        // initialise waypoint speed
    if (is_positive(g2.rtl_speed)) {
        gspeed = g2.rtl_speed;
    }
    bool rt = control_mode->set_desired_speed(gspeed);
    gcs().send_text(MAV_SEVERITY_INFO, "Set Guided speed: %f,rt:%d.",gspeed,(int)rt);
    return control_mode->set_desired_location(target_loc);
    
}
//uint_8 frame:0-home,1-current
//float  angle
//float  distance
Location Rover::calc_desired_location(float distance, float angle, uint8_t frame)
{
    //prepare target position
    Location target_loc = current_loc;
    //Based home
    if(frame == 0)
    {
        if(ahrs.home_is_set())
            target_loc = ahrs.get_home();
        else
            gcs().send_text(MAV_SEVERITY_INFO, "calc desired location: home is not set.");
    }
    //calculate offset
    // rotate from body-frame to NE frame
    float rad = angle * 3.1415926f / 180.0f;
    const float ne_x = distance * cosf(rad);
    const float ne_y = distance * sinf(rad);
    // add offset to current location
    target_loc.offset(ne_x, ne_y);

    return target_loc;
}