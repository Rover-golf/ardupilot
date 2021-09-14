#include "Rover.h"

#if HAL_WITH_UAVCAN
#include <AP_RangeFinder/RangeFinder_Backend.h>  // Josh
#include <AP_Proximity/AP_Proximity_Backend.h>  // Josh
#include <AP_Proximity/AP_Proximity_RangeFinder.h> // Josh
#include <AP_Proximity/AP_Proximity.h> // Josh
#include <AP_RangeFinder/RangeFinder.h>  // Josh
//#include <AP_RangeFinder/AP_RangeFinder_SR73F.h>  // Josh
// static RangeFinder lidar;
#endif

void Rover::hundred_hz_loop(void)
{
    if (yaw_enable)
    {
        float yaw_get = degrees(ahrs.yaw);
        float yaw_rate_to = 0;
        // 全部转到0-360比较
        if (yaw_get < 0)
            yaw_get += 360.0f;

        yaw_rate_to = yaw_desire - yaw_get;
        if (yaw_rate_to < 0)
            yaw_rate_to += 360.0f;
        if (yaw_rate_to > 180)
            //yaw_rate_to = 360-yaw_rate_to;//反向转yaw_rate_to
            yaw_rate_to = -1;

        //否则正向
        yaw_rate_to = yaw_rate_to > 0 ? g.steer_rate_use : -g.steer_rate_use;
        if (fabsf(yaw_desire - yaw_get) < g.steer_error)
        {
            yaw_enable = false;
            yaw_complete = true;
            yaw_rate_to = 0;
        }
        rover.mode_gobatt.set_para(0.0f, yaw_rate_to);
    }
}

void Rover::one_hz_loop(void)
{
    if (work_enable && (AP_HAL::millis() - rover_golf_start > 5 * 3600 * 1000) && rover_golf_start!=0)
    {
        rover.golf_end_mission();
    }


    //sr73f_can.update();
    //golf: regular start&return
    static uint16_t test_work_s = 0;
    //uint8_t hour, min, sec;
    //uint16_t ms;
    //获取现在的UTC时间 时 分 秒
    //if (!AP::rtc().get_local_time(hour, min, sec, ms))
    //     gcs().send_text(MAV_SEVERITY_DEBUG, "UTC get time faild!");
    //else
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "H:M:S %d:%d:%d", hour, min, sec);
    batt_nd_charge = false;

    float batt_volt = battery.voltage();

    bool batt_is_low = (batt_volt < g.batt_nd_rtl) ? true : false;
    bool batt_is_full = (batt_volt > g.batt_charge_to) ? true : false;
    
    // 
    static bool door_nd_close = false;
    golf_is_full = !(rover.check_digital_pin(AUX_GOLF_PIN));
    if(golf_is_full) gcs().send_text(MAV_SEVERITY_INFO, "golf full %i", golf_is_full); 
    nd_collision = !(rover.check_digital_pin(AUX_AVOID_PIN));
    if(nd_collision) gcs().send_text(MAV_SEVERITY_INFO, "collision %i", nd_collision); 
    uint8_t nd_avd = 0;

    // Begin Josh
 
    if(pi_ctl == false && rover.control_mode->is_autopilot_mode())
    {
        motor_pull();
    } 
 

//#if HAL_WITH_UAVCAN
     float distance_cm=0.0;
//#endif
    // char            c = 0;
//	static char   buf[16];
//	static unsigned int i;
//      while(hal.uartD->available() > 0 && c != '\n' && i < sizeof(buf)){
 //   if(hal.uartD->available() > 0 && c != '\n' ){
//		c 		 = hal.uartD->read();
//		buf[i++] = c;
//        gcs().send_text(MAV_SEVERITY_INFO, "UART D %d", c);   // Josh for IR on uartD
	//}

    //float last_time;
    //bool hasDate;
    if (golf_work_state != GOLF_PI_CTL && nd_collision) // Josh  collision
    {
         golf_work_state = GOLF_COLLISION;
    }
//#if HAL_WITH_UAVCAN
    else
    {
/*
        // Josh  2021August12
           float distance0=0.0, distance45=0.0, distance315=0.0;
           if(g2.proximity.get_status() == AP_Proximity::Proximity_Good)
           {
            g2.proximity.get_horizontal_distance(0, distance0);
            g2.proximity.get_horizontal_distance(45, distance45);
            g2.proximity.get_horizontal_distance(315, distance315);
           }
           
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
            }
//            gcs().send_text(MAV_SEVERITY_INFO, "0=%lf 45=%lf 315=%lf v=%i", distance0,distance45,distance315, nd_avd);
        // end of 2021August12
*/ 
        for (uint8_t i = 0; i < rover.rangefinder.num_sensors(); i++)
        {

            AP_RangeFinder_Backend *s = rover.rangefinder.get_backend(i);
            if (s == nullptr)
            {
                continue;
            }

            distance_cm = s->distance_cm();
            target_deg = s->target_deg();
            //last_time = s->last_reading_ms();
            //hasDate = s->has_data;
//            gcs().send_text(MAV_SEVERITY_INFO, "lidar #%i  status = %i  count = %i", i,  s->status(), s->range_valid_count());
                          //   gcs().send_text(MAV_SEVERITY_INFO, "dist #%i =  %lf", i, distance_cm);
            if (distance_cm > 50.0 && distance_cm < 500.0 && pi_ctl == false && rover.control_mode->is_autopilot_mode() && s->status() == 4)
            //           &&  s->has_data() )
            {
                 nd_avd = 1;
                 golf_work_state = GOLF_WORK;
            }
            else
            {
//                nd_avd = 0;
            }
    //        gcs().send_text(MAV_SEVERITY_INFO, "#i=%i dist=%lf deg=%f status=%i  avd=%i", i,distance_cm,target_deg, s->status(), nd_avd);
            
        }

    }

//    gcs().send_text(MAV_SEVERITY_INFO, "mode=%i work_enable=%i state=%i colli=%i", rover.control_mode->mode_number(),
//                    work_enable, golf_work_state, nd_collision);
//#endif

    // End Josh

    if (batt_is_low) { batt_nd_charge = true; }

    //定时启动
    // bool time1_avaiable = !((g2.start_1_hour == 0) && (g2.start_1_min == 0) && (g2.end_1_hour == 0) && (g2.end_1_min == 0));
    // bool time1_to_start = ((hour == (uint8_t)g2.start_1_hour) && (min == (uint8_t)g2.start_1_min) && (sec < 4));
    // //    bool time1_to_start = ((hour*24+min >= (uint8_t)g2.start_1_hour*24+(uint8_t)g2.start_1_min)
    // //                        && (hour*24+min < (uint8_t)g2.end_1_hour*24 +(uint8_t)g2.end_1_min) && (sec < 4));
    // bool time1_to_end = ((hour == (uint8_t)g2.end_1_hour) && (min == (uint8_t)g2.end_1_min) && (sec < 4));
    // bool time2_avaiable = !((g2.start_2_hour == 0) && (g2.start_2_min == 0) && (g2.end_2_hour == 0) && (g2.end_2_min == 0));
    // bool time2_to_start = ((hour == (uint8_t)g2.start_2_hour) && (min == (uint8_t)g2.start_2_min) && (sec < 4));
    // //    bool time2_to_start = ((hour*24+min >= (uint8_t)g2.start_2_hour*24+(uint8_t)g2.start_2_min)
    // //                        && (hour*24+min < (uint8_t)g2.end_2_hour*24 +(uint8_t)g2.end_2_min) && (sec < 4));
    // bool time2_to_end = ((hour == (uint8_t)g2.end_2_hour) && (min == (uint8_t)g2.end_2_min) && (sec < 4));
    // bool time3_avaiable = !((g2.start_3_hour == 0) && (g2.start_3_min == 0) && (g2.end_3_hour == 0) && (g2.end_3_min == 0));
    // bool time3_to_start = ((hour == (uint8_t)g2.start_3_hour) && (min == (uint8_t)g2.start_3_min) && (sec < 4));
    // //    bool time3_to_start = ((hour*24+min >= (uint8_t)g2.start_3_hour*24+(uint8_t)g2.start_3_min)
    // //                        && (hour*24+min < (uint8_t)g2.end_3_hour*24 +(uint8_t)g2.end_3_min) && (sec < 4));
    // bool time3_to_end = ((hour == (uint8_t)g2.end_3_hour) && (min == (uint8_t)g2.end_3_min) && (sec < 4));

//    gcs().send_text(MAV_SEVERITY_INFO, "t2s=%i ta=%i, slp=%i", time1_to_start, time1_avaiable, isSleep);
    




    /*
    if ((time1_to_start && time1_avaiable) ||
        (time2_to_start && time2_avaiable) ||
        (time3_to_start && time3_avaiable))
    {
        //      if(!work_enable)
        //      {
        work_enable = true;
        if (isSleep) // Josh
        {
            golf_work_state = GOLF_HOLD;
            isSleep = false;
        }
        //      }
    }

    if ((time1_to_end && time1_avaiable) ||
        (time2_to_end && time2_avaiable) ||
        (time3_to_end && time3_avaiable)

    )
    {
        // 结束了之后回去充电
        work_enable = false;
        batt_nd_charge = true;
        golf_work_state = GOLF_BACK;
        work_golf_back = true; // Josh
        isSleep = true;        // Josh
        rover.set_mode(rover.mode_rtl, MODE_REASON_EVERYDAY_END);
    }
*/
    // 状态机
//    gcs().send_text(MAV_SEVERITY_DEBUG, "golf_work_state = %d ", golf_work_state);

    switch (golf_work_state)
    {
    case GOLF_COLLISION:
        if (work_enable)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf collision");
            rover.set_mode(rover.mode_hold, MODE_REASON_EVERYDAY_END);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf collision backward");
            if (pi_ctl) //control_mode->mode_number() != Mode::Number::HOLD)
            {
                rover.set_mode(rover.mode_hold, MODE_REASON_EVERYDAY_END);
                // SleepForSeconds(10);
            }
            rover.set_mode(rover.mode_gobatt, MODE_REASON_EVERYDAY_END);
            pi_ctl_id = 9010;
            golf_send_cmd(pi_ctl_id, rover.ahrs.yaw, target_deg); // Josh added parameters
//            target_deg = target_deg > 0 ? fabsf(target_deg) - 45.0 : 45.0 - fabsf(target_deg);
            yaw_desire = constrain_deg(constrain_deg(degrees(rover.ahrs.yaw)) + 0);
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
            rover.set_mode(rover.mode_auto, MODE_REASON_EVERYDAY_START);
            arming.arm(AP_Arming::Method::RUDDER);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf Time Start");
            golf_work_state = GOLF_WORK;
            work_golf_back = false; // Josh
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
            rover.set_mode(rover.mode_rtl, MODE_REASON_EVERYDAY_END);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf is full");
            test_work_s = 0;
            golf_work_state = GOLF_BACK;
            work_golf_back = true; // Josh
        }
        if (batt_nd_charge)
        {
            rover.set_mode(rover.mode_rtl, MODE_REASON_EVERYDAY_END);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf batt is low");
            test_work_s = 0;
            golf_work_state = GOLF_BACK;
            work_golf_back = true; // Josh
        }
        if (nd_avd)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf avd now");
            rover.set_mode(rover.mode_gobatt, MODE_REASON_EVERYDAY_END);
            golf_work_state = GOLF_PI_AVOID;
            pi_ctl_id = 9020;
            yaw_complete = true;
            pi_ctl_start = AP_HAL::millis();
            target_deg = target_deg > 0 ? fabsf(target_deg) - 45.0 : 45.0 - fabsf(target_deg);
            golf_send_cmd(pi_ctl_id, rover.ahrs.yaw, target_deg); // Josh added parameters
            pi_ctl = true;
        }

        if (g2.wp_nav.reached_destination())
        {
            golf_work_state = GOLF_BACK;
            work_golf_back = true; // Josh
            rover.set_mode(rover.mode_rtl, MODE_REASON_EVERYDAY_END);

            // golf_work_state = GOLF_PREP_PI;
        }
        break;
    case GOLF_BACK:
        if (g2.wp_nav.reached_destination())
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf near home");
 //           rover.set_mode(rover.mode_rtl, MODE_REASON_EVERYDAY_END);
            golf_work_state = GOLF_PREP_PI;
        }
        break;
    case GOLF_PREP_PI:
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf prep for pi");
        rover.set_mode(rover.mode_gobatt, MODE_REASON_EVERYDAY_END);
        golf_work_state = GOLF_PI_CTL;
        pi_ctl_id = 9000;
//        yaw_enable = true;
//        yaw_complete = false;
//        yaw_desire = 0;
        pi_ctl = true;
//        golf_send_cmd(pi_ctl_id, rover.ahrs.yaw, target_deg); // Josh added parameters
        break;
    case GOLF_PI_CTL:
        // 等树莓派控制完成后pi_ctl会设置成false
        if (!pi_ctl)
        {
            if (batt_nd_charge)
                golf_work_state = GOLF_LOW_BATT;
            else
                golf_work_state = GOLF_HOLD;
        }
        break;
    case GOLF_LOW_BATT:
        if (batt_is_full)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Golf batt is ready");
            golf_work_state = GOLF_HOLD;
        }
        break;
    case GOLF_PI_AVOID:
        if (!pi_ctl)
        {
            if (work_golf_back) //  Josh during RTL avoidance, after avoid, keep going RTL
            {
                golf_work_state = GOLF_BACK;
                work_golf_back = true; // Josh
                rover.set_mode(rover.mode_rtl, MODE_REASON_EVERYDAY_END);

            } // Josh END
            else
            {
                rover.set_mode(rover.mode_auto, MODE_REASON_EVERYDAY_START);
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
                if (AP_HAL::millis() - pi_ctl_start >3000)
                    pi_ctl_step++;
                break;
            case 2:
                rover.mode_gobatt.set_para(-50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 3:
                rover.mode_gobatt.set_para(-50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 4:
                if (AP_HAL::millis() - pi_ctl_start > 3000)
                {
                    yaw_desire = constrain_deg(constrain_deg(degrees(rover.ahrs.yaw)) + 45.0);
                    yaw_enable = true;
                    yaw_complete = false;
                    pi_ctl_start = AP_HAL::millis();
                    pi_ctl_step++;
                }
                break;
            case 5:
                if (AP_HAL::millis() - pi_ctl_start > 3000)
                    pi_ctl_step++;
                break;
            case 6:
                rover.mode_gobatt.set_para(50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 7:
                rover.mode_gobatt.set_para(50);
                pi_ctl_start = AP_HAL::millis();
                pi_ctl_step++;
                break;
            case 8:
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
        case 9020:  // obscale avoid
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
//                gcs().send_text(MAV_SEVERITY_INFO, "yaw = %f targetDeg = %f ", degrees(rover.ahrs.yaw) , target_deg);
                yaw_desire = constrain_deg(constrain_deg(degrees(rover.ahrs.yaw)) + target_deg);
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
            case 7:
//                gcs().send_text(MAV_SEVERITY_INFO, "yaw = %f targetDeg = %f ", degrees(rover.ahrs.yaw) , target_deg);
                yaw_desire = constrain_deg(constrain_deg(degrees(rover.ahrs.yaw)) );
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
        case 9000:  // reached home then unload ball
        {
            switch (pi_ctl_step)
            {
            case 0:
//                if (yaw_complete)
//                {
                    pi_ctl_step++;
                    pi_ctl_step++;  // Josh Sept.4 2021, skip sim_pi_guide AOA
                    pi_ctl_start = AP_HAL::millis();
//                }
                break;
            case 1:
                // run guided
                sim_pi_guide();
                if (rover_reached_stick)
                {
                    rover_reached_stick = false;
                    pi_ctl_start = AP_HAL::millis();
                    pi_ctl_step++;
                }
                break;
            case 2:
                // open door and wait 10s
                motor_push();     
                if (AP_HAL::millis() - pi_ctl_start > 10000)
                {
                    pi_ctl_start = AP_HAL::millis();
                    pi_ctl_step++;
                }
                break;
            case 3:
                rover.mode_gobatt.set_para(-50);
                if (AP_HAL::millis() - pi_ctl_start > 5000)
                {
                    pi_ctl_start = AP_HAL::millis();
                    pi_ctl_step++;
                }
                break;
            case 4:
                // close door and wait 3s
                motor_pull();    
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
        default:
            break;
        }
    }
}

void Rover::sim_pi_guide(void)
{
    // if (pi_ctl != true && pi_ctl_id != 9000 && pi_ctl_step != 1)
    //     return;
    
    float dis = 0.0f, angel = 0.0f, trun = 0.0f;
    g2.beacon.get_data(dis, angel);
    if (fabs(angel)<g.golf_max_degerr)
        angel = 0.0;
    
    if(angel < 0)    
        trun = g.golf_yawrate_k*fabs(angel) > g.golf_max_turn? g.golf_max_turn:g.golf_yawrate_k*angel;
    else
        trun = g.golf_yawrate_k*fabs(angel) > g.golf_max_turn? -g.golf_max_turn:-g.golf_yawrate_k*angel;
 
           gcs().send_text(MAV_SEVERITY_NOTICE, "pi_gd_state=%d dis=%.2f angel=%.2f trun=%.2f", 
                                                    sim_pi_guide_state,dis, angel, trun);
 

    if(!nd_collision)
    {
        switch (sim_pi_guide_state)
        {
        // wait AOA data stable
        case 0:
            if (AP_HAL::millis() - pi_ctl_start > 3000)
            {
                pi_ctl_start = AP_HAL::millis();
                sim_pi_guide_state++;
            }
            break;
        // try to turn till angle in error file
        case 1:
            rover.mode_gobatt.set_para(0,trun);
            if (fabs(angel)<g.golf_max_degerr)
            {
                pi_ctl_start = AP_HAL::millis();
                sim_pi_guide_state++;
            }
            break;
        case 2:
//            gcs().send_text(MAV_SEVERITY_NOTICE, "pi_gd_state=%d dis=%.2f angel=%.2f trun=%.2f", 
//                                                    sim_pi_guide_state,dis, angel, trun);
            rover.mode_gobatt.set_para(g.golf_forward,0);
//            if (AP_HAL::millis() - pi_ctl_start > 3000)
//            {
//                rover.mode_gobatt.set_para(0,0);
//                pi_ctl_start = AP_HAL::millis();
                sim_pi_guide_state = 0;
//            }
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

void Rover::golf_start_mission(void)
{
    rover.rover_golf_start = AP_HAL::millis();
    work_enable = true;
    start_auto = false;
    if (isSleep) // Josh
    {
        golf_work_state = GOLF_HOLD;
        isSleep = false;
    }
}

void Rover::golf_end_mission(void)
{
    rover_golf_start = 0;
    work_enable = false;
    batt_nd_charge = true;
    golf_work_state = GOLF_BACK;
    work_golf_back = true; // Josh
    isSleep = true;        // Josh
    start_auto = true;
    rover.set_mode(rover.mode_rtl, MODE_REASON_EVERYDAY_END);
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