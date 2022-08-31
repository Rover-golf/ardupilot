#include "mode.h"
#include "Rover.h"

void ModeGoBatt::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

bool ModeGoBatt::_enter()
{
    gcs().send_text(MAV_SEVERITY_DEBUG, "start mode gobatt");
    set_para();
    //rover.start_debug();
    return true;
}
bool ModeGoBatt::stop_rover()
{
    set_para();
    return stop_vehicle();
}
void ModeGoBatt::set_para(float throttle, float steering)
{
    _throttle = throttle;
    _steering = steering;
    _last_time_ms = AP_HAL::millis();
}

bool ModeGoBatt::set_yaw(float yaw)
{

    float yaw_err = degrees(ahrs.yaw_sensor - yaw);
    if (fabsf(yaw_err) < 2.0f)
    {
        return true;
        set_para(0.0f, 0.0f);
    }
    set_para(0.0f, 1400.0f);
    return false;
}

void ModeGoBatt::update()
{

    if ((AP_HAL::millis() - _last_time_ms) > 2000)
    {
        // gcs().send_text(MAV_SEVERITY_WARNING, "target not received last 1secs, stopping");
 //       _steering = 0;
 //       _throttle = 0;
        _lateral = 0;
        _mainsail = 0;
    }
    //check RC throttle
    float RC_throttle = 0.0;
    float RC_steering = 0.0;
    get_pilot_input(RC_steering,RC_throttle);

    float desired_steering = _steering;
    float desired_throttle = _throttle + RC_throttle;
    float desired_lateral = _lateral;
    float desired_mainsail = _mainsail;
 
    g2.motors.set_mainsail(desired_mainsail);

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, false);
    g2.motors.set_lateral(desired_lateral);
   
    //gcs().send_text(MAV_SEVERITY_INFO, "gobattmode: throttle= %f, steering= %f, lateral= %f",desired_throttle,desired_steering,desired_lateral);   
    // gcs().send_text(MAV_SEVERITY_CONTINUE, "%f/%f/%f/%f",
    //                 desired_mainsail, desired_throttle, desired_steering, desired_lateral);
}