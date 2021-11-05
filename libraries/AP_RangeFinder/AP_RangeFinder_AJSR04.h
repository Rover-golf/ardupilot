#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_AJSR04 : public AP_RangeFinder_Backend
{

public:

    // constructor
    AP_RangeFinder_AJSR04(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            uint8_t serial_instance);

    // static detection function
    static bool detect(uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_cm
    bool get_reading(uint16_t &reading_cm,float& target_deg);
    //get single sensor distance returned in reading_cm
    bool get_reading_single(uint16_t &reading_cm);
    //get multi sensor distance returned in reading_cm and number in sensornum
    bool get_reading_multi(uint16_t &reading_cm, uint8_t &sensornum);
    //require reading distance information
    bool require_reading_distance(uint8_t sensornum);    
    AP_HAL::UARTDriver *uart = nullptr;
    uint8_t linebuf[10];
    uint8_t linebuf_len;
    //sensor max address
    uint8_t _Address;
    //current sensor id
    uint8_t _cursensorid = 0x01;
};
