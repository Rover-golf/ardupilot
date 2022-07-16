#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"
#include "CygbotParser.h"

class AP_RangeFinder_CYGLidar : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;
protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;
    bool get_reading_2d(uint16_t &reading_cm,float &target_deg);
    bool Get2DMinDistance(uint16_t &reading_cm,float &target_deg);
    //require command
    bool require_command(uint8_t require_command = 0x01);     

    uint8_t bufferPtr[340];
    //sensor max address
    uint8_t _Address; 
    int last_reading_CYGLidar = 0; 
    bool isreading = false;
};
