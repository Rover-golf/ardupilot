#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_AJSR04 : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_cm
    bool get_reading(uint16_t &reading_cm) override;

    uint8_t linebuf[10];
    uint8_t linebuf_len;
};
