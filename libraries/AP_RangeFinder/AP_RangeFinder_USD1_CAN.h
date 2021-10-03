#pragma once

#include "AP_RangeFinder_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

class AP_RangeFinder_USD1_CAN : public CANSensor, public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_USD1_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    void update() override;

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    //   
    void init(uint8_t driver_index, bool enable_filters) override;
protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }
private:
    bool new_data;
    uint16_t _distance_cm;
    uint32_t _last_reading_ms;
    //Radar
    bool CH30_Startup();
    bool CH30_Analysis(AP_HAL::CANFrame &frame,float& fdist, float& fdeg);
    bool SR73_Analysis(AP_HAL::CANFrame &frame,float& fdist, float& fdeg);
    bool SetRadarAddress(uint8_t iaddress = 0x00);
    bool ObjectInRegion(float fdist, float fdeg = 0,float long1 = 0.f,float lat1 = 0.6f,float long2 = 2.0f,float lat2 = -0.6f);
    enum{
        //SR73
        OBJECT_STATUS = 0x60A,
        OBJECT_INFORMATION = 0x60B,
        //CH30
        CH30_HEART_BEAT=1415,
	    CH30_DATA=1414
    };
    typedef struct{          // byte description
 	    uint8_t Id:8;         //0:7      Object ID
 	    uint8_t DistLongH:8;  //8:15     V dist H 8
 	    uint8_t DistLatH:3;   //16:18    H dist H 3
 	    uint8_t DistLongL:5;  //19:23    V dist H 5
 	    uint8_t DistLatL:8;   //24:31    H dist L 8
 	    uint8_t VrelLongH:8;  //32:39    V speed H 8
 	    uint8_t VrelLatH:6;   //40:45    H speed H 6
 	    uint8_t VrelLongL:2;  //46:47    V speed L 2
 	    uint8_t DynProp:3;    //48:50    object motion property，default 0
 	    uint8_t undefined:2;  //51:52    object motion property，default 0
        uint8_t VrelLatL:3;   //53:55    H speed L 3
        uint8_t RCS:8;        //56:63    object motion property，default 0
    }ObjectsInforBits;
    //
    ObjectsInforBits objectsinformation;

    typedef struct { 
   		float Objects_ID;           //Object ID
   		float Objects_DistLong;     //V dist unit:m
   		float Objects_DistLat;      //H dist unit:m
   		float Objects_VrelLong;     //V speed
   		float Objects_DynProp;      //object motion property
   		float Objects_VrelLat;      //H speed
   		float Objects_RCS;          //RCS default 0
   		float target_distance;      //target distance
   		float target_deg;           //target angle
   		float target_speed;         //target speed
    }Object_dataBytes;

    Object_dataBytes get_object_data;

    bool _bRt;
    bool new_information;//radar start new scan
    uint8_t _Address;//radar address(0-7)
    float fdata,ftarget_distance, ftarget_deg;
};
#endif //HAL_MAX_CAN_PROTOCOL_DRIVERS