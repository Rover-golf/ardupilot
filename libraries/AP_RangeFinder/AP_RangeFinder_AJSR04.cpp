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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_AJSR04.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define AJSR04_FRAME_HEADER 0xff
#define AJSR04_FRAME_LENGTH 4
#define AJSR04_DIST_MAX_CM 800
#define AJSR04_OUT_OF_RANGE_ADD_CM 100
#define AJSR04_FRAME_HEADER_M 0x7f
#define AJSR04_FRAME_END_M 0x03
#define AJSR04_FRAME_LENGTH_M 10

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_AJSR04::AP_RangeFinder_AJSR04(RangeFinder::RangeFinder_State &_state,
                                                             AP_RangeFinder_Params &_params,
                                                             uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params)
{
    _Address = _params.address;
    const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

/* 
   detect if a Benewake rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_AJSR04::detect(uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}


/* 
   update the state of the sensor
*/
void AP_RangeFinder_AJSR04::update(void)
{
    if (get_reading(state.distance_cm, state.target_deg)) {
        // update range_valid state based on distance measured
  	 //   hal.console->printf("AP_RangeFinder_AJSR04 Update=%d, deg=%.0f\n",state.distance_cm,state.target_deg);
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 500) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_AJSR04::get_reading(uint16_t &reading_cm,float& target_deg)
{
    //hal.console->printf("sr-04: _Address=%d.\n",_Address);
    bool brt = false;
    if(_Address == 0)//single channel radar
    {
        target_deg = 0.f;
        brt = get_reading_single(reading_cm);
    }
    else//multi channel radar
    {
        if(_cursensorid > _Address)
            _cursensorid = 0x01;
       //require reading distance
        uint8_t sensornum = _cursensorid;
        //for next sensor
        _cursensorid++;
        brt = require_reading_distance(sensornum);
//        hal.console->printf("sr-04: require=%d.\n",(int)brt);
         if(!brt)
            return false;
        //read distance 
        brt = get_reading_multi(reading_cm, sensornum);
        //define sensor anlge
        target_deg = 0.f;
        switch(sensornum)
        {
            case 1:
                target_deg = 45.f;
                break;
            case 2:            
                target_deg = 15.f;
                break;
            case 3:
                target_deg = -15.f;
                break;
            case 4:
                target_deg = -45.f;
                break;
            default:
                target_deg = 0.f;
                break;
        }
    }

    return brt;     
}

// format of serial packets received from benewake lidar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0xFF
// byte 1               DIST_H          Distance (in mm) high 8 bits
// byte 2               DIST_L          Distance (in mm) low 8 bits
// byte 3               Checksum        Checksum byte, sum of byte 1 and byte 2 just confirm low 8 bits

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_AJSR04::get_reading_single(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    uint16_t count = 0;
    uint16_t count_out_of_range = 0;
    linebuf_len = 0;
    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0xff, add to buffer
        if (linebuf_len == 0) {
            if (c == AJSR04_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } 
        else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 4 items try to decode it
            if (linebuf_len == AJSR04_FRAME_LENGTH)
             {
                // calculate checksum
                uint8_t checksum = 0;
                
                checksum = (linebuf[1] + linebuf[2]) & 0x00ff;
               
                // if checksum matches extract contents
                if (checksum == linebuf[3]) {
                    // calculate distance
                    reading_cm = ((uint16_t)linebuf[1] << 8) | linebuf[2];
                    count = 2;
                    if (reading_cm >= 7000) {
                        // this reading is out of range
                        count_out_of_range = 1;
                    }

                }
                // clear buffer
                linebuf_len = 0;
                break;//just return one distance 
             }
        }
    }   

    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_cm = MAX(AJSR04_DIST_MAX_CM, max_distance_cm() + AJSR04_OUT_OF_RANGE_ADD_CM);
        //hal.console->printf("sr-04: distance=%d.\n",reading_cm);
        return true;
    }
    else if (count > 0) {
        // return distance of readings
        reading_cm = reading_cm / 10;
        //hal.console->printf("sr-04: distance=%d.\n",reading_cm);
        return true;
    }

    // no readings so return false
    return false;
}  

//----------------Multi-Sensor--------------
// format of serial packets received from ajsr04 radar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x7f
// byte 1               sensor number   01-255
// byte 2               command number  0x12 read distance
// byte 3               Data direction  0x00 send to sensor; 0x01 receive from sensor         
// byte 4               data0           Distance (in mm) data0 * 0x100
// byte 5               data1           Distance (in mm) low 8 bits
// byte 6               data2
// byte 7               data3
// byte 8               End flag        0x03
// byte 9               Checksum        Checksum byte, sum from byte 1 to byte 8 just confirm low 8 bits

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_AJSR04::get_reading_multi(uint16_t &reading_cm, uint8_t &sensornum)
{
    if (uart == nullptr) {
        return false;
    }

    uint16_t count = 0;
    uint16_t count_out_of_range = 0;
    linebuf_len = 0;
    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0xff, add to buffer
        if (linebuf_len == 0) {
            if (c == AJSR04_FRAME_HEADER_M) {
                linebuf[linebuf_len++] = c;
            }
        } 
        else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 10 items try to decode it
            if (linebuf_len == AJSR04_FRAME_LENGTH_M && linebuf[8] == AJSR04_FRAME_END_M)
             {
                // calculate checksum
                uint8_t checksum = 0;
                for(uint8_t i=1;i<9; i++)
                    checksum += linebuf[i];  
                //checksum = checksum & 0x00ff;
               
                // if checksum matches extract contents
                if (checksum == linebuf[9]) {
                    // calculate distance
                    reading_cm = ((uint16_t)linebuf[4] * 0X100 + linebuf[5]) / 10;
                    sensornum = linebuf[1];
                    count = 2;
                    if (reading_cm >= 700) {
                        // this reading is out of range
                        count_out_of_range = 1;
                    }

                }
                // clear buffer
                linebuf_len = 0;
                break;//just return one distance 
             }
        }
    }   

    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_cm = MAX(AJSR04_DIST_MAX_CM, max_distance_cm() + AJSR04_OUT_OF_RANGE_ADD_CM);
 //       hal.console->printf("sr-04:sensor=%d, distance=%d.\n",sensornum,reading_cm);
        return true;
    }
    else if (count > 0) {
//        hal.console->printf("sr-04:sensor=%d, distance=%d.\n",sensornum,reading_cm);
        return true;
    }

    // no readings so return false
    return false;
}

//require reading distance information
bool AP_RangeFinder_AJSR04::require_reading_distance(uint8_t sensornum)
{
    if (uart == nullptr) {
        return false;
    }

    uint8_t commandbuf[10];
    bool brt = false;

    memset(commandbuf,0x00,10);
    commandbuf[0] = 0x7f;
    commandbuf[1] = sensornum;
    commandbuf[2] = 0x12;
    commandbuf[3] = 0x00;
    commandbuf[8] = 0x03;
    // calculate checksum
    uint8_t checksum = 0;
    for(uint8_t i=1;i<9; i++)
        checksum += commandbuf[i];  
    checksum = checksum & 0x00ff;   
    commandbuf[9] = checksum;
    
    uint8_t irt = uart->write(commandbuf,10);
    if(irt == 10)
        brt = true;

    return brt;
}



