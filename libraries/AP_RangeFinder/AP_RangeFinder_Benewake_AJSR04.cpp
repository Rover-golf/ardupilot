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

#include "AP_RangeFinder_Benewake_AJSR04.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define AJSR04_FRAME_HEADER 0xff
#define AJSR04_FRAME_LENGTH 4
#define AJSR04_DIST_MAX_CM 800
#define AJSR04_OUT_OF_RANGE_ADD_CM 100
#define AJSR04_FRAME_HEADER_M 0x7f
#define AJSR04_FRAME_END_M 0x03
#define AJSR04_FRAME_LENGTH_M 10
#define LAkIBEAM1S_FRAME_LENGTH 6

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_AJSR04::get_reading(float &reading_m)
{
    uint16_t reading_cm = 0;
    float target_deg = 0;
    _Address = params.address;

    //hal.console->printf("sr-04: _Address=%d.\n",_Address);
    bool brt = false;
    if(_Address == 0)//LakiBeam1S radar
    {
        target_deg = 0.f;
        brt = get_reading_LakiBeam1S(reading_cm,target_deg);
    }
    else if(_Address == 1)//single channel radar
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
    state.target_deg = target_deg;
    reading_m = reading_cm * 0.01;

    return brt;     
}

// format of serial packets received from LakiBeam1S lidar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0xFF
// byte 1               DIST_H          Distance (in mm) high 8 bits
// byte 2               DIST_L          Distance (in mm) low 8 bits
// byte 3               ANGL_H          angle (in degree*100) high 8 bits
// byte 4               ANGL_L          angle (in degree*100) low 8 bits
// byte 5               Checksum        Checksum byte, sum of byte 1 to byte 4 just confirm low 8 bits
//get LakiBeam1S Ladar distance returned in reading_cm
bool AP_RangeFinder_AJSR04::get_reading_LakiBeam1S(uint16_t &reading_cm, float &reading_deg)
{
    //gcs().send_text(MAV_SEVERITY_INFO, "get_reading_LakiBeam1S:1");  
    if (uart == nullptr) {
        return false;
    }
    //gcs().send_text(MAV_SEVERITY_INFO, "get_reading_LakiBeam1S:2");  
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
        //gcs().send_text(MAV_SEVERITY_INFO, "get_reading_LakiBeam1S:3 read=%i",c);  
        // if buffer is empty and this byte is 0xff, add to buffer
        if (linebuf_len == 0) {
            if (c == AJSR04_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } 
        else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 6 items try to decode it
            if (linebuf_len == LAkIBEAM1S_FRAME_LENGTH)
             {
                // calculate checksum
                uint8_t checksum = 0;
                
                checksum = (linebuf[1] + linebuf[2]+ linebuf[3] + linebuf[4]) & 0x00ff;
               
                // if checksum matches extract contents
                if (checksum == linebuf[5]) {
                    // calculate distance
                    reading_cm = ((uint16_t)linebuf[1] << 8) | linebuf[2];
                    reading_deg = (((uint16_t)linebuf[3] << 8) | linebuf[4])/100.f;
                    //
                    //gcs().send_text(MAV_SEVERITY_INFO, "dis= %d, deg=%f", reading_cm,reading_deg);
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
    //    hal.console->printf("sr-04: distance=%d.\n",reading_cm);
        return true;
    }
    else if (count > 0) {
        // return distance of readings
        reading_cm = reading_cm / 10;
    //    hal.console->printf("sr-04: distance=%d.\n",reading_cm);
        return true;
    }

    // no readings so return false
    return false;
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
    //    hal.console->printf("sr-04: distance=%d.\n",reading_cm);
        return true;
    }
    else if (count > 0) {
        // return distance of readings
        reading_cm = reading_cm / 10;
    //    hal.console->printf("sr-04: distance=%d.\n",reading_cm);
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
    //    hal.console->printf("sr-04:sensor=%d, distance=%d.\n",sensornum,reading_cm);
        return true;
    }
    else if (count > 0) {
    //    hal.console->printf("sr-04:sensor=%d, distance=%d.\n",sensornum,reading_cm);
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