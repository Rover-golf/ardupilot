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

#include "AP_RangeFinder_CYGLidar.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define CYGLIDAR_DIST_MAX_CM 800
#define CYGLIDAR_OUT_OF_RANGE_ADD_CM 100
#define POS_PAYLOAD_DATA	6

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_CYGLidar::get_reading(uint16_t &reading_cm)
{
    if(isreading)
        return false;
    isreading = true;

    float target_deg = 0;
    bool brt = false;

     if( AP_HAL::millis() - last_reading_CYGLidar > 1000)
     {
      //  hal.console->printf("CYGLidar get_reading require_command.\n");
        brt = require_command(0x01);
        if(!brt)
        {
            isreading = false;
            return false;
        }
        else
            last_reading_CYGLidar = AP_HAL::millis();
     }

    //read distance 
    brt = get_reading_2d(reading_cm,target_deg);
    //anlge
    state.target_deg = target_deg;
    isreading = false;
    return brt;     
}

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_CYGLidar::get_reading_2d(uint16_t &reading_cm,float &target_deg)
{
    if (uart == nullptr) {
        return false;
    }
    uint8_t count = 0;
    uint8_t count_out_of_range = 0;
    uint8_t parser;
    bool brt = false;
    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    //hal.console->printf("CYGLidar: get_reading_2d nbytes=%d.\n",nbytes);
    if(nbytes<=0)
        return false;

    // uint8_t buffer[330];
    // int16_t rdnbytes = uart->read(buffer, nbytes);
    // hal.console->printf("CYGLidar: get_reading_2d Rd_nbytes=%d.\n",rdnbytes);
    // for(uint16_t i=rdnbytes-20;i<rdnbytes;i++)
    //     hal.console->printf("CYGLidar: get_reading_2d %d:r=%x.\n",i,buffer[i]);
    while (nbytes-- > 0) {
        int16_t r = uart->read();
       // hal.console->printf("CYGLidar: get_reading_2d %d:r=%x.\n",nbytes,r);
        if (r < 0) {
            continue;
        //    hal.console->printf("CYGLidar: get_reading_2d r<0 continue.\n");
        }
        uint8_t c = (uint8_t)r;
        // Check on the validity of dataset
        parser = CygParser(bufferPtr, c);
      //  hal.console->printf("CYGLidar: get_reading_2d parser=%d.\n",parser);      
        switch (parser)
        {
            case 0x01:
                switch (bufferPtr[POS_PAYLOAD_HEADER])
                {
                    case 0x01: // 2D
                        brt = Get2DMinDistance(reading_cm,target_deg);
                        if(brt)
                            count = 1;
                        else
                            count_out_of_range = 1;
                        break;
                    case 0x08: // 3D
                        break;
                }
                break;
            case 0x00: // passed through header 1
                
                //current_time_laser = ros::Time::now(); 
                break;
            default:
                break;
        }    
    }   
    last_reading_CYGLidar = AP_HAL::millis();

    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_cm = MAX(CYGLIDAR_DIST_MAX_CM, max_distance_cm() + CYGLIDAR_OUT_OF_RANGE_ADD_CM);
        //hal.console->printf("CYGLidar: count_out_of_range distance=%d, target_deg=%f.\n",reading_cm,target_deg);
        return true;
    }
    else if (count > 0) {
        //hal.console->printf("CYGLidar: distance=%d,target_deg=%f.\n",reading_cm,target_deg);
        return true;
    }
    //hal.console->printf("CYGLidar: distance=false.\n");
    // no readings so return false
    return false;
}  


//require command
bool AP_RangeFinder_CYGLidar::require_command(uint8_t commandid)
{
    if (uart == nullptr) {
        return false;
    }

    uint8_t commandbuf[10];
    bool brt = false;

    memset(commandbuf,0x00,10);
    commandbuf[0] = 0x5A;
    commandbuf[1] = 0x77;
    commandbuf[2] = 0xFF;
    switch(commandid)
    {
        case 0x01://2D 5A 77 FF 02 00 01 00 03
            commandbuf[3] = 0x02;
            commandbuf[4] = 0x00;
            commandbuf[5] = 0x01;
            commandbuf[6] = 0x00;
            //commandbuf[7] = 0x03;           
            break;
        case 0x08://3D
            break;
        case 0x02://Stop 0x5A 0x77 0xFF 0x02 0x00 0x02 0x00 0x00
            commandbuf[3] = 0x02;
            commandbuf[4] = 0x00;
            commandbuf[5] = 0x02;
            commandbuf[6] = 0x00;
            //commandbuf[7] = 0x00;          
            break;
        default:
            break;
    }
    // calculate checksum
    uint8_t checksum = 0;
    for(int i = 3; i < 8 - 1; i++)
    { 
        checksum ^= commandbuf[i]; 
    }   
    commandbuf[7] = checksum;
    //hal.console->printf("CYGLidar: checksum=%d.\n",checksum);
    uint8_t irt = uart->write(commandbuf,8);
    if(irt == 8)
        brt = true;
    hal.console->printf("CygLidar: require=%d.\n",(int)brt);
    return brt;
}

bool AP_RangeFinder_CYGLidar::Get2DMinDistance(uint16_t &reading_cm,float &target_deg)
{
    uint16_t PayloadSize = ((bufferPtr[POS_LENGTH_2] << 8) & 0xff00) | (bufferPtr[POS_LENGTH_1] & 0x00ff);
    uint16_t datalen = (PayloadSize - 1) / 2;
    uint16_t currentdata = 0;//cm
    uint16_t mindistance = CYGLIDAR_DIST_MAX_CM;//meter (20-800)
    uint16_t mindistId = 0;

    for (uint16_t i = 0; i < datalen; i++)
    {
        currentdata = ((bufferPtr[POS_PAYLOAD_DATA+i*2] << 8) & 0xff00) | (bufferPtr[POS_PAYLOAD_DATA+i*2+1] & 0x00ff);
        currentdata /= 10;//mm -> cm
        if(currentdata >= min_distance_cm() && currentdata <= max_distance_cm() && currentdata < mindistance)
        {
            mindistance = currentdata;
            mindistId = i;
        }
    }

    reading_cm = mindistance;
    target_deg = 0.75 * mindistId;
    //find target
    if(mindistance < CYGLIDAR_DIST_MAX_CM)
    {
        return true;
    }

    return false;
}