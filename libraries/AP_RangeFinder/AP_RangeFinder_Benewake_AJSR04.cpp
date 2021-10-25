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

// format of serial packets received from benewake lidar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0xFF
// byte 1               DIST_H          Distance (in mm) high 8 bits
// byte 2               DIST_L          Distance (in mm) low 8 bits
// byte 3               Checksum        Checksum byte, sum of byte 1 and byte 2 just confirm low 8 bits

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_AJSR04::get_reading(uint16_t &reading_cm)
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
