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
#include "AP_RangeFinder_NoopLoop_TOF.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;


/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_NoopLoop_TOF::AP_RangeFinder_NoopLoop_TOF(RangeFinder::RangeFinder_State &_state,
                                                             AP_RangeFinder_Params &_params,
                                                             uint8_t serial_instance,
                                                             uint8_t num
                                                             ) :
    AP_RangeFinder_Backend(_state, _params)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (_uart != nullptr) {
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
        _num = num;
    }
}

/* 
   detect if a Benewake rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_NoopLoop_TOF::detect(uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_NoopLoop_TOF::get_reading(uint16_t &reading_cm)
{
    if (_uart == nullptr) { return false; }
    reading_cm = (uint16_t) _disL[0]; 
    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_NoopLoop_TOF::update(void)
{
    if (_uart == nullptr) { return; }
    // gcs().send_text(MAV_SEVERITY_DEBUG, "NoopLoop Update %d", _num);


    // check uart for any incoming messages
    uint32_t nbytes = MIN(_uart->available(), 256U);
    while (nbytes-- > 0)
    {
        int16_t b = _uart->read();
        if (b >= 0)
        {
            // gcs().send_text(MAV_SEVERITY_DEBUG, "0x%02x",b);
            MsgType type = parse_byte((uint8_t) b);

            if (type == MsgType::TOFSense_Frame)
            {
                parse_tof_frame();
                // gcs().send_text(MAV_SEVERITY_DEBUG, "test from AP_RangeFinder_NoopLoop_TOF");
            }

        }
    }


    if (AP_HAL::millis() - _last_update_ms > 2000 || data_ok)
    {
        _last_update_ms = AP_HAL::millis();
        request_data(cnt);
        data_ok = 0;
        cnt++;


        if (cnt >= _num)
            cnt = 0;
    }
}


void AP_RangeFinder_NoopLoop_TOF::request_data(uint8_t id)
{
    uint8_t buff[] = {0x57,0x10,0xFF,0xFF,0x00,0xFF,0xFF,0x63};
    buff[4] = id;
    buff[7] += id;
    _uart->write(buff,sizeof(buff));  
    
}


// process one byte received on serial port
// message is stored in _msgbuf
AP_RangeFinder_NoopLoop_TOF::MsgType AP_RangeFinder_NoopLoop_TOF::parse_byte(uint8_t b)
{
    // process byte depending upon current state
    switch (parese_state)
    {
        case ParseState::HEADER:
            if (b == NOOPLOOP_HEADER)
            {
                _msgbuf[0] = b;
                _msg_len = 1;
                _frame_len = 16;
                _crc_expected = b;
                parese_state = ParseState::FUNCTION_MARK;
            }
            break;
        case ParseState::FUNCTION_MARK:
            _msgbuf[1] = b;
            _msg_len++;
            _crc_expected += b;
            parese_state = ParseState::PAYLOAD;
            break;
        case ParseState::PAYLOAD:
            // add byte to buffer if there is room
            if (_msg_len < NOOPLOOP_MSG_BUF_MAX)
            {
                _msgbuf[_msg_len] = b;
            }
            _msg_len++;

            if (_msg_len >= _frame_len)
            {
                parese_state = ParseState::HEADER;
                // _msg_len = 0;
                // check crc
                if (b == _crc_expected)
                {
                    return MsgType::TOFSense_Frame;
                }
            }
            else
            {
                _crc_expected += b;
            }
            break;
        default:
            break;
    }
    // gcs().send_text(MAV_SEVERITY_DEBUG, "_msg_len:%d", _msg_len);
    return MsgType::INVALID;
}


void AP_RangeFinder_NoopLoop_TOF::parse_tof_frame()
{
    // 有接收到block才解析
    if (_frame_len >= 16)
    {
        _last_update_ms = AP_HAL::millis();
        const uint8_t id = _msgbuf[3];
        const int32_t dis = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_DIS + 2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_DIS + 1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_DIS] << 8) >> 8;
        const int16_t singal = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_SINGAL + 1] << 8 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_SINGAL]);
        _dis = dis/1000.0f;
        _singal = singal;
        data_ok = 1;
        _disL[id] = _dis;
        // gcs().send_text(MAV_SEVERITY_DEBUG, "id: %d %.2f %.2f",id, _dis, _singal);
    }

}

float AP_RangeFinder_NoopLoop_TOF::get_data(uint8_t id) { return _disL[id]; }