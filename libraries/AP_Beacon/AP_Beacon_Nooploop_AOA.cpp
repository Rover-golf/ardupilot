#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Beacon_Nooploop_AOA.h"
#include <ctype.h>
#include <stdio.h>

AP_Beacon_Nooploop_AOA::AP_Beacon_Nooploop_AOA(AP_Beacon &frontend, AP_SerialManager &serial_manager) : 
    AP_Beacon_Backend(frontend)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);
    if (_uart != nullptr)
    {
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Beacon, 0));
    }
}

bool AP_Beacon_Nooploop_AOA::healthy() { return true; }

void AP_Beacon_Nooploop_AOA::update(void)
{
    // set_beacon_distance(0, 3.14159f);

    // return immediately if not serial port
    if (_uart == nullptr) { return; }

    // check uart for any incoming messages
    uint32_t nbytes = MIN(_uart->available(), 1024U);
    while (nbytes-- > 0)
    {
        int16_t b = _uart->read();
        if (b >= 0)
        {
            // gcs().send_text(MAV_SEVERITY_DEBUG, "0x%02x",b);
            MsgType type = parse_byte((uint8_t) b);

            if (type == MsgType::AOA_Node_Frame0)
            {
                parse_aoa_node_frame0();
            }

        }
    }
}

// process one byte received on serial port
// message is stored in _msgbuf
AP_Beacon_Nooploop_AOA::MsgType AP_Beacon_Nooploop_AOA::parse_byte(uint8_t b)
{
    // process byte depending upon current state
    switch (_state)
    {
        case ParseState::HEADER:
            if (b == NOOPLOOP_HEADER)
            {
                _msgbuf[0] = b;
                _msg_len = 1;
                _crc_expected = b;
                _state = ParseState::H55_FUNCTION_MARK;
            }
            break;
        case ParseState::H55_FUNCTION_MARK:
            if (b == NOOPLOOP_FUNCTION_MARK_07)
            {
                _msgbuf[1] = b;
                _msg_len++;
                _crc_expected += b;
                _state = ParseState::LEN_L;
            }
            else
            {
                _state = ParseState::HEADER;
            }
            break;

        case ParseState::LEN_L:
            _msgbuf[2] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::LEN_H;
            break;

        case ParseState::LEN_H:
            // extract and sanity check frame length
            _frame_len = UINT16_VALUE(b, _msgbuf[2]);
            if (_frame_len > NOOPLOOP_NODE_FRAME2_FRAMELEN_MAX)
            {
                _state = ParseState::HEADER;
            }
            else
            {
                _msgbuf[3] = b;
                _msg_len++;
                _crc_expected += b;
                _state = ParseState::NF2_PAYLOAD;
            }
            break;

        case ParseState::NF2_PAYLOAD:
            // add byte to buffer if there is room
            if (_msg_len < NOOPLOOP_MSG_BUF_MAX)
            {
                _msgbuf[_msg_len] = b;
            }
            _msg_len++;
            if (_msg_len >= _frame_len)
            {
                _state = ParseState::HEADER;
                // check crc
                if (b == _crc_expected)
                {
                    return MsgType::AOA_Node_Frame0;
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

    return MsgType::INVALID;
}


// 拿到dis和angel
void AP_Beacon_Nooploop_AOA::parse_aoa_node_frame0()
{
    // 有接收到block才解析
    if (_frame_len > 4+17)
    {
        _last_update_ms = AP_HAL::millis();
        const int32_t dis = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_DIS + 2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_DIS + 1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_DIS] << 8) >> 8;
        const int16_t angel = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_ANGEL + 1] << 8 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_ANGEL]);
        _dis = dis/1000.0f;
        _angel = angel/100.0f;
        L_angel[angel_index] = _angel;
        angel_index++;
        if(angel_index >= AVERAGE_ANGEL_MAX)
                angel_index=0;
        //gcs().send_text(MAV_SEVERITY_DEBUG, "%.2f %.2f", _dis, _angel);
    }

}

void AP_Beacon_Nooploop_AOA::get_data_raw(float &dis, float &angel)
{
    dis = _dis;
    angel = _angel;
}

void AP_Beacon_Nooploop_AOA::get_data(float &dis, float &angel)
{
    float temp = 0;
    for (uint8_t i = 0; i < AVERAGE_ANGEL_MAX; i++)
    {
        temp += L_angel[i];
    }
    
    dis = _dis;
    angel = temp/AVERAGE_ANGEL_MAX;


}