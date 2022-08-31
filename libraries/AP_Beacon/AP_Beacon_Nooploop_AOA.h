#pragma once

#include "AP_Beacon_Backend.h"

#define NOOPLOOP_HEADER 0x55    // message header
#define NOOPLOOP_FUNCTION_MARK_07 0x07
#define NOOPLOOP_MSG_BUF_MAX 256

#define NOOPLOOP_NODE_FRAME2_FRAMELEN_MAX 4096  // frames should be less than 4k bytes
#define NOOPLOOP_NODE_FRAME2_DIS    23               // 20+3
#define NOOPLOOP_NODE_FRAME2_ANGEL  26


#define AVERAGE_ANGEL_MAX   20 //100 uwb202207

class AP_Beacon_Nooploop_AOA : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_Nooploop_AOA(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update() override;

    void get_data(float &dis, float &angel) override;
    void get_data_raw(float &dis, float &angel) override;

    float _dis = 0.0f, _angel = 0.0f;
    
    float L_angel[AVERAGE_ANGEL_MAX] = {0.0f};
    uint8_t angel_index = 0;

private:
    // // 解析AOA_Node_Frame0
    enum class MsgType : uint8_t
    {
        INVALID = 0,
        AOA_Node_Frame0,
        NODE_FRAME2,
        SETTING_FRAME0
    };

    // // process one byte received on serial port
    // // message is stored in _msgbuf
    MsgType parse_byte(uint8_t b);

    void parse_aoa_node_frame0();


    enum class ParseState : uint8_t
    {
        HEADER = 0,        // waiting for header
        H55_FUNCTION_MARK, // waiting for function mark
        LEN_L,             // waiting for low byte of length
        LEN_H,             // waiting for high byte of length
        NF2_PAYLOAD,       // receiving payload bytes
        SF0_PAYLOAD,       // receiving payload bytes
    } _state = ParseState::HEADER;

    // // members
    AP_HAL::UARTDriver *_uart;             // pointer to uart configured for use with nooploop
    uint8_t _msgbuf[NOOPLOOP_MSG_BUF_MAX]; // buffer to hold most recent message from tag
    uint16_t _msg_len;                     // number of bytes received from the current message (may be larger than size of _msgbuf)
    uint16_t _frame_len;                   // message supplied frame length
    uint8_t _crc_expected;                 // calculated crc which is compared against actual received crc
    uint32_t _last_update_ms;              // last time we receive data from tag

};

