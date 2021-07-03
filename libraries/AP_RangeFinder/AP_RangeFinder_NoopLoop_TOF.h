#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define NOOPLOOP_HEADER 0x57    // message header
#define NOOPLOOP_MSG_BUF_MAX 256
#define NOOPLOOP_NODE_FRAME2_DIS    8
#define NOOPLOOP_NODE_FRAME2_SINGAL  12

#define NOOPLOOP_MAX_TOF 8


class AP_RangeFinder_NoopLoop_TOF : public AP_RangeFinder_Backend
{

public:

    // constructor
    AP_RangeFinder_NoopLoop_TOF(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            uint8_t serial_instance,
                            uint8_t num
                            );

    // static detection function
    static bool detect(uint8_t serial_instance);

    // update state
    void update(void) override;


    void request_data(uint8_t id);
    float get_data(uint8_t id) override;
   

    float _dis,_singal;
    float _disL[NOOPLOOP_MAX_TOF] = {0.0f};
    uint32_t _last_update_ms;
    uint8_t cnt=0;
    uint8_t _num=2;
    uint8_t data_ok = 0;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_cm
    bool get_reading(uint16_t &reading_cm);




     // // 解析AOA_Node_Frame0
    enum class MsgType : uint8_t
    {
        INVALID = 0,
        TOFSense_Frame
    };

    // // process one byte received on serial port
    // // message is stored in _msgbuf
    MsgType parse_byte(uint8_t b);

    void parse_tof_frame();
    enum class ParseState : uint8_t
    {
        HEADER = 0,        // waiting for header
        FUNCTION_MARK, // waiting for function mark
        PAYLOAD,       // receiving payload bytes
    } parese_state = ParseState::HEADER;

    // // members
    AP_HAL::UARTDriver *_uart = nullptr;             // pointer to uart configured for use with nooploop
    uint8_t _msgbuf[NOOPLOOP_MSG_BUF_MAX]; // buffer to hold most recent message from tag
    uint16_t _msg_len;                     // number of bytes received from the current message (may be larger than size of _msgbuf)
    uint16_t _frame_len=16;                   // message supplied frame length
    uint8_t _crc_expected;                 // calculated crc which is compared against actual received crc

};
