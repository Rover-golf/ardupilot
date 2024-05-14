#pragma once


#include "AP_BattMonitor_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#ifndef HAL_229_CAN_ENABLE
    #define HAL_229_CAN_ENABLE (!defined(HAL_BUILD_AP_PERIPH) && HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024) || (defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PERIPH_ENABLE_BATTERY_229))
#endif

#if HAL_229_CAN_ENABLE

//receive data length
#define RECEIVE_BUF_LEN 45

class AP_BattMonitor_229 : public CANSensor, public AP_BattMonitor_Backend {
public:

    // construct the CAN Sensor
    AP_BattMonitor_229(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params):
        AP_BattMonitor_Backend(mon, mon_state, params),
        CANSensor("MK229")
    {
        register_driver(AP_CANManager::Driver_Type_Batt229);
    }

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

protected:
    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    //check connect and send data
    void Check_SendData() override;
private:
//设备地址 功能代码 数据段   CRC16校验码
//1个byte 1个byte N个byte 2个byte(低字节在前)
//主机发送： 01 03    00 00   00 02 CRC
//       地址 功能码 起始地址 数据长度 CRC 码
//从机响应： 01 03      04     12 45 56 68           CRC
//       地址 功能码 返回字节数 寄存器数据 1 寄存器数据 2 CRC 码
//发送数据：01 03 01 00 00 14 44 39 （读 0100 开始的 20 个寄存器）
//接收数据：01 03 28 00 3C FC C8 00 03 0D DB 04 C6 89 50 00 00 00 00 04 C6 89 50 00 00 03 E8
//00 00 00 00 00 00 00 28 00 00 00 04 00 00 00 02 47 F8
    //设备地址
    uint8_t Device_Adress = 0x01;

    //功能代码
    enum class PacketType : uint8_t
    {
        STREAM_INPUT = 0x10,  // 写多路寄存器
        STREAM_OUTPUT = 0x03, // 读多路寄存器
        RELAY_SET = 0x4B,     // 写 1 路继电器的输出状态
        RELAY_GET = 0x4C,     // 读 1 路继电器的输出状态
    };
    //
    uint8_t receive_buffer[RECEIVE_BUF_LEN];
    uint8_t current_buflen = 0x00;

    enum class FaultFlags : uint8_t {
        OVER_VOLTAGE        = (1<<0),
        UNDER_VOLTAGE       = (1<<1),
        OVER_CURRENT        = (1<<2),
        OVER_TEMPERATURE    = (1<<3),
    };

    enum class ReportMode : uint8_t {
        DISABLED            = 0,
        REPORT_INPUTS       = 1,
        REPORT_OUTPUTS      = 2,
        REPORT_VOUT_CVT_ALG = 3,
    };
    // send command to MPPT device
    void send_command(const PacketType type,  const float data = 0.0f);

    bool is_healthy( ) const {
        return MK229_device.is_healthy();
    }

    const char* get_fault_code_string(const FaultFlags fault) const;

    //Frames received from the MPPT will use CAN Extended ID: 0x0042XXXX,
    // the least significant 16 bits contain the serial number of the MPPT.
    static constexpr uint32_t EXPECTED_FRAME_ID = (0x00420000 | AP_HAL::CANFrame::FlagEFF);

    bool get_voltage_and_current_and_temp( float &voltage, float &current, float &temperature) const;

    void perform_logging() const;
    uint16_t Check_CRC16(uint8_t* pchMsg,uint8_t wDataLen);
    HAL_Semaphore _sem_static;
    uint32_t logger_last_ms;

    PacketType packet_sent_prev;

    uint32_t presendcmd_ms = 0;
    
    struct MK229_device {
        uint32_t timestamp_ms;
        uint8_t sequence;

        FaultFlags faults;
        int8_t temperature;
        uint8_t algorithm;
        float output_voltage_fixed;
        float cvt;

        struct {
            float voltage;
            float current;
            float power;
        } input, output;

        bool is_healthy() const {
            return ((timestamp_ms > 0) && ((AP_HAL::millis() - timestamp_ms) < 2000));
        }
    } MK229_device;
};

#endif // HAL_48_CAN_ENABLE

