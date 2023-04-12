#pragma once


#include "AP_BattMonitor_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

#ifndef HAL_48_CAN_ENABLE
    #define HAL_48_CAN_ENABLE (!defined(HAL_BUILD_AP_PERIPH) && HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024) || (defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PERIPH_ENABLE_BATTERY_48))
#endif

#if HAL_48_CAN_ENABLE


class AP_BattMonitor_48 : public CANSensor, public AP_BattMonitor_Backend {
public:

    // construct the CAN Sensor
    AP_BattMonitor_48(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params):
        AP_BattMonitor_Backend(mon, mon_state, params),
        CANSensor("B48")
    {
        register_driver(AP_CANManager::Driver_Type_Batt48);
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
    // Message groups form part of the CAN ID of each frame
    enum class PacketType : uint32_t {
        ACK             = 0x21,
        NACK            = 0x22,
        PING            = 0x23,
        FAULT           = 0x42,
        ALGORITHM_SET   = 0x44,
        ALGORITHM_GET   = 0x45,
        STREAM_FAULT    = 0x47,
        STREAM_INPUT    = 0x101,
        STREAM_OUTPUT   = 0x100,
        VOLTAGE_SET     = 0x4B,
        VOLTAGE_GET     = 0x4C,
        CVT_SET         = 0x4D,
        CVT_GET         = 0x4E,
    };

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
        return B48_devices.is_healthy();
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

    struct B48_device {
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
    } B48_devices;
};

#endif // HAL_48_CAN_ENABLE

