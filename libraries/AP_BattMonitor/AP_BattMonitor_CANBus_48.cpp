#include "AP_BattMonitor_CANBus_48.h"

#if HAL_48_CAN_ENABLE
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// read - read the voltage and current
void AP_BattMonitor_48::read()
{
    WITH_SEMAPHORE(_sem_static);

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - logger_last_ms >= 1000) {
        logger_last_ms = now_ms;
        perform_logging();
    }

    // get output voltage and current but allow for getting input if serial number is negative
    // Using _params._serial_number == 0 will give you the average output of all devices
    if (get_voltage_and_current_and_temp( _state.voltage, _state.current_amps, _state.temperature)) {
        _state.temperature_time = now_ms;
        _state.last_time_micros = AP_HAL::micros();
        _state.healthy = true;
        return;
    }

    _state.voltage = 0;
    _state.current_amps = 0;
    _state.healthy = false;
}

void AP_BattMonitor_48::perform_logging() const
{
#ifndef HAL_BUILD_AP_PERIPH

    AP_Logger *logger = AP_Logger::get_singleton();
    if (!logger || !logger->logging_enabled()) {
        return;
    }

    // log to AP_Logger
    // @LoggerMessage: MPPT
    // @Description: Information about the Maximum Power Point Tracker sensor
    // @Field: TimeUS: Time since system startup
    // @Field: F: Faults
    // @FieldBits: F: Over-Voltage,Under-Voltage,Over-Current,Over-Temperature
    // @Field: Temp: Temperature
    // @Field: InV: Input Voltage
    // @Field: InC: Input Current
    // @Field: InP: Input Power
    // @Field: OutV: Output Voltage
    // @Field: OutC: Output Current
    // @Field: OutP: Output Power
    
    AP::logger().Write("B48", "TimeUS,F,Temp,InV,InC,InP,OutV,OutC,OutP",
                       "s#--OVAWVAW",
                       "F----------",
                        "QBHBbffffff",
                       AP_HAL::micros64(),
                        (uint8_t)B48_devices.faults,
                        B48_devices.temperature,
                        (double)B48_devices.input.voltage,
                        (double)B48_devices.input.current,
                        (double)B48_devices.input.power,
                        (double)B48_devices.output.voltage,
                        (double)B48_devices.output.current,
                        (double)B48_devices.output.power);
#endif
}
// send command
void AP_BattMonitor_48::Check_SendData()
{
    send_command(PacketType::STREAM_OUTPUT);
    //TEST BATT
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BattMonitor::Check_SendData(0X5A)");
    
}
// parse inbound frames
void AP_BattMonitor_48::handle_frame(AP_HAL::CANFrame &frame)
{
    const uint16_t canid = frame.id & 0x0000FFFF;
    //TEST BATT
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PDCAN: %u NACK 0x%2X", canid, (unsigned)packet_sent_prev);
    if (canid == 0) {
        // This is for broadcast and I don't think we should allow this inbound.
        return;
    }

    //check
    uint16_t chknum = Check_CRC16(frame.data,6);
    if(chknum != UINT16_VALUE(frame.data[6],frame.data[7]))
        return;
    //TEST BATT
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PDCAN: %u checknum 0x%u", canid, chknum); 
    WITH_SEMAPHORE(_sem_static);

    switch ((PacketType)frame.id) {
    case PacketType::STREAM_FAULT:
        B48_devices.faults = (FaultFlags)frame.data[2];
        B48_devices.temperature = frame.data[3];
        break;

    case PacketType::STREAM_INPUT:
        B48_devices.input.voltage = (float)(UINT16_VALUE(frame.data[0], frame.data[1]))*0.01;//10mV -> V
        B48_devices.input.current = (float)((int32_t)(frame.data[2]<<8) | (int32_t)frame.data[3])*0.01; //10mA -> A
        B48_devices.input.power =  (float)(UINT16_VALUE(frame.data[4], frame.data[5]))*0.01; // 10mAh -> Ah
        break;
    case PacketType::STREAM_OUTPUT:
        B48_devices.output.voltage = (float)(UINT16_VALUE(frame.data[0], frame.data[1]))*0.01;//10mV -> V
        if(frame.data[2]>>7 > 0)//negative
            B48_devices.output.current = ((float)(UINT16_VALUE(frame.data[2], frame.data[3]))-0xffff-1)*0.01; //10mA -> A
        else
            B48_devices.output.current = (float)(UINT16_VALUE(frame.data[2], frame.data[3]))*0.01; // 10mA -> A

            
        B48_devices.output.power = (float)(UINT16_VALUE(frame.data[4], frame.data[5]))*0.01; // 10mAh -> Ah
        //TEST BATT
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "D0:%02X D1:%02X D2:%02X D3:%02X", frame.data[0],frame.data[1],frame.data[2],frame.data[3]); 
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "V: %f cur: %f pow:%f", B48_devices.output.voltage, B48_devices.output.current,B48_devices.output.power); 
        break;

    case PacketType::FAULT: {
        // This msg is received when a new fault event happens. It contains the bitfield of all faults.
        // We use this event to compare against existing faults to notify the user of just the new fault
        const uint8_t all_current_faults = frame.data[2];
        const uint8_t prev_faults = (uint8_t)B48_devices.faults;
        const uint8_t new_single_fault = (~prev_faults & all_current_faults);
        if (new_single_fault != 0) {
        //    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "PDCAN: New Fault! %d: %s", (int)new_single_fault, get_fault_code_string((FaultFlags)new_single_fault));
        }
        B48_devices.faults = (FaultFlags)frame.data[2];
        }
        break;

    case PacketType::ACK:
        break;
    case PacketType::NACK:
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PDCAN: %u NACK 0x%2X", serialnumber, (unsigned)packet_sent_prev);
        break;

    case PacketType::ALGORITHM_SET:
        B48_devices.algorithm = frame.data[2];
        break;

    case PacketType::VOLTAGE_SET:
        B48_devices.output_voltage_fixed = fixed2float(UINT16_VALUE(frame.data[2], frame.data[3]));
        break;

    case PacketType::CVT_SET:
        B48_devices.cvt = fixed2float(UINT16_VALUE(frame.data[2], frame.data[3]));
        break;

    case PacketType::ALGORITHM_GET: // this is a request so we never expect it inbound
    case PacketType::VOLTAGE_GET:   // this is a request so we never expect it inbound
    case PacketType::CVT_GET:       // this is a request so we never expect it inbound
    case PacketType::PING:          // this is never received. When sent it generates an ACK
        // nothing to do
        return;
    }
    B48_devices.timestamp_ms = AP_HAL::millis();
}

void AP_BattMonitor_48::send_command(const PacketType type, const float data)
{
    //1 second send once.
    if(AP_HAL::millis() - presendcmd_ms < 600)
        return;
    presendcmd_ms = AP_HAL::millis(); 

    AP_HAL::CANFrame txFrame;

    switch (type) {
    case PacketType::STREAM_FAULT:
    case PacketType::STREAM_INPUT:
    case PacketType::STREAM_OUTPUT:
    case PacketType::FAULT:
    case PacketType::ACK:
    case PacketType::NACK:
    case PacketType::ALGORITHM_GET:
    case PacketType::VOLTAGE_GET:
    case PacketType::CVT_GET:
    case PacketType::PING:
        txFrame.dlc = 1;
        txFrame.data[0] = 0X5A;
        break;

    case PacketType::ALGORITHM_SET:
        txFrame.dlc = 1;
        txFrame.data[2] = data;
        break;

    case PacketType::VOLTAGE_SET:
    case PacketType::CVT_SET:
        {
        txFrame.dlc = 2;
        const uint16_t value = float2fixed(data);
        txFrame.data[2] = HIGHBYTE(value);
        txFrame.data[3] = LOWBYTE(value);
        }
        break;
    }

    txFrame.id = (uint32_t)type;

    if (write_frame(txFrame, 50000)) {
        // keep track of what we sent last in case we get an ACK/NACK
        packet_sent_prev = type;
        //TEST BATT
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "writeframe data0 0x%2X",txFrame.data[0]);
    }
    //TEST BATT
    //else
    //    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "writeframe error.");
}

// return fault code as string
const char* AP_BattMonitor_48::get_fault_code_string(const FaultFlags fault) const
{
    switch (fault) {
    case FaultFlags::OVER_VOLTAGE:
        return "Over-Voltage";
    case FaultFlags::UNDER_VOLTAGE:
        return "Under-Voltage";
    case FaultFlags::OVER_CURRENT:
        return "Over-Current";
    case FaultFlags::OVER_TEMPERATURE:
        return "Over-Temperature";
    default:
        return "Unknown";
    }
}
// when returning false, no values were changed.
bool AP_BattMonitor_48::get_voltage_and_current_and_temp( float &voltage, float &current, float &temperature) const
{

    voltage = 0.0f;
    current = 0.0f;
    temperature = 0.0f;

    if (!B48_devices.is_healthy()) {
        return false;
    }
            
    voltage = B48_devices.output.voltage;
    current = B48_devices.output.current;
    temperature = (float)B48_devices.temperature;
    return true;

}
//CRC16 check
#define         CRC_16_POLYNOMIALS  0xa001    
// X^16 +X^15 +X^2+1
uint16_t AP_BattMonitor_48::Check_CRC16(uint8_t* pchMsg,uint8_t wDataLen)
{
    uint8_t i, chChar;
    uint16_t wCRC = 0xFFFF;
    while (wDataLen--)
    {
        chChar = *pchMsg++;
        wCRC ^= (uint16_t) chChar;
        for (i = 0; i < 8; i++)
        {
            if (wCRC & 0x0001)
                wCRC = (wCRC >> 1) ^ CRC_16_POLYNOMIALS;
            else
                wCRC >>= 1;
       }
   }
   return wCRC;
}


#endif // HAL_48_CAN_ENABLE
