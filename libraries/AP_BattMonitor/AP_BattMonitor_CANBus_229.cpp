#include "AP_BattMonitor_CANBus_229.h"

#if HAL_229_CAN_ENABLE
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// read - read the voltage and current
void AP_BattMonitor_229::read()
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

void AP_BattMonitor_229::perform_logging() const
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
    
    AP::logger().Write("MK229", "TimeUS,F,Temp,InV,InC,InP,OutV,OutC,OutP",
                       "s#--OVAWVAW",
                       "F----------",
                        "QBHBbffffff",
                       AP_HAL::micros64(),
                        (uint8_t)MK229_device.faults,
                        MK229_device.temperature,
                        (double)MK229_device.input.voltage,
                        (double)MK229_device.input.current,
                        (double)MK229_device.input.power,
                        (double)MK229_device.output.voltage,
                        (double)MK229_device.output.current,
                        (double)MK229_device.output.power);
#endif
}
// send command
void AP_BattMonitor_229::Check_SendData()
{
    send_command(PacketType::STREAM_OUTPUT);
    //TEST BATT
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BattMonitor::Check_SendData(0X03)");
    
}
// parse inbound frames
void AP_BattMonitor_229::handle_frame(AP_HAL::CANFrame &frame)
{
    const uint16_t canid = frame.id & 0x0000FFFF;
    //TEST BATT
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "handle_frame_cid: %u pretype 0x%2X", canid, (unsigned)packet_sent_prev);
    if (canid == 0) {
        // This is for broadcast and I don't think we should allow this inbound.
        return;
    }
    if(current_buflen == 0x00)
    {
        if(frame.data[0] == Device_Adress && frame.data[1] == (uint8_t)PacketType::STREAM_OUTPUT)
        {
            for(int i = 0; i< 8; i++)
                receive_buffer[current_buflen++] = frame.data[i];
        }
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Add: %u type 0x%2X,len 0x%2X", frame.data[0], frame.data[1],frame.data[2]);     
    }
    else if(current_buflen < RECEIVE_BUF_LEN)
    {
        for(int i = 0; i< 8 && current_buflen < RECEIVE_BUF_LEN; i++)
            receive_buffer[current_buflen++] = frame.data[i];
    }
    
    if(current_buflen < RECEIVE_BUF_LEN)//get all data
        return;

    //check
    uint16_t chknum = Check_CRC16(receive_buffer,RECEIVE_BUF_LEN-2);
    //TEST BATT
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PDCAN: %u,%u,%u", receive_buffer[0],receive_buffer[1],receive_buffer[2]); 
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "readck: %2X,%2X,checknum 0x%4X", receive_buffer[RECEIVE_BUF_LEN-2],receive_buffer[RECEIVE_BUF_LEN-1], chknum); 
    if(chknum != UINT16_VALUE(receive_buffer[RECEIVE_BUF_LEN-1],receive_buffer[RECEIVE_BUF_LEN-2]))
        return;

    WITH_SEMAPHORE(_sem_static);

    switch ((PacketType)receive_buffer[1]) {

    case PacketType::STREAM_INPUT:
        MK229_device.output.voltage = (float)(UINT32_VALUE(receive_buffer[3], receive_buffer[4],receive_buffer[5],receive_buffer[6]))*0.0001;//10000 -> V
        MK229_device.output.current = (float)(UINT32_VALUE(receive_buffer[7], receive_buffer[8],receive_buffer[9],receive_buffer[10]))*0.0001;//10000 -> A
        MK229_device.output.power = (float)(UINT32_VALUE(receive_buffer[11], receive_buffer[12],receive_buffer[13],receive_buffer[14]))*0.0001;//10000 -> W
        break;
    case PacketType::STREAM_OUTPUT:
        MK229_device.output.voltage = (float)(UINT32_VALUE(receive_buffer[3], receive_buffer[4],receive_buffer[5],receive_buffer[6]))*0.0001;//10000 -> V
        //if(receive_buffer[3]>>7 > 0)//negative
        //    MK229_device.output.current = ((float)(UINT16_VALUE(frame.data[2], frame.data[3]))-0xffff-1)*0.01; //10mA -> A
        //else
        MK229_device.output.current = (float)(UINT32_VALUE(receive_buffer[7], receive_buffer[8],receive_buffer[9],receive_buffer[10]))*0.0001;//10000 -> A         
        MK229_device.output.power = (float)(UINT32_VALUE(receive_buffer[11], receive_buffer[12],receive_buffer[13],receive_buffer[14]))*0.0001;//10000 -> W
        //TEST BATT
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "D0:%02X D1:%02X D2:%02X D3:%02X", frame.data[0],frame.data[1],frame.data[2],frame.data[3]); 
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "V: %f cur: %f pow:%f", MK229_device.output.voltage, MK229_device.output.current,MK229_device.output.power); 
        break;

    case PacketType::RELAY_SET: 
    case PacketType::RELAY_GET:
        // nothing to do
        return;
    }
    MK229_device.timestamp_ms = AP_HAL::millis();
}

void AP_BattMonitor_229::send_command(const PacketType type, const float data)
{
    if(current_buflen < RECEIVE_BUF_LEN && MK229_device.is_healthy())
        return;
    //1 second send once.
    if(AP_HAL::millis() - presendcmd_ms < 600)
        return;
    presendcmd_ms = AP_HAL::millis(); 

    current_buflen = 0; //send requirement and get new data

    AP_HAL::CANFrame txFrame;

    switch (type) {
    case PacketType::STREAM_INPUT://write register
        break;
    case PacketType::STREAM_OUTPUT://read start adress:0100  20registers
        txFrame.dlc = 8;
        txFrame.data[0] = Device_Adress; //device adress
        txFrame.data[1] = (uint8_t)type; //Function code
        txFrame.data[2] = 0x01; //start adress H
        txFrame.data[3] = 0x00; //start adress L
        txFrame.data[4] = 0x00; //data len H
        txFrame.data[5] = 0x14; //data len L
        txFrame.data[6] = 0x44; //Checksum L
        txFrame.data[7] = 0x39; //Checksum H
        break;
    case PacketType::RELAY_SET: //write 1 relay
        break;
    case PacketType::RELAY_GET: //read 1 relay
        break;
    }

    txFrame.id = (uint32_t)1;

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
const char* AP_BattMonitor_229::get_fault_code_string(const FaultFlags fault) const
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
bool AP_BattMonitor_229::get_voltage_and_current_and_temp( float &voltage, float &current, float &temperature) const
{

    voltage = 0.0f;
    current = 0.0f;
    temperature = 0.0f;

    if (!MK229_device.is_healthy()) {
        return false;
    }
            
    voltage = MK229_device.output.voltage;
    current = MK229_device.output.current;
    temperature = (float)MK229_device.temperature;
    return true;

}
//CRC16 check
#define         CRC_16_POLYNOMIALS  0xa001    
// X^16 +X^15 +X^2+1
uint16_t AP_BattMonitor_229::Check_CRC16(uint8_t* pchMsg,uint8_t wDataLen)
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
