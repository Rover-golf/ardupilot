#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_USD1_CAN.h"

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

extern const AP_HAL::HAL& hal;
/*
  constructor
 */
AP_RangeFinder_USD1_CAN::AP_RangeFinder_USD1_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    CANSensor("USD1"),
    AP_RangeFinder_Backend(_state, _params)
{
   _Address = _params.address;// Radar ID (0-7)
    new_information = false;
    new_data = false;
    register_driver(AP_CANManager::Driver_Type_USD1);
}

// update state
void AP_RangeFinder_USD1_CAN::update(void)
{
   WITH_SEMAPHORE(_sem);
    if ((AP_HAL::millis() - _last_reading_ms) > 500) {
        // if data is older than 500ms, report NoData
        set_status(RangeFinder::Status::NoData);
    } else if (new_data) {
        state.distance_cm = _distance_cm;
        state.last_reading_ms = _last_reading_ms;
        update_status();
        new_data = false;
    }
}

// handler for incoming frames
void AP_RangeFinder_USD1_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    
    if(frame.id <= 0)
	{
		return;
	}
    switch(_Address)
    {
        case 0:
            _bRt = CH30_Analysis(frame,ftarget_distance, ftarget_deg); 
            break;
        case 1:
            _bRt = SR73_Analysis(frame,ftarget_distance, ftarget_deg);
            break;
        default:
            _bRt = false;
            break;
    }
      
    if(_bRt)
    {
        _distance_cm = (uint16_t)(ftarget_distance * 100);
        _last_reading_ms = AP_HAL::millis();
        new_data = true;          
     //   hal.console->printf("handle_frame: FrmId:%d, distance=%d, angle=%d.\n",(int)(frame.id),(int)(_distance_cm),(int)ftarget_deg);
    }
    else
    {
     //   hal.console->printf("handle_frame: FrmId:%d.\n",(int)(frame.id));
    }
}

void AP_RangeFinder_USD1_CAN::init(uint8_t driver_index, bool enable_filters)
{
    CANSensor::init( driver_index,  enable_filters);
    //set radar address to 1 
    //_bRt = SetRadarAddress(1);
    if(_Address == 0)//CH30 startup
        CH30_Startup();
}

bool AP_RangeFinder_USD1_CAN::CH30_Startup()
{
    AP_HAL::CANFrame can_tx1, can_rx1;

	can_tx1.id= 0x606;      // CH30 startup command
	can_tx1.data[0]=0xc1;
	can_tx1.data[1]=0x00;
	can_tx1.data[2]=0xFF;   // ROI 254 cm  (half 127 cm))
	can_tx1.data[3]=0x28;   // 0x23;   // deep 350 cm
	can_tx1.data[4]=0x00;
	can_tx1.dlc=0x05;

    _bRt = false;
    uint32_t CH30_init_start = AP_HAL::millis();
    while(!_bRt && AP_HAL::millis() - CH30_init_start < 20000)
    {
        _bRt = write_frame(can_tx1,500);
        if(_bRt)
        {
            hal.scheduler->delay(50); 
            _bRt = read_frame(can_rx1, 500);
        }
        if(!_bRt)
            hal.scheduler->delay(500);
    }

    return _bRt;
}

bool AP_RangeFinder_USD1_CAN::CH30_Analysis(AP_HAL::CANFrame &frame,float& fdist, float& fdeg)
{
    switch(frame.id)
    {
        case CH30_HEART_BEAT: //
            break;
		case CH30_DATA:   // Benewake CH30 data
        {
            fdist = (((frame.data[1]&(0x0f))<<8) | (frame.data[0]&(0xff)));
            fdeg = (float)(int8_t)frame.data[3];
			fdist =fdist / 100.0;
			//hal.console->printf("CH30_Analysis: distance=%f, deg=%f\n",fdist,fdeg);
            //check in region
            //_bRt = ObjectInRegion( fdist, fdeg);
            //if(_bRt)
                return true; 
            //else
            //    return false;
        }
			break;
	    default:
            break;
    }

    return false;
}

bool AP_RangeFinder_USD1_CAN::SR73_Analysis(AP_HAL::CANFrame &frame,float& fdist, float& fdeg)
{
    if(frame.id == OBJECT_STATUS + _Address * 0x10)
    {
        new_information = true;
    }
    else if( frame.id == OBJECT_INFORMATION + _Address * 0x10 && new_information)
    {
        
        //Object ID
        objectsinformation.Id = frame.data[0];
        //V dist H
        objectsinformation.DistLongH = frame.data[1];
        //H dist H
        objectsinformation.DistLatH = frame.data[2]&(0x07);
        //V dist L
        objectsinformation.DistLongL = frame.data[2]>>3;
        //H dist L
        objectsinformation.DistLatL = frame.data[3];
        //V speed H
        objectsinformation.VrelLongH = frame.data[4];
        //H spped H
        objectsinformation.VrelLatH = frame.data[5]&(0x3F);
        //V speed L
        objectsinformation.VrelLongL = frame.data[5]>>6;
        //
        objectsinformation.DynProp = frame.data[6]&(0x07);
        //
        objectsinformation.undefined = (frame.data[6]&(0x18))>>3;
        //H spped L
        objectsinformation.VrelLatL = frame.data[6]>>5;
        //
        objectsinformation.RCS = frame.data[7];
        //-----------------gain data -----------------------/
        //object Id
        get_object_data.Objects_ID=objectsinformation.Id;
        //V dist unit:m
        get_object_data.Objects_DistLong = ((objectsinformation.DistLongH<<5)|objectsinformation.DistLongL)*0.2-500;
        //H dist unit:m
        get_object_data.Objects_DistLat = ((objectsinformation.DistLatH<<8)|objectsinformation.DistLatL)*0.2-204.6;
        //V speed
        get_object_data.Objects_VrelLong = ((objectsinformation.VrelLongH<<2)|objectsinformation.VrelLongL)*0.25-128;
        //
        get_object_data.Objects_DynProp = objectsinformation.DynProp;
        //H speed
        get_object_data.Objects_VrelLat = ((objectsinformation.VrelLatH<<3)|objectsinformation.VrelLatL)*0.25-64;
        //
        get_object_data.Objects_RCS = objectsinformation.RCS*0.25-64;
        //ratio H/V
        fdata = get_object_data.Objects_DistLat / get_object_data.Objects_DistLong;
        //dist
        get_object_data.target_distance = sqrtf(get_object_data.Objects_DistLong*get_object_data.Objects_DistLong
                +get_object_data.Objects_DistLat*get_object_data.Objects_DistLat);
        //angle
        get_object_data.target_deg = atanf(fdata);
        //speed
        get_object_data.target_speed = get_object_data.Objects_VrelLong*(cosf(get_object_data.target_deg))+
                get_object_data.Objects_VrelLat*(sinf(get_object_data.target_deg));
                
        fdist = get_object_data.target_distance;
        fdeg = get_object_data.target_deg * 57.29;
        //check in region
        _bRt = ObjectInRegion( fdist, fdeg);
        if(_bRt)
        {
            new_information = false;
            return true; 
        }
        else
            return false;
    }

    return false;
}

bool AP_RangeFinder_USD1_CAN::SetRadarAddress(uint8_t iaddress)
{
    AP_HAL::CANFrame frame;
    //SR73 set address: 1
	frame.id= 0x200;  // setup parameter
	frame.data[0]=0x82;//NVM valid, sensorID valid
	frame.data[1]=0x00;
	frame.data[2]=0x00;
	frame.data[3]=0x00;
	frame.data[4]=iaddress; //sensorID
    frame.data[5]=0x80; //StoreNVM
    frame.data[6]=0x00;
    frame.data[7]=0x00;
	frame.dlc=0x08;
    //SR73 set collision detection region configuration(0x401)
	frame.id= 0x401;   // setup region
	frame.data[0]=0xFF;//Coordinates valid, Activation valid num:63
	frame.data[1]=0x01;//RegionID 
	frame.data[2]=0x4E;
	frame.data[3]=0x24;
	frame.data[4]=0x02;
    frame.data[5]=0x4E;
    frame.data[6]=0x73;
    frame.data[7]=0xFC;
	frame.dlc=0x08;
    _bRt = write_frame(frame,20000);
    int irt = (int) _bRt;
    hal.console->printf("SetRadarAddress rt:%d\n",irt);
    return _bRt;
}

bool AP_RangeFinder_USD1_CAN::ObjectInRegion(float fdist, float fdeg,float long1,float lat1,float long2,float lat2)
{
    float objlong,objlat;
    objlong = fdist * cosf(fdeg / 57.29f);
    objlat = fdist * sinf(fdeg / 57.29f);
    if(objlong>= long1 && objlong <= long2 && objlat >= lat2 && objlat<= lat1)
    {
     //   hal.console->printf("ObjectInRegion obj fdist=(%f) fdeg=%f in region.\n",fdist,fdeg);
        return true;
    }
    //hal.console->printf("ObjectInRegion obj fdist=(%f) fdeg=%f out region.\n",fdist,fdeg);
    return false;
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
