/****************************************************************************************
*文件原型：恩曌 -CAN类型的传感器驱动
*函数功能：
*修改日期：2021-1-28
*修改作者：ymuav
*备注信息：
***************************************************************************************/

#include "AP_RangeFinder_SR73F.h"
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL_ChibiOS/CAN.h>
#include <uavcan/driver/can.hpp>
#include <uavcan/time.hpp>
#include "AP_RangeFinder_Params.h"


//定义结构体,获取目标状态数据
AP_RangeFinder_SR73F::ObjectsStatusBits objectstatus_data;
//定义目标信息位
AP_RangeFinder_SR73F::ObjectsInforBits objectsinformation;
AP_RangeFinder_SR73F::Object_dataBytes get_object_data;
uavcan::ICanDriver* sr73f_can_driver;

uint8_t sr73_driver_index; //驱动号
uint32_t sr73_driver_rate; //波特率
extern const AP_HAL::HAL& hal;

/****************************************************************************************
*函数原型：AP_RangeFinder_TR24DA100_CAN::AP_RangeFinder_TR24DA100_CAN()
*函数功能：can设备外设初始化
*修改日期：2020-8-1
*修改作者：zuav
*备注信息：
***************************************************************************************/
/**********************************************************************************************
*函数原型：AP_RangeFinder_TR24DA100_CAN *AP_RangeFinder_TR24DA100_CAN::get_tcan(uint8_t driver_index)
*函数功能：获取设备驱动
*修改日期：2020-8-1
*修改作者：zuav
*备注信息：
*********************************************************************************************/
AP_RangeFinder_SR73F::AP_RangeFinder_SR73F(RangeFinder::RangeFinder_State &_state,
		                                   AP_RangeFinder_Params &_params): AP_RangeFinder_Backend(_state, _params)
{
}

/*****************************************************************************************************
 *函数原型：void AP_RangeFinder_TR24DA100_CAN::init(uint8_t driver_index, bool enable_filters)
 *函数功能：读取近距离传感器数据
 *修改日期：2019-2-18
 *修改作者：cihang_uav
 *备注信息：return rangefinder altitude in centimeters
 ******************************************************************************************************/
bool AP_RangeFinder_SR73F::init(uint32_t rate,uint8_t driver_index)
{
	sr73_driver_index= driver_index;
	sr73_driver_rate=rate;
    hal.console->printf("sr73f_driver_index=%d\r\n",sr73_driver_index);
    hal.console->printf("SR73_CAN: starting init\n\r");
    if (_initialized)
    {
    	hal.console->printf("SR73_CAN： already initialized\r");
        return false;
    }
    const_cast <AP_HAL::HAL&> (hal).can_mgr[sr73_driver_index ] = new ChibiOS::CANManager;
    hal.can_mgr[sr73_driver_index]->begin(sr73_driver_rate,sr73_driver_index);
    hal.console->printf("can_mgr=%d\r\n",hal.can_mgr[sr73_driver_index]);
    if (hal.can_mgr[sr73_driver_index ]  == nullptr)
    {
    	hal.console->printf("SR73_CAN: no mgr for this driver\n\r");
        return false;
    }
    hal.can_mgr[sr73_driver_index]->initialized(true);
    if (!hal.can_mgr[sr73_driver_index ]->is_initialized())
    {
    	hal.console->printf("SR73_CAN: mgr not initialized\n\r");
        return false;
    }
    //存储设备
    sr73f_can_driver = hal.can_mgr[sr73_driver_index ]->get_driver();
    hal.console->printf("_can_driver2=%d\r\n",sr73f_can_driver);
    if (sr73f_can_driver == nullptr)
    {
    	hal.console->printf("SR73_CAN: no CAN driver\n\r");
        return false;
    }


    _initialized = true;

	hal.console->printf("SR73_CAN Init FINISH \r\n");
    return true;

}

/*****************************************************************************************************
 *函数原型：bool AP_RangeFinder_TR24DA100_CAN::detect(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        uint8_t serial_instance )
 *函数功能：读取近距离传感器数据
 *修改日期：2019-2-18
 *修改作者：cihang_uav
 *备注信息：return rangefinder altitude in centimeters
 ******************************************************************************************************/
bool AP_RangeFinder_SR73F::detect(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
{
	AP_RangeFinder_SR73F *sensor= new AP_RangeFinder_SR73F(_state,_params);

    if(sensor->init(_params.rate,_params.can_driver)) //初始化对应的串口
    {
    	 hal.console->printf("RNGFND SR73F Init Finish\r\n");
    	 return true;
    }
    else
    {
    	 hal.console->printf("RNGFND SR73F Init Fail \r\n");
    	return false;
    }
}


/******************************************************************************************************
 *函数原型：uint16_t AP_RangeFinder_TR24DA100_CAN::get_reading(uint16_t &reading_cm)
 *函数功能：读取传感器数据
 *修改日期：2021-1-22
 *修改作者：ymuav
 *备注信息：
 ******************************************************************************************************/
bool AP_RangeFinder_SR73F::get_reading(uint16_t &reading_cm, float &target_deg)
{
	float data=0;
	 //定义CAN接收数据
	 uavcan::CanRxFrame can_rx;
	 //输出标记
	 uavcan::CanIOFlags out_flags=0;
     //读取数据,这里暂时只能使用CAN1

	 sr73f_can_driver->getIface(0)->receive(can_rx,can_rx.ts_mono,can_rx.ts_utc,out_flags);

	 gcs().send_text(MAV_SEVERITY_INFO, "can_rx ID : %d ", can_rx.id);
	 	switch(can_rx.id)
	 	{
	 	    //如果是目标状态信息
	 	    case OBJECT_STATUS:
	 			//获取目标个数
	 			objectstatus_data.Objects_NofObjects=can_rx.data[0];
	 			//循环计数高8位
	 			objectstatus_data.Objects_MeasCountH=can_rx.data[1];
	 			//循环计数低8位
	 			objectstatus_data.Objects_MeasCountL=can_rx.data[2];
	 			//只要高4位,接口版本
	 			objectstatus_data.Objects_InterfaceVersion=(can_rx.data[3]>>4);
	 			//循环计数数据=循环计数高8位|循环计数低8位
	 			objectstatus_data.Objects_MeasCount=(objectstatus_data.Objects_MeasCountH<<8)|objectstatus_data.Objects_MeasCountL;
	 			hal.console->printf("num=%d\r\n",objectstatus_data.Objects_NofObjects);
	 			hal.console->printf("count=%d\r\n",objectstatus_data.Objects_MeasCount);
	 			hal.console->printf("version=%d\r\n",objectstatus_data.Objects_InterfaceVersion);
	 	    	break;
	 	    case OBJECT_INFORMATION:
	 			//雷达ID
	 			objectsinformation.Id=can_rx.data[0];
	 			//目标纵向距离高8位
	 			objectsinformation.DistLongH=can_rx.data[1];

	 			//目标横向距离高3位
	 			objectsinformation.DistLatH=can_rx.data[2]&(0x07);
	 			//目标纵向距离低5位
	 			objectsinformation.DistLongL=can_rx.data[2]>>3;
	 			//目标横向距离低8位
	 			objectsinformation.DistLatL=can_rx.data[3];
	 			//目标纵向速度高8位
	 			objectsinformation.VrelLongH=can_rx.data[4];
	 			//目标横向速度高6位
	 			objectsinformation.VrelLatH=can_rx.data[5]&(0x3F);
	 			//目标纵向速度低2位
	 			objectsinformation.VrelLongL=can_rx.data[5]>>6;

	 			//目标的运动属性，默认是0
	 			objectsinformation.DynProp=can_rx.data[6]&(0x07);
	 			//目标未定义位
	 			objectsinformation.undefined=(can_rx.data[6]&(0x18))>>3;
	 			//目标横向速度低3位
	 			objectsinformation.VrelLatL=can_rx.data[6]>>5;

	 			//目标横向速度低3位
	 			objectsinformation.RCS=can_rx.data[7];

	 			/******获取需要的数据*************************************/
	 			//目标 ID 8位
	 			get_object_data.Objects_ID=objectsinformation.Id;
	 			//目标纵向距离13位,单位是m
	 			get_object_data.Objects_DistLong=((objectsinformation.DistLongH<<5)|objectsinformation.DistLongL)*0.2-500;
	 			//目标横向距离11位,单位是m
	 			get_object_data.Objects_DistLat=((objectsinformation.DistLatH<<8)|objectsinformation.DistLatL)*0.2-204.6;
	 			//目标纵向速度10位,
	 			get_object_data.Objects_VrelLong=((objectsinformation.VrelLongH<<2)|objectsinformation.VrelLongL)*0.25-128;
	 			//目标动态属性， 暂不支持分类， 现默认值均为 0
	 			get_object_data.Objects_DynProp=objectsinformation.DynProp;
	 			//目标横向速度9位
	 			get_object_data.Objects_VrelLat=((objectsinformation.VrelLatH<<3)|objectsinformation.VrelLatL)*0.25-64;
	 			//目标RCS默认是0
	 			get_object_data.Objects_RCS=objectsinformation.RCS*0.25-64;

	 			data=(float)get_object_data.Objects_DistLat/(float)get_object_data.Objects_DistLong;
	 			//获取实际的距离
	 			get_object_data.target_distance=sqrtf(get_object_data.Objects_DistLong*get_object_data.Objects_DistLong
	 					+get_object_data.Objects_DistLat*get_object_data.Objects_DistLat);
	 			//获取实际的速度
	 				get_object_data.target_deg=atanf(data);
	 			//获取实际的速度
	 			get_object_data.target_speed=get_object_data.Objects_VrelLong*(cosf(get_object_data.target_deg))+
	 					get_object_data.Objects_VrelLat*(sinf(get_object_data.target_deg));

	 			hal.console->printf("data=%f\r\n",data);
	 			hal.console->printf("myid=%f\r\n",get_object_data.Objects_ID);
	 			hal.console->printf("distance=%f\r\n",get_object_data.target_distance);
	 			hal.console->printf("deg=%f\r\n",get_object_data.target_deg*57.29);
	 			hal.console->printf("speed=%f\r\n",get_object_data.target_speed);
	 			hal.console->printf("rcs=%f\r\n",get_object_data.Objects_RCS);
	 			break;
	 	    default:
	 	    	break;
	 	}
	 	if(get_object_data.target_distance * 100 < 20 || get_object_data.target_distance * 100 > 600
	 	   || can_rx.id != OBJECT_INFORMATION)
	 	//if(can_rx.id == OBJECT_STATUS)
	 	{
	     	reading_cm = 0;
	 		target_deg = 0.0;
	 	    return false;
	 	}
	     reading_cm = get_object_data.target_distance * 100;
	 	target_deg = get_object_data.target_deg*57.29;
	     return true;





}



/****************************************************************************************
 *函数原型：void AP_RangeFinder_NRA24::update(void)
 *函数功能：更新传感器数据
 *修改日期：2019-2-18
 *修改作者：
 *备注信息：update the state of the sensor
 ***************************************************************************************/
void AP_RangeFinder_SR73F::update(void)
{
	  if (get_reading(state.distance_cm, state.target_deg))
	{
		hal.console->printf("alt5=%d\r\n",state.distance_cm);
		// update range_valid state based on distance measured
		//基于测量距离的更新范围有效状态
		last_reading_ms = AP_HAL::millis();
		//更新状态
		update_status();
	} else if (AP_HAL::millis() - last_reading_ms > 2000) //如果2s还没数据，提示没数据
	{
		set_status(RangeFinder::RangeFinder_NoData);
	}
}




