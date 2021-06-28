/*
 * AP_RangeFinder_SR73F.cpp
 *
 *  Created on: Aug 10, 2020
 *      Author: coco
 */
#include "RangeFinder.h"
#include "AP_RangeFinder_SR73F.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <uavcan/driver/can.hpp>
#include <uavcan/time.hpp>
#include <AP_HAL_ChibiOS/CAN.h>
#include <GCS_MAVLink/GCS.h>   
#include <uavcan/time.hpp>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
extern const AP_HAL::HAL& hal;


uavcan::ICanDriver* sr73_can_driver;
uint8_t sr73_driver_index; //驱动号
uint32_t sr73_driver_rate; //波特率
/****************************************************************************************
*函数原型：void AP_RangeFinder_SR73F::init()
*函数功能：can设备外设初始化
*修改日期：2020-8-1
*修改作者：zuav
*备注信息：
***************************************************************************************/
AP_RangeFinder_SR73F::AP_RangeFinder_SR73F(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params)
    : AP_RangeFinder_Backend(_state, _params)
{
}
/****************************************************************************************
*函数原型：bool AP_RangeFinder_SR73F::detect(RangeFinder::RangeFinder_State &_state)
*函数功能：识别设备
*修改日期：2020-8-11
*修改作者：zuav
*备注信息：
***************************************************************************************/
bool AP_RangeFinder_SR73F::detect(RangeFinder::RangeFinder_State &_state,AP_RangeFinder_Params &_params)
{
	AP_RangeFinder_SR73F *sensor= new AP_RangeFinder_SR73F(_state,_params);
	//进行设备初始化
    sensor->init(_params.can_driver,_params.rate);
    return sensor;
}

/****************************************************************************************
*函数原型：void AP_RangeFinder_SR73F::init()
*函数功能：can设备外设初始化
*修改日期：2020-8-1
*修改作者：zuav
*备注信息：
***************************************************************************************/
bool AP_RangeFinder_SR73F::init(uint8_t driver_index,uint32_t rate)
{
	//获取驱动号
	sr73_driver_index = driver_index;
	//获取波特率
	sr73_driver_rate=rate;
    hal.console->printf("sr73_driver_index=%d\r\n",sr73_driver_index);
	hal.console->printf("sr73_driver_rate=%d\r\n",sr73_driver_rate);
    hal.console->printf("SR73F: starting init\n\r");
    if (_initialized)
    {
    	hal.console->printf("SR73F： already initialized\r");
        return false;
    }
    const_cast <AP_HAL::HAL&> (hal).can_mgr[sr73_driver_index ] = new ChibiOS::CANManager;
    //初始化对应的端口号和波特率
    hal.can_mgr[sr73_driver_index]->begin(sr73_driver_index,sr73_driver_rate);
    hal.console->printf("can_mgr=%d\r\n",hal.can_mgr[sr73_driver_index]);
    if (hal.can_mgr[sr73_driver_index ]  == nullptr)
    {
    	hal.console->printf("SR73F: no mgr for this driver\n\r");
        return false;
    }
    hal.can_mgr[sr73_driver_index]->initialized(true);
    if (!hal.can_mgr[sr73_driver_index ]->is_initialized())
    {
    	hal.console->printf("SR73F: mgr not initialized\n\r");
        return false;
    }
    //存储设备
    sr73_can_driver = hal.can_mgr[sr73_driver_index ]->get_driver();
    hal.console->printf("_can_driver2=%d\r\n",sr73_can_driver);
    if (sr73_can_driver == nullptr)
    {
    	hal.console->printf("SR73F: no CAN driver\n\r");
        return false;
    }


    _initialized = true;

	hal.console->printf("SR73F Init FINISH \r\n");
    return true;
}

/*********************************************************************************
*函数原型：bool AP_RangeFinder_SR73F::get_reading(uint16_t &reading_cm)
*函数功能：can总线数据更新
*修改日期：2020-8-3
*修改作者：zuav
*备注信息：
**********************************************************************************/
bool AP_RangeFinder_SR73F::get_reading(uint16_t &reading_cm, float &target_deg)
{

	float data=0;
	//定义can接收
	uavcan::CanRxFrame can_rx;
	uavcan::CanIOFlags out_flags=0;
	hal.can_mgr[0]->_driver[0].getIface(0)->receive(can_rx,can_rx.ts_mono,can_rx.ts_utc,out_flags);
	//根据收到的ID进行判断
			hal.console->printf("id=%lu\r\n",can_rx.id);
//			hal.console->printf("data[0]=%d\r\n",can_rx.data[0]);
//			hal.console->printf("data[1]=%d\r\n",can_rx.data[1]);
//			hal.console->printf("data[2]=%d\r\n",can_rx.data[2]);
//			return 1;

//    gcs().send_text(MAV_SEVERITY_INFO, "can_rx ID : %d ", can_rx.id);
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

/*********************************************************************************
*函数原型：void AP_RangeFinder_SR73F::update(void)
*函数功能：定时器更新
*修改日期：2020-8-3
*修改作者：zuav
*备注信息：
**********************************************************************************/
void AP_RangeFinder_SR73F::update(void)
{
	//读取数据
    if (get_reading(state.distance_cm, state.target_deg))
    {
        //更新测量数据
		state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else
    {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}





