/*
 * AP_SR73F_CAN.cpp
 *
 *  Created on: Jul 6, 2021
 *      Author: coco
 */
/****************************************************************************************
*文件原型：正方电池协议驱动
*函数功能：解析正方电池D类数据（位速率是5K，数据更新2Hz）,采样速率需要足够快
*修改日期：2020-12-29
*修改作者：ymuav
*备注信息：
***************************************************************************************/
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL_ChibiOS/CAN.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_SR73F_CAN.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
extern const AP_HAL::HAL& hal;


static const uint8_t CAN_IFACE_INDEX = 0;
//距离和目标角度
uint16_t sr73f_reading_cm=0;
float sr73f_target_deg=0;
//定义使能参数
bool sr73f_senseor_driver_enable=0;












/****************************************************************************************
*函数原型：void AP_BattMonitor_ZhengFang_UavCan::init()
*函数功能：can设备外设初始化
*修改日期：2020-8-1
*修改作者：zuav
*备注信息：
***************************************************************************************/
AP_SR73F_CAN::AP_SR73F_CAN()
{
  //不执行任何操作
}


/****************************************************************************************
*函数原型：void AP_BatCAN::init(void)
*函数功能：can设备外设初始化
*修改日期：2020-8-1
*修改作者：zuav
*备注信息：
***************************************************************************************/
void AP_SR73F_CAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    hal.console->printf( "AP_SR73F_CAN: starting init\r\n");

    if (_initialized)
    {
    	hal.console->printf( "AP_SR73F_CAN: already initialized\r\n");
        return;
    }

    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr)
    {
    	hal.console->printf( "AP_SR73F_CAN:  no mgr for this driver\r\n");
    	sr73f_senseor_driver_enable=0;
        return;
    }

    if (!can_mgr->is_initialized())
    {
    	hal.console->printf( "AP_SR73F_CAN:  mgr not initialized \r\n");
    	sr73f_senseor_driver_enable=0;
        return;
    }

    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr)
    {
    	hal.console->printf( "AP_SR73F_CAN:  no CAN driver\r\n");
    	sr73f_senseor_driver_enable=0;
        return;
    }

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_SR73F_CAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 1)) {

    	hal.console->printf( "AP_SR73F_CAN:  couldn't create thread\r\n");
    	sr73f_senseor_driver_enable=0;
        return;
    }

    _initialized = true;
    sr73f_senseor_driver_enable=1;
	hal.console->printf( "AP_SR73F_CAN: init done\r\n");

	uavcan::CanRxFrame can_tx1;
	can_tx1.id= 0x606;
	can_tx1.data[0]=0xC1;
	can_tx1.data[1]=0x00;
	can_tx1.data[2]=0x23;
	can_tx1.data[3]=0x28;
	can_tx1.data[4]=0x00;
	can_tx1.dlc=0x05;
	_can_driver->getIface(0)->send(can_tx1, can_tx1.ts_mono, 1);

    return;
}

/*********************************************************************************
*函数原型：AP_SR73F_CAN *AP_SR73F_CAN::get_tcan(uint8_t driver_index)
*函数功能：can总线数据更新
*修改日期：2021-7-37
*修改作者：ymuav
*备注信息：
*备注信息：
*备注信息：
**********************************************************************************/
AP_SR73F_CAN *AP_SR73F_CAN::get_tcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_SR73CAN)
    {
        return nullptr;
    }
    return static_cast<AP_SR73F_CAN*>(AP::can().get_driver(driver_index));
}
/*********************************************************************************
*函数原型：void AP_SR73F_CAN::loop()
*函数功能：can总线数据更新
*修改日期：2021-7-37
*修改作者：ymuav
*备注信息：
*备注信息：
*备注信息：
**********************************************************************************/
void AP_SR73F_CAN::loop()
{
	bool valid_data=0;
	float data=0;
	 //输出标记
	uavcan::CanIOFlags out_flags=0;
	//定义CAN接收数据
	uavcan::CanRxFrame can_rx;

	hal.console->printf( "KKKKK\r\n");
    while (true)
    {
        if (!_initialized)
        {
     	    hal.console->printf( "AP_SR73F_CAN not initialized\\r\n");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }
        valid_data= _can_driver->getIface(CAN_IFACE_INDEX)->receive(can_rx,can_rx.ts_mono,can_rx.ts_utc,out_flags);
        hal.console->printf( "valid_data ID=%d \r\n", valid_data);
        hal.console->printf( "can_rx ID=%ld \r\n", can_rx.id);
        if(valid_data!=0)
        {

       	   hal.console->printf( "can_rx ID=%d \r\n", can_rx.id);

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
       	 	        hal.scheduler->delay_microseconds(500);
       	 	    	break;
       	 	}
       	 	if(
			//		get_object_data.target_distance * 100 < 20 || get_object_data.target_distance * 100 > 600
       	 	//   || 
				   can_rx.id != OBJECT_INFORMATION)
       	 	{

       	 	  sr73f_reading_cm=0;
       	 	  sr73f_target_deg=0;

       	 	}
       	    sr73f_reading_cm = get_object_data.target_distance * 100;
       	    sr73f_target_deg = get_object_data.target_deg*57.29;

        }
        else
        {
        	 hal.scheduler->delay_microseconds(500);
        }
    }

}





#endif
















