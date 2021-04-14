
/*
 *   特殊的板层配置为CAN接口
 *   AP_BoardConfig_CAN - board specific configuration for CAN interface
 */
#include "AP_SR73F_CAN.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <uavcan/driver/can.hpp>
#include <uavcan/time.hpp>
#include <AP_HAL_ChibiOS/CAN.h>
extern const AP_HAL::HAL& hal;



AP_SR73F_CAN::AP_SR73F_CAN()
{
}

AP_SR73F_CAN::~AP_SR73F_CAN()
{
}


/**************************************************************************************************************
*函数原型：void AP_SR73F_CAN::init()
*函数功能：can设备外设初始化
*修改日期：2020-8-1
*修改作者：zuav
*备注信息：
****************************************************************************************************************/
void AP_SR73F_CAN::init()
{
	//定义设备对象
    uint8_t inum = 0;
    //初始化can_mgr对象
    const_cast <AP_HAL::HAL&> (hal).can_mgr[inum] = new ChibiOS::CANManager();
	//初始化can的波特率
	hal.can_mgr[0]->begin(500000,0);
}

/**************************************************************************************************************
*函数原型：void AP_SR73F_CAN::update()
*函数功能：can总线数据更新
*修改日期：2020-8-3
*修改作者：zuav
*备注信息：
****************************************************************************************************************/
void AP_SR73F_CAN::update()
{

	float data=0;
	//定义can接收
	uavcan::CanRxFrame can_rx;
	uavcan::CanIOFlags out_flags=0;
	hal.can_mgr[0]->_driver[0].getIface(0)->receive(can_rx,can_rx.ts_mono,can_rx.ts_utc,out_flags);
	if(can_rx.id<=0)
	{
		return;
	}
	//根据收到的ID进行判断
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


}






