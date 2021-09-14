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

extern const AP_HAL::HAL& hal;


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
    sensor->init();
    return sensor;
}

/****************************************************************************************
*函数原型：void AP_RangeFinder_SR73F::init()
*函数功能：can设备外设初始化
*修改日期：2020-8-1
*修改作者：zuav
*备注信息：
***************************************************************************************/
bool AP_RangeFinder_SR73F::init(void)
{
	//定义设备对象
    uint8_t inum = 0;
    //初始化can_mgr对象
    const_cast <AP_HAL::HAL&> (hal).can_mgr[inum] = new ChibiOS::CANManager();
	//初始化can的波特率
	hal.can_mgr[0]->begin(500000,0);
    hal.console->printf("SR73 init250 \r\n");

    // benewake CH30 init     Josh Sept.2, 2021
	uavcan::CanRxFrame can_tx1;


	can_tx1.id= 0x606;  // CH30 startup command
	can_tx1.data[0]=0xc1;
	can_tx1.data[1]=0x00;
	can_tx1.data[2]=0xFF;   // ROI 254 cm  (half 127 cm))
	can_tx1.data[3]=0x28; //    0x23;   // deep 350 cm
	can_tx1.data[4]=0x00;
	can_tx1.dlc=0x05;
    
	uavcan::CanRxFrame can_rx;
	uavcan::CanIOFlags out_flags=0;
	uint32_t CH30_init_start = AP_HAL::millis();
    while(can_rx.id != CH30_DATA && (AP_HAL::millis() - CH30_init_start < 20000))
	{
		hal.can_mgr[0]->_driver[0].getIface(0)->send(can_tx1, can_tx1.ts_mono, 1);
		hal.can_mgr[0]->_driver[0].getIface(0)->receive(can_rx,can_rx.ts_mono,can_rx.ts_utc,out_flags);
		hal.console->printf("activate CH30 id=%lu\r\n",can_rx.id);

	}
	/*
	can_tx1.id= 0x607;  // CH30 set baud rate command
	can_tx1.data[0]=0x86;
	can_tx1.data[1]=0x75;
	can_tx1.data[2]=0x58;   
	can_tx1.data[3]=0x06;   
	can_tx1.data[4]=0x76;
	can_tx1.data[5]=0x60;
	can_tx1.data[6]=0xF4;
	can_tx1.data[7]=0x01;
	can_tx1.dlc=0x08;

	CH30_init_start = AP_HAL::millis();
    while(AP_HAL::millis() - CH30_init_start < 10000)
	{
		hal.can_mgr[0]->_driver[0].getIface(0)->send(can_tx1, can_tx1.ts_mono, 1);
		hal.console->printf("change buad rate to 500K CH30\r\n");

	}

	hal.can_mgr[0]->begin(500000,0);
*/
    // end  Josh Sept. 2, 2021

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
//			hal.console->printf("data[5]=%d\r\n",can_rx.data[5]);
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
//			hal.console->printf("num=%d\r\n",objectstatus_data.Objects_NofObjects);
//			hal.console->printf("count=%d\r\n",objectstatus_data.Objects_MeasCount);
//			hal.console->printf("version=%d\r\n",objectstatus_data.Objects_InterfaceVersion);
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

/*
			hal.console->printf("data=%f\r\n",data);
			hal.console->printf("myid=%f\r\n",get_object_data.Objects_ID);
			hal.console->printf("distance=%f\r\n",get_object_data.target_distance);
			hal.console->printf("deg=%f\r\n",get_object_data.target_deg*57.29);
			hal.console->printf("speed=%f\r\n",get_object_data.target_speed);
			hal.console->printf("rcs=%f\r\n",get_object_data.Objects_RCS);
*/
			break;

	    case CH30_HEART_BEAT:    // Benewake CH30  heart beat
/*
	        hal.console->printf("is_run=%d\r\n", (can_rx.data[0] & 0x01));
	        hal.console->printf("is_error=%d\r\n", (can_rx.data[0] & 0x04) >> 2);
	        hal.console->printf("value=%d\r\n", (can_rx.data[0] & 0xE0) >> 5);
	        hal.console->printf("centre=%d\r\n", (can_rx.data[0] & 0x10) >> 4);
	        hal.console->printf("version_valid=%d\r\n", (can_rx.data[0] & 0x02) >> 1);
*/
		    break;
		case CH30_DATA:   // Benewake CH30 data
/*
	        hal.console->printf("data0=%d\r\n", can_rx.data[0]);
	        hal.console->printf("data1=%d\r\n", can_rx.data[1]);
	        hal.console->printf("data2=%d\r\n", can_rx.data[2]);

	        hal.console->printf("data3=%x\r\n", can_rx.data[3]);
	        hal.console->printf("data3=%f\r\n", (float)(int8_t)can_rx.data[3]);

	        hal.console->printf("data4=%d\r\n", can_rx.data[4]);
	        hal.console->printf("data5=%d\r\n", can_rx.data[5]);
	        hal.console->printf("data6=%d\r\n", can_rx.data[6]);
	        hal.console->printf("data7=%d\r\n", can_rx.data[7]);
	        hal.console->printf("data-long=%d\r\n", can_rx.dlc);
  */
            get_object_data.target_distance = (((can_rx.data[1]&(0x0f))<<8) | (can_rx.data[0]&(0xff)));
            get_object_data.target_deg = (float)(int8_t)can_rx.data[3];
			get_object_data.target_distance =get_object_data.target_distance / 100.0;
			hal.console->printf("distance=%f\r\n",get_object_data.target_distance);
			hal.console->printf("deg=%f\r\n",get_object_data.target_deg);
			break;

	    default:
	    	break;
	}
	if(get_object_data.target_distance * 100 < 20 || get_object_data.target_distance * 100 > 600 
	   || (can_rx.id != OBJECT_INFORMATION 
	   && can_rx.id != CH30_DATA
	   ))
	{
    	reading_cm = 0;
		target_deg = 0.0;
	hal.console->printf("reading return false\r\n");
	    return false;
	}
	hal.console->printf("reading return true\r\n");
    reading_cm = get_object_data.target_distance * 100;
    if(can_rx.id == OBJECT_INFORMATION)
		target_deg = get_object_data.target_deg*57.29;
	else
		target_deg = get_object_data.target_deg;

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
		hal.console->printf("final distance=%d\r\n",state.distance_cm);
		hal.console->printf("final deg=%f\r\n",state.target_deg);
        //更新测量数据
		state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else
    {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}





