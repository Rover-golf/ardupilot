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
#include <AP_SR73F_CAN/AP_SR73F_CAN.h>

////距离和目标角度
extern uint16_t sr73f_reading_cm;
extern float sr73f_target_deg;
////定义使能参数
//extern bool sr73f_senseor_driver_enable;
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
AP_RangeFinder_Backend * AP_RangeFinder_SR73F::detect(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
{
	AP_RangeFinder_SR73F *sensor= new AP_RangeFinder_SR73F(_state,_params);

    if((!sensor) ||(!sensor->init(_params.rate,_params.can_driver))) //初始化对应的串口
    {
   	    hal.console->printf("RNGFND SR73F Init Fail \r\n");
        delete sensor;
        return nullptr;
    }
    hal.console->printf("RNGFND SR73F Init Success \r\n");
    return sensor;
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
	reading_cm=sr73f_reading_cm;
	target_deg=sr73f_target_deg;
    if(reading_cm>0) //必须输出数据
    {
      return true;
    }
    else
    {
    	 return false;
    }



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




