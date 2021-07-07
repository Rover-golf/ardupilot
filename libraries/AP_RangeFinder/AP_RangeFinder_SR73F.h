/*
 * AP_RangeFinder_SR73F.h
 *
 *  Created on: Aug 10, 2020
 *      Author: coco
 */

#ifndef DZUAV_FC_FIRMWARE_LIBRARIES_AP_RANGEFINDER_H_
#define DZUAV_FC_FIRMWARE_LIBRARIES_AP_RANGEFINDER_H_


#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include "AP_RangeFinder_Params.h"



//定义恩曌TR24DA100类继承AP_RangeFinder_Backend和CANProtocol类
class AP_RangeFinder_SR73F  : public AP_RangeFinder_Backend
{
public:
	AP_RangeFinder_SR73F();
	//定义构造函数
	AP_RangeFinder_SR73F(RangeFinder::RangeFinder_State &_state,
                              AP_RangeFinder_Params &_params);
	//识别传感器
	static AP_RangeFinder_Backend * detect(RangeFinder::RangeFinder_State &_state,
           AP_RangeFinder_Params &_params);
   bool get_reading(uint16_t &reading_cm, float &target_deg);
   bool init(uint32_t rate,uint8_t driver_index);
	//对数据进行更新
	void update(void)override;








protected:
	//定义保护类型的函数，获取无人机距离传感器类型---所有通用格式这里可以先不用
	virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
	{
		return MAV_DISTANCE_SENSOR_UNKNOWN;
	}
private:
      uint32_t last_reading_ms=0;
      bool _initialized;

};








#endif
