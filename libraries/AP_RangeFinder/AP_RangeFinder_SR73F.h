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
	//定义构造函数
	AP_RangeFinder_SR73F(RangeFinder::RangeFinder_State &_state,
                              AP_RangeFinder_Params &_params);
	//识别传感器
   static bool detect(RangeFinder::RangeFinder_State &_state,
           AP_RangeFinder_Params &_params);
   bool get_reading(uint16_t &reading_cm, float &target_deg);
   bool init(uint32_t rate,uint8_t driver_index);
	//对数据进行更新
	void update(void)override;
    enum{
  	  OBJECT_STATUS=1546,
  	  OBJECT_INFORMATION=1547
    };
    typedef struct{
 	   uint8_t Objects_NofObjects:8 ; //目标个数
 	   uint8_t Objects_MeasCountH:8 ; //循环计数高8位
 	   uint8_t Objects_MeasCountL:8 ; //循环计数低8位
 	   uint8_t Objects_InterfaceVersion:4 ; //只要高4位,接口版本
 	   uint16_t Objects_MeasCount:16 ; //循环计数数据=循环计数高8位|循环计数低8位
    }ObjectsStatusBits;


   typedef struct{
 	  uint8_t Id:8; /*0:7雷达ID*/
 	  uint8_t DistLongH:8; /*8:15 纵向距离高8位*/
 	  uint8_t DistLatH:3; /*16:18    横向距离高3位*/
 	  uint8_t DistLongL:5; /*19:23    纵向距离低5位*/
 	  uint8_t DistLatL:8;    /*24:31    横向距离低8位*/
 	  uint8_t VrelLongH:8;  /*32:39    纵向速度高8位*/
 	  uint8_t VrelLatH:6;  /*40:45    横向速度高6位*/
 	  uint8_t VrelLongL:2;  /*46:47    纵向速度低2位*/
 	  uint8_t DynProp:3;  /*48:50    目标的运动属性，默认是0*/
 	  uint8_t undefined:2;  /*51:52    目标的运动属性，默认是0*/
 	  uint8_t VrelLatL:3;  /*53:55    横向速度低3位*/
 	  uint8_t RCS:8;  /*56:63    目标的运动属性，默认是0*/
   }ObjectsInforBits;




   typedef struct { // byte description
   		float Objects_ID; //目标ID
   		float Objects_DistLong;  //目标纵向距离
   		float Objects_DistLat;   //目标横向距离
   		float Objects_VrelLong;  // 目标纵向速度
   		float Objects_DynProp;     // 目标运动属性
   		float Objects_VrelLat;    //目标横向速度
   		float Objects_RCS;         //目标RCS默认是0
   		float target_distance;      //目标距离
   		float target_deg;           //目标角度
   		float target_speed;         //目标速度
   }Object_dataBytes;
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
