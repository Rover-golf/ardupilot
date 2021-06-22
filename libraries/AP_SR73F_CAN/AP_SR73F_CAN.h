/*
 * APC_SR73F_CAN.h
 *
 *  Created on: Aug 1, 2020
 *      Author: coco
 */

#ifndef DZUAV_FC_FIRMWARE_LIBRARIES_AP_SR73FCAN_H_
#define DZUAV_FC_FIRMWARE_LIBRARIES_AP_SR73FCAN_H_


#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>


class AP_SR73F_CAN
{
  public:
	  AP_SR73F_CAN();
	  ~AP_SR73F_CAN();

       /* Do not allow copies */
	   AP_SR73F_CAN(const AP_SR73F_CAN &other) = delete;
	   AP_SR73F_CAN &operator=(const AP_SR73F_CAN&) = delete;

       //初始化
       void init(uint8_t inum,uint32_t buad) ;
       //更新
       void update();
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
       //定义结构体,获取目标状态数据
       ObjectsStatusBits objectstatus_data;

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
      //定义目标信息位
      ObjectsInforBits objectsinformation;

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

      Object_dataBytes get_object_data;


};






#endif
