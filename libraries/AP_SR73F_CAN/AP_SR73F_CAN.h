/*
 * AP_SR73F_CAN.h
 *
 *  Created on: Jul 6, 2021
 *      Author: coco
 */

#ifndef ARDUPILOT_LIBRARIES_AP_SR73F_CAN_AP_SR73F_CAN_H_
#define ARDUPILOT_LIBRARIES_AP_SR73F_CAN_AP_SR73F_CAN_H_


#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_Math/AP_Math.h>

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

class AP_SR73F_CAN: public AP_HAL::CANProtocol
{
  public:

	AP_SR73F_CAN();
    ~AP_SR73F_CAN();


    AP_SR73F_CAN(const AP_SR73F_CAN &other) = delete;
    AP_SR73F_CAN &operator=(const AP_SR73F_CAN&) = delete;


    static AP_SR73F_CAN *get_tcan(uint8_t driver_index);

    // initialise AP_SR73F_CAN bus
    void init(uint8_t driver_index, bool enable_filters) override;


    void update();

    // send sr73  messages over MAVLink
    void send_ap_sr73_can_telemetry_mavlink(uint8_t mav_chan);

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

   //定义结构体,获取目标状态数据
   ObjectsStatusBits objectstatus_data;
   //定义目标信息位
   ObjectsInforBits objectsinformation;
   Object_dataBytes get_object_data;

  private:

      // loop to send output to ESCs in background thread
      void loop();
      bool _initialized;
      char _thread_name[9];
      uint8_t _driver_index;
      uavcan::ICanDriver* _can_driver;


};




#endif /* ARDUPILOT_LIBRARIES_AP_SR73F_CAN_AP_SR73F_CAN_H_ */
