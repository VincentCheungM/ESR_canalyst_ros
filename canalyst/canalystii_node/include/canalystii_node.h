#ifndef CANALYSTII_NODE_H
#define CANALYSTII_NODE_H

#include "ros/ros.h"
#include "canalystii.h"
#include "canalystii_node_msg/can.h"

// Simple usage:
// CANalystii_node can_device;
// can_device.start_device();
// can_device.init_can_interface();
// can_device.receive_can_frame();//listening from CAN bus
// can_device.send_can_frame();//send to CAN bus
class CANalystii_node:public CANalystii{
private:
    ros::NodeHandle node_handle_;    
    std::string can_msg_pub_topic_;

    //TODO:(vincent.cheung.mcer@gmail.com) Noy yet implement subscriber.
    //ros::Subscriber can_msg_sub_; 
    //std::string can_msg_sub_topic_;
    //void can_msg_cb();
      
public:
    CANalystii_node(int vci_device_type=4, int vci_device_ind=0);
    //~CANalystii_node();
    static canalystii_node_msg::can can_obj2msg(const VCI_CAN_OBJ& can_obj);
    static VCI_CAN_OBJ can_msg2obj(const canalystii_node_msg::can& can_msg);
    ros::Publisher can_msg_pub_;
};
#endif