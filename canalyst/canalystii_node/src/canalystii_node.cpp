#include "canalystii_node.h"

CANalystii_node::CANalystii_node(int vci_device_type, int vci_device_ind) : 
CANalystii(vci_device_type,vci_device_ind), node_handle_("~")
{
    ROS_INFO("Inititalizing CANalyst-ii node...");

    node_handle_.param<std::string>("can_msg_pub_topic", can_msg_pub_topic_, "/canalyst_can");
    ROS_INFO("can_msg_pub_topic_: %s", can_msg_pub_topic_.c_str());
    can_msg_pub_ = node_handle_.advertise<canalystii_node_msg::can>(can_msg_pub_topic_, 10);

    //TODO:(vincent.cheung.mcer@gmail.com) Noy yet implement subscriber.
    //node_handle_.param<std::string>("can_msg_sub_topic", can_msg_sub_topic_, "/not_used_can");
    //ROS_INFO("can_msg_sub_topic_: %s", can_msg_sub_topic_.c_str());
    //can_msg_sub_ = node_handle_.subscribe(can_msg_sub_topic_, 2, &CANalystii_node::can_msg_cb, this);
}

canalystii_node_msg::can CANalystii_node::can_obj2msg(const VCI_CAN_OBJ& can_obj){
    canalystii_node_msg::can can_msg;
    //set Header with ros time rather than can_obj time, in case there are no timestamp
    //TODO: (vincent.cheung.mcer@gmail.com) miss seq here
    can_msg.header.frame_id = "/canalystii";
    can_msg.header.stamp = ros::Time::now();

    //set data    
    can_msg.id = can_obj.ID;
    can_msg.timeflag = can_obj.TimeFlag;
    can_msg.sendtype = can_obj.SendType;
    can_msg.remoteflag = can_obj.RemoteFlag;
    can_msg.externflag = can_obj.ExternFlag;
    can_msg.datalen = can_obj.DataLen;
    for(int i =0;i<8;i++){
        can_msg.data[i] = can_obj.Data[i];
    }
    return can_msg;
}

VCI_CAN_OBJ CANalystii_node::can_msg2obj(const canalystii_node_msg::can& can_msg){
    VCI_CAN_OBJ can_obj;
    //TODO: (vincent.cheung.mcer@gmail.com) type not consistent
    //can_obj.TimeStamp = can_msg.header.stamp
    can_obj.ID = can_msg.id;
    can_obj.TimeFlag = can_msg.timeflag;
    can_obj.SendType = can_msg.sendtype;
    can_obj.RemoteFlag = can_msg.remoteflag;
    can_obj.ExternFlag = can_msg.externflag;
    can_obj.DataLen = can_msg.datalen;
    for(int i =0;i<8;i++){
        can_obj.Data[i] = can_msg.data[i];
    }
    return can_obj;    
}
