#include "ros/ros.h"
#include "canalystii_node.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "canalystii_node");
    CANalystii_node can_node;
    
    if(!can_node.start_device()){
        ROS_WARN("device starts error");
        return -1;
    }

    VCI_INIT_CONFIG vci_conf;
    vci_conf.AccCode = 0x80000008;
    vci_conf.AccMask = 0xFFFFFFFF;
    vci_conf.Filter = 1;//receive all frames
    vci_conf.Timing0 = 0x00;
    vci_conf.Timing1 = 0x1C;//baudrate 500kbps
    vci_conf.Mode = 0;//normal mode
    unsigned int can_idx = 0;
    if(!can_node.init_can_interface(can_idx,vci_conf)){
        ROS_WARN("device port init error");
        return -1;
    }

    ROS_INFO("listening to can bus");
    VCI_CAN_OBJ can_obj;
    while(ros::ok()){
        unsigned int recv_len = 1;
        
        //int len = can_node.receive_can_frame(can_idx,can_obj,recv_len,0);
        if(can_node.receive_can_frame(can_idx,can_obj,recv_len,20)){
            //ROS_INFO("received:%u",can_obj.ID);
            canalystii_node_msg::can msg = CANalystii_node::can_obj2msg(can_obj);
            can_node.can_msg_pub_.publish(msg);
        }
        //ros::spinOnce();
    }
    //ros::spin();
    return 0;
}