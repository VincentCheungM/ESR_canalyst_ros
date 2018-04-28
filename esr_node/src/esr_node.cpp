/*
 * cluster.cpp
 *
 * Created on   : Oct 21, 2017
 * Author   : Vincent Cheung
 * @brief Below algorithm is based on Euclidean Clustering provided in PCL.
 */
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "canalystii_node_msg/can.h"
#include "delphi_esr_msgs/EsrTrack.h"
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/common/common.h>
// #include <pcl/common/centroid.h>

// bytes to sign
#define B2SIGN(x) ((x)==1?-1:1)
// bytes to complement
#define B2COMP1(x) ((64)-(x))
#define B2COMP2(x) ((1024)-(x))
#define B2COMP3(x) ((16384)-(x))

class Esr_node
{
public:
    
    Esr_node();

private:

    ros::NodeHandle node_handle_;
    ros::Subscriber can_node_sub_;
    ros::Publisher esr_track_pub_;
    ros::Publisher esr_marker_array_pub_;
    
    std::string can_topic_;
    std::string esr_track_topic_;    
    std::string marker_topic_;
    std::string esr_frame_id_;

    double marker_height_;
    double marker_length_;
    double marker_width_;
    double m_pi_;

    void CAN_callback(const canalystii_node_msg::can::ConstPtr &in_can_msg);
    delphi_esr_msgs::EsrTrack set_track_msg(const canalystii_node_msg::can::ConstPtr &in_can_msg);
    visualization_msgs::Marker track_msg2marker(const delphi_esr_msgs::EsrTrack & track_msg);
};

Esr_node::Esr_node() : node_handle_("esr_n"),m_pi_(3.1415926)
{
    ROS_INFO("Inititalizing ESR Rader Node...");

    node_handle_.param<std::string>("can_topic_", can_topic_, "/canalyst_can");
    ROS_INFO("can_topic_: %s", can_topic_.c_str());

    node_handle_.param<std::string>("esr_track_topic_", esr_track_topic_, "/esr_track");
    ROS_INFO("esr_track_topic_: %s", esr_track_topic_.c_str());

    node_handle_.param<std::string>("marker_topic_", marker_topic_, "/esr_track_marker");
    ROS_INFO("marker_topic_: %s", marker_topic_.c_str());

    node_handle_.param<std::string>("esr_frame_id_", esr_frame_id_, "/esr_node");
    ROS_INFO("esr_frame_id_: %s", esr_frame_id_.c_str());

    node_handle_.param<double>("marker_height_", marker_height_, 0.5);
    ROS_INFO("marker_height_: %lf", marker_height_);
    node_handle_.param<double>("marker_length_", marker_length_, 5.0);
    ROS_INFO("marker_length_: %lf", marker_length_);
    node_handle_.param<double>("marker_width_", marker_width_, 2.0);
    ROS_INFO("marker_width_: %lf", marker_width_);

    can_node_sub_ = node_handle_.subscribe(can_topic_, 2, &Esr_node::CAN_callback, this);
    esr_track_pub_ = node_handle_.advertise<delphi_esr_msgs::EsrTrack>(esr_track_topic_, 10);
    esr_marker_array_pub_ = node_handle_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
}

//TODO: (vincent.cheung.mcer@gmail.com) need to add comments to these magic bits operation
delphi_esr_msgs::EsrTrack Esr_node::set_track_msg(const canalystii_node_msg::can::ConstPtr &in_can_msg){
    delphi_esr_msgs::EsrTrack track_msg;
    //set header
    track_msg.header.frame_id = esr_frame_id_;
    track_msg.header.stamp = in_can_msg->header.stamp; 
    
    track_msg.track_ID = in_can_msg->id - 0x500;//from [0x500, 0x53F] to [0x0,0x3F]
    //is_on_coming_ = (in_can_msg->data[0]&0x01);//not inside track_msg
    track_msg.track_group_changed = (in_can_msg->data[0] & 0x02)>>1;

    //sign value
    if(B2SIGN((in_can_msg->data[0]&0x80)>>7)==-1){
        track_msg.track_lat_rate = (-B2COMP1((in_can_msg->data[0]&0xFC)>>2))*0.25;
    }else{
        track_msg.track_lat_rate = ((in_can_msg->data[0]&0xFC)>>2)*0.25;
    }
    
    track_msg.track_status = (in_can_msg->data[1]&0xE0)>>5;

    //sign value
    if(B2SIGN((in_can_msg->data[1]&0x10)>>4)==-1){
        track_msg.track_angle = -B2COMP2((((in_can_msg->data[1]&0x1F)<<5)|((in_can_msg->data[2]&0xF8)>>3)))*0.1;
    }else{
        track_msg.track_angle = (((in_can_msg->data[1]&0x1F)<<5)|((in_can_msg->data[2]&0xF8)>>3))*0.1;
    }

    track_msg.track_range = (((in_can_msg->data[2]&0x07)<<8)|in_can_msg->data[3])*0.1;
    track_msg.track_bridge_object = (in_can_msg->data[4]&0x80)>>7;
    track_msg.track_rolling_count = (in_can_msg->data[4]&0x40)>>6;
    track_msg.track_width = ((in_can_msg->data[4]&0x3C)>>2)*0.5;

    //sign value
    if(B2SIGN((in_can_msg->data[4]&0b00000010)>>1)==-1){
        track_msg.track_range_accel = -B2COMP2((((in_can_msg->data[4]&0x03)<<8)+in_can_msg->data[5]))*0.05;
    }else{
        track_msg.track_range_accel = (((in_can_msg->data[4]&0x03)<<8)+in_can_msg->data[5])*0.05;
    }    

    track_msg.track_med_range_mode = ((in_can_msg->data[6]&0xC0)>>6);

    //sign value
    if(B2SIGN((in_can_msg->data[6]&0b00100000)>>5)==-1){
        track_msg.track_range_rate = -B2COMP3((((in_can_msg->data[6]&0x3F)<<8)|in_can_msg->data[7]))*0.01;
    }else{
        track_msg.track_range_rate = (((in_can_msg->data[6]&0x3F)<<8)|in_can_msg->data[7])*0.01;//sign
    }

    return track_msg;
}


visualization_msgs::Marker Esr_node::track_msg2marker(const delphi_esr_msgs::EsrTrack & track_msg){
    uint32_t shape = visualization_msgs::Marker::CUBE; 
    visualization_msgs::Marker marker; 
    marker.header = track_msg.header;
    
    marker.ns = std::to_string(track_msg.track_ID); 
    marker.id = track_msg.track_ID; 
    marker.type = shape; 
    marker.action = visualization_msgs::Marker::ADD; 
    const double angle_rad = track_msg.track_angle * m_pi_/ 180;
    marker.pose.position.x = track_msg.track_range*std::sin(angle_rad); 
    marker.pose.position.y = track_msg.track_range*std::cos(angle_rad); 
    marker.pose.position.z = marker_height_; 
    marker.pose.orientation.x = 0.0; 
    marker.pose.orientation.y = 0.0; 
    marker.pose.orientation.z = 0.0; 
    marker.pose.orientation.w = 1.0; 
    
    marker.scale.x = marker_length_; 
    marker.scale.y = marker_width_; 
    marker.scale.z = marker_height_; 
          
    marker.color.r = 10*track_msg.track_ID % 255; 
    marker.color.g = 20*track_msg.track_ID % 255; 
    marker.color.b = 40*track_msg.track_ID % 255; 
    marker.color.a = 0.5; 
  
    marker.lifetime = ros::Duration(0.5); 
    return marker; 

}
void Esr_node::CAN_callback(const canalystii_node_msg::can::ConstPtr &in_can_msg)
{
    ROS_INFO("Listening CAN frame id:%d",in_can_msg->id);
    //check if it is not a esr track msg
    if(in_can_msg->id<0x500 || in_can_msg->id>0x53F){
        //TODO: (vincent.cheung.mcer@gmail.com) currently do nothing, for track_msg.id = [0x500,0x53f]
        return ;
    }

    delphi_esr_msgs::EsrTrack track_msg = set_track_msg(in_can_msg);

    visualization_msgs::Marker ma = track_msg2marker(track_msg); 

    esr_marker_array_pub_.publish(ma);
    esr_track_pub_.publish (track_msg);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Esr_radar_node");
    Esr_node node;
    ROS_INFO("Listening CAN frame");
    ros::spin();

    return 0;

}
