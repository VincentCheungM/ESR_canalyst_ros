#ifndef CANALYSTII_H
#define CANALYSTII_H
#include "controlcan.h"

// CANalyst-ii is can-usb device class, which offers
// start_device, close_device, init_can_interface,
// receive_can_frame, send_can_frame methods. By
// using the provided libcontrol.so 
// Simple usage:
// CANalystii can_device;
// can_device.start_device();
// can_device.init_can_interface();
// can_device.receive_can_frame();//listening from CAN bus
// can_device.send_can_frame();//send to CAN bus
class CANalystii{
public:
    CANalystii(int vci_device_type=4, int vci_device_ind=0);
    ~CANalystii();
    // start device with default parameters, and return status
    bool start_device();
    // close device with default parameters, and return status    
    bool close_device();
    // init CAN interface:port 1 or port 2
    bool init_can_interface(unsigned int can_idx, const VCI_INIT_CONFIG& vci_conf);
    // receive CAN frame from CAN bus
    unsigned int receive_can_frame(unsigned int can_idx, VCI_CAN_OBJ &recv_obj, unsigned int recv_len, int wait_time=0);
    // send CAN frame from CAN bus
    bool send_can_frame(unsigned int can_idx, VCI_CAN_OBJ &send_obj, unsigned int send_len);
private:
    unsigned int vci_device_type_;
    unsigned int vci_device_ind_;
    
    const unsigned int can_ind_port_[2];
    VCI_INIT_CONFIG vci_conf_[2];

    bool is_dev_start_;
    bool is_port_init_[2];
    bool is_port_start_[2];
};



#endif