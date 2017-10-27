#include "canalystii.h"

CANalystii::CANalystii(int vci_device_type, int vci_device_ind):can_ind_port_({0,1}){
    vci_device_type_ = vci_device_type;
    vci_device_ind_ = vci_device_ind;
    is_dev_start_ = false;
    for(int i=0;i<2;i++){
        vci_conf_[i].AccCode = 0x80000008;
        vci_conf_[i].AccMask = 0xFFFFFFFF;
        vci_conf_[i].Filter = 1;//receive all frames
        vci_conf_[i].Timing0 = 0x00;
        vci_conf_[i].Timing1 = 0x1C;//baudrate 500kbps
        vci_conf_[i].Mode = 0;//normal mode
        is_port_init_[i] = false;
        is_port_start_[i] = false;
    }
}

CANalystii::~CANalystii(){
    if(is_dev_start_ == true){
        close_device();
    }
}

bool CANalystii::start_device(){
    if(!is_dev_start_){
        is_dev_start_ = (VCI_OpenDevice(vci_device_type_, vci_device_ind_, 0)==1);
    }
    return is_dev_start_;
}

bool CANalystii::close_device(){
    if(is_dev_start_){
        is_dev_start_ = !(VCI_CloseDevice(vci_device_type_, vci_device_ind_)==1);
    }
    is_port_init_[0] = false;
    is_port_init_[1] = false;
    is_port_start_[0] = false;
    is_port_start_[1] = false;
    return !is_dev_start_;
}

bool CANalystii::init_can_interface(unsigned int can_idx, const VCI_INIT_CONFIG& vci_conf){
    unsigned int can_port_idx = can_idx<=1?can_ind_port_[can_idx]:0;
    vci_conf_[can_port_idx] = vci_conf;
    is_port_init_[can_port_idx] = (VCI_InitCAN(vci_device_type_, vci_device_ind_, can_port_idx,&vci_conf_[can_port_idx])==1);
    is_port_start_[can_port_idx] = (VCI_StartCAN(vci_device_type_, vci_device_ind_, can_port_idx)==1);
    VCI_ClearBuffer(vci_device_type_, vci_device_ind_, can_port_idx);
    VCI_ClearBuffer(vci_device_type_, vci_device_ind_, can_port_idx);
    return is_port_init_[can_port_idx]&&is_port_start_[can_port_idx];
}

unsigned int CANalystii::receive_can_frame(unsigned int can_idx, VCI_CAN_OBJ &recv_obj, unsigned int recv_len, int wait_time){
    unsigned int can_port_idx = can_idx<=1?can_ind_port_[can_idx]:0;
    unsigned int receive_len = 0;
    receive_len = VCI_Receive(vci_device_type_, vci_device_ind_, can_port_idx, &recv_obj, recv_len, wait_time);
    return receive_len;
}

bool CANalystii::send_can_frame(unsigned int can_idx, VCI_CAN_OBJ &send_obj, unsigned int send_len){
    //TODO: (vincent.cheung.mcer@gmail.com) Not yet implemented.
    unsigned int can_port_idx = can_idx<=1?can_ind_port_[can_idx]:0;    
    bool status;
    status = (VCI_Transmit(vci_device_type_, vci_device_ind_, can_port_idx, &send_obj, send_len)==1);
    return status;
}
