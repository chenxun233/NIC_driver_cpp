#include "implementation.h"

dev_ptr_type create_device(
        const  basic_para_type& basic_para
){
    dev_ptr_type device;
    device = std::make_unique<vfio_dev>(basic_para);
    if (device->initialize()){
        return device;
    }
    device = std::make_unique<non_vfio_dev>(basic_para);
    if (device->initialize()){
        return device;
    }
    return nullptr;

}   