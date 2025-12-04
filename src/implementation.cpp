#include "implementation.h"

dev_ptr_type create_device(
                            std::string pci_addr,
                            uint8_t     bar_index_max,
                            uint16_t    num_rx_queues,
                            uint16_t    num_tx_queues,
                            uint16_t    interrupt_timeout_ms
){
    dev_ptr_type device;
    device = std::make_unique<vfio_dev>(
                                        pci_addr, 
                                        bar_index_max, 
                                        num_rx_queues, 
                                        num_tx_queues, 
                                        interrupt_timeout_ms);
    if (device->initialize()){
        return device;
    }
    device = std::make_unique<non_vfio_dev>(
                                        pci_addr, 
                                        bar_index_max, 
                                        num_rx_queues, 
                                        num_tx_queues, 
                                        interrupt_timeout_ms);
    if (device->initialize()){
        return device;
    }
    return nullptr;

}   