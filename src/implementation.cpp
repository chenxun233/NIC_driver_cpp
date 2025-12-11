#include "implementation.h"

std::unique_ptr<BasicDev> return_p_device(
                            std::string pci_addr,
                            uint8_t     bar_index_max,
                            uint16_t    num_rx_queues,
                            uint16_t    num_tx_queues,
                            uint16_t    interrupt_timeout_ms
)
{
    std::unique_ptr<BasicDev> p_device;
    p_device = std::make_unique<vfio_dev>(
                                        pci_addr, 
                                        bar_index_max, 
                                        num_rx_queues, 
                                        num_tx_queues, 
                                        interrupt_timeout_ms);
    if (p_device->initialize()){
        debug("VFIO device initialized successfully");
        return p_device;
    }
    p_device = std::make_unique<non_vfio_dev>(
                                        pci_addr, 
                                        bar_index_max, 
                                        num_rx_queues, 
                                        num_tx_queues, 
                                        interrupt_timeout_ms);
    if (p_device->initialize()){
        return p_device;
    }
    return nullptr;

}   