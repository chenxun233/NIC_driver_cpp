#ifndef NON_VFIO_DEV_H
#define NON_VFIO_DEV_H
#include "basic_dev.h"
class non_vfio_dev : public BasicDev{
    public:
        non_vfio_dev(
                    std::string pci_addr,
                    uint8_t     bar_index_max,
                    uint16_t    num_rx_queues,
                    uint16_t    num_tx_queues,
                    uint16_t    interrupt_timeout_ms )          ;
        ~non_vfio_dev()                                         ;
        bool initialize()                           override    ;
        bool map_bar ()                             override    ;
        bool enable_dma()                           override    ;
        bool remove_ixgbe_driver()                              ;
    private:    
};


#endif // NON_VFIO_DEV_H