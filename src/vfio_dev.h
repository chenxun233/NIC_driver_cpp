#ifndef VFIO_DEV_H
#define VFIO_DEV_H
#include "basic_dev.h"
#include "interrupts.h"
#include "hardware_op.h"

    

class vfio_dev : public basic_dev{
    public:
        vfio_dev(
                std::string pci_addr,
                uint8_t     bar_index_max,
                uint16_t    num_rx_queues,
                uint16_t    num_tx_queues,
                uint16_t    interrupt_timeout_ms
            
        )                                                               ;
        ~vfio_dev()                                                     ;
        bool initialize()                           override            ;
        bool map_bar ()                             override            ;
        bool enable_dma()                           override            ;
        bool tx_batch()                             override            ;
        bool rx_batch()                             override            ;
        bool read_stats()                           override            ;
        bool set_promisc()                          override            ;
        bool get_link_speed()                       override            ;
        bool set_mac_address()                      override            ;
        bool get_mac_address()                      override            ;
        bool reset()                                override            ;
        bool set_interrupt_host()                                       ;
        bool set_hardware()                                             ;
        vfio_fd_type get_fds(){return m_fds;}                           ;
    private:        
        bool _get_group_id()                                            ;
        bool _get_group_fd()                                            ;
        bool _get_container_fd()                                        ;
        bool _get_device_fd()                                           ;
        bool _add_group_to_container()                                  ;
    private:        
        vfio_fd_type                                m_fds               ;
        std::unique_ptr<interrupt>                  p_interrupt         ;
        std::unique_ptr<hardware_op>                m_hardware_op       ;

};

#endif // VFIO_DEV_H
