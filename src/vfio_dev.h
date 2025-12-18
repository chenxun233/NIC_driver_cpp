#ifndef VFIO_DEV_H
#define VFIO_DEV_H
#include "basic_dev.h"
#include <cstdint>
struct QueuesPtr {
    void*                   rx;
    void*                   tx;
};
    

class VfioDev : public BasicDev{
    public:
        VfioDev(
                std::string pci_addr,
                uint16_t    num_rx_queues,
                uint16_t    num_tx_queues)                                                 ;
        ~VfioDev()                                                                         ;
        bool getFD()                       override                                        ;
        bool mapBAR (uint8_t bar_index)    override                                        ;
        bool enableDMA()                   override                                        ;
        bool initHardware()                override                                        ;
        bool initRingBuffer()              override                                        ;
        bool enable_interrupt()            override                                        ;
        bool initMemPool()                 override                                        ;
        bool set_promisc(bool enable)      override                                        ;
        bool wait_for_link()               override                                        ;
    private:                           
        bool _get_group_id()                                                               ;
        bool _get_group_fd()                                                               ;
        bool _get_container_fd()                                                           ;
        bool _get_device_fd()                                                              ;
        bool _add_group_to_container()                                                     ;
    private:
        bool        _dev_disable_IRQ()                                                            ;
        bool        _dev_clear_interrupts()                                                       ;
        bool        _dev_rst_hardware()                                                           ;
        bool        _get_mac_address()                                                            ;
        bool        _init_eeprom_n_dma()                                                          ;
        bool        _init_link_nego()                                                             ;
        bool        _read_stats()                                                                 ;
    private:
        bool        _init_rx()                                                                  ;
        bool        _init_tx()                                                                  ;
        bool        _prepare_rx_queue()                                                         ;
        bool        _prepare_tx_queue()                                                         ;
        void        _enable_msi_interrupt(uint16_t queue_id)                                    ;
        void        _enable_msix_interrupt(uint16_t queue_id)                                   ;
        uint32_t    _get_link_speed()                                                           ;
        bool        _initialize_interrupt()                                                 ;
        bool        _host_setup_IRQ_type()                                                  ;
        bool        _host_alloc_IRQ_queues()                                                ;
        bool        _host_setup_IRQ_queues()                                                ;
        int         _vfio_enable_msi()                                                      ;
        int         _vfio_enable_msix(int index)                                            ;
        int         _vfio_epoll_ctl(int event_fd)                                           ;
    private:                   
        MemPool*                           p_mempool{nullptr}                              ;        
        uint16_t                           _calc_ip_checksum  (uint8_t* data, uint32_t len);
        void                               _pkt_buf_free      (struct pkt_buf* buf)        ;
        QueuesPtr                          m_queues;

};

#endif // VFIO_DEV_H
