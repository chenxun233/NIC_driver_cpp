#pragma once
#include "basic_dev.h"
#include <cstdint>
#include <vector>
#include "memory_pool.h"
#include "ixgbe_ring_buffer.h"
struct QueuesPtr {
    void*                   rx;
    void*                   tx;
};
    


class VFIODev : public BasicDev{
    public:
        VFIODev(
                std::string pci_addr,
                uint16_t    num_rx_queues,
                uint16_t    num_tx_queues)                                                 ;
        ~VFIODev()                                                                         ;
        bool        getFD()                                                 override                                 ;
        bool        mapBAR (uint8_t bar_index)                              override                                 ;
        bool        enableDMA()                                             override                                 ;
        bool        initHardware()                                          override                                 ;
        bool        setDMAMemory()                                          override                                 ;
        bool        prepareQueues()                                         override                                 ;
        bool        enable_interrupt()                                      override                                 ;
        bool        initMemoryPool(uint32_t num_buf, uint32_t buf_size)     override                                 ;
        bool        setRingBuffers()                                        override                                    ;
        bool        initMemPool()                                           override                                 ;
        bool        set_promisc(bool enable)                                override                                 ;
        bool        wait_for_link()                                         override                                 ;
    private:                                  
        bool        _get_group_id()                                                        ;
        bool        _get_group_fd()                                                        ;
        bool        _get_container_fd()                                                    ;
        bool        _get_device_fd()                                                       ;
        bool        _add_group_to_container()                                              ;
    private:
        bool        _dev_disable_IRQ()                                                     ;
        bool        _dev_clear_interrupts()                                                ;
        bool        _dev_rst_hardware()                                                    ;
        bool        _get_mac_address()                                                     ;
        bool        _init_eeprom_n_dma()                                                   ;
        bool        _init_link_nego()                                                      ;
        bool        _read_stats()                                                          ;
    private:
        bool        _set_rx_DMA()                                                         ;
        bool        _set_tx_DMA()                                                         ;
        bool        _prepare_rx_queue()                                                    ;
        bool        _prepare_tx_queue()                                                    ;
        void        _enable_msi_interrupt(uint16_t queue_id)                               ;
        void        _enable_msix_interrupt(uint16_t queue_id)                              ;
        uint32_t    _get_link_speed()                                                      ;
        bool        _initialize_interrupt()                                                ;
        bool        _host_setup_IRQ_type()                                                 ;
        bool        _host_alloc_IRQ_queues()                                               ;
        bool        _host_setup_IRQ_queues()                                               ;
        int         _vfio_enable_msi()                                                     ;
        int         _vfio_enable_msix(int index)                                           ;
        int         _vfio_epoll_ctl(int event_fd)                                          ;
        uint16_t    _calc_ip_checksum  (uint8_t* data, uint32_t len)                       ;
    private:
        uint32_t                        m_num_rx_bufs{0}                                        ;   
        uint32_t                        m_buf_rx_size{0}                                        ;
        uint32_t                        m_num_tx_bufs{0}                                           ;
        uint32_t                        m_buf_tx_size{0}                                        ;
        MemoryPool*                     m_mempool{nullptr}                                 ;
        MemoryPool*                     m_tx_mempool{nullptr}                              ;
        // MemPool*                     p_mempool{nullptr}                                 ;
        std::vector<IXGBE_RingBuffer*>  p_rx_ring_buffers                                    ;
        std::vector<IXGBE_RingBuffer*>  p_tx_ring_buffers                                    ;     
        QueuesPtr                       m_queues;

};

