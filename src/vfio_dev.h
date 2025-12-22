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
        VFIODev(std::string pci_addr, uint8_t max_bar_index)                                                     ;
        ~VFIODev()                                                                                               ;
        bool        initHardware(const int interrupt_interval)                override                           ;
        bool        setDescriptorRings()                                      override                           ;
        bool        enableDevQueues()                                         override                           ;
        bool        enableDevInterrupt()                                      override                           ;
        bool        setRxRingBuffers(uint16_t num_tx_queues,uint32_t num_buf, uint32_t buf_size)     override    ;
        bool        setTxRingBuffers(uint16_t num_tx_queues,uint32_t num_buf, uint32_t buf_size)     override    ;
        bool        sendOnQueue(uint8_t* p_data, size_t size, uint16_t queue_id)                     override    ;
        bool        fillTxMemPool(uint32_t num_buf)                      override                           ;
        void        send()                                    override                           ;

        bool        setPromisc(bool enable)                             override                                 ;
        bool        wait4Link()                                         override                                 ;
    private:
        bool        _getFD()                                                 override                                 ;
        bool        _mapBAR (uint8_t bar_index)                              override                                 ;
        bool        _enableDMA()                                             override                                 ;                                  
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
        DevStatus    _readStatus()                                      override      ;
        void        _initStatus(DevStatus* stats)                       override      ;
    private:
        bool        _setRxDescriptorRing()                                                 ;
        bool        _setTxDescriptorRing()                                                 ;
        bool        _enableDevRxQueue()                                                    ;
        bool        _enableDevTxQueue()                                                    ;
        void        _enableDevMSIInterrupt(uint16_t queue_id)                              ;
        void        _enableDevMSIxInterrupt(uint16_t queue_id)                             ;
        uint32_t    _get_link_speed()                                                      ;
        bool        _initialize_interrupt(const int &interrupt_interval)                   ;
        bool        _getDevIRQType()                                                       ;
        bool        _allocIRQQueues()                                                      ;
        bool        _setupIRQQueues(const int &interrupt_interval)                         ;
        int         _injectEventFdToVFIODev_msi()                                          ;
        int         _injectEventFdToVFIODev_msix(int index)                                ;
        int         _vfio_epoll_ctl(int event_fd)                                          ;
        uint16_t    _calc_ip_checksum  (uint8_t* data, uint32_t len)                       ;
    private:
        uint32_t                        m_num_rx_bufs{0}                                   ;   
        uint32_t                        m_buf_rx_size{0}                                   ;
        uint32_t                        m_num_tx_bufs{0}                                   ;
        uint32_t                        m_buf_tx_size{0}                                   ;
        // std::vector<MemoryPool*>        p_mempool                                          ;
        MemoryPool*                       p_tx_mempool{nullptr}                              ;
        std::vector<IXGBE_RxRingBuffer*>  p_rx_ring_buffers                                  ;
        std::vector<IXGBE_TxRingBuffer*>  p_tx_ring_buffers                                  ;
        uint32_t                          m_used_tx_buf_num{0}                               ;     

};

