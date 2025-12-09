#ifndef BASIC_DEV_H
#define BASIC_DEV_H
#include <cstdint>
#include <string>
#include <array>
#include <memory>
#include "memory_op.h"
// #include "ixgbe_type.h"
struct __attribute__((__packed__)) mac_address_type {
	uint8_t	addr[6];
};

struct dev_stats_type {
    uint64_t    rx_pkts;
    uint64_t    tx_pkts;
    uint64_t    rx_bytes;
    uint64_t    tx_bytes;
};

struct basic_para_type{
	std::string                 pci_addr;
    uint8_t                     bar_index_max;
	uint16_t                    num_rx_queues;
	uint16_t                    num_tx_queues;
    uint16_t                    interrupt_timeout_ms;
    std::array<uint8_t*,6>      p_bar_addr;
    mac_address_type            mac_address;
};

struct VfioFd{
    int                         container_fd;
    int                         group_id;
    int                         group_fd;
    int                         device_fd;
};

class BasicDev{
    public:
        explicit            BasicDev(
                            	        std::string pci_addr,
                                        uint8_t     bar_index_max,
	                                    uint16_t    num_rx_queues,
	                                    uint16_t    num_tx_queues,
                                        uint16_t    interrupt_timeout_ms
        );
        virtual             ~BasicDev()                   = default   ;
        virtual bool        initialize()                   = 0         ;
        virtual bool        map_bar ()                     = 0         ;
        virtual bool        enable_dma()                   = 0         ;
        virtual bool        tx_batch()                     = 0         ;
        virtual bool        rx_batch()                     = 0         ;
        virtual bool        read_stats()                   = 0         ;
        virtual bool        set_promisc()                  = 0         ;
        virtual bool        get_link_speed()               = 0         ;
        virtual bool        set_mac_address()              = 0         ;
        virtual bool        get_mac_address()              = 0         ;
        virtual bool        reset()                        = 0         ;
        basic_para_type     get_basic_para()                           ;                                             
    protected:
        basic_para_type     m_basic_para                               ;
        dev_stats_type      m_dev_stats                                ;
                                    
};
#endif // BASIC_DEV_H

