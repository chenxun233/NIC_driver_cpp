#ifndef BASIC_DEV_H
#define BASIC_DEV_H
#include <cstdint>
#include <string>
#include <array>
#include <memory>
// #include "ixgbe_type.h"
struct __attribute__((__packed__)) mac_address_type {
	uint8_t	addr[6];
};
struct basic_para_type{
	std::string                 pci_addr;
    uint8_t                     bar_index;
	uint16_t                    num_rx_queues;
	uint16_t                    num_tx_queues;
    uint16_t                    interrupt_timeout_ms;
    std::array<uint8_t*,6>      p_bar_addr;
    mac_address_type            m_mac_address;
};

// struct mempool {
// 	void*                       base_addr;
// 	uint32_t                    buf_size;
// 	uint32_t                    num_entries;
// 	// memory is managed via a simple stack
// 	// replacing this with a lock-free queue (or stack) makes this thread-safe
// 	uint32_t                    free_stack_top;
// 	// the stack contains the entry id, i.e., base_addr + entry_id * buf_size is the address of the buf
// 	uint32_t                    free_stack[];
// };

// struct rx_queue_type{
//     volatile union ixgbe_adv_rx_desc* descriptors;
// 	struct mempool* mempool;
// 	uint16_t num_entries;
// 	// position we are reading from
// 	uint16_t rx_index;
// 	// virtual addresses to map descriptors back to their mbuf for freeing
// 	void* virtual_addresses[];
// };

// struct tx_queue_type {
// 	volatile union ixgbe_adv_tx_desc* descriptors;
// 	uint16_t num_entries;
// 	// position to clean up descriptors that where sent out by the nic
// 	uint16_t clean_index;
// 	// position to insert packets for transmission
// 	uint16_t tx_index;
// 	// virtual addresses to map descriptors back to their mbuf for freeing
// 	void* virtual_addresses[];
// };


typedef struct{
    int                         container_fd;
    int                         group_id;
    int                         group_fd;
    int                         device_fd;
}vfio_fd_type;


class basic_dev{
    public:
        explicit            basic_dev(
                            	        std::string pci_addr,
                                        uint8_t     bar_index,
	                                    uint16_t    num_rx_queues,
	                                    uint16_t    num_tx_queues,
                                        uint16_t    interrupt_timeout_ms
        );
        virtual             ~basic_dev()                   = default   ;
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
                                    
};
#endif // BASIC_DEV_H

