#ifndef BASIC_DEV_H
#define BASIC_DEV_H
#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include "memory_op.h"
#include <linux/vfio.h>

#define MOVING_AVERAGE_RANGE 5
#define IRQ_SET_BUF_LEN (sizeof(struct vfio_irq_set) + sizeof(int))
#define MAX_INTERRUPT_VECTORS 32
#define MSIX_IRQ_SET_BUF_LEN (sizeof(struct vfio_irq_set) + sizeof(int) * (MAX_INTERRUPT_VECTORS + 1))

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
struct interrupt_moving_avg {
	uint32_t index; // The current index
	uint32_t length; // The moving average length
	uint64_t sum; // The moving average sum
	uint64_t measured_rates[MOVING_AVERAGE_RANGE]; // The moving average window
};

struct InterruptQueue {
	int vfio_event_fd; // event fd
	int vfio_epoll_fd; // epoll fd
	bool interrupt_enabled; // Whether interrupt for this queue is enabled or not
	uint64_t last_time_checked; // Last time the interrupt flag was checked
	uint64_t instr_counter; // Instruction counter to avoid unnecessary calls to monotonic_time
	uint64_t rx_pkts; // The number of received packets since the last check
	uint64_t interval; // The interval to check the interrupt flag
	struct interrupt_moving_avg moving_avg; // The moving average of the hybrid interrupt
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

struct interruptPara{
    uint32_t                                    itr_rate                ;
    std::vector<InterruptQueue>                 interrupt_queues        ;
    uint8_t                                     interrupt_type          ;
};

class BasicDev{
    public:
                            BasicDev(                                                                          
                                std::string pci_addr,
                                uint16_t    num_rx_queues,
                                uint16_t    num_tx_queues
                            );
        virtual             ~BasicDev()                    = default   ;
        virtual bool        getFD()                        = 0         ;
        virtual bool        mapBAR (uint8_t bar_index)       = 0       ;
        virtual bool        enableDMA()                    = 0         ;
        virtual bool        initHardware()                  = 0        ;
        virtual bool        initRingBuffer()                = 0        ;
        virtual bool        enable_interrupt()              = 0        ;
        virtual bool        wait_for_link()                 = 0        ;
        virtual bool        set_promisc(bool enable)         = 0        ;
        virtual bool        initMemPool()                  = 0         ;    
        basic_para_type     get_basic_para()                           ;                                             
    protected:
        basic_para_type     m_basic_para                               ;
        dev_stats_type      m_dev_stats{0,0,0,0}                       ;
        VfioFd              m_fds{-1,-1,-1,-1}                         ;                                    
        interruptPara       m_interrupt_para                            ;
};
#endif // BASIC_DEV_H

