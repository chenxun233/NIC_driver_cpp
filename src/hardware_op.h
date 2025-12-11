#ifndef DEV_OPERATION_H
#define DEV_OPERATION_H
#include <stdint.h>
#include <unistd.h>
#include "basic_dev.h"
#include "memory_op.h"

constexpr int NUM_OF_BUF_RX_QUEUE = 512;
constexpr int NUM_OF_BUF_TX_QUEUE = 512;

#define MOVING_AVERAGE_RANGE 5
#define IRQ_SET_BUF_LEN (sizeof(struct vfio_irq_set) + sizeof(int))
#define MAX_INTERRUPT_VECTORS 32
#define MSIX_IRQ_SET_BUF_LEN (sizeof(struct vfio_irq_set) + sizeof(int) * (MAX_INTERRUPT_VECTORS + 1))



struct QueuesPtr {
    void*                   rx;
    void*                   tx;
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

struct FullPara {
    basic_para_type&                         basic                          ;
    struct VfioFd                               fds                            ;
    uint32_t                                 itr_rate{0x028}                ;
    std::unique_ptr<InterruptQueue[]>      interrupt_queues{nullptr}      ;
    uint8_t                                  interrupt_type{0}              ;
    dev_stats_type&                          stats                          ;
};


class hardware_op {
    public:
        hardware_op(basic_para_type& basic_para,::VfioFd& fds,dev_stats_type& stats);
        ~hardware_op();
        bool                                    dev_reset_n_init()                  ;
    private:
        bool                                    _dev_disable_IRQ()                  ;
        bool                                    _dev_clear_interrupts()             ;
        bool                                    _dev_rst_hardware()                 ;
        bool                                    _get_mac_address()                  ;
        bool                                    _init_eeprom_n_dma()                ;
        bool                                    _init_link_nego()                   ;
        bool                                    _read_stats(dev_stats_type& stats)  ;               
        bool                                    _init_rx()                          ;
        bool                                    _init_tx()                          ;
        bool                                    _prepare_rx_queue()                 ;
        bool                                    _prepare_tx_queue()                 ;
        void                                    _enable_msi_interrupt(uint16_t queue_id);
        void                                    _enable_msix_interrupt(uint16_t queue_id);
        bool                                    _enable_interrupt()                 ;
        bool                                    _set_promisc(bool enable)           ;
        bool                                    _wait_for_link()                    ;
        uint32_t                                _get_link_speed()                   ;
    private:
        FullPara                                m_para                              ;      
        QueuesPtr                               m_queues                            ;

    private:
        bool                                    _initialize_interrupt()             ;
        bool                                    _host_setup_IRQ_type()              ;
        bool                                    _host_alloc_IRQ_queues()            ;
        bool                                    _host_setup_IRQ_queues()            ;

    private:
        int                                     _vfio_enable_msi()                  ;
        int                                     _vfio_enable_msix(int index)        ;
        int                                     _vfio_epoll_ctl(int event_fd)       ;
  

};

#endif // DEV_OPERATION_H