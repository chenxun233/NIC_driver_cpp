#ifndef INTERRUPTS_H
#define INTERRUPTS_H
#include <cstdint>
#include <memory>
#define MOVING_AVERAGE_RANGE 5
#define IRQ_SET_BUF_LEN (sizeof(struct vfio_irq_set) + sizeof(int))
#define MAX_INTERRUPT_VECTORS 32
#define MSIX_IRQ_SET_BUF_LEN (sizeof(struct vfio_irq_set) + sizeof(int) * (MAX_INTERRUPT_VECTORS + 1))

struct interrupt_moving_avg {
	uint32_t index; // The current index
	uint32_t length; // The moving average length
	uint64_t sum; // The moving average sum
	uint64_t measured_rates[MOVING_AVERAGE_RANGE]; // The moving average window
};

struct interrupt_queues {
	int vfio_event_fd; // event fd
	int vfio_epoll_fd; // epoll fd
	bool interrupt_enabled; // Whether interrupt for this queue is enabled or not
	uint64_t last_time_checked; // Last time the interrupt flag was checked
	uint64_t instr_counter; // Instruction counter to avoid unnecessary calls to monotonic_time
	uint64_t rx_pkts; // The number of received packets since the last check
	uint64_t interval; // The interval to check the interrupt flag
	struct interrupt_moving_avg moving_avg; // The moving average of the hybrid interrupt
};


class interrupt {
    public:
        interrupt(
            int device_fd,int 
            interrupt_timeout_ms,
            uint16_t rx_queue_num,
            uint16_t tx_queue_num);
        ~interrupt();

    private:
        bool                                    _initialize()                       ;
        bool                                    _get_interrupt_type()               ;
        bool                                    _alloc_interrupt_queues()           ;
        int                                     _vfio_enable_msi()                  ;
        int                                     _vfio_enable_msix(int index)        ;
        int                                     _vfio_epoll_ctl(int event_fd)                   ;
        bool                                    _setup_interrupts_queues()          ;
    private:
        int                                     m_device_fd                         ; 
        int                                     m_interrupt_timeout_ms              ;
        uint32_t                                m_itr_rate                          ;
        uint16_t                                m_rx_queue_num                      ;
        uint16_t                                m_tx_queue_num                      ;
        std::unique_ptr<interrupt_queues[]>     p_interrupt_queues                  ;
        uint8_t                                 m_interrupt_type                    ;

};
#endif // INTERRUPTS_H