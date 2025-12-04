#ifndef DEV_OPERATION_H
#define DEV_OPERATION_H
#include <stdint.h>
#include <unistd.h>
#include "basic_dev.h"

constexpr int NUM_RX_QUEUE_ENTRIES = 512;
constexpr int NUM_TX_QUEUE_ENTRIES = 512;

struct basic_and_fd_type {
    basic_para_type&          basic;
    vfio_fd_type&             fds;
    dev_stats_type&           stats;
};

struct queues_ptr_type {
    void*                   rx;
    void*                   tx;
};

class hardware_op {
    public:
        hardware_op(basic_para_type& basic_para,vfio_fd_type& fds,dev_stats_type& stats);
        ~hardware_op();
        bool                                    dev_reset_n_init()                  ;
    private:
        bool                                    _dev_disable_IRQ()                  ;
        bool                                    _dev_clear_interrupts()             ;
        bool                                    _dev_rst_hardware()                 ;
        bool                                    _get_mac_address()                  ;
        bool                                    _init_eeprom_n_dma()                ;
        bool                                    _init_link_nego()                   ;
        bool                                    _read_stats(dev_stats_type& stats)  ;                 ;
        bool                                    _init_rx()                          ;
        bool                                    _init_tx()                          ;
    private:
        basic_and_fd_type                       m_para                              ;
        queues_ptr_type                         m_queues                            ;

        

};

#endif // DEV_OPERATION_H