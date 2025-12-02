#ifndef BASIC_DEV_H
#define BASIC_DEV_H
#include <cstdint>
#include <string>
#include <array>
#include <memory>
#include "interrupts.h"
typedef struct{
	std::string                 pci_addr;
    uint8_t                     bar_index;
	uint16_t                    num_rx_queues;
	uint16_t                    num_tx_queues;
    uint16_t                    interrupt_timeout_ms;
} basic_para_type;

typedef   
    std::array<uint8_t*,6>       bar_addr_type;

typedef
    std::unique_ptr<interrupt>    interrupt_ptr_type;

typedef struct{
    int                         container_fd;
    int                         group_id;
    int                         group_fd;
    int                         device_fd;
}vfio_fd_type;


class basic_dev{
    public:
        explicit basic_dev(const basic_para_type& basic_para);
        virtual         ~basic_dev()                   = default   ;
        virtual bool    initialize()                   = 0         ;
        virtual bool    map_bar ()                     = 0         ;
        virtual bool    enable_dma()                   = 0         ;                                         
        basic_para_type get_basic_para()                           ;                                             
    protected:
        basic_para_type m_basic_para                                ;
        bar_addr_type   p_bar_addr                                  ;    
};
#endif // BASIC_DEV_H

