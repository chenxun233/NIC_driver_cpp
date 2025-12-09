#include "basic_dev.h"

BasicDev::BasicDev(
                    std::string pci_addr,
                    uint8_t     bar_index_max,
	                uint16_t    num_rx_queues,
	                uint16_t    num_tx_queues,
                    uint16_t    interrupt_timeout_ms
):
m_dev_stats({})
{
    if (pci_addr.empty()){
        throw std::invalid_argument("PCI address cannot be empty");
    };
    if( bar_index_max > 5) {
        throw std::out_of_range("BAR index must be between 0 and 5");
    };
    if(num_rx_queues <=0 || num_tx_queues <=0){
        throw std::invalid_argument("Number of RX and TX queues must be greater than 0");
    };
    m_basic_para.pci_addr                   = pci_addr;
    m_basic_para.bar_index_max              = bar_index_max;
    m_basic_para.num_rx_queues              = num_rx_queues;
    m_basic_para.num_tx_queues              = num_tx_queues;
    m_basic_para.interrupt_timeout_ms       = interrupt_timeout_ms;
    m_basic_para.p_bar_addr.fill(nullptr);
    m_basic_para.mac_address = {};
    
};


basic_para_type BasicDev::get_basic_para(){
    return m_basic_para;
}