#include "basic_dev.h"

BasicDev::BasicDev(                                                                          
                                std::string pci_addr,
                                uint16_t    num_rx_queues,
                                uint16_t    num_tx_queues
                            )

{
    m_basic_para.pci_addr        = pci_addr;
    m_basic_para.num_rx_queues   = num_rx_queues;
    m_basic_para.num_tx_queues   = num_tx_queues;
    m_basic_para.bar_index_max   = 1; // default max bar index
    m_basic_para.interrupt_timeout_ms = 100; // default interrupt timeout
    for (auto& addr : m_basic_para.p_bar_addr) {
        addr = nullptr;
    }
}

basic_para_type BasicDev::get_basic_para(){
    return m_basic_para;
}