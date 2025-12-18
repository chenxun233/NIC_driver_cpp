#include "basic_dev.h"

BasicDev::BasicDev(std::string pci_addr,uint8_t max_bar_index):
m_basic_para()
{
    // initialize struct members in the constructor body
    m_basic_para.pci_addr = pci_addr;
    m_basic_para.num_rx_queues = 0;
    m_basic_para.num_tx_queues = 0;
    m_basic_para.max_bar_index = max_bar_index;
    m_basic_para.interrupt_timeout_ms = 100;
    for (auto& addr : m_basic_para.p_bar_addr) {
        addr = nullptr;
    }
}

basic_para_type BasicDev::get_basic_para(){
    return m_basic_para;
}