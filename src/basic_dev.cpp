#include "basic_dev.h"
static double diff_mpps(uint64_t pkts_new, uint64_t pkts_old, uint64_t nanos) {
	return (double) (pkts_new - pkts_old) / 1000000.0 / ((double) nanos / 1000000000.0);
}


static uint32_t diff_mbit(uint64_t bytes_new, uint64_t bytes_old, uint64_t pkts_new, uint64_t pkts_old, uint64_t nanos) {
	// take stuff on the wire into account, i.e., the preamble, SFD and IFG (20 bytes)
	// otherwise it won't show up as 10000 mbit/s with small packets which is confusing
	return (uint32_t) (((bytes_new - bytes_old) / 1000000.0 / ((double) nanos / 1000000000.0)) * 8
		+ diff_mpps(pkts_new, pkts_old, nanos) * 20 * 8);
}

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


uint64_t BasicDev::_monotonic_time(){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000 + static_cast<uint64_t>(ts.tv_nsec);
}

basic_para_type BasicDev::get_basic_para(){
    return m_basic_para;
}


void BasicDev::_print_stats_diff(DevStatus* stats_new, DevStatus* stats_old, uint64_t nanos){
	printf("[%s] RX: %d Mbit/s %.2f Mpps\n", m_basic_para.pci_addr.c_str(),
		diff_mbit(stats_new->rx_bytes, stats_old->rx_bytes, stats_new->rx_pkts, stats_old->rx_pkts, nanos),
		diff_mpps(stats_new->rx_pkts, stats_old->rx_pkts, nanos)
	);
	printf("[%s] TX: %d Mbit/s %.2f Mpps\n", m_basic_para.pci_addr.c_str(),
		diff_mbit(stats_new->tx_bytes, stats_old->tx_bytes, stats_new->tx_pkts, stats_old->tx_pkts, nanos),
		diff_mpps(stats_new->tx_pkts, stats_old->tx_pkts, nanos)
	);

}