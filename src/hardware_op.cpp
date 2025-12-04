#include "device.h"
#include "ixgbe_type.h"
#include "log.h"
#include "hardware_op.h"
#include "memory_op.h"

using namespace memory_op;

hardware_op::hardware_op(
    basic_para_type& basic_para,
    vfio_fd_type& fds,
	dev_stats_type& stats
):
m_para{basic_para, fds, stats}
{

}


bool hardware_op::dev_reset_n_init(){
	info("Resetting device...");

	// section 4.6.3.1 - disable all interrupts
	this->_dev_disable_IRQ();
	this->_dev_rst_hardware();
	usleep(10000);
	// section 4.6.3.1 - disable interrupts again after reset
	this->_dev_disable_IRQ();
	this->_get_mac_address();
    this->_init_eeprom_n_dma();

	// section 4.6.4 - initialize link (auto negotiation)
	this->_init_link_nego();
	// section 4.6.5 - statistical counters
	// reset-on-read registers, just read them once
	dev_stats_type dev = {};
	this->_read_stats(dev);

	// section 4.6.7 - init rx
	this->_init_rx();

	// section 4.6.8 - init tx
	this->_init_tx();

	// enables queues after initializing everything
	for (uint16_t i = 0; i < dev->ixy.num_rx_queues; i++) {
		start_rx_queue(dev, i);
	}
	for (uint16_t i = 0; i < dev->ixy.num_tx_queues; i++) {
		start_tx_queue(dev, i);
	}

	// enable interrupts
	for (uint16_t queue = 0; queue < dev->ixy.num_rx_queues; queue++) {
		enable_interrupt(dev, queue);
	}

	// finally, enable promisc mode by default, it makes testing less annoying
	ixgbe_set_promisc(&dev->ixy, true);

	// wait for some time for the link to come up
	wait_for_link(dev);
	return true;
}

bool hardware_op::_dev_disable_IRQ(){
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EIMS, 0x00000000);
	_dev_clear_interrupts();
	return true;
}

bool hardware_op::_dev_clear_interrupts(){
	// Clear interrupt mask
	// Clear interrupt mask to stop from interrupts being generated
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);
	get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EICR);
	return true;
}

bool hardware_op::_dev_rst_hardware(){
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
	wait_clear_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
	return true;
}

bool hardware_op::_get_mac_address(){
	mac_address_type mac;
	uint32_t rar_low = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RAL(0));
	uint32_t rar_high = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RAH(0));

	mac.addr[0] = rar_low;
	mac.addr[1] = rar_low >> 8;
	mac.addr[2] = rar_low >> 16;
	mac.addr[3] = rar_low >> 24;
	mac.addr[4] = rar_high;
	mac.addr[5] = rar_high >> 8;
    m_para.basic.mac_address = mac;
    return true;
}

bool hardware_op::_init_eeprom_n_dma(){
	// section 4.6.3 - Wait for EEPROM auto read completion
	wait_set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EEC, IXGBE_EEC_ARD);
	// section 4.6.3 - Wait for DMA initialization done (RDRXCTL.DMAIDONE)
	wait_set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RDRXCTL, IXGBE_RDRXCTL_DMAIDONE);
    return true;
}
bool hardware_op::_init_link_nego(){
	// should already be set by the eeprom config, maybe we shouldn't override it here to support weirdo nics?
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_AUTOC, (get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_AUTOC) & ~IXGBE_AUTOC_LMS_MASK) | IXGBE_AUTOC_LMS_10G_SERIAL);
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_AUTOC, (get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_AUTOC) & ~IXGBE_AUTOC_10G_PMA_PMD_MASK) | IXGBE_AUTOC_10G_XAUI);
	// negotiate link
	set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_AUTOC, IXGBE_AUTOC_AN_RESTART);
	// datasheet wants us to wait for the link here, but we can continue and wait afterwards
}

bool hardware_op::_read_stats(dev_stats_type& stats){
	uint32_t rx_pkts = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_GPRC);
	uint32_t tx_pkts = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_GPTC);
	uint64_t rx_bytes = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_GORCL) + (((uint64_t) get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_GORCH)) << 32);
	uint64_t tx_bytes = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_GOTCL) + (((uint64_t) get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_GOTCH)) << 32);

	stats.rx_pkts += rx_pkts;
	stats.tx_pkts += tx_pkts;
	stats.rx_bytes += rx_bytes;
	stats.tx_bytes += tx_bytes;
	return true;
}

bool hardware_op::_init_rx(){
	// make sure that rx is disabled while re-configuring it
	// the datasheet also wants us to disable some crypto-offloading related rx paths (but we don't care about them)
	clear_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
	// no fancy dcb or vt, just a single 128kb packet buffer for us
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RXPBSIZE(0), IXGBE_RXPBSIZE_128KB);
	for (int i = 1; i < 8; i++) {
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RXPBSIZE(i), 0);
	}

	// always enable CRC offloading
	set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_HLREG0, IXGBE_HLREG0_RXCRCSTRP);
	set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_RDRXCTL, IXGBE_RDRXCTL_CRCSTRIP);

	// accept broadcast packets
	set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_FCTRL, IXGBE_FCTRL_BAM);

	// per-queue config, same for all queues
	for (uint16_t i = 0; i < m_para.basic.num_rx_queues; i++) {
		debug("initializing rx queue %d", i);
		// enable advanced rx descriptors, we could also get away with legacy descriptors, but they aren't really easier
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_SRRCTL(i), (get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_SRRCTL(i)) & ~IXGBE_SRRCTL_DESCTYPE_MASK) | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF);
		// drop_en causes the nic to drop packets if no rx descriptors are available instead of buffering them
		// a single overflowing queue can fill up the whole buffer and impact operations if not setting this flag
		set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_SRRCTL(i), IXGBE_SRRCTL_DROP_EN);
		// setup descriptor ring, see section 7.1.9
		uint32_t ring_size_bytes = NUM_RX_QUEUE_ENTRIES * sizeof(union ixgbe_adv_rx_desc);
		struct dma_memory_type mem = memory_allocate_dma(ring_size_bytes, true);
		// neat trick from Snabb: initialize to 0xFF to prevent rogue memory accesses on premature DMA activation
		memset(mem.virt, -1, ring_size_bytes);
		// tell the device where it can write to (its iova, so its view)
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RDBAL(i), (uint32_t) (mem.phy & 0xFFFFFFFFull));
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RDBAH(i), (uint32_t) (mem.phy >> 32));
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RDLEN(i), ring_size_bytes);
		debug("rx ring %d phy addr:  0x%012lX", i, mem.phy);
		debug("rx ring %d virt addr: 0x%012lX", i, (uintptr_t) mem.virt);
		// set ring to empty at start
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RDH(i), 0);
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RDT(i), 0);
		// private data for the driver, 0-initialized
		struct rx_ringbuffer_type* queue = ((struct rx_ringbuffer_type*)(m_queues.rx)) + i;
		queue->num_entries = NUM_RX_QUEUE_ENTRIES;
		queue->rx_index = 0;
		queue->descriptors = (union ixgbe_adv_rx_desc*) mem.virt;
	}

	// last step is to set some magic bits mentioned in the last sentence in 4.6.7
	set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_CTRL_EXT, IXGBE_CTRL_EXT_NS_DIS);
	// this flag probably refers to a broken feature: it's reserved and initialized as '1' but it must be set to '0'
	// there isn't even a constant in ixgbe_types.h for this flag
	for (uint16_t i = 0; i < m_para.basic.num_rx_queues; i++) {
		clear_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_DCA_RXCTRL(i), 1 << 12);
	}

	// start RX
	set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
	return true;
}