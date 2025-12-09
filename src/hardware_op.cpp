#include "device.h"
#include "ixgbe_type.h"
#include "log.h"
#include "hardware_op.h"

#include <sys/ioctl.h>
#include <linux/vfio.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>

const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;

hardware_op::hardware_op(
    basic_para_type& basic_para,
    VfioFd& fds,
	dev_stats_type& stats
):
m_para{basic_para, fds,0x028,nullptr,0, stats}
{
	_initialize_interrupt();
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

	this->_prepare_rx_queue();
	this->_prepare_tx_queue();

	this->_enable_interrupt();
	

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
	return true;
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
		uint32_t ring_size_bytes = NUM_OF_BUF_RX_QUEUE * sizeof(union ixgbe_adv_rx_desc);
		DmaMemoryPair mem = memory_allocate_dma(this->m_para.fds, ring_size_bytes, true);
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
		RxRingBuffer* queue = ((RxRingBuffer*)(m_queues.rx)) + i;
		queue->total_num_of_buf = NUM_OF_BUF_RX_QUEUE;
		queue->rx_index = 0;
		//descriptor shares the same memory as virt
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

bool hardware_op::_init_tx(){
	// crc offload and small packet padding
	set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_HLREG0, IXGBE_HLREG0_TXCRCEN | IXGBE_HLREG0_TXPADEN);

	// set default buffer size allocations
	// see also: section 4.6.11.3.4, no fancy features like DCB and VTd
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TXPBSIZE(0), IXGBE_TXPBSIZE_40KB);
	for (int i = 1; i < 8; i++) {
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TXPBSIZE(i), 0);
	}
	// required when not using DCB/VTd
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_DTXMXSZRQ, 0xFFFF);
	clear_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_RTTDCS, IXGBE_RTTDCS_ARBDIS);

	// per-queue config for all queues
	for (uint16_t i = 0; i < m_para.basic.num_tx_queues; i++) {
		debug("initializing tx queue %d", i);

		// setup descriptor ring, see section 7.1.9
		uint32_t ring_size_bytes = NUM_OF_BUF_TX_QUEUE * sizeof(union ixgbe_adv_tx_desc);
		DmaMemoryPair mem = memory_allocate_dma(this->m_para.fds,ring_size_bytes, true);
		memset(mem.virt, -1, ring_size_bytes);
		// tell the device where it can write to (its iova, so its view)
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TDBAL(i), (uint32_t) (mem.phy & 0xFFFFFFFFull));
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TDBAH(i), (uint32_t) (mem.phy >> 32));
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TDLEN(i), ring_size_bytes);
		debug("tx ring %d phy addr:  0x%012lX", i, mem.phy);
		debug("tx ring %d virt addr: 0x%012lX", i, (uintptr_t) mem.virt);

		// descriptor writeback magic values, important to get good performance and low PCIe overhead
		// see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
		// we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
		uint32_t txdctl = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TXDCTL(i));
		// there are no defines for this in ixgbe_type.h for some reason
		// pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
		txdctl &= ~(0x7F | (0x7F << 8) | (0x7F << 16)); // clear bits
		txdctl |= (36 | (8 << 8) | (4 << 16)); // from DPDK
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TXDCTL(i), txdctl);

		// private data for the driver, 0-initialized
		TxRingBuffer* queue = ((TxRingBuffer*)(m_queues.tx)) + i;
		queue->total_num_of_buf = NUM_OF_BUF_TX_QUEUE;
		queue->descriptors = (union ixgbe_adv_tx_desc*) mem.virt;
	}
	// final step: enable DMA
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_DMATXCTL, IXGBE_DMATXCTL_TE);

}
bool hardware_op::_prepare_rx_queue(){
	for (uint16_t queue_id = 0; queue_id < m_para.basic.num_rx_queues; queue_id++){
		debug("starting rx queue %d", queue_id);
		struct RxRingBuffer* queue = ((struct RxRingBuffer*)(m_queues.rx)) + queue_id;
		// 2048 as pktbuf size is strictly speaking incorrect:
		// we need a few headers (1 cacheline), so there's only 1984 bytes left for the device
		// but the 82599 can only handle sizes in increments of 1 kb; but this is fine since our max packet size
		// is the default MTU of 1518
		// this has to be fixed if jumbo frames are to be supported
		// mempool should be >= the number of rx and tx descriptors for a forwarding application
		int num_of_buf_mempool = NUM_OF_BUF_RX_QUEUE + NUM_OF_BUF_TX_QUEUE;
		// mempool and the pkt buffers inside are all created.
		queue->mempool = memory_allocate_mempool(this->m_para.fds,num_of_buf_mempool < MIN_NUM_OF_BUF ? MIN_NUM_OF_BUF : num_of_buf_mempool, PKT_BUF_SIZE);
		if (queue->total_num_of_buf & (queue->total_num_of_buf - 1)) {
			error("number of queue entries must be a power of 2");
		}
		for (int i = 0; i < queue->total_num_of_buf; i++) {
			volatile union ixgbe_adv_rx_desc* rxd = queue->descriptors + i;
			struct pkt_buf* buf = pkt_buf_alloc(queue->mempool);
			if (!buf) {
				error("failed to allocate rx descriptor");
			}
			// where the data buffer is
			rxd->read.pkt_addr = buf->phy_addr + offsetof(struct pkt_buf, data);
			rxd->read.hdr_addr = 0;
			// we need to return the virtual address in the rx function which the descriptor doesn't know by default
			queue->buf_virtual_addr[i] = buf;
		}
		// enable queue and wait if necessary
		set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_RXDCTL(queue_id), IXGBE_RXDCTL_ENABLE);
		wait_set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RXDCTL(queue_id), IXGBE_RXDCTL_ENABLE);
		// rx queue starts out full
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RDH(queue_id), 0);
		// was set to 0 before in the init function
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_RDT(queue_id), queue->total_num_of_buf - 1);
		// Implementation of RX queue preparation
	}
	return true;
}

bool hardware_op::_prepare_tx_queue(){
	for (uint16_t queue_id = 0; queue_id < m_para.basic.num_tx_queues; queue_id++){
		debug("starting tx queue %d", queue_id);
		struct TxRingBuffer* queue = ((struct TxRingBuffer*)(m_queues.tx)) + queue_id;
		if (queue->total_num_of_buf & (queue->total_num_of_buf - 1)) {
			error("number of queue entries must be a power of 2");
		}
		// tx queue starts out empty
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TDH(queue_id), 0);
		set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TDT(queue_id), 0);
		// enable queue and wait if necessary
		set_bar_flags32(m_para.basic.p_bar_addr[0], IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
		wait_set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
		// Implementation of TX queue preparation
	return true;
	}
}
void hardware_op::_enable_msi_interrupt(uint16_t queue_id){
	// Step 1: The software driver associates between Tx and Rx interrupt causes and the EICR
	// register by setting the IVAR[n] registers.
	set_ivar(m_para.basic.p_bar_addr[0], 0, queue_id, 0);

	// Step 2: Program SRRCTL[n].RDMTS (per receive queue) if software uses the receive
	// descriptor minimum threshold interrupt
	// We don't use the minimum threshold interrupt

	// Step 3: All interrupts should be set to 0b (no auto clear in the EIAC register). Following an
	// interrupt, software might read the EICR register to check for the interrupt causes.
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EIAC, 0x00000000);

	// Step 4: Set the auto mask in the EIAM register according to the preferred mode of operation.
	// In our case we prefer not auto-masking the interrupts

	// Step 5: Set the interrupt throttling in EITR[n] and GPIE according to the preferred mode of operation.
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EITR(queue_id), m_para.itr_rate);

	// Step 6: Software clears EICR by writing all ones to clear old interrupt causes
	_dev_clear_interrupts();

	// Step 7: Software enables the required interrupt causes by setting the EIMS register
	u32 mask = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EIMS);
	mask |= (1 << queue_id);
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EIMS, mask);
	debug("Using MSI interrupts");
}

void hardware_op::_enable_msix_interrupt(uint16_t queue_id){
	// Step 1: The software driver associates between interrupt causes and MSI-X vectors and the
	// throttling timers EITR[n] by programming the IVAR[n] and IVAR_MISC registers.
	uint32_t gpie = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_GPIE);
	gpie |= IXGBE_GPIE_MSIX_MODE | IXGBE_GPIE_PBA_SUPPORT | IXGBE_GPIE_EIAME;
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_GPIE, gpie);
	set_ivar(m_para.basic.p_bar_addr[0], 0, queue_id, queue_id);

	// Step 2: Program SRRCTL[n].RDMTS (per receive queue) if software uses the receive
	// descriptor minimum threshold interrupt
	// We don't use the minimum threshold interrupt

	// Step 3: The EIAC[n] registers should be set to auto clear for transmit and receive interrupt
	// causes (for best performance). The EIAC bits that control the other and TCP timer
	// interrupt causes should be set to 0b (no auto clear).
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EIAC, IXGBE_EIMS_RTX_QUEUE);

	// Step 4: Set the auto mask in the EIAM register according to the preferred mode of operation.
	// In our case we prefer to not auto-mask the interrupts

	// Step 5: Set the interrupt throttling in EITR[n] and GPIE according to the preferred mode of operation.
	// 0x000 (0us) => ... INT/s
	// 0x008 (2us) => 488200 INT/s
	// 0x010 (4us) => 244000 INT/s
	// 0x028 (10us) => 97600 INT/s
	// 0x0C8 (50us) => 20000 INT/s
	// 0x190 (100us) => 9766 INT/s
	// 0x320 (200us) => 4880 INT/s
	// 0x4B0 (300us) => 3255 INT/s
	// 0x640 (400us) => 2441 INT/s
	// 0x7D0 (500us) => 2000 INT/s
	// 0x960 (600us) => 1630 INT/s
	// 0xAF0 (700us) => 1400 INT/s
	// 0xC80 (800us) => 1220 INT/s
	// 0xE10 (900us) => 1080 INT/s
	// 0xFA7 (1000us) => 980 INT/s
	// 0xFFF (1024us) => 950 INT/s
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EITR(queue_id), m_para.itr_rate);

	// Step 6: Software enables the required interrupt causes by setting the EIMS register
	u32 mask = get_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EIMS);
	mask |= (1 << queue_id);
	set_bar_reg32(m_para.basic.p_bar_addr[0], IXGBE_EIMS, mask);
	debug("Using MSIX interrupts");
}

void hardware_op::_enable_interrupt(){
	for (uint16_t queue_id = 0; queue_id < m_para.basic.num_rx_queues; queue_id++)
	{
		if (!m_para.interrupt_queues[queue_id].interrupt_enabled) {
		return;
	}
	switch (m_para.interrupt_type) {
		case VFIO_PCI_MSIX_IRQ_INDEX:
			_enable_msix_interrupt(queue_id);
			break;
		case VFIO_PCI_MSI_IRQ_INDEX:
			_enable_msi_interrupt(queue_id);
			break;
		default:
			warn("Interrupt type not supported: %d", m_para.interrupt_type);
			return;
	}
	}
}


bool hardware_op::_initialize_interrupt(){
	return
	this->_host_setup_IRQ_type()			&&
	this->_host_alloc_IRQ_queues()			&&
	this->_host_setup_IRQ_queues()		;
}

bool hardware_op::_host_setup_IRQ_type(){
	if (!m_para.fds.device_fd) {
		return false;
	}
	info("Setup VFIO Interrupts");

	for (int i = VFIO_PCI_MSIX_IRQ_INDEX; i >= 0; i--) {
		struct vfio_irq_info irq = {};
        irq.argsz = sizeof(irq);
        irq.index = i;
		ioctl(m_para.fds.device_fd, VFIO_DEVICE_GET_IRQ_INFO, &irq);
		/* if this vector cannot be used with eventfd continue with next*/
		if ((irq.flags & VFIO_IRQ_INFO_EVENTFD) == 0) {
			debug("IRQ doesn't support Event FD");
			continue;
		}
		this->m_para.interrupt_type = i;
        debug("Using IRQ type %d with %d vectors", i, irq.count);
        return true;
	}
    return false;
}
bool hardware_op::_host_alloc_IRQ_queues(){
	this->m_para.interrupt_queues = std::make_unique<interrupt_queues[]>(this->m_para.basic.num_rx_queues);
	return true;
}
int hardware_op::_vfio_enable_msi(){
	info("Enable MSI Interrupts");
	char irq_set_buf[IRQ_SET_BUF_LEN];
	struct vfio_irq_set* irq_set;
	int* fd_ptr;

	// setup event fd
	int event_fd = eventfd(0, 0);

	irq_set = reinterpret_cast<struct vfio_irq_set*>(irq_set_buf);
	irq_set->argsz = sizeof(irq_set_buf);
	irq_set->count = 1;
	irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSI_IRQ_INDEX;
	irq_set->start = 0;
	fd_ptr = reinterpret_cast<int*>(&irq_set->data);
	*fd_ptr = event_fd;
	int ret = ioctl(m_para.fds.device_fd, VFIO_DEVICE_SET_IRQS, irq_set);
	if (ret < 0 )
	{
		error("Failed to set MSIX IRQS");
		return -1;
	}
	return event_fd;
}

int hardware_op::_vfio_enable_msix(int index){
	info("Enable MSIX Interrupts");
	char irq_set_buf[MSIX_IRQ_SET_BUF_LEN];
	struct vfio_irq_set* irq_set;
	int* fd_ptr;

	// setup event fd
	int event_fd = eventfd(0, 0);

	irq_set = reinterpret_cast<struct vfio_irq_set*>(irq_set_buf);
	irq_set->argsz = sizeof(irq_set_buf);

	if (!index) {
		index = 1;
	} else if (index > MAX_INTERRUPT_VECTORS)
		index = MAX_INTERRUPT_VECTORS + 1;

	irq_set->count = index;
	irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSIX_IRQ_INDEX;
	irq_set->start = 0;
	fd_ptr = reinterpret_cast<int*>(&irq_set->data);
	*fd_ptr = event_fd;

	int ret = ioctl(m_para.fds.device_fd, VFIO_DEVICE_SET_IRQS, irq_set);
	if (ret < 0) {
		error("Failed to set MSIX IRQS");
		return -1;
	}
	return event_fd;
}

int hardware_op::_vfio_epoll_ctl(int event_fd){
	struct epoll_event event;
	event.events = EPOLLIN;
	event.data.fd = event_fd;

	int epoll_fd = (int) check_err(epoll_create1(0), "to created epoll");

	int ret = epoll_ctl(epoll_fd, EPOLL_CTL_ADD, event_fd, &event);
	if (ret < 0) {
		error("Failed to add event fd to epoll instance");
		return -1;
	}
	return epoll_fd;
}

bool hardware_op::_host_setup_IRQ_queues(){
	switch (m_para.interrupt_type) {	
		case VFIO_PCI_MSIX_IRQ_INDEX: {
			for (uint32_t rx_queue = 0; rx_queue < m_para.basic.num_rx_queues; rx_queue++) {
				int vfio_event_fd = _vfio_enable_msix(rx_queue);
				int vfio_epoll_fd = _vfio_epoll_ctl(vfio_event_fd);
				m_para.interrupt_queues[rx_queue].vfio_event_fd = vfio_event_fd;
				m_para.interrupt_queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				m_para.interrupt_queues[rx_queue].moving_avg.length = 0;
				m_para.interrupt_queues[rx_queue].moving_avg.index = 0;
				m_para.interrupt_queues[rx_queue].interval = INTERRUPT_INITIAL_INTERVAL;
			}
			break;
		}
		case VFIO_PCI_MSI_IRQ_INDEX: {
			int vfio_event_fd = _vfio_enable_msi();
			int vfio_epoll_fd = _vfio_epoll_ctl(vfio_event_fd);
			for (uint32_t rx_queue = 0; rx_queue < m_para.basic.num_rx_queues; rx_queue++) {
				m_para.interrupt_queues[rx_queue].vfio_event_fd = vfio_event_fd;
				m_para.interrupt_queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				m_para.interrupt_queues[rx_queue].moving_avg.length = 0;
				m_para.interrupt_queues[rx_queue].moving_avg.index = 0;
				m_para.interrupt_queues[rx_queue].interval = INTERRUPT_INITIAL_INTERVAL;
			}
			break;
		}
		default:
			warn("Interrupt type not supported: %d", m_para.interrupt_type);
			return false;
	}
	return true;
}


