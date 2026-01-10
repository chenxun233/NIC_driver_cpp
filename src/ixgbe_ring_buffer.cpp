#include "ixgbe_ring_buffer.h"
#include "device.h"
#include "log.h"
#define wrap_ring(index, ring_size) (uint16_t) ((index + 1) & (ring_size - 1))
using namespace std;




bool IXGBE_RxRingBuffer::_bindDescMemVirt(){
	if (!m_desc_mem_pair.virt) {
		error("invalid DMA memory provided to RX ring buffer for descriptor ring");
		return false;
	}
	p_desc_ring_start = (union ixgbe_adv_rx_desc*) m_desc_mem_pair.virt;
	
	return true;
};

bool IXGBE_RxRingBuffer::linkMemoryPool(DMAMemoryPool* const mem_pool){
	p_mem_pool = mem_pool;
	m_num_buf = mem_pool->getNumOfBufs();
	if (!p_mem_pool) return false;
	return true;
};

bool IXGBE_RxRingBuffer::linkPKTBufToDesc(pkt_buf* buf, uint16_t desc_idx){
	if (p_mem_pool == nullptr) {
		error("memory pool not linked, call linkMemoryPool first");
		return false;
	}
	if (p_desc_ring_start == nullptr) {
		error("descriptor ring not linked to DMA memory, call bindDMAMemVirtWithDesc first");
		return false;
	}
		volatile union ixgbe_adv_rx_desc* rxd = p_desc_ring_start + desc_idx;
		if (!buf) {
			error("failed to allocate rx descriptor");
			return false;
		}
		// where the data buffer is
		uintptr_t data_offset = (uintptr_t)(buf->data - (uint8_t*)buf);
		rxd->read.pkt_addr = buf->iova + data_offset;
		rxd->read.hdr_addr = 0;
		// we need to return the virtual address in the rx function which the descriptor doesn't know by default
	
	return true;
};



bool IXGBE_RxRingBuffer::_bindDescMemIOVA(uint8_t* BAR_addr, uint8_t ring_index){
		// enable advanced rx descriptors, we could also get away with legacy descriptors, but they aren't really easier
		set_bar_reg32(BAR_addr, IXGBE_SRRCTL(ring_index), (get_bar_reg32(BAR_addr, IXGBE_SRRCTL(ring_index)) & ~IXGBE_SRRCTL_DESCTYPE_MASK) | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF);
		// drop_en causes the nic to drop packets if no rx descriptors are available instead of buffering them
		// a single overflowing queue can fill up the whole buffer and impact operations if not setting this flag
		set_bar_flags32(BAR_addr, IXGBE_SRRCTL(ring_index), IXGBE_SRRCTL_DROP_EN);
		// tell the device where it can write to (its iova, so its view)
		// neat trick from Snabb: initialize to 0xFF to prevent rogue memory accesses on premature DMA activation
		set_bar_reg32(BAR_addr, IXGBE_RDBAL(ring_index), (uint32_t) (m_desc_mem_pair.iova & 0xFFFFFFFFull));
		set_bar_reg32(BAR_addr, IXGBE_RDBAH(ring_index), (uint32_t) (m_desc_mem_pair.iova >> 32));
		set_bar_reg32(BAR_addr, IXGBE_RDLEN(ring_index), m_num_desc * m_size_desc);
		// set ring to empty at start
		set_bar_reg32(BAR_addr, IXGBE_RDH(ring_index), 0);
		set_bar_reg32(BAR_addr, IXGBE_RDT(ring_index), 0);
		return true;
};

IXGBE_TxRingBuffer::IXGBE_TxRingBuffer(){
}

IXGBE_TxRingBuffer::~IXGBE_TxRingBuffer(){
	if (p_used_buf_addr){
		delete[] p_used_buf_addr;
		p_used_buf_addr = nullptr;
	}
	if (p_linked_buf_addr){
		delete[] p_linked_buf_addr;
		p_linked_buf_addr = nullptr;
	}
};

bool IXGBE_TxRingBuffer::linkMemoryPool(DMAMemoryPool* const mem_pool){
	p_mem_pool = mem_pool;
	m_num_buf = mem_pool->getNumOfBufs();
	if (!p_mem_pool) return false;
	p_used_buf_addr = new pkt_buf*[m_num_buf];
	p_linked_buf_addr = new pkt_buf*[m_num_buf];
	return true;
}


bool IXGBE_TxRingBuffer::_bindDescMemIOVA(uint8_t* BAR_addr, uint8_t ring_index){
		// tell the device where it can write to (its iova, so its view)
		set_bar_reg32(BAR_addr, IXGBE_TDBAL(ring_index), (uint32_t) (m_desc_mem_pair.iova & 0xFFFFFFFFull));
		set_bar_reg32(BAR_addr, IXGBE_TDBAH(ring_index), (uint32_t) (m_desc_mem_pair.iova >> 32));
		set_bar_reg32(BAR_addr, IXGBE_TDLEN(ring_index), m_num_desc * m_size_desc);
		// descriptor writeback magic values, important to get good performance and low PCIe overhead
		// see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
		// we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
		uint32_t txdctl = get_bar_reg32(BAR_addr, IXGBE_TXDCTL(ring_index));
		// there are no defines for this in ixgbe_type.h for some reason
		// pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
		txdctl &= ~(0x7F | (0x7F << 8) | (0x7F << 16)); // clear bits
		txdctl |= (36 | (8 << 8) | (4 << 16)); // from DPDK
		set_bar_reg32(BAR_addr, IXGBE_TXDCTL(ring_index), txdctl);
		return true;
};

bool IXGBE_TxRingBuffer::_bindDescMemVirt(){
	if (!m_desc_mem_pair.virt) {
		error("invalid DMA memory provided to TX ring buffer for descriptor ring");
		return false;
	}
	p_desc_ring_start = (union ixgbe_adv_tx_desc*) m_desc_mem_pair.virt;
	return true;
};


uint16_t IXGBE_TxRingBuffer::linkPktBufWithDesc(){
	struct pkt_buf* buf = getUsedBufAddr();
	
	while (buf) {
		uint16_t next_index = wrap_ring(m_desc_tail, m_num_desc);
		if (next_index == m_desc_head) {
			// ring full, return buffer to pool (can't push back to FIFO front)
			p_mem_pool->freePktBuf(buf);
			// Also return any remaining buffers in the used queue back to pool
			while ((buf = getUsedBufAddr()) != nullptr) {
				p_mem_pool->freePktBuf(buf);
			}
			return m_desc_tail;
		}
		// Track buffer AFTER confirming ring has space
		setLinkedBufAddr(buf);
		volatile union ixgbe_adv_tx_desc* txd = p_desc_ring_start + m_desc_tail;
		
		// NIC reads from here
		uintptr_t data_offset = (uintptr_t)(buf->data - (uint8_t*) buf);
		txd->read.buffer_addr = buf->iova + data_offset;
		// always the same flags: one buffer (EOP), advanced data descriptor, CRC offload, data length
		txd->read.cmd_type_len =
			IXGBE_ADVTXD_DCMD_EOP | IXGBE_ADVTXD_DCMD_RS | IXGBE_ADVTXD_DCMD_IFCS | IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_DATA | buf->size;
		// no fancy offloading stuff - only the total payload length
		// implement offloading flags here:
		// 	* ip checksum offloading is trivial: just set the offset
		// 	* tcp/udp checksum offloading is more annoying, you have to precalculate the pseudo-header checksum
		txd->read.olinfo_status = buf->size << IXGBE_ADVTXD_PAYLEN_SHIFT;
		m_desc_tail = next_index;
		buf = getUsedBufAddr();
	}
	return m_desc_tail;
}

bool IXGBE_TxRingBuffer::linkPKTBufToDesc(pkt_buf* buf, uint16_t desc_idx){
	if (p_mem_pool == nullptr) {
		error("memory pool not linked, call linkMemoryPool first");
		return false;
	}
	if (p_desc_ring_start == nullptr) {
		error("descriptor ring not linked to DMA memory, call bindDMAMemVirtWithDesc first");
		return false;
	}
	if (!buf) {
		error("failed to allocate rx descriptor");
		return false;
	}
	volatile union ixgbe_adv_tx_desc* txd = p_desc_ring_start + desc_idx;
	uintptr_t data_offset = (uintptr_t)(buf->data - (uint8_t*)buf);
		txd->read.buffer_addr = buf->iova + data_offset;
		// always the same flags: one buffer (EOP), advanced data descriptor, CRC offload, data length
		txd->read.cmd_type_len =
			IXGBE_ADVTXD_DCMD_EOP | IXGBE_ADVTXD_DCMD_RS | IXGBE_ADVTXD_DCMD_IFCS | IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_DATA | buf->size;
		// no fancy offloading stuff - only the total payload length
		// implement offloading flags here:
		// 	* ip checksum offloading is trivial: just set the offset
		// 	* tcp/udp checksum offloading is more annoying, you have to precalculate the pseudo-header checksum
		txd->read.olinfo_status = buf->size << IXGBE_ADVTXD_PAYLEN_SHIFT;
	return true;

}

uint16_t IXGBE_TxRingBuffer::_calcIPChecksum(const uint8_t* data, uint32_t size) {
	if (size % 1) error("odd-sized checksums NYI"); // we don't need that
	uint32_t cs = 0;
	for (uint32_t i = 0; i < size / 2; i++) {
		cs += ((uint16_t*)data)[i];
		if (cs > 0xFFFF) {
			cs = (cs & 0xFFFF) + 1; // 16 bit one's complement
		}
	}
	return ~((uint16_t) cs);
}
// it will automatically forward to a free pkt_buf from the mempool
bool IXGBE_TxRingBuffer::fillPktBuf (const char* data, uint32_t size) {
	struct pkt_buf* buf = p_mem_pool->popOutOnePktBufFromTop();
	if (!buf) {
		// error("failed to allocate pkt_buf from mempool");
		return false;
	}
	if (size > p_mem_pool->getBufSize() - sizeof(struct pkt_buf)) {
		warn("data size %u exceeds pkt_buf capacity %zu, truncating",
		     size,
		     p_mem_pool->getBufSize() - sizeof(struct pkt_buf));
		size = p_mem_pool->getBufSize() - sizeof(struct pkt_buf);
	}
	memcpy(buf->data, data, size);
	buf->size = size;
	*(uint16_t*) (buf->data + 24) = _calcIPChecksum(buf->data + 14, 20);
	if (setUsedBufAddr(buf) == false) {
		p_mem_pool->freePktBuf(buf);
		error("failed to set used buf addr");
		return false;
	}
	return true;
}

bool IXGBE_TxRingBuffer::freeUsedBuf() {
	struct pkt_buf* buf = getUsedBufAddr();
	if (!buf) {
		warn("no used buf to free");
		return true;
	}
	p_mem_pool->freePktBuf(buf);
	return true;
}

bool IXGBE_TxRingBuffer::cleanDescriptorRing(uint16_t min_clean_num){
	if (!p_desc_ring_start || !p_mem_pool) {
		error("TX ring not initialized");
		return false;
	}
	int16_t cleanable = m_desc_tail - m_desc_head;
	if (cleanable < 0) { // handle wrap-around
		cleanable = m_num_desc + cleanable;
	}

	if (cleanable < min_clean_num) {
		return false;
	}

	uint16_t cleanup_to = m_desc_head + min_clean_num - 1;
	if (cleanup_to >= m_num_desc) {
		cleanup_to -= m_num_desc;
	}
	volatile union ixgbe_adv_tx_desc* txd = p_desc_ring_start + cleanup_to;
	uint32_t status = txd->wb.status;
	// only clean if the last descriptor in the batch is done
	if (!(status & IXGBE_ADVTXD_STAT_DD)) {
		return false;
	}

	// Clean exactly min_clean_num descriptors and their corresponding buffers
	for (uint16_t cleaned = 0; cleaned < min_clean_num; cleaned++) {
		struct pkt_buf* buf = getLinkedBufAddr();
		if (buf) {
			p_mem_pool->freePktBuf(buf);
		}
	}
	m_desc_head = wrap_ring(cleanup_to, m_num_desc);
	return true;
}
