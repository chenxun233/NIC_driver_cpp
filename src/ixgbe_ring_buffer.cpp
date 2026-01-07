#include "ixgbe_ring_buffer.h"
#include "device.h"
#include "log.h"

using namespace std;



bool IXGBE_RxRingBuffer::linkMemoryPool(DMAMemoryPool* const mem_pool){
	if (!mem_pool) {
		error("null memory pool provided to RX ring buffer");
		return false;
	}
	p_mem_pool = mem_pool;
	return true;
};

bool IXGBE_RxRingBuffer::bindDMAMemVirtWithDesc(){
	if (!m_DMA_mem_pair.virt) {
		error("invalid DMA memory provided to RX ring buffer for descriptor ring");
		return false;
	}
	_p_descriptors_start_addr = (union ixgbe_adv_rx_desc*) m_DMA_mem_pair.virt;
	
	return true;
};

bool IXGBE_RxRingBuffer::linkDescWithPKTBuf(){
	if (p_mem_pool == nullptr) {
		error("memory pool not linked, call linkMemoryPool first");
		return false;
	}
	if (_p_descriptors_start_addr == nullptr) {
		error("descriptor ring not linked to DMA memory, call bindDMAMemVirtWithDesc first");
		return false;
	}
	for (uint32_t i = 0; i < p_mem_pool->getNumOfBufs(); i++) {
		volatile union ixgbe_adv_rx_desc* rxd = _p_descriptors_start_addr + i;
		struct pkt_buf* buf = p_mem_pool->takeOutOnePktBuf();
		if (!buf) {
			error("failed to allocate rx descriptor");
			return false;
		}
		// where the data buffer is
		uintptr_t data_offset = (uintptr_t)(buf->data - (uint8_t*)buf);
		rxd->read.pkt_addr = buf->iova + data_offset;
		rxd->read.hdr_addr = 0;
		// we need to return the virtual address in the rx function which the descriptor doesn't know by default
	}
	return true;
};

bool IXGBE_RxRingBuffer::allocDMAMemory(uint8_t index, int container_fd){
		debug("initializing rx queue %d", index);
		// setup descriptor ring, see section 7.1.9
		uint32_t ring_size_bytes = p_mem_pool->getNumOfBufs() * sizeof(union ixgbe_adv_rx_desc);
		DMAMemoryPair DMA_mem_pair = DMAMemoryAllocator::getInstance().allocDMAMemory(ring_size_bytes, container_fd);
		memset(DMA_mem_pair.virt, -1, ring_size_bytes);
		m_DMA_mem_pair = DMA_mem_pair;
		
	return true;
};

bool IXGBE_RxRingBuffer::bindDMAMemIOVAWithNIC(uint8_t* BAR_addr, uint8_t index){
		// enable advanced rx descriptors, we could also get away with legacy descriptors, but they aren't really easier
		set_bar_reg32(BAR_addr, IXGBE_SRRCTL(index), (get_bar_reg32(BAR_addr, IXGBE_SRRCTL(index)) & ~IXGBE_SRRCTL_DESCTYPE_MASK) | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF);
		// drop_en causes the nic to drop packets if no rx descriptors are available instead of buffering them
		// a single overflowing queue can fill up the whole buffer and impact operations if not setting this flag
		set_bar_flags32(BAR_addr, IXGBE_SRRCTL(index), IXGBE_SRRCTL_DROP_EN);
		// tell the device where it can write to (its iova, so its view)
		// neat trick from Snabb: initialize to 0xFF to prevent rogue memory accesses on premature DMA activation
		set_bar_reg32(BAR_addr, IXGBE_RDBAL(index), (uint32_t) (m_DMA_mem_pair.iova & 0xFFFFFFFFull));
		set_bar_reg32(BAR_addr, IXGBE_RDBAH(index), (uint32_t) (m_DMA_mem_pair.iova >> 32));
		set_bar_reg32(BAR_addr, IXGBE_RDLEN(index), p_mem_pool->getNumOfBufs() * sizeof(union ixgbe_adv_rx_desc));
		// set ring to empty at start
		set_bar_reg32(BAR_addr, IXGBE_RDH(index), 0);
		set_bar_reg32(BAR_addr, IXGBE_RDT(index), 0);
		return true;
};

bool IXGBE_TxRingBuffer::linkMemoryPool(DMAMemoryPool* const mem_pool){
	if (!mem_pool) {
		error("null memory pool provided to RX ring buffer");
		return false;
	}
	p_mem_pool = mem_pool;
	
	return true;
};

bool IXGBE_TxRingBuffer::allocDMAMemory(uint8_t index, int container_fd){
		uint32_t ring_size_bytes = p_mem_pool->getNumOfBufs() * sizeof(union ixgbe_adv_tx_desc);
		DMAMemoryPair DMA_mem_pair = DMAMemoryAllocator::getInstance().allocDMAMemory(ring_size_bytes, container_fd);
		memset(DMA_mem_pair.virt, -1, ring_size_bytes);
		m_DMA_mem_pair = DMA_mem_pair;
		debug("Configured TX ring buffer %d with DMA memory IOVA 0x%llx and virtual address %p", index, (unsigned long long)DMA_mem_pair.iova, DMA_mem_pair.virt);
		return true;
}

bool IXGBE_TxRingBuffer::bindDMAMemIOVAWithNIC(uint8_t* BAR_addr, uint8_t index){
		// tell the device where it can write to (its iova, so its view)
		set_bar_reg32(BAR_addr, IXGBE_TDBAL(index), (uint32_t) (m_DMA_mem_pair.iova & 0xFFFFFFFFull));
		set_bar_reg32(BAR_addr, IXGBE_TDBAH(index), (uint32_t) (m_DMA_mem_pair.iova >> 32));
		set_bar_reg32(BAR_addr, IXGBE_TDLEN(index), p_mem_pool->getNumOfBufs() * sizeof(union ixgbe_adv_tx_desc));
		debug("tx ring %d phy addr:  0x%012lX", index, m_DMA_mem_pair.iova);
		debug("tx ring %d virt addr: 0x%012lX", index, (uintptr_t) m_DMA_mem_pair.virt);
		// descriptor writeback magic values, important to get good performance and low PCIe overhead
		// see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
		// we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
		uint32_t txdctl = get_bar_reg32(BAR_addr, IXGBE_TXDCTL(index));
		// there are no defines for this in ixgbe_type.h for some reason
		// pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
		txdctl &= ~(0x7F | (0x7F << 8) | (0x7F << 16)); // clear bits
		txdctl |= (36 | (8 << 8) | (4 << 16)); // from DPDK
		set_bar_reg32(BAR_addr, IXGBE_TXDCTL(index), txdctl);
		return true;
};

bool IXGBE_TxRingBuffer::bindDMAMemVirtWithDesc(){
	if (!m_DMA_mem_pair.virt) {
		error("invalid DMA memory provided to TX ring buffer for descriptor ring");
		return false;
	}
	_p_descriptors_start_addr = (union ixgbe_adv_tx_desc*) m_DMA_mem_pair.virt;
	return true;
};

bool IXGBE_TxRingBuffer::linkDescWithPKTBuf(){
	// if (p_mem_pool == nullptr) {
	// 	error("memory pool not linked, call linkMemoryPool first");
	// 	return false;
	// }
	// if (_p_descriptors_start_addr == nullptr) {
	// 	error("descriptor ring not linked to DMA memory, call bindDMAMemVirtWithDesc first");
	// 	return false;
	// }
	// for (uint32_t i = 0; i < p_mem_pool->getNumOfBufs(); i++) {
	// 	volatile union ixgbe_adv_tx_desc* rxd = _p_descriptors_start_addr + i;
	// 	struct pkt_buf* buf = p_mem_pool->takeOutOnePktBuf();
	// 	if (!buf) {
	// 		error("failed to allocate rx descriptor");
	// 		return false;
	// 	}
	// 	// where the data buffer is
	// 	uintptr_t data_offset = (uintptr_t)(buf->data - (uint8_t*)buf);
	// 	txd->read.buffer_addr = buf->iova + data_offset;
	// 	// always the same flags: one buffer (EOP), advanced data descriptor, CRC offload, data length
	// 	txd->read.cmd_type_len =
	// 		IXGBE_ADVTXD_DCMD_EOP | IXGBE_ADVTXD_DCMD_RS | IXGBE_ADVTXD_DCMD_IFCS | IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_DATA | buf->size;
	// 	// no fancy offloading stuff - only the total payload length
	// 	// implement offloading flags here:
	// 	// 	* ip checksum offloading is trivial: just set the offset
	// 	// 	* tcp/udp checksum offloading is more annoying, you have to precalculate the pseudo-header checksum
	// 	txd->read.olinfo_status = buf->size << IXGBE_ADVTXD_PAYLEN_SHIFT;
	// 	// we need to return the virtual address in the rx function which the descriptor doesn't know by default
	// }
	return true;
};