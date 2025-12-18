#include "ixgbe_ring_buffer.h"
#include "device.h"

using namespace std;

IXGBE_RingBuffer::IXGBE_RingBuffer(bool is_rx):
m_is_rx{is_rx}
{
    // Implementation for RxRingBuffer constructor
};


bool IXGBE_RingBuffer::linkMemoryPool(MemoryPool* const mem_pool){
	if (!m_is_rx) {
		info("TX ring buffer does not need memory pool");
		return true;
	}
	if (!mem_pool) {
		error("null memory pool provided to RX ring buffer");
		return false;
	}
	p_mem_pool = mem_pool;
	
	return true;
};

bool IXGBE_RingBuffer::allocDMAMem2DescRing(const DMAMemoryPair& DMA_mem_pair){
	if (!DMA_mem_pair.virt || DMA_mem_pair.size == 0) {
		error("invalid DMA memory provided to RX ring buffer for descriptor ring");
		return false;
	}
	p_descriptors = (union ixgbe_adv_rx_desc*) DMA_mem_pair.virt;
	_linkDescWithPKTBuf();
	return true;
};

bool IXGBE_RingBuffer::_linkDescWithPKTBuf(){
	if (!m_is_rx) {
		info("TX ring buffer does not need to prepare pkt buffer");
		return true;
	}
	if (p_mem_pool == nullptr) {
		error("memory pool not linked, call linkMemoryPool first");
		return false;
	}
	if (p_descriptors == nullptr) {
		error("descriptor ring not linked to DMA memory, call allocDMAMem2DescRing first");
		return false;
	}
	for (uint32_t i = 0; i < p_mem_pool->getNumOfBufs(); i++) {
		volatile union ixgbe_adv_rx_desc* rxd = p_descriptors + i;
		struct pkt_buf* buf = p_mem_pool->popOnePktBuf();
		if (!buf) {
			error("failed to allocate rx descriptor");
			return false;
		}
		// where the data buffer is
		rxd->read.pkt_addr = buf->phy_addr + offsetof(struct pkt_buf, data);
		rxd->read.hdr_addr = 0;
		// we need to return the virtual address in the rx function which the descriptor doesn't know by default
		v_buf_virtual_addr.push_back((void*) buf);
	}
	return true;
};
