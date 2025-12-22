#include "ixgbe_ring_buffer.h"
#include "device.h"

using namespace std;



bool IXGBE_RxRingBuffer::linkMemoryPool(MemoryPool* const mem_pool){
	if (!mem_pool) {
		error("null memory pool provided to RX ring buffer");
		return false;
	}
	p_mem_pool = mem_pool;
	
	return true;
};

bool IXGBE_RxRingBuffer::allocDMAMem2DescRing(const DMAMemoryPair& DMA_mem_pair){
	if (!DMA_mem_pair.virt || DMA_mem_pair.size == 0) {
		error("invalid DMA memory provided to RX ring buffer for descriptor ring");
		return false;
	}
	_p_descriptors = (union ixgbe_adv_rx_desc*) DMA_mem_pair.virt;
	
	return true;
};

bool IXGBE_RxRingBuffer::linkDescWithPKTBuf(){
	if (p_mem_pool == nullptr) {
		error("memory pool not linked, call linkMemoryPool first");
		return false;
	}
	if (_p_descriptors == nullptr) {
		error("descriptor ring not linked to DMA memory, call allocDMAMem2DescRing first");
		return false;
	}
	for (uint32_t i = 0; i < p_mem_pool->getNumOfBufs(); i++) {
		volatile union ixgbe_adv_rx_desc* rxd = _p_descriptors + i;
		struct pkt_buf* buf = p_mem_pool->takeOutPktBuf();
		if (!buf) {
			error("failed to allocate rx descriptor");
			return false;
		}
		// where the data buffer is
		uintptr_t data_offset = (uintptr_t)(buf->data - (uint8_t*)buf);
		rxd->read.pkt_addr = buf->iova + data_offset;
		rxd->read.hdr_addr = 0;
		// we need to return the virtual address in the rx function which the descriptor doesn't know by default
		v_buf_addr.push_back((void*) buf);
	}
	return true;
};
bool IXGBE_TxRingBuffer::linkMemoryPool(MemoryPool* const mem_pool){
	if (!mem_pool) {
		error("null memory pool provided to RX ring buffer");
		return false;
	}
	p_mem_pool = mem_pool;
	
	return true;
};

bool IXGBE_TxRingBuffer::allocDMAMem2DescRing(const DMAMemoryPair& DMA_mem_pair){
	if (!DMA_mem_pair.virt || DMA_mem_pair.size == 0) {
		error("invalid DMA memory provided to RX ring buffer for descriptor ring");
		return false;
	}
	_p_descriptors = (union ixgbe_adv_tx_desc*) DMA_mem_pair.virt;
	return true;
};
