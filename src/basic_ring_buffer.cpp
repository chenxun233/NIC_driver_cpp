#include "basic_ring_buffer.h"
#include "dma_memory_allocator.h"
#include "log.h"
#include "ixgbe_type.h"
#include <cstring>
//This function allocate DMA memory for descriptors, whose number of elements is as same as the linked memory pool
bool RingBuffer::_allocDescMemory(int container_fd, uint32_t num_desc, uint32_t size_desc){
    if (p_mem_pool == nullptr) {
        error("No memory pool linked yet");
        return false;
    }
	uint32_t total_size = num_desc * size_desc;
	DMAMemoryPair desc_mem_pair = DMAMemoryAllocator::getInstance().allocDMAMemory(total_size, container_fd);
	memset(desc_mem_pair.virt, -1, total_size);
	m_desc_mem_pair = desc_mem_pair;
	return true;

}




bool RingBuffer::createDescriptorRing(int container_fd, uint8_t* BAR_addr, uint32_t num_desc, uint32_t size_desc, uint8_t ring_index){
	m_num_desc = num_desc;
	m_size_desc = size_desc;
	this->_allocDescMemory(container_fd, num_desc, size_desc);
	this->_bindDescMemIOVA(BAR_addr, ring_index);
	this->_bindDescMemVirt();
	if (!a_linked_buf_addr) {
		a_linked_buf_addr = new pkt_buf*[m_num_desc]();
	}
	return true;
}
