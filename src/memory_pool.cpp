#include "memory_pool.h"
#include <stddef.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <linux/mman.h>
#include <unistd.h>
#include "log.h"
#include <sys/ioctl.h>
#include <linux/vfio.h>
#include "dma_memory_allocator.h"



MemoryPool::MemoryPool(uint32_t num_bufs, uint32_t buf_size, int container_fd):
    m_num_bufs(num_bufs),
    m_buf_size(buf_size),
    m_total_size(static_cast<uint64_t>(num_bufs) * static_cast<uint64_t>(buf_size)),
    m_container_fd(container_fd)
{
    m_free_stack.resize(num_bufs);
    _allocateMemory();
    _initEachPktBuf();
}

bool MemoryPool::_allocateMemory(){
    DMAMemoryAllocator& dma_allocator = DMAMemoryAllocator::getInstance();
    DMAMemoryPair DMA_mem_pair = dma_allocator.allocDMAMemory(m_total_size, m_container_fd);
    p_base_virtual_addr = DMA_mem_pair.virt;
    m_base_io_virtual_addr = DMA_mem_pair.phy;
    if (!p_base_virtual_addr) {
        error("failed to allocate DMA memory for MemoryPool");
        return false;
    }
    return true;
}

bool MemoryPool::_initEachPktBuf(){
    if (!p_base_virtual_addr) {
        error("memory not allocated yet");
        return false;
    }
    for (uint32_t idx = 0; idx < m_num_bufs; idx++) {
        m_free_stack[idx] = idx;
        // the start virtual address of this pkt_buf
        struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) p_base_virtual_addr) + idx * m_buf_size);
        // the offset is shared by virtual and physical address
        uintptr_t offset = (uintptr_t) (idx * m_buf_size);
        buf->phy_addr = (uintptr_t) m_base_io_virtual_addr + offset;
        buf->idx = idx;
        buf->size = 0;
        buf->data = (uint8_t*) buf + sizeof(struct pkt_buf);
    }
    m_free_stack_top = m_num_bufs;
    return true;
}

struct pkt_buf* MemoryPool::popOnePktBuf(){
    if (m_free_stack_top == 0) {
        warn("no free pkt_buf available");
        return nullptr;
    }
    uint32_t idx = m_free_stack[--m_free_stack_top];
    struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) p_base_virtual_addr) + idx * m_buf_size);
    return buf;
}

void MemoryPool::freeOnePktBuf(struct pkt_buf* buf){
    m_free_stack[m_free_stack_top++] = buf->idx;
}


