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
    m_free_stack_top(num_bufs),
    m_container_fd(container_fd)
{
    m_free_stack.resize(num_bufs);
}

bool MemoryPool::allocateMemory(){
    // allocate memory via DMA allocator
    // Here we use a simple method to allocate memory, which may be improved later.
    // For example, we can use a singleton DMA memory allocator to manage all DMA memory allocations.
    DMAMemoryAllocator& dma_allocator = DMAMemoryAllocator::getInstance();
    DmaMemoryPair mem = dma_allocator.allocDMAMemory(m_total_size, m_container_fd);
    p_base_virtual_addr = mem.virt;
    m_base_io_virtual_addr = mem.phy;
    if (!p_base_virtual_addr) {
        error("failed to allocate DMA memory for MemoryPool");
        return false;
    }
    return true;
}

bool MemoryPool::initEachPktBuf(){
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


