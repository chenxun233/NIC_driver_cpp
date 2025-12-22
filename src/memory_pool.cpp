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
    v_p_used_buf_addr.resize(num_bufs);
    _allocateMemory();
    _initEachPktBuf();
}

bool MemoryPool::_allocateMemory(){
    if (m_container_fd<=0) {
        error("No valid container fd provided, DMA memory may not be IOMMU mapped");
        return false;
    }
    DMAMemoryAllocator& dma_allocator = DMAMemoryAllocator::getInstance();
    m_DMA_mem_pair = dma_allocator.allocDMAMemory(m_total_size, m_container_fd);
    return true;
}

bool MemoryPool::_initEachPktBuf(){
    if (m_DMA_mem_pair.virt == nullptr) {
        error("memory not allocated yet");
        return false;
    }
    for (uint32_t idx = 0; idx < m_num_bufs; idx++) {
        m_free_stack[idx] = idx;
        // the start virtual address of this pkt_buf
        struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) m_DMA_mem_pair.virt) + idx * m_buf_size);
        // the offset is shared by virtual and physical address
        uintptr_t offset = (uintptr_t) (idx * m_buf_size);
        buf->iova = (uintptr_t) m_DMA_mem_pair.iova + offset;
        buf->idx = idx;
        buf->size = 0;
        buf->data = (uint8_t*) buf + sizeof(struct pkt_buf);
    }
    m_free_stack_top = m_num_bufs;
    return true;
}

uint32_t MemoryPool::takePktBuf(struct pkt_buf** v_p_bufs, uint32_t num_bufs){
    uint32_t actual_num = 0;
    if (num_bufs > m_free_stack_top) {
        num_bufs = m_free_stack_top;
    }
    for (uint32_t i = 0; i < num_bufs; i++) {
        struct pkt_buf* buf = takeOutPktBuf();
        if (!buf) {
            warn("Failed to take out pkt_buf");
            break;
        }
        v_p_bufs[i] = buf;
        actual_num++;
    }
    return actual_num;
}

struct pkt_buf* MemoryPool::takeOutPktBuf(){
    if (m_free_stack_top == 0) {
        warn("no free pkt_buf available");
        return nullptr;
    }
    uint32_t idx = m_free_stack[--m_free_stack_top];
    struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) m_DMA_mem_pair.virt) + idx * m_buf_size);
    return buf;
}

void MemoryPool::pushBackPktBuf(struct pkt_buf* buf){
    m_free_stack[m_free_stack_top++] = buf->idx;
}


