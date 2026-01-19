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



DMAMemoryPool::DMAMemoryPool(uint32_t num_bufs, uint32_t buf_size, int container_fd):
    m_num_bufs(num_bufs),
    m_buf_size(buf_size),
    m_container_fd(container_fd)
{
    v_free_stack.resize(num_bufs);
    _allocateMemory();
    _createPktBufRing();
    info("MemoryPool created");
}

DMAMemoryPool::~DMAMemoryPool(){
}

bool DMAMemoryPool::_allocateMemory(){
    if (m_container_fd<=0) {
        error("No valid container fd provided, DMA memory may not be IOMMU mapped");
        return false;
    }
    DMAMemoryAllocator& dma_allocator = DMAMemoryAllocator::getInstance();
    m_DMA_mem_pair = dma_allocator.allocDMAMemory(m_num_bufs * m_buf_size, m_container_fd);
    return true;
}

bool DMAMemoryPool::_createPktBufRing(){
    if (m_DMA_mem_pair.virt == nullptr) {
        error("memory not allocated yet");
        return false;
    }
    for (uint32_t idx = 0; idx < m_num_bufs; idx++) {
        v_free_stack[idx] = idx;
        // the start virtual address of this pkt_buf
        struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) m_DMA_mem_pair.virt) + idx * m_buf_size);
        // the offset is shared by virtual and physical address
        uintptr_t offset = (uintptr_t) (idx * m_buf_size);
        // iova has already bound to the virtual address in DMA memory allocator
        buf->iova = (uintptr_t) m_DMA_mem_pair.iova + offset;
        buf->idx = idx;
        buf->size = 0;
        buf->data = (uint8_t*) buf + sizeof(struct pkt_buf);
    }
    m_free_stack_top = m_num_bufs;
    return true;
}

uint32_t DMAMemoryPool::popOutMultiPktBuf(struct pkt_buf** v_p_bufs, uint32_t num_bufs){
    uint32_t actual_num = 0;
    if (num_bufs > m_free_stack_top) {
        num_bufs = m_free_stack_top;
    }
    for (uint32_t i = 0; i < num_bufs; i++) {
        struct pkt_buf* buf = popOutOnePktBufFromTop();
        if (!buf) {
            warn("Failed to take out pkt_buf");
            break;
        }
        v_p_bufs[i] = buf;
        actual_num++;
    }
    return actual_num;
}
// this function will reduce m_free_stack_top by 1
struct pkt_buf* DMAMemoryPool::popOutOnePktBufFromTop(){
    if (m_free_stack_top == 0) {
        // warn("no free pkt_buf available");
        return nullptr;
    }
    uint32_t idx = v_free_stack[--m_free_stack_top];
    struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) m_DMA_mem_pair.virt) + idx * m_buf_size);
    return buf;
}
// this function does not reduce m_free_stack_top
struct pkt_buf* DMAMemoryPool::getBuf(uint16_t idx){
    if (idx >= m_num_bufs) {
        warn("pkt_buf index %u out of range", idx);
        return nullptr;
    }
    struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) m_DMA_mem_pair.virt) + idx * m_buf_size);
    return buf;
}

void DMAMemoryPool::freePktBuf(struct pkt_buf* buf){
    if (m_free_stack_top >= m_num_bufs) {
        warn("freePktBuf: free stack overflow, possible double-free of buf idx %u", buf->idx);
        return;
    }
    v_free_stack[m_free_stack_top++] = buf->idx;
}


