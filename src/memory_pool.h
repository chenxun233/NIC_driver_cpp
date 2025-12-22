#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include "dma_memory_allocator.h"
#define SIZE_PKT_BUF_HEADROOM 40

struct pkt_buf {
	// physical address to pass a buffer to a nic
	uintptr_t iova;
    // index of this pkt_buf in the mempool
	uint32_t idx;
    // actual size of the data in the buffer, initialized to 0
	uint32_t size;
	uint8_t head_room[SIZE_PKT_BUF_HEADROOM];
	uint8_t* data __attribute__((aligned(64)));
};


class MemoryPool{

    public:
        MemoryPool(uint32_t num_buf, uint32_t buf_size, int container_fd = -1);
        ~MemoryPool();
        struct pkt_buf*             takeOutPktBuf();
        uint32_t                    takePktBuf(struct pkt_buf** v_p_bufs, uint32_t num_bufs);
        void                        pushBackPktBuf(struct pkt_buf* buf);
        uint32_t                    getNumOfBufs() const     { return m_num_bufs; }
        uint32_t                    getBufSize()   const     { return m_buf_size; }
        void*                       getUsedBufAddr(uint32_t index) const { return index>m_num_bufs ? nullptr : v_p_used_buf_addr[index]; }
        bool                        setUsedBufAddr(uint32_t index, void* addr) {
                                                                                if (index>m_num_bufs) return false;
                                                                                v_p_used_buf_addr[index] = addr; return true; }
        uintptr_t                   getBaseIOVirtualAddr() const { return m_DMA_mem_pair.iova; }
    private:
        bool                        _allocateMemory();
        bool                        _initEachPktBuf();
        uint32_t                    m_num_bufs{0}                   ;
        uint32_t                    m_buf_size{0}                   ;
        uint64_t                    m_total_size{0}                 ;
        uint32_t                    m_free_stack_top{0}             ;
        int                         m_container_fd{-1}              ;
        std::vector<void*>          v_p_used_buf_addr               ;        
        std::vector<uint32_t>       m_free_stack                    ;
        DMAMemoryPair               m_DMA_mem_pair                  ; 

};