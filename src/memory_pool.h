#pragma once
#include <cstdint>
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


class DMAMemoryPool{

    public:
        DMAMemoryPool(uint32_t num_buf, uint32_t buf_size, int container_fd = -1);
        ~DMAMemoryPool();
        struct pkt_buf*             popOutOnePktBufFromTop();
        uint32_t                    popOutMultiPktBuf(struct pkt_buf** v_p_bufs, uint32_t num_bufs);
        void                        freePktBuf(struct pkt_buf* buf);
        struct pkt_buf*             getBuf(uint16_t idx);
        uint32_t                    getNumOfBufs() const     { return m_num_bufs; }
        uint32_t                    getBufSize()   const     { return m_buf_size; }
                                                       
    private:
        bool                        _allocateMemory();
        bool                        _createPktBufRing();
        uint32_t                    m_num_bufs{0};
        uint32_t                    m_buf_size{0};
        uint32_t                    m_free_stack_top{0};
        int                         m_container_fd{-1} ;   
        std::vector<uint32_t>       v_free_stack;
        DMAMemoryPair               m_DMA_mem_pair; 

};