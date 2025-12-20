#pragma once
#include <cstdint>
#include "dma_memory_allocator.h"
#include "memory_pool.h"


class RxRingBuffer{

    public:
        
        virtual ~RxRingBuffer() = default;
        virtual bool linkMemoryPool( MemoryPool* const mem_pool) = 0;
        virtual bool allocDMAMem2DescRing( const DMAMemoryPair& DMA_mem_pair) = 0;
    protected:
        virtual bool _linkDescWithPKTBuf() = 0;
    protected:
        MemoryPool*                                     _p_mem_pool{nullptr}              ;
        std::vector<void*>                              _v_buf_virtual_addr               ;

};

class TxRingBuffer{

    public:
        
        virtual ~TxRingBuffer() = default;
        virtual bool allocDMAMem2DescRing( const DMAMemoryPair& DMA_mem_pair) = 0;
    protected:
        std::vector<void*>                              _v_buf_virtual_addr               ;

};