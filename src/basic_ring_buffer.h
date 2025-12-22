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
        virtual bool             linkDescWithPKTBuf() = 0;
    protected:
        MemoryPool*              p_mem_pool{nullptr}              ;
        std::vector<void*>       v_buf_addr               ;
        uint16_t                 m_rx_idx{0}                      ;

};

class TxRingBuffer{

    public:
        
        virtual ~TxRingBuffer() = default;
        virtual bool linkMemoryPool( MemoryPool* const mem_pool) = 0;
        virtual bool allocDMAMem2DescRing( const DMAMemoryPair& DMA_mem_pair) = 0;
        uint16_t getTxIndex() const { return m_tx_idx; }
        uint16_t getCleanIndex() const { return clean_index; }
        void    setTxIndex(const uint16_t& idx) { m_tx_idx = idx; }
        void    setCleanIndex(const uint16_t& idx) { clean_index = idx; }
    protected:
        MemoryPool*             p_mem_pool{nullptr}              ;
        std::vector<void*>      v_buf_addr                        ;
        uint16_t                m_tx_idx{0}                       ;
        uint16_t                clean_index{0}                    ;        
};