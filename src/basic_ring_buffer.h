#pragma once
#include <cstdint>
#include "dma_memory_allocator.h"
#include "memory_pool.h"


class RxRingBuffer{

    public:
        
        virtual ~RxRingBuffer() = default;
        virtual bool linkMemoryPool( DMAMemoryPool* const mem_pool) = 0;
        virtual bool allocDMAMemory(uint8_t index, int device_fd) = 0;
        virtual bool bindDMAMemIOVAWithNIC(uint8_t* BAR_addr, uint8_t index) = 0;
        virtual bool bindDMAMemVirtWithDesc()= 0;
    protected:
        virtual bool             linkDescWithPKTBuf() = 0;
    protected:
        DMAMemoryPool*           p_mem_pool{nullptr};
        std::vector<void*>       v_buf_addr         ;
        uint16_t                 m_rx_idx{0}        ;
        DMAMemoryPair            m_DMA_mem_pair{0,0,0};


};

class TxRingBuffer{

    public:
        virtual ~TxRingBuffer() = default;
        virtual bool linkMemoryPool( DMAMemoryPool* const mem_pool) = 0;
        virtual bool allocDMAMemory(uint8_t index, int device_fd) = 0;
        virtual bool bindDMAMemIOVAWithNIC(uint8_t* BAR_addr, uint8_t index) = 0;
        virtual bool bindDMAMemVirtWithDesc() = 0;
        uint16_t getTxIndex() const { return m_tx_idx; }
        uint16_t getCleanIndex() const { return clean_index; }
        void    setTxIndex(const uint16_t& idx) { m_tx_idx = idx; }
        void    setCleanIndex(const uint16_t& idx) { clean_index = idx; }
    protected:
        DMAMemoryPool*          p_mem_pool{nullptr};
        std::vector<void*>      v_buf_addr         ;
        uint16_t                m_tx_idx{0}        ;
        uint16_t                clean_index{0}     ;  
        DMAMemoryPair           m_DMA_mem_pair{0,0,0};      
};