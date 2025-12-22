#pragma once
#include "memory_pool.h"
#include "basic_ring_buffer.h"
#include <memory>
#include "ixgbe_type.h"
#include <vector>



class IXGBE_RxRingBuffer:public RxRingBuffer {
    public:
        IXGBE_RxRingBuffer          (){}                                               ;
        ~IXGBE_RxRingBuffer         (){}                                               ;
        bool linkMemoryPool         (MemoryPool* const mem_pool)  override             ;
        bool allocDMAMem2DescRing   (const DMAMemoryPair& DMA_mem_pair)    override    ;
        bool linkDescWithPKTBuf    ()               override                           ;
        MemoryPool* getMemPool() const { return p_mem_pool; } 
    private:
        volatile union ixgbe_adv_rx_desc*               _p_descriptors                    ;
};


class IXGBE_TxRingBuffer:public TxRingBuffer {
    public:
        IXGBE_TxRingBuffer          (){}                                               ;
        ~IXGBE_TxRingBuffer         (){}                                               ;
        bool linkMemoryPool         (MemoryPool* const mem_pool)  override               ;
        bool allocDMAMem2DescRing   (const DMAMemoryPair& DMA_mem_pair)    override      ;
        MemoryPool* getMemPool() const { return p_mem_pool; }
        volatile union ixgbe_adv_tx_desc* getDescriptors() const { return _p_descriptors; }
    private:
        volatile union ixgbe_adv_tx_desc*               _p_descriptors                    ;

};