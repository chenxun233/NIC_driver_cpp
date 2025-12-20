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
        bool linkMemoryPool         (MemoryPool* const mem_pool)  override               ;
        bool allocDMAMem2DescRing   (const DMAMemoryPair& DMA_mem_pair)    override      ;
    private:
        bool _linkDescWithPKTBuf    ()               override                            ; 
        volatile union ixgbe_adv_rx_desc*               _p_descriptors                    ;
};


class IXGBE_TxRingBuffer:public TxRingBuffer {
    public:
        IXGBE_TxRingBuffer          (){}                                               ;
        ~IXGBE_TxRingBuffer         (){}                                               ;
        bool allocDMAMem2DescRing   (const DMAMemoryPair& DMA_mem_pair)    override      ;
    private:
        volatile union ixgbe_adv_rx_desc*               _p_descriptors                    ;
};