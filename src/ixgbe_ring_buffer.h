#pragma once
#include "memory_pool.h"
#include "basic_ring_buffer.h"
#include <memory>
#include "ixgbe_type.h"
#include <vector>


// a ring buffer is one queue
class IXGBE_RingBuffer:public BasicRingBuffer {
    public:
    IXGBE_RingBuffer                (bool is_rx)                                         ;
    bool linkMemoryPool             (MemoryPool* const mem_pool)  override               ;
    bool allocDMAMem2DescRing       (const DMAMemoryPair& DMA_mem_pair)                  ;


    private:
        bool _linkDescWithPKTBuf()                                                       ; 
        bool                                            m_is_rx{true}                    ;
        MemoryPool*                                     p_mem_pool{nullptr}              ;
        std::vector<void*>                              v_buf_virtual_addr               ;
        volatile union ixgbe_adv_rx_desc*               p_descriptors                    ;

};
