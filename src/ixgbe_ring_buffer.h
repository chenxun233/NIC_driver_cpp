#pragma once
#include "memory_pool.h"
#include "basic_ring_buffer.h"
#include <memory>
#include "ixgbe_type.h"
#include <vector>


// a ring buffer is one queue
class IXGBE_RingBuffer:public BasicRingBuffer {
    public:
    IXGBE_RingBuffer(bool is_rx)                                                         ;
    bool linkMemoryPool    (MemoryPool* const mem_pool)  override                       ;
    bool linkDescriptor2DMAMemory(const DMAMemoryPair& mem)                              ;
    bool preparePktBuffer()                                                              ; 

    private:
        bool                                            m_is_rx{true}                    ;
        MemoryPool*                                     p_mem_pool{nullptr}              ;
        std::vector<void*>                              p_buf_virtual_addr               ;
        volatile union ixgbe_adv_rx_desc*               p_descriptors                    ;

};
