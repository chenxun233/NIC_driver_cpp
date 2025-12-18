#include <cstdint>
#include "dma_memory_allocator.h"

class BasicRingBuffer{

    public:
        ~BasicRingBuffer() = default;
        virtual bool link2MemoryPool( MemoryPool* const mem_pool) = 0;

};