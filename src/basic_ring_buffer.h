#pragma once
#include <cstdint>
#include "dma_memory_allocator.h"

class BasicRingBuffer{

    public:
        ~BasicRingBuffer() = default;
        virtual bool linkMemoryPool( MemoryPool* const mem_pool) = 0;

};