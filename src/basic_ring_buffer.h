#pragma once
#include <cstdint>
#include "dma_memory_allocator.h"
#include "memory_pool.h"
#ifndef wrap_ring
#define wrap_ring(index, ring_size) (uint16_t) ((index + 1) & (ring_size - 1))
#endif


class RingBuffer{
    public:
        virtual         ~RingBuffer() = default;
        virtual bool    linkMemoryPool( DMAMemoryPool* const mem_pool) = 0;
        bool            createDescriptorRing(int container_fd, uint8_t* BAR_addr,uint32_t num_desc, uint32_t size_desc, uint8_t ring_index);
        virtual bool    linkPKTBufToDesc (pkt_buf* buf, uint16_t desc_idx)  = 0;
        uint16_t        getDescTailIdx() const { return m_desc_tail; }
        uint16_t        getDescHeadIdx() const { return m_desc_head; }
        void            setDescTailIdx(uint16_t idx) { m_desc_tail = idx; }
        void            setDescHeadIdx(uint16_t idx) { if (idx > p_mem_pool->getNumOfBufs()) idx-=p_mem_pool->getNumOfBufs(); m_desc_head = idx; }
        bool            setUsedBufAddr      (pkt_buf* used_buf_addr) { if(m_used_buf_addr_offset>=p_mem_pool->getNumOfBufs()) return false; p_used_buf_addr[m_used_buf_addr_offset++] = used_buf_addr;return true; }
        pkt_buf*        getUsedBufAddr      (){if (m_used_buf_addr_offset == 0) return nullptr ;return p_used_buf_addr[--m_used_buf_addr_offset]; }
        bool            setLinkedBufAddr    (pkt_buf* used_buf_addr) { if(m_linked_buf_addr_offset>=p_mem_pool->getNumOfBufs()) return false; p_linked_buf_addr[m_linked_buf_addr_offset++] = used_buf_addr;return true; }
        pkt_buf*        getLinkedBufAddr    (){if (m_linked_buf_addr_offset == 0) return nullptr ;return p_linked_buf_addr[--m_linked_buf_addr_offset]; }

    protected:
        bool            _allocDescMemory(int container_fd, uint32_t num_desc, uint32_t size_desc);
        virtual bool    _bindDescMemIOVA(uint8_t* BAR_addr, uint8_t ring_index) = 0;
        virtual bool    _bindDescMemVirt() = 0;
    protected:
        uint32_t        m_size_desc{0}   ;
        uint32_t        m_num_desc{0}    ;
        DMAMemoryPool*  p_mem_pool{nullptr};
        DMAMemoryPair   m_desc_mem_pair{0,0,0};  
        uint16_t        m_desc_tail{0}        ;
        uint16_t        m_desc_head{0}        ;
        pkt_buf**       p_used_buf_addr{nullptr};    
        uint32_t        m_used_buf_addr_offset{0};
        pkt_buf**       p_linked_buf_addr{nullptr};
        uint32_t        m_linked_buf_addr_offset{0};

};

