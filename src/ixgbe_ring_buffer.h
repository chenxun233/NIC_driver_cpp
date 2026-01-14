#pragma once
#include "memory_pool.h"
#include "basic_ring_buffer.h"
#include "ixgbe_type.h"



class IXGBE_RxRingBuffer:public RingBuffer {
    public:
                        IXGBE_RxRingBuffer (){};
                        ~IXGBE_RxRingBuffer(){};
        bool            linkMemoryPool           ( DMAMemoryPool* const mem_pool) override;
        uint16_t        fillDescRing        (uint16_t batch_size);
        uint16_t        readDescriptors(uint16_t batch_size, struct pkt_buf** bufs);
        void            releasePktBufs(struct pkt_buf** bufs, uint16_t num_bufs){
                                                                                    for (uint16_t i = 0; i < num_bufs; i++) {
                                                                                        if (bufs[i])
                                                                                            p_mem_pool->freePktBuf(bufs[i]);
                                                                                    }
                                                                                }
        int             vfio_epoll_wait(int epoll_fd, uint16_t timeout);
        DMAMemoryPool*  getMemPool   () const { return p_mem_pool; } 
    private:
        bool            _bindDescMemIOVA          (uint8_t* BAR_addr, uint8_t index) override;
        bool            _bindDescMemVirt          () override    ;
        volatile union ixgbe_adv_rx_desc*               p_desc_ring_start;
};


class IXGBE_TxRingBuffer:public RingBuffer {
    public:
                        IXGBE_TxRingBuffer      ();
                        ~IXGBE_TxRingBuffer     ();
        bool            linkMemoryPool         ( DMAMemoryPool* const mem_pool) override;
        uint16_t        linkPktWithDesc     (uint16_t batch_size);
        bool            fillPktBuf              (const char* data, uint32_t size);
        bool            cleanDescriptorRing     (uint16_t min_clean_num);
        DMAMemoryPool*  getMemPool              () const { return p_mem_pool; }

        bool            setUsedBufAddr      (pkt_buf* buf) {
                                                                uint32_t next_tail = wrap_ring(m_used_buf_tail, m_num_buf);
                                                                if (next_tail == m_used_buf_head) return false;  // Queue full
                                                                a_used_buf_addr[m_used_buf_tail] = buf;
                                                                m_used_buf_tail = next_tail;
                                                                return true;
                                                            }
        pkt_buf*        getUsedBufAddr      () {
                                                    if (m_used_buf_head == m_used_buf_tail) return nullptr;  // Queue empty
                                                    pkt_buf* buf = a_used_buf_addr[m_used_buf_head];
                                                    m_used_buf_head = wrap_ring(m_used_buf_head, m_num_buf);
                                                    return buf;
                                                }
    private:
        bool            _bindDescMemIOVA        (uint8_t* BAR_addr, uint8_t index) override;        
        bool            _bindDescMemVirt        ()    override    ;
        uint16_t        _calcIPChecksum         (const uint8_t* data, uint32_t size);
    private:
        volatile union ixgbe_adv_tx_desc*   p_desc_ring_start;
        pkt_buf**       a_used_buf_addr{nullptr};    
        uint32_t        m_used_buf_head{0};   // Dequeue from head (FIFO)
        uint32_t        m_used_buf_tail{0};   // Enqueue at tail
        
        


        


};
