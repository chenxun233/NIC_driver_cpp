#pragma once
#include "memory_pool.h"
#include "basic_ring_buffer.h"
#include "ixgbe_type.h"



class IXGBE_RxRingBuffer:public RingBuffer {
    public:
                        IXGBE_RxRingBuffer (){};
                        ~IXGBE_RxRingBuffer(){};
        bool            linkMemoryPool           ( DMAMemoryPool* const mem_pool) override;
        uint16_t        linkPktBufWithDesc       (uint16_t batch_size);
        DMAMemoryPool*  getMemPool   () const { return p_mem_pool; } 
    private:
        bool            _bindDescMemIOVA          (uint8_t* BAR_addr, uint8_t index) override;
        bool            _bindDescMemVirt   () override    ;
        volatile union ixgbe_adv_rx_desc*               p_desc_ring_start;
};


class IXGBE_TxRingBuffer:public RingBuffer {
    public:
                        IXGBE_TxRingBuffer      ();
                        ~IXGBE_TxRingBuffer     ();
        bool            linkMemoryPool         ( DMAMemoryPool* const mem_pool) override;
        uint16_t        linkPktBufWithDesc     (uint16_t batch_size);
        bool            fillPktBuf              (const char* data, uint32_t size);
        bool            freeUsedBuf             ();
        bool            cleanDescriptorRing     (uint16_t min_clean_num);
        DMAMemoryPool*  getMemPool              () const { return p_mem_pool; }
    private:
        bool            _bindDescMemIOVA        (uint8_t* BAR_addr, uint8_t index) override;        
        bool            _bindDescMemVirt        ()    override    ;
        uint16_t        _calcIPChecksum         (const uint8_t* data, uint32_t size);
    private:
        volatile union ixgbe_adv_tx_desc*   p_desc_ring_start;
        


        


};
