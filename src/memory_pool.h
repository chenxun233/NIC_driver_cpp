#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#define SIZE_PKT_BUF_HEADROOM 40

struct pkt_buf {
	// physical address to pass a buffer to a nic
	uintptr_t phy_addr;
    // index of this pkt_buf in the mempool
	uint32_t idx;
    // actual size of the data in the buffer, initialized to 0
	uint32_t size;
	uint8_t head_room[SIZE_PKT_BUF_HEADROOM];
	uint8_t* data __attribute__((aligned(64)));
};


class MemoryPool{

    public:
        MemoryPool(uint32_t num_buf, uint32_t buf_size, int container_fd = -1);
        ~MemoryPool();
        struct pkt_buf*             popOnePktBuf();
        void                        freeOnePktBuf(struct pkt_buf* buf);
    public:
        uint32_t                    getNumOfBufs() const     { return m_num_bufs; }
        uint32_t                    getBufSize()   const     { return m_buf_size; }
        void*                       getBaseVirtualAddr() const { return p_base_virtual_addr; }
        uintptr_t                   getBaseIOVirtualAddr() const { return m_base_io_virtual_addr; }
    private:
        bool                        _allocateMemory();
        bool                        _initEachPktBuf();
        uint32_t                    m_num_bufs{0}                   ;
        uint32_t                    m_buf_size{0}                   ;
        uint64_t                    m_total_size{0}                 ;
        uint32_t                    m_free_stack_top{0}             ;
        int                         m_container_fd{-1}              ;
        void*                       p_base_virtual_addr{nullptr}    ;
        uint64_t                    m_base_io_virtual_addr{0}       ;
        std::vector<uint32_t>       m_free_stack                    ;

};