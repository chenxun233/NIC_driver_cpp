#ifndef MEMORY_H
#define MEMORY_H
#include "basic_dev.h"
#include "log.h"
#include "ixgbe_type.h"



#define HUGE_PAGE_BITS 21
#define HUGE_PAGE_SIZE (1 << HUGE_PAGE_BITS) // 2_097_152 = 2MiB
#define SIZE_PKT_BUF_HEADROOM 40
#define MIN_NUM_OF_BUF 4096
#define PKT_BUF_SIZE 2048
#define PKT_SIZE 60


#define NUM_OF_BUF_RX_QUEUE 512
#define NUM_OF_BUF_TX_QUEUE 512




struct DmaMemoryPair {
    // start of the virtual address
    void*   virt;
    // start of the physical/IO virtual address
    uint64_t phy;
};

struct MemPool {
    //CPU-accessible virtual address, pointing to the start of all packet buffers
    void*                       base_virtual_addr;
    //size of each pkt buffer
    uint32_t                    buf_size;
    //total number of pkt buffers
    uint32_t                    num_pkt_buf;
    // memory is managed via a simple stack
    // replacing this with a lock-free queue (or stack) makes this thread-safe
    // an index, pointing to the top of the free stack. It tracks how many free pkt buffers are left
    uint32_t                    free_stack_top;
    // the stack contains the entry id, i.e., base_addr + entry_id * buf_size is the address of the buf
    uint32_t                    free_stack[NUM_OF_BUF_RX_QUEUE+NUM_OF_BUF_TX_QUEUE];
};

struct RxRingBuffer{
    volatile union ixgbe_adv_rx_desc* descriptors;
    // it stores all the pkt buffers (used, unused)
    struct MemPool* mempool;
    uint16_t total_num_of_buf;
    // position we are reading from
    uint16_t rx_index;
    // virtual addresses to map descriptors back to their mbuf for freeing
    // it stores the pointer to each pkt buffer that to be used.
    void* buf_virtual_addr[NUM_OF_BUF_RX_QUEUE];
};

struct TxRingBuffer {
    volatile union ixgbe_adv_tx_desc* descriptors;
    uint16_t total_num_of_buf;
    // position we are writing to
    uint16_t tx_index;
    // position up to which the device has processed
    uint16_t clean_index;
    // virtual addresses to map descriptors back to their mbuf for freeing
    void* buf_virtual_addr[NUM_OF_BUF_TX_QUEUE];
};

struct pkt_buf {
	// physical address to pass a buffer to a nic
	uintptr_t phy_addr;
    // the mempool this pkt_buf belongs to
	struct MemPool* mempool;
    // index of this pkt_buf in the mempool
	uint32_t mempool_idx;
    // actual size of the data in the buffer, initialized to 0
	uint32_t size;

	uint8_t head_room[SIZE_PKT_BUF_HEADROOM];
	uint8_t* data __attribute__((aligned(64)));
};

struct DmaMemoryPair        memory_allocate_dma             (const struct VfioFd vfio_fds,size_t size, bool require_contiguous);
struct DmaMemoryPair        memory_allocate_via_vfiomap     (const struct VfioFd vfio_fds,size_t size, bool require_contiguous);
uint64_t                    setup_dma_ret_iova              (const struct VfioFd vfio_fds,void* virt_addr, size_t size);
struct DmaMemoryPair        memory_allocate_via_hugepage    (size_t size, bool require_contiguous);
uint64_t                    get_page_size                   ();
uint64_t                    align_up_u64                    (uint64_t val, uint64_t align);
uintptr_t                   virt_to_phys                    (void* virt_addr);
struct MemPool*             memory_allocate_mempool         (const struct VfioFd vfio_fds,uint32_t num_pkt_buf, uint32_t buf_size);
struct pkt_buf*             pkt_buf_alloc                   (struct MemPool* mempool);
uint32_t                    pkt_buf_alloc_batch             (struct MemPool* mempool, struct pkt_buf* bufs[], uint32_t num_bufs);


#endif // MEMORY_H