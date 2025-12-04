#ifndef MEMORY_H
#define MEMORY_H
#include "basic_dev.h"
#include "log.h"
#include "ixgbe_type.h"

static uint64_t MIN_DMA_MEMORY = 4096; // we can not allocate less than page_size memory
static uint64_t iova_start = 0x10000; //start from a low address
static uint64_t iova_end = UINT64_MAX; 
static uint64_t next_iova = 0;
#define HUGE_PAGE_BITS 21
#define HUGE_PAGE_SIZE (1 << HUGE_PAGE_BITS) // 2_097_152 = 2MiB
#define SIZE_PKT_BUF_HEADROOM 40







namespace memory_op{

    struct dma_memory_type {
    void*   virt;
    uint64_t phy;
    };

    struct mempool_type {
        void*                       base_addr;
        uint32_t                    buf_size;
        uint32_t                    num_entries;
        // memory is managed via a simple stack
        // replacing this with a lock-free queue (or stack) makes this thread-safe
        uint32_t                    free_stack_top;
        // the stack contains the entry id, i.e., base_addr + entry_id * buf_size is the address of the buf
        uint32_t                    free_stack[];
    };

    struct rx_ringbuffer_type{
        volatile union ixgbe_adv_rx_desc* descriptors;
    	struct mempool_type* mempool;
    	uint16_t num_entries;
    	// position we are reading from
    	uint16_t rx_index;
    	// virtual addresses to map descriptors back to their mbuf for freeing
    	void* virtual_addresses[];
    };

    struct tx_ringbuffer_type {
    	volatile union ixgbe_adv_tx_desc* descriptors;
    	uint16_t num_entries;
    	// position we are writing to
    	uint16_t tx_index;
    	// position up to which the device has processed
    	uint16_t clean_index;
    	// virtual addresses to map descriptors back to their mbuf for freeing
    	void* virtual_addresses[];
    };

    struct dma_memory_type  memory_allocate_dma(size_t size, bool require_contiguous);
    struct dma_memory_type  memory_allocate_via_hugepage(size_t size, bool require_contiguous);
    struct dma_memory_type  memory_allocate_via_vfiomap(size_t size, bool require_contiguous);
    uint64_t                vfio_map_dma(void* virt_addr, size_t size);
    uint64_t                get_page_size();
    uint64_t                align_up_u64(uint64_t val, uint64_t align);
    uintptr_t               virt_to_phys(void* virt_addr);
    uint32_t                huge_pg_id;

}
#endif // MEMORY_H