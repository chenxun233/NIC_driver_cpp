#include "dma_memory_allocator.h"
#include <unistd.h>
#include <sys/mman.h>
#include <linux/mman.h>
#include <linux/vfio.h>
#include "log.h"
#include <sys/ioctl.h>

constexpr uint64_t  MIN_DMA_MEMORY   = 4096; // we can not allocate less than page_size memory
constexpr uint64_t  iova_end         = UINT64_MAX;
constexpr size_t    min_payload_size = 2048;


DMAMemoryAllocator::DMAMemoryAllocator()
{
}

DMAMemoryAllocator::~DMAMemoryAllocator()
{
    _unmapVirtualAddr();
    _unmapIOVirtualAddr();
}

DMAMemoryPair DMAMemoryAllocator::allocDMAMemory(size_t size, int container_fd){
    size = _alignUpU64(size, m_page_size);
    //allocate virtual address
    void* virt_addr = _allocDMAVirtualAddr(size);
    // create IOMMU mapping
    // allocate IO virtual address
    uint64_t iova = m_next_iova;
    // bind the two addresses.
    _bindIOVAWithVirtAddr(virt_addr, iova, size, container_fd);
    DMAMemoryPair DMA_mem_pair;
    DMA_mem_pair.virt = virt_addr;
    DMA_mem_pair.iova = iova;
    DMA_mem_pair.size = size;
    m_allocated_memories.push_back(DMA_mem_pair);
    return  DMA_mem_pair;
}

void*  DMAMemoryAllocator::_allocDMAVirtualAddr(size_t size){
    void* virtual_address = (void*) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_HUGETLB | MAP_HUGE_2MB, -1, 0);
    if (virtual_address == MAP_FAILED) {
        error("Failed to mmap DMA memory using huge page. Huge page may have not been enabled. The error code is %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    return virtual_address;
}

// this function makes the physical address in DRAM shared both by virtual address space and IOVA. one is for CPU access, the other is for device DMA access.
bool DMAMemoryAllocator::_bindIOVAWithVirtAddr(void* virt_addr, uint64_t iova, size_t size, int container_fd){
	m_next_iova = _alignUpU64(m_next_iova, m_page_size);
	
	if (m_next_iova > iova_end || m_next_iova + size - 1 > iova_end) {
		error("IOMMU aperture exhausted: need 0x%llx bytes", (unsigned long long) size);
		exit(EXIT_FAILURE);
	}
	
	struct vfio_iommu_type1_dma_map dma_map ={};
	dma_map.vaddr = (uint64_t) virt_addr;
	// dma_map.vaddr = (uint64_t) 0x100000;
	dma_map.iova = iova;
	dma_map.size = size;
	dma_map.argsz = sizeof(dma_map);
	dma_map.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;
	check_err(ioctl(container_fd, VFIO_IOMMU_MAP_DMA, &dma_map), "IOMMU Map DMA Memory");
	m_next_iova = iova + size;
	return true;
}

bool DMAMemoryAllocator::_unmapVirtualAddr(){
    //unmap all allocated virtual addresses
    for (const auto& DMA_mem_pair : m_allocated_memories) {
        if (munmap(DMA_mem_pair.virt, DMA_mem_pair.size) == -1) {
            error("Failed to unmap virtual address %p: %s", DMA_mem_pair.virt, strerror(errno));
            return false;
        }
    }
    return true;
}

bool DMAMemoryAllocator::_unmapIOVirtualAddr(){

    return true;
}



uint64_t DMAMemoryAllocator::_alignUpU64(uint64_t value, uint64_t alignment){
    if (!alignment){
        return value;
    }
    return (value + alignment - 1) & ~(alignment - 1);
}