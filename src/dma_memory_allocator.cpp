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

DmaMemoryPair DMAMemoryAllocator::allocDMAMemory(size_t size, int container_fd){
    debug("DMAMemoryAllocator: allocating dma memory of size %zu", size);
    size = _alignUpU64(size, m_page_size);
    //allocate virtual address
    void* virt_addr = _mapVirtualAddr(size);
    // create IOMMU mapping
    // allocate IO virtual address
    uint64_t iova = _mapIOVirtualAddr(virt_addr, size, container_fd);
    DmaMemoryPair mem;
    mem.virt = virt_addr;
    mem.phy = iova;
    mem.size = size;
    debug("DMAMemoryAllocator: finished allocating dma memory of size %zu", size);
    m_allocated_memories.push_back(mem);
    return  mem;
}

void*  DMAMemoryAllocator::_mapVirtualAddr(size_t size){
    void* virtual_address = (void*) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_HUGETLB | MAP_HUGE_2MB, -1, 0);
    if (virtual_address == MAP_FAILED) {
        error("Failed to mmap DMA memory using huge page. Huge page may have not been enabled. The error code is %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    return virtual_address;
}

uint64_t DMAMemoryAllocator::_mapIOVirtualAddr(void* virt_addr, size_t size, int container_fd){
	m_next_iova = _alignUpU64(m_next_iova, m_page_size);
	
	if (m_next_iova > iova_end || m_next_iova + size - 1 > iova_end) {
		error("IOMMU aperture exhausted: need 0x%llx bytes", (unsigned long long) size);
		exit(EXIT_FAILURE);
	}
	uint64_t iova = m_next_iova;
	struct vfio_iommu_type1_dma_map dma_map ={};
	dma_map.vaddr = (uint64_t) virt_addr;
	// dma_map.vaddr = (uint64_t) 0x100000;
	dma_map.iova = iova;
	dma_map.size = size;
	dma_map.argsz = sizeof(dma_map);
	dma_map.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;
	check_err(ioctl(container_fd, VFIO_IOMMU_MAP_DMA, &dma_map), "IOMMU Map DMA Memory");
	m_next_iova = iova + size;
	return iova;
}

bool DMAMemoryAllocator::_unmapVirtualAddr(){
    //unmap all allocated virtual addresses
    for (const auto& mem : m_allocated_memories) {
        if (munmap(mem.virt, mem.size) == -1) {
            error("Failed to unmap virtual address %p: %s", mem.virt, strerror(errno));
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