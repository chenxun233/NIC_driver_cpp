#include "memory_op.h"
#include <linux/mman.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/vfio.h>
#include <sys/ioctl.h>
#include "log.h"
#include <limits.h>
#include <fcntl.h>
#include "basic_dev.h"

using namespace memory_op;

struct dma_memory_type memory_allocate_dma(struct vfio_fd_type vfio_fds,size_t size, bool require_contiguous){
	if (vfio_fds.container_fd != -1) {
		// VFIO == -1 means that there is no VFIO container set, i.e. VFIO / IOMMU is not activated
		return memory_allocate_via_vfiomap(size, require_contiguous);
	} else {
		return memory_allocate_via_hugepage(size, require_contiguous);
	}
}

struct dma_memory_type memory_op::memory_allocate_via_vfiomap(size_t size, bool require_contiguous){
	debug("allocating dma memory via VFIO");
	void* virt_addr = (void*) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_HUGETLB | MAP_HUGE_2MB, -1, 0);
	// create IOMMU mapping
	uint64_t iova = (uint64_t) vfio_map_dma(virt_addr, size);
	return (struct dma_memory_type){
		// for VFIO, this needs to point to the device view memory = IOVA!
		.virt = virt_addr,
		.phy = iova
	};
}

uint64_t memory_op::vfio_map_dma(struct vfio_fd_type vfio_fds,void* virt_addr, size_t size){
	uint64_t page_size = get_page_size();
	uint64_t map_size = size < MIN_DMA_MEMORY ? MIN_DMA_MEMORY : size;
	map_size = align_up_u64(map_size, page_size);

	if (!next_iova) {
		next_iova = align_up_u64(iova_start, page_size);
	}

	if (next_iova > iova_end || next_iova + map_size - 1 > iova_end) {
		error("IOMMU aperture exhausted: need 0x%llx bytes", (unsigned long long) map_size);
		exit(EXIT_FAILURE);
	}

	uint64_t iova = next_iova;
	struct vfio_iommu_type1_dma_map dma_map ={};

	dma_map.vaddr = (uint64_t) virt_addr;
	dma_map.iova = iova;
	dma_map.size = map_size;
	dma_map.argsz = sizeof(dma_map);
	dma_map.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;

	int cfd = vfio_fds.container_fd;
	check_err(ioctl(cfd, VFIO_IOMMU_MAP_DMA, &dma_map), "IOMMU Map DMA Memory");
	next_iova = iova + map_size;
	return iova;
}

uint64_t memory_op::get_page_size(){
	long page_size = sysconf(_SC_PAGESIZE);
	if (page_size <= 0) {
		page_size = 4096;
	}
	return (uint64_t) page_size;
}

uint64_t memory_op::align_up_u64(uint64_t val, uint64_t align){
	if (!align){
		return val;
	}
	return (val + align - 1) & ~(align - 1);
}


struct dma_memory_type memory_op::memory_allocate_via_hugepage(size_t size, bool require_contiguous){
	debug("allocating dma memory via huge page");
	// round up to multiples of 2 MB if necessary, this is the wasteful part
	// this could be fixed by co-locating allocations on the same page until a request would be too large
	// when fixing this: make sure to align on 128 byte boundaries (82599 dma requirement)
	if (size % HUGE_PAGE_SIZE) {
		size = ((size >> HUGE_PAGE_BITS) + 1) << HUGE_PAGE_BITS;
	}
	if (require_contiguous && size > HUGE_PAGE_SIZE) {
		// this is the place to implement larger contiguous physical mappings if that's ever needed
		error("could not map physically contiguous memory");
	}
	// unique filename, C11 stdatomic.h requires a too recent gcc, we want to support gcc 4.8
	uint32_t id = __sync_fetch_and_add(&huge_pg_id, 1);
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/mnt/huge/ixy-%d-%d", getpid(), id);
	// temporary file, will be deleted to prevent leaks of persistent pages
	int fd = check_err(open(path, O_CREAT | O_RDWR, S_IRWXU), "open hugetlbfs file, check that /mnt/huge is mounted");
	check_err(ftruncate(fd, (off_t) size), "allocate huge page memory, check hugetlbfs configuration");
	void* virt_addr = (void*) check_err(mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_HUGETLB, fd, 0), "mmap hugepage");
	// never swap out DMA memory
	check_err(mlock(virt_addr, size), "disable swap for DMA memory");
	// don't keep it around in the hugetlbfs
	close(fd);
	unlink(path);
	return (struct dma_memory_type){
		.virt = virt_addr,
		.phy = virt_to_phys(virt_addr)
	};
}

uintptr_t memory_op::virt_to_phys(void* virt_addr){
	long pagesize = sysconf(_SC_PAGESIZE);
	int fd = check_err(open("/proc/self/pagemap", O_RDONLY), "getting pagemap");
	// pagemap is an array of pointers for each normal-sized page
	check_err(lseek(fd, (uintptr_t) virt_addr / pagesize * sizeof(uintptr_t), SEEK_SET), "getting pagemap");
	uintptr_t phy = 0;
	check_err(read(fd, &phy, sizeof(phy)), "translating address");
	close(fd);
	if (!phy) {
		warn("failed to translate virtual address %p to physical address", virt_addr);
		exit(EXIT_FAILURE);
	}
	// bits 0-54 are the page number
	return (phy & 0x7fffffffffffffULL) * pagesize + ((uintptr_t) virt_addr) % pagesize;
}