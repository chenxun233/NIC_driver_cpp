#include "memory_op.h"
#include <linux/mman.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/vfio.h>
#include <sys/ioctl.h>
#include "log.h"
#include <limits.h>
#include <fcntl.h>

static uint64_t MIN_DMA_MEMORY = 4096; // we can not allocate less than page_size memory
static uint64_t iova_start = 0x10000; //start from a low address
static uint64_t iova_end = UINT64_MAX; 
static uint64_t next_iova = 0;
static uint32_t huge_pg_id;
static int 		 call_time;
static int	   	call_time_memory_allocate_dma = 0;
static int 		call_time_memory_allocate_mempool = 0;




struct DmaMemoryPair memory_allocate_dma(struct VfioFd vfio_fds,size_t size, bool require_contiguous){
	call_time_memory_allocate_dma++;
	debug("entered memory_allocate_dma, call time: %d", call_time_memory_allocate_dma);
	if (vfio_fds.container_fd != -1) {
		// VFIO == -1 means that there is no VFIO container set, i.e. VFIO / IOMMU is not activated
		return memory_allocate_via_vfiomap(vfio_fds, size, require_contiguous);
	} else {
		return memory_allocate_via_hugepage(size, require_contiguous);
	}
}

struct DmaMemoryPair memory_allocate_via_vfiomap(struct VfioFd vfio_fds,size_t size, bool require_contiguous){
	(void) require_contiguous;
	debug("allocating dma memory via VFIO");
	//allocate virtual address
	void* virt_addr = (void*) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_HUGETLB | MAP_HUGE_2MB, -1, 0);
	// create IOMMU mapping
	// allocate IO virtual address
	uint64_t iova = (uint64_t) setup_dma_ret_iova(vfio_fds, virt_addr, size);
	struct DmaMemoryPair mem;
	mem.virt = virt_addr;
	mem.phy = iova;
	debug("finished allocating dma memory via VFIO");
	return mem;
}

uint64_t setup_dma_ret_iova(struct VfioFd vfio_fds,void* virt_addr, size_t size){
	call_time++;
	debug("entered setup_dma_ret_iova, call time: %d", call_time);
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
	// dma_map.vaddr = (uint64_t) 0x100000;
	dma_map.iova = iova;
	dma_map.size = map_size;
	dma_map.argsz = sizeof(dma_map);
	dma_map.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;
	int cfd = vfio_fds.container_fd;
	check_err(ioctl(cfd, VFIO_IOMMU_MAP_DMA, &dma_map), "IOMMU Map DMA Memory");
	next_iova = iova + map_size;
	debug("finished setup_dma_ret_iova, time: %d", call_time);
	return iova;
}

uint64_t get_page_size(){
	long page_size = sysconf(_SC_PAGESIZE);
	if (page_size <= 0) {
		page_size = 4096;
	}
	return (uint64_t) page_size;
}

uint64_t align_up_u64(uint64_t val, uint64_t align){
	if (!align){
		return val;
	}
	return (val + align - 1) & ~(align - 1);
}


struct DmaMemoryPair memory_allocate_via_hugepage(size_t size, bool require_contiguous){
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
	void* virt_addr = (void*) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_HUGETLB, fd, 0);
	// never swap out DMA memory
	check_err(mlock(virt_addr, size), "disable swap for DMA memory");
	// don't keep it around in the hugetlbfs
	close(fd);
	unlink(path);
	struct DmaMemoryPair mem;
	mem.phy = virt_to_phys(virt_addr);
	mem.virt = virt_addr;
	return mem;
}

uintptr_t virt_to_phys(void* virt_addr){
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

struct MemPool* memory_allocate_mempool(struct VfioFd vfio_fds,uint32_t num_pkt_buf, uint32_t buf_size){
	call_time_memory_allocate_mempool++;
	debug("entered memory_allocate_mempool, call time: %d", call_time_memory_allocate_mempool);
	buf_size = buf_size ? buf_size : 2048;
	// require entries that neatly fit into the page size, this makes the memory pool much easier
	// otherwise our base_virtual_addr + index * size formula would be wrong because we can't cross a page-boundary
	if ((vfio_fds.container_fd == -1) && HUGE_PAGE_SIZE % buf_size) {
		error("entry size must be a divisor of the huge page size (%d)", HUGE_PAGE_SIZE);
	}
	struct MemPool* mempool = (struct MemPool*) malloc(sizeof(struct MemPool) + num_pkt_buf * sizeof(uint32_t));
	struct DmaMemoryPair mem = memory_allocate_dma(vfio_fds, buf_size * num_pkt_buf, false);
	mempool->num_pkt_buf = num_pkt_buf;
	mempool->buf_size = buf_size;
	mempool->base_virtual_addr = mem.virt;
	mempool->free_stack_top = num_pkt_buf;
	for (uint32_t i = 0; i < num_pkt_buf; i++) {
		mempool->free_stack[i] = i;
		// the start virtual address of this pkt_buf
		struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) mempool->base_virtual_addr) + i * buf_size);
		// the offset is shared by virtual and physical address
		uintptr_t offset = (uintptr_t) ((uint8_t*) buf - (uint8_t*) mempool->base_virtual_addr);
		if (vfio_fds.container_fd != -1) {
			buf->phy_addr = (uintptr_t) mem.phy + offset;
		} else {
			buf->phy_addr = virt_to_phys(buf);
		}
		buf->mempool_idx = i;
		buf->mempool = mempool;
		buf->size = 0;
		buf->data = (uint8_t*) buf + sizeof(struct pkt_buf);
	}
	debug("finished memory_allocate_mempool, call time: %d", call_time_memory_allocate_mempool);
	return mempool;
}

uint32_t pkt_buf_alloc_batch(struct MemPool* mempool, struct pkt_buf* bufs[], uint32_t num_bufs){
	if (mempool->free_stack_top < num_bufs) {
		warn("memory pool %p only has %d free bufs, requested %d", (void*)mempool, mempool->free_stack_top, num_bufs);
		num_bufs = mempool->free_stack_top;
	}
	for (uint32_t i = 0; i < num_bufs; i++) {
		uint32_t buf_id = mempool->free_stack[--mempool->free_stack_top];
		bufs[i] = (struct pkt_buf*) (((uint8_t*) mempool->base_virtual_addr) + buf_id * mempool->buf_size);
	}
	return num_bufs;
}

struct pkt_buf* pkt_buf_alloc(struct MemPool* mempool){
	struct pkt_buf* buf = nullptr;
	pkt_buf_alloc_batch(mempool, &buf, 1);
	return buf;
}

