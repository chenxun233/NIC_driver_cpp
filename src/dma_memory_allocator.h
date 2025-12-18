#include <cstdint>
#include <cstddef>
#include <vector>

struct DmaMemoryPair {
    // start of the virtual address
    void*   virt;
    // start of the physical/IO virtual address
    uint64_t phy;
    size_t  size;
};

class DMAMemoryAllocator {
    
    public:
        static DMAMemoryAllocator& getInstance               ()
        {
            static DMAMemoryAllocator instance; 
            return instance;
        }
                                    ~DMAMemoryAllocator         ()                                            ;
        DmaMemoryPair               allocDMAMemory              (size_t size, int container_fd)                         ;

    private:                    
                                    DMAMemoryAllocator          ()                                                      ;
        uint64_t                    _alignUpU64                 (uint64_t value, uint64_t alignment)                    ;
        void*                       _mapVirtualAddr             (size_t ring_size)                                      ;
        uint64_t                    _mapIOVirtualAddr           (void* virt_addr, size_t ring_size, int container_fd)   ;
        bool                        _unmapVirtualAddr           ()                                                      ;
        bool                        _unmapIOVirtualAddr         ()                                                      ;
    private:                
        uint64_t                    m_page_size                 {2*1024*1024}                                           ;
        uint64_t                    m_next_iova                 {0x10000000}                                            ;
        std::vector<DmaMemoryPair>  m_allocated_memories                                                                ;

};