#ifndef LIB_VFIO_H 
#define LIB_VFIO_H

#include <cstdint>
#include <string>
#include <array>
#include <linux/vfio.h>
#include <memory>
#include "interrupts.h"

struct vfio_fd{
    int container_fd;
    int group_id;
    int group_fd;
    int device_fd;
};


struct iommu_aperture{
    uint64_t start;
    uint64_t end;
};

class vfio_global{
    public:
        static bool get_page_size();
        static long page_size;
};

class vfio_device{

    public:
        vfio_device(std::string pci_address,uint8_t bar_index, int interrupt_timeout_ms);
        ~vfio_device();
        
        int get_device_fd(){return this->m_fds.device_fd;};
        bool get_interrupt_enabled(){return m_interrupt_enabled;};
        bool get_iommu_aperture();
        
    private:
        bool _get_group_id();
        bool _get_group_fd();
        bool _get_container_fd();
        bool _get_device_fd();
        bool _add_group_to_container();
        bool _map_bar_addr(uint8_t bar_index);

        bool m_interrupt_enabled;
        std::unique_ptr<interrupt_handler> p_interrupt_handler;
        std::array<uint8_t*,6> p_bar_addr;
        iommu_aperture m_iommu_aperture;
        std::string m_pci_addr;
        vfio_fd m_fds;

};
#endif /* LIB_VFIO_H */