#include <cstdint>
#include <string>
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
        vfio_device(std::string pci_address);
        ~vfio_device();

        bool get_iommu_aperture();
        
    private:
        bool _get_group_id();
        bool _get_group_fd();
        bool _get_container_fd();
        bool _get_device_fd();
        bool _add_group_to_container();

        iommu_aperture m_iommu_aperture;
        std::string m_pci_addr;
        vfio_fd m_fds;

};