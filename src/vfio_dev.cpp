#include "vfio_dev.h"
#include <filesystem>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "log.h"
#include <linux/vfio.h>
#include <memory>

vfio_dev::vfio_dev(
                    std::string pci_addr,
                    uint8_t     bar_index,
                    uint16_t    num_rx_queues,
                    uint16_t    num_tx_queues,
                    uint16_t    interrupt_timeout_ms
):
basic_dev(
    pci_addr, 
    bar_index, 
    num_rx_queues, 
    num_tx_queues, 
    interrupt_timeout_ms
),
m_fds{-1, -1, -1, -1}   
{
};

vfio_dev::~vfio_dev(){

};
bool vfio_dev::initialize() {
    return
    this->_get_group_id()               &&
    this->_get_container_fd()           &&
    this->_get_group_fd()               &&
    this->_add_group_to_container()     &&
    this->_get_device_fd()              &&
    this->enable_dma()                  &&
    this->map_bar()                     &&
    this->set_interrupt_host();            
};


bool vfio_dev::_get_group_id(){
    std::filesystem::path device_dir = std::filesystem::path("/sys/bus/pci/devices") / this->m_basic_para.pci_addr.c_str();
    struct stat st;
    int ret = stat(device_dir.c_str(), &st);
    if (ret < 0) {
        warn("PCI device %s not found in sysfs", this->m_basic_para.pci_addr.c_str());
        return false;
    }
    std::filesystem::path group_link = device_dir / "iommu_group";
    std::error_code ec;
    std::filesystem::path group_target = std::filesystem::read_symlink(group_link, ec);
    if (ec) {
        warn("find the iommu_group for the device: %s", ec.message().c_str());
        return false;
    }
    std::string group_name = group_target.filename().string();
    int group_id = std::stoi(group_name);
    this->m_fds.group_id = group_id;
    debug("VFIO group id for device %s is %d", this->m_basic_para.pci_addr.c_str(), group_id);
    return true;
}


bool vfio_dev::_get_container_fd(){
    int cfd = m_fds.container_fd;
    if (cfd == -1) {
        cfd = ::open("/dev/vfio/vfio", O_RDWR);
        if (cfd == -1){
            warn("filed to open /dev/vfio/vfio");
            return false;
        }
        m_fds.container_fd = cfd;
    }
    debug("VFIO container fd acquired: %d", cfd);
    return true;
}

bool vfio_dev::_get_group_fd(){
    if (this->m_fds.group_id == -1) {
        warn("Group ID is invalid");
        return false;
    }
    std::string group_path = "/dev/vfio/" + std::to_string(this->m_fds.group_id);
    int gfd =::open(group_path.c_str(), O_RDWR);
    if (gfd == -1){
        warn("filed to open %s", group_path.c_str());
        return false;
    }
    this->m_fds.group_fd = gfd;
    debug("VFIO group fd acquired: %d", gfd);
    return true;
}

bool vfio_dev::_add_group_to_container(){
    if (this->m_fds.container_fd == -1 || this->m_fds.group_fd == -1) {
        warn("Container fd or group fd is invalid");
        return false;
    }
	if(!(ioctl(this->m_fds.container_fd, VFIO_GET_API_VERSION) == VFIO_API_VERSION))
    {
        warn("the API version of the container is not compatible");
        return false;
    };
	// check if type1 is supported
	if(!(ioctl(this->m_fds.container_fd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) == 1))
    {
        warn("the container does not support Type1 IOMMU");
        return false;
    };

// check if group is viable
	struct vfio_group_status group_status;
    group_status.argsz = sizeof(group_status);
	if(ioctl(this->m_fds.group_fd, VFIO_GROUP_GET_STATUS, &group_status)== -1)
    {
        warn("failed to get VFIO group status");
        return false;
    };
	if(!((group_status.flags & VFIO_GROUP_FLAGS_VIABLE) > 0))
    {
        warn("VFIO group is not viable - are all devices in the group bound to the VFIO driver?");
        return false;
    };

	if(ioctl(this->m_fds.group_fd, VFIO_GROUP_SET_CONTAINER, &this->m_fds.container_fd) == -1)
    {
        warn("failed to set container for VFIO group");
        return false;
    };
    int ret = ::ioctl(this->m_fds.container_fd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);
    if (ret == -1 && errno != EBUSY) {
        warn("set Type1 IOMMU for the container: %s", strerror(errno));
        return false;
    }
    debug("VFIO group %d added to container %d", this->m_fds.group_fd, this->m_fds.container_fd);
    return true;

}

bool vfio_dev::_get_device_fd(){
    if (this->m_fds.group_fd == -1) {
        warn("Group fd is invalid"); 
        return false;
    }
    int dfd = ioctl(this->m_fds.group_fd, VFIO_GROUP_GET_DEVICE_FD, this->m_basic_para.pci_addr.c_str());
    if (dfd == -1){
        warn("failed to get device fd from group");
        return false;
    }
    this->m_fds.device_fd = dfd;
    debug("VFIO device fd acquired: %d", dfd);
    return true;
}

bool vfio_dev::map_bar () {
    if (m_basic_para.bar_index > VFIO_PCI_BAR5_REGION_INDEX){
        warn("BAR index %d is out of range", m_basic_para.bar_index);
        return false;
    }
    if (this->m_fds.device_fd == -1) {
        warn("Device fd is invalid");
        return false;
    }
    for (int i = 0; i <= m_basic_para.bar_index; i++) {
        struct vfio_region_info region_info = {};
        region_info.argsz = sizeof(region_info);
	    region_info.index = i;
	    int ret = ioctl(this->m_fds.device_fd, VFIO_DEVICE_GET_REGION_INFO, &region_info);
        if (ret == -1) {
            warn("Failed to get region info for BAR %d: %s", i, strerror(errno));
            return false; // MAP_FAILED == ((void *) -1)
        }
        uint8_t* temp_addr = static_cast<uint8_t*> (::mmap(NULL, region_info.size, PROT_READ | PROT_WRITE, MAP_SHARED, this->m_fds.device_fd, region_info.offset));
        if (temp_addr == MAP_FAILED) {
            error("Failed to mmap BAR %d: %s", i, strerror(errno));
            return false;
        }
        m_basic_para.p_bar_addr[i] = temp_addr;
        debug("Mapped BAR %d: addr=%p", i, m_basic_para.p_bar_addr[i]);    
    }
    return true;         
};


bool vfio_dev::enable_dma() {
	int command_register_offset = 4;
	// bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
	int bus_master_enable_bit = 2;
	// Get region info for config region
	struct vfio_region_info conf_reg ={};
    conf_reg.argsz = sizeof(conf_reg);
	conf_reg.index = VFIO_PCI_CONFIG_REGION_INDEX;
	check_err(ioctl(this->m_fds.device_fd, VFIO_DEVICE_GET_REGION_INFO, &conf_reg), "get vfio config region info");
	uint16_t dma = 0;
	assert(pread(this->m_fds.device_fd, &dma, 2, conf_reg.offset + command_register_offset) == 2);
	dma |= 1 << bus_master_enable_bit;
	assert(pwrite(this->m_fds.device_fd, &dma, 2, conf_reg.offset + command_register_offset) == 2);
    return true;
}

bool vfio_dev::tx_batch() {

    return true;
}
bool vfio_dev::rx_batch(){
    return true;
}       
bool vfio_dev::read_stats(){
    return true;
}     
bool vfio_dev::set_promisc(){
    return true;
}    
bool vfio_dev::get_link_speed(){
    return true;
} 
bool vfio_dev::set_mac_address(){
    return true;
}
bool vfio_dev::get_mac_address(){
    return true;
}
bool vfio_dev::reset(){
    return true;
}

bool vfio_dev::set_interrupt_host() {
    if (m_basic_para.interrupt_timeout_ms == 0) {
        info("Interrupt is disabled");
        return true;
    }
    p_interrupt = std::make_unique<interrupt>(
        this->m_fds.device_fd,
        this->m_basic_para.interrupt_timeout_ms,
        this->m_basic_para.num_rx_queues,
        this->m_basic_para.num_tx_queues
    );
    return true;
};