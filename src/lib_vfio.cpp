#include "lib_vfio.h"
#include <linux/vfio.h>
#include <filesystem>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "log.h"


vfio_device::vfio_device(std::string pci_address,uint8_t bar_index, int interrupt_timeout_ms): 
m_pci_addr(pci_address),
m_fds({-1, -1, -1, -1}),
m_interrupt_timeout_ms(interrupt_timeout_ms),
m_is_interrupt_enabled(interrupt_timeout_ms > 0),
m_is_vfio_enabled(false)
{
this->_initialize(bar_index);
}

vfio_device::~vfio_device(){

}

bool vfio_device::_initialize(uint8_t bar_index){            

    return
    this->_get_group_id()               &&
    this->_get_container_fd()           &&
    this->_get_group_fd()               &&
    this->_add_group_to_container()     &&
    this->_get_device_fd()              && 
    this->_map_bar(bar_index)           &&
    this->_interrupt_handler();
}



bool vfio_device::_get_group_id(){
    std::filesystem::path device_dir = std::filesystem::path("/sys/bus/pci/devices") / this->m_pci_addr;
    struct stat st;
    int ret = stat(device_dir.c_str(), &st);
    if (ret < 0) {
        warn("PCI device %s not found in sysfs", this->m_pci_addr.c_str());
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
    debug("VFIO group id for device %s is %d", this->m_pci_addr.c_str(), group_id);
    return true;
}


bool vfio_device::_get_container_fd(){
    int cfd = m_fds.container_fd;
    if (cfd == -1) {
        cfd = ::open("/dev/vfio/vfio", O_RDWR);
        if (cfd == -1){
            error("filed to open /dev/vfio/vfio");
            return false;
        }
        m_fds.container_fd = cfd;
    }
    debug("VFIO container fd acquired: %d", cfd);
    return true;
}

bool vfio_device::_get_group_fd(){
    if (this->m_fds.group_id == -1) {
        warn("Group ID is invalid");
        return false;
    }
    std::string group_path = "/dev/vfio/" + std::to_string(this->m_fds.group_id);
    int gfd =check_err(::open(group_path.c_str(), O_RDWR), "open vfio group");
    this->m_fds.group_fd = gfd;
    debug("VFIO group fd acquired: %d", gfd);
    return true;
}

bool vfio_device::_add_group_to_container(){
    if (this->m_fds.container_fd == -1 || this->m_fds.group_fd == -1) {
        warn("Container fd or group fd is invalid");
        return false;
    }
        // check if the container's API version is the same as the VFIO API's
	check_err((ioctl(this->m_fds.container_fd, VFIO_GET_API_VERSION) == VFIO_API_VERSION) - 1, "get a valid API version from the container");
	// check if type1 is supported
	check_err((ioctl(this->m_fds.container_fd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) == 1) - 1, "get Type1 IOMMU support from the IOMMU container");

// check if group is viable
	struct vfio_group_status group_status;
    group_status.argsz = sizeof(group_status);

	check_err(ioctl(this->m_fds.group_fd, VFIO_GROUP_GET_STATUS, &group_status), "get VFIO group status");
	check_err(((group_status.flags & VFIO_GROUP_FLAGS_VIABLE) > 0) - 1, "get viable VFIO group - are all devices in the group bound to the VFIO driver?");

	// Add group to container
	check_err(ioctl(this->m_fds.group_fd, VFIO_GROUP_SET_CONTAINER, &this->m_fds.container_fd), "set container");
    int ret = ::ioctl(this->m_fds.container_fd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);
    if (ret == -1 && errno != EBUSY) {
        warn("set Type1 IOMMU for the container: %s", strerror(errno));
        return false;
    }

    debug("VFIO group %d added to container %d", this->m_fds.group_fd, this->m_fds.container_fd);
    return true;

}

bool vfio_device::_get_device_fd(){
    if (this->m_fds.group_fd == -1) {
        warn("Group fd is invalid"); 
        return false;
    }
    int dfd = check_err(ioctl(this->m_fds.group_fd, VFIO_GROUP_GET_DEVICE_FD, this->m_pci_addr.c_str()), "get device fd from group");
    this->m_fds.device_fd = dfd;
    this->m_is_vfio_enabled = true;
    debug("VFIO device fd acquired: %d", dfd);
    return true;
}

bool vfio_device::_map_bar(uint8_t bar_index){
    if (m_is_vfio_enabled){
        return 
        this->_map_bar_via_vfio(bar_index);
    } else {
        return
        this->_remove_ixgbe_driver()    &&
        this->_enable_dma()             &&
        this->_map_bar_directly(bar_index);
    }
    error("Failed to map BAR");
    return false;
}

bool vfio_device::_map_bar_via_vfio(uint8_t bar_index){
    if (bar_index > VFIO_PCI_BAR5_REGION_INDEX){
        warn("BAR index %d is out of range", bar_index);
        return false;
    }
    if (this->m_fds.device_fd == -1) {
        warn("Device fd is invalid");
        return false;
    }
    for (int i = 0; i <= bar_index; i++) {
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
        p_bar_addr[i] = temp_addr;
        debug("Mapped BAR %d: addr=%p", i, p_bar_addr[i]);    
    }
    return true;
}

bool vfio_device::_remove_ixgbe_driver(){
    std::filesystem::path device_dir = std::filesystem::path("/sys/bus/pci/devices") / this->m_pci_addr.c_str()/"driver/unbind";
	int fd = open(device_dir.c_str(), O_WRONLY);
	if (fd == -1) {
		debug("no driver loaded");
		return false;
	}
	if (write(fd, m_pci_addr.c_str(), strlen(m_pci_addr.c_str())) != (ssize_t) strlen(m_pci_addr.c_str())) {
		warn("failed to unload driver for device %s", m_pci_addr.c_str());
        return false;
	}
	check_err(close(fd), "close");
    return true;
}

bool vfio_device::_enable_dma(){
    std::filesystem::path res = std::filesystem::path("/sys/bus/pci/devices")
            / m_pci_addr / "config";
	int fd = check_err(open(res.c_str(), O_RDWR), "open pci config");
	// write to the command register (offset 4) in the PCIe config space
	// bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
	assert(lseek(fd, 4, SEEK_SET) == 4);
	uint16_t dma = 0;
	assert(read(fd, &dma, 2) == 2);
	dma |= 1 << 2;
	assert(lseek(fd, 4, SEEK_SET) == 4);
	assert(write(fd, &dma, 2) == 2);
	check_err(close(fd), "close");
    return true;
}

bool vfio_device::_map_bar_directly(uint8_t bar_index){
    for (int bar = 0; bar <= bar_index; ++bar) {
        std::filesystem::path res = std::filesystem::path("/sys/bus/pci/devices")
            / m_pci_addr / ("resource" + std::to_string(bar));
        int fd = check_err(::open(res.c_str(), O_RDWR | O_SYNC), "open pci resource");
        struct stat st {};
        check_err(::fstat(fd, &st), "stat pci resource");
        uint8_t* addr = static_cast<uint8_t*>(
            ::mmap(nullptr, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
        if (addr == MAP_FAILED) {
            check_err(::close(fd), "close pci resource");
            return false;
        }
        this->p_bar_addr[bar] = addr;
        debug("BAR%d mapped: %p len %zu", bar, addr, static_cast<size_t>(st.st_size));
        check_err(::close(fd), "close pci resource");
    }   
    return true;
}

bool vfio_device::_interrupt_handler(){
    if (m_is_interrupt_enabled && m_is_vfio_enabled){ 
    debug("Interrupts enabled with timeout %d ms", m_interrupt_timeout_ms);
    p_interrupt = std::make_unique<interrupt>(*this);
    }
    else{
        debug("Interrupts disabled");
    }
    return true;
}

