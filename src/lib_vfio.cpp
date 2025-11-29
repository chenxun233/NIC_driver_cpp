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



vfio_device::vfio_device(std::string pci_address): 
m_pci_addr(pci_address),
m_fds({-1, -1, -1, -1})
{
this->_get_group_id();
this->_get_container_fd();
this->_get_group_fd();
this->_add_group_to_container();
this->_get_device_fd();
}

vfio_device::~vfio_device(){

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
        cfd = check_err(::open("/dev/vfio/vfio", O_RDWR), "open /dev/vfio/vfio");
        m_fds.container_fd = cfd;

        if (::ioctl(cfd, VFIO_GET_API_VERSION) != VFIO_API_VERSION) {
            warn("get a valid API version from the container");
            return false;
        }
        if (::ioctl(cfd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) != 1) {
            warn("get Type1 IOMMU support from the container");
            return false;
        }
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
    int gfd = check_err(::open(group_path.c_str(), O_RDWR), "open VFIO group");
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
    debug("VFIO device fd acquired: %d", dfd);
    return true;

}