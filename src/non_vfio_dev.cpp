#include "non_vfio_dev.h"
#include <filesystem>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include "log.h"
#include <linux/vfio.h>
#include <memory>

non_vfio_dev::non_vfio_dev(
                    std::string pci_addr,
                    uint8_t     bar_index_max,
                    uint16_t    num_rx_queues,
                    uint16_t    num_tx_queues,
                    uint16_t    interrupt_timeout_ms
):
BasicDev(
    pci_addr, 
    bar_index_max, 
    num_rx_queues, 
    num_tx_queues, 
    interrupt_timeout_ms
)
{

};
non_vfio_dev::~non_vfio_dev(){

};
bool non_vfio_dev::initialize() {
    return
    this->remove_ixgbe_driver()   &&
    this->enable_dma()            &&
    this->map_bar()               ;          
};

bool non_vfio_dev::map_bar () {
    for (int bar = 0; bar <= m_basic_para.bar_index_max; ++bar) {
        std::filesystem::path res = std::filesystem::path("/sys/bus/pci/devices")
            / m_basic_para.pci_addr / ("resource" + std::to_string(bar));
        int fd = check_err(::open(res.c_str(), O_RDWR | O_SYNC), "open pci resource");
        struct stat st {};
        check_err(::fstat(fd, &st), "stat pci resource");
        uint8_t* addr = static_cast<uint8_t*>(
            ::mmap(nullptr, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
        if (addr == MAP_FAILED) {
            check_err(::close(fd), "close pci resource");
            return false;
        }
        this->m_basic_para.p_bar_addr[bar] = addr;
        debug("BAR%d mapped: %p len %zu", bar, addr, static_cast<size_t>(st.st_size));
        check_err(::close(fd), "close pci resource");
    }   
    return true;
}
bool non_vfio_dev::remove_ixgbe_driver() {
    std::filesystem::path device_dir = std::filesystem::path("/sys/bus/pci/devices") / this->m_basic_para.pci_addr.c_str()/"driver/unbind";
	int fd = open(device_dir.c_str(), O_WRONLY);
	if (fd == -1) {
		debug("no driver loaded");
		return false;
	}
	if (write(fd, m_basic_para.pci_addr.c_str(), strlen(m_basic_para.pci_addr.c_str())) != (ssize_t) strlen(m_basic_para.pci_addr.c_str())) {
		warn("failed to unload driver for device %s", m_basic_para.pci_addr.c_str());
        return false;
	}
	check_err(close(fd), "close");
    return true;
};
bool non_vfio_dev::enable_dma() {
    std::filesystem::path res = std::filesystem::path("/sys/bus/pci/devices")
            / m_basic_para.pci_addr / "config";
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
};

bool non_vfio_dev::tx_batch() {
    return true;
}
bool non_vfio_dev::rx_batch(){
    return true;
}       
bool non_vfio_dev::read_stats(){
    return true;
}     
bool non_vfio_dev::set_promisc(){
    return true;
}    
bool non_vfio_dev::get_link_speed(){
    return true;
} 
bool non_vfio_dev::set_mac_address(){
    return true;
}
bool non_vfio_dev::get_mac_address(){
    return true;
}
bool non_vfio_dev::reset(){
    return true;
}