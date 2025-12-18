#include "vfio_dev.h"
#include <filesystem>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "log.h"
#include <linux/vfio.h>
#include <memory>
#include "device.h"
#include "ixgbe_type.h"
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include "dma_memory_allocator.h"
#include "ixgbe_ring_buffer.h"
#define PKT_SIZE 60

const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;
const int MAX_RX_QUEUE_ENTRIES = 4096;
const int MAX_TX_QUEUE_ENTRIES = 4096;

static const uint8_t pkt_data[] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // dst MAC
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, // src MAC
	0x08, 0x00,                         // ether type: IPv4
	0x45, 0x00,                         // Version, IHL, TOS
	(PKT_SIZE - 14) >> 8,               // ip len excluding ethernet, high byte
	(PKT_SIZE - 14) & 0xFF,             // ip len exlucding ethernet, low byte
	0x00, 0x00, 0x00, 0x00,             // id, flags, fragmentation
	0x40, 0x11, 0x00, 0x00,             // TTL (64), protocol (UDP), checksum
	0x0A, 0x00, 0x00, 0x01,             // src ip (10.0.0.1)
	0x0A, 0x00, 0x00, 0x02,             // dst ip (10.0.0.2)
	0x00, 0x2A, 0x05, 0x39,             // src and dst ports (42 -> 1337)
	(PKT_SIZE - 20 - 14) >> 8,          // udp len excluding ip & ethernet, high byte
	(PKT_SIZE - 20 - 14) & 0xFF,        // udp len exlucding ip & ethernet, low byte
	0x00, 0x00,                         // udp checksum, optional
	'i', 'x', 'y'                       // payload
	// rest of the payload is zero-filled because mempools guarantee empty bufs
};

VFIODev::VFIODev(
                    std::string pci_addr,
                    uint16_t    num_rx_queues,
                    uint16_t    num_tx_queues
):
BasicDev(
            pci_addr,
            num_rx_queues,
            num_tx_queues
        )
{

}

VFIODev::~VFIODev(){
};

bool VFIODev::getFD() {
    return
    this->_get_group_id()               &&
    this->_get_container_fd()           &&
    this->_get_group_fd()               &&
    this->_add_group_to_container()     &&
    this->_get_device_fd();              
};


bool VFIODev::_get_group_id(){
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


bool VFIODev::_get_container_fd(){
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

bool VFIODev::_get_group_fd(){
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

bool VFIODev::_add_group_to_container(){
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

bool VFIODev::_get_device_fd(){
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

bool VFIODev::mapBAR (uint8_t bar_index) {
    m_basic_para.bar_index_max = bar_index;
    if (m_basic_para.bar_index_max > VFIO_PCI_BAR5_REGION_INDEX){
        warn("BAR index %d is out of range", m_basic_para.bar_index_max);
        return false;
    }
    if (this->m_fds.device_fd == -1) {
        warn("Device fd is invalid");
        return false;
    }
    for (int i = 0; i <= m_basic_para.bar_index_max; i++) {
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


bool VFIODev::enableDMA() {
	int command_register_offset = 4;
	// bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
	int bus_master_enable_bit = 2;
	// Get region info for config region
	struct vfio_region_info conf_reg ={};
    conf_reg.argsz = sizeof(conf_reg);
	conf_reg.index = VFIO_PCI_CONFIG_REGION_INDEX;
    debug("device_fd is %d", this->m_fds.device_fd);
	check_err(ioctl(this->m_fds.device_fd, VFIO_DEVICE_GET_REGION_INFO, &conf_reg), "get vfio config region info");
	uint16_t dma = 0;
	assert(pread(this->m_fds.device_fd, &dma, 2, conf_reg.offset + command_register_offset) == 2);
	dma |= 1 << bus_master_enable_bit;
	assert(pwrite(this->m_fds.device_fd, &dma, 2, conf_reg.offset + command_register_offset) == 2);
    return true;
}





bool VFIODev::initHardware() {
	info("Resetting device...");
	// section 4.6.3.1 - disable all interrupts
	this->_dev_disable_IRQ();
	this->_dev_rst_hardware();
	usleep(10000);
	// section 4.6.3.1 - disable interrupts again after reset
	this->_dev_disable_IRQ();
	this->_get_mac_address();
    this->_init_eeprom_n_dma();

	// section 4.6.4 - initialize link (auto negotiation)
	this->_init_link_nego();
	// section 4.6.5 - statistical counters
	// reset-on-read registers, just read them once
	this->_read_stats();
    this->_initialize_interrupt();
    return true;
};

bool VFIODev::setDMAMemory() {
	this->_set_rx_DMA();
	// section 4.6.8 - init tx
	this->_set_tx_DMA();
    return true;
}

bool VFIODev::prepareQueues() {
    debug("entered VFIODev::prepareQueues");
	this->_prepare_rx_queue();
	this->_prepare_tx_queue();
    return true;
}

bool VFIODev::initMemoryPool(uint32_t num_buf, uint32_t buf_size){
    m_num_rx_bufs = num_buf;
    m_num_tx_bufs = num_buf;
    m_buf_rx_size = buf_size;
    m_buf_tx_size = buf_size;
    this->m_mempool = new MemoryPool(num_buf, buf_size, m_fds.container_fd);
    this->m_mempool->allocateMemory();
    this->m_mempool->initEachPktBuf();
    return true;
}

bool VFIODev::setRingBuffers(){
    for (uint16_t i = 0; i < m_basic_para.num_rx_queues; i++) {
        p_rx_ring_buffers.push_back(new IXGBE_RingBuffer(true));
        printf(p_rx_ring_buffers[i] == nullptr ? "nullptr\n" : "not null\n");
        p_rx_ring_buffers[i]->linkMemoryPool(this->m_mempool);
    }
    for (uint16_t i = 0; i < m_basic_para.num_tx_queues; i++) {
        p_tx_ring_buffers.push_back(new IXGBE_RingBuffer(false));
        p_tx_ring_buffers[i]->linkMemoryPool(this->m_mempool);
    }
    return true;
}

bool VFIODev::_read_stats(){
    info("entered VFIODev::_read_stats");
	uint32_t rx_pkts = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GPRC);
	uint32_t tx_pkts = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GPTC);
	uint64_t rx_bytes = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GORCL) + (((uint64_t) get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GORCH)) << 32);
	uint64_t tx_bytes = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GOTCL) + (((uint64_t) get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GOTCH)) << 32);

	m_dev_stats.rx_pkts  += rx_pkts;
	m_dev_stats.tx_pkts  += tx_pkts;
	m_dev_stats.rx_bytes += rx_bytes;
	m_dev_stats.tx_bytes += tx_bytes;
	return true;
}


bool VFIODev::_dev_disable_IRQ(){
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EIMS, 0x00000000);
	_dev_clear_interrupts();
	return true;
}

bool VFIODev::_dev_clear_interrupts(){
	// Clear interrupt mask
	// Clear interrupt mask to stop from interrupts being generated
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);
	get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EICR);
	return true;
}

bool VFIODev::_dev_rst_hardware(){
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
	wait_clear_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
	return true;
}

bool VFIODev::_get_mac_address(){
	mac_address_type mac;
	uint32_t rar_low = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RAL(0));
	uint32_t rar_high = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RAH(0));

	mac.addr[0] = rar_low;
	mac.addr[1] = rar_low >> 8;
	mac.addr[2] = rar_low >> 16;
	mac.addr[3] = rar_low >> 24;
	mac.addr[4] = rar_high;
	mac.addr[5] = rar_high >> 8;
    m_basic_para.mac_address = mac;
    return true;
}

bool VFIODev::_init_eeprom_n_dma(){
	// section 4.6.3 - Wait for EEPROM auto read completion
	wait_set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EEC, IXGBE_EEC_ARD);
	// section 4.6.3 - Wait for DMA initialization done (RDRXCTL.DMAIDONE)
	wait_set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDRXCTL, IXGBE_RDRXCTL_DMAIDONE);
    return true;
}
bool VFIODev::_init_link_nego(){
	// should already be set by the eeprom config, maybe we shouldn't override it here to support weirdo nics?
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_AUTOC, (get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_AUTOC) & ~IXGBE_AUTOC_LMS_MASK) | IXGBE_AUTOC_LMS_10G_SERIAL);
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_AUTOC, (get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_AUTOC) & ~IXGBE_AUTOC_10G_PMA_PMD_MASK) | IXGBE_AUTOC_10G_XAUI);
	// negotiate link
	set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_AUTOC, IXGBE_AUTOC_AN_RESTART);
	// datasheet wants us to wait for the link here, but we can continue and wait afterwards
	return true;
}


bool VFIODev::initMemPool() {
    debug("entered VFIODev::initMemPool");
	const int NUM_BUFS = 2048;
	m_tx_mempool = new MemoryPool(NUM_BUFS, 2048, m_fds.container_fd);
    m_tx_mempool->allocateMemory();
    m_tx_mempool->initEachPktBuf();
	// pre-fill all our packet buffers with some templates that can be modified later
	// we have to do it like this because sending is async in the hardware; we cannot re-use a buffer immediately
	struct pkt_buf* bufs[NUM_BUFS];
	for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
		struct pkt_buf* buf = m_tx_mempool->popOnePktBuf();
		buf->size = PKT_SIZE;
		memcpy(buf->data, pkt_data, sizeof(pkt_data));
		*(uint16_t*) (buf->data + 24) = _calc_ip_checksum(buf->data + 14, 20);
		bufs[buf_id] = buf;
	}
	// return them all to the mempool, all future allocations will return bufs with the data set above
	for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
		m_tx_mempool->freeOnePktBuf(bufs[buf_id]);
	}
	return true;
}



uint16_t VFIODev::_calc_ip_checksum(uint8_t* data, uint32_t len) {
	if (len % 1) error("odd-sized checksums NYI"); // we don't need that
	uint32_t cs = 0;
	for (uint32_t i = 0; i < len / 2; i++) {
		cs += ((uint16_t*)data)[i];
		if (cs > 0xFFFF) {
			cs = (cs & 0xFFFF) + 1; // 16 bit one's complement
		}
	}
	return ~((uint16_t) cs);
}


bool VFIODev::_set_rx_DMA(){
	// make sure that rx is disabled while re-configuring it
	// the datasheet also wants us to disable some crypto-offloading related rx paths (but we don't care about them)
	clear_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
	// no fancy dcb or vt, just a single 128kb packet buffer for us
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RXPBSIZE(0), IXGBE_RXPBSIZE_128KB);
	for (int i = 1; i < 8; i++) {
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RXPBSIZE(i), 0);
	}

	// always enable CRC offloading
	set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_HLREG0, IXGBE_HLREG0_RXCRCSTRP);
	set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_RDRXCTL, IXGBE_RDRXCTL_CRCSTRIP);

	// accept broadcast packets
	set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_FCTRL, IXGBE_FCTRL_BAM);

	// per-queue config, same for all queues
	for (uint16_t i = 0; i < m_basic_para.num_rx_queues; i++) {
		debug("initializing rx queue %d", i);
		// enable advanced rx descriptors, we could also get away with legacy descriptors, but they aren't really easier
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_SRRCTL(i), (get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_SRRCTL(i)) & ~IXGBE_SRRCTL_DESCTYPE_MASK) | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF);
		// drop_en causes the nic to drop packets if no rx descriptors are available instead of buffering them
		// a single overflowing queue can fill up the whole buffer and impact operations if not setting this flag
		set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_SRRCTL(i), IXGBE_SRRCTL_DROP_EN);
		// setup descriptor ring, see section 7.1.9
		uint32_t ring_size_bytes = m_num_rx_bufs * sizeof(union ixgbe_adv_rx_desc);
		DMAMemoryPair mem = DMAMemoryAllocator::getInstance().allocDMAMemory(ring_size_bytes, this->m_fds.container_fd);
		// neat trick from Snabb: initialize to 0xFF to prevent rogue memory accesses on premature DMA activation
		memset(mem.virt, -1, ring_size_bytes);
		// tell the device where it can write to (its iova, so its view)
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDBAL(i), (uint32_t) (mem.phy & 0xFFFFFFFFull));
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDBAH(i), (uint32_t) (mem.phy >> 32));
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDLEN(i), ring_size_bytes);
		// set ring to empty at start
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDH(i), 0);
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDT(i), 0);

		// private data for the driver, 0-initialized
        if (p_rx_ring_buffers[i]== nullptr){
            error("RX ring buffer %d is nullptr", i);
        }
        p_rx_ring_buffers[i]->linkDescriptor2DMAMemory(mem);
        p_rx_ring_buffers[i]->preparePktBuffer();

	}

	// last step is to set some magic bits mentioned in the last sentence in 4.6.7
	set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_CTRL_EXT, IXGBE_CTRL_EXT_NS_DIS);
	// this flag probably refers to a broken feature: it's reserved and initialized as '1' but it must be set to '0'
	// there isn't even a constant in ixgbe_types.h for this flag
	for (uint16_t i = 0; i < m_basic_para.num_rx_queues; i++) {
		clear_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_DCA_RXCTRL(i), 1 << 12);
	}
	// start RX
	set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
	debug("finished RX initialization");
	return true;
}

bool VFIODev::_set_tx_DMA(){
	// crc offload and small packet padding
	set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_HLREG0, IXGBE_HLREG0_TXCRCEN | IXGBE_HLREG0_TXPADEN);

	// set default buffer size allocations
	// see also: section 4.6.11.3.4, no fancy features like DCB and VTd
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TXPBSIZE(0), IXGBE_TXPBSIZE_40KB);
	for (int i = 1; i < 8; i++) {
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TXPBSIZE(i), 0);
	}
	// required when not using DCB/VTd
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_DTXMXSZRQ, 0xFFFF);
	clear_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_RTTDCS, IXGBE_RTTDCS_ARBDIS);

	// per-queue config for all queues
	for (uint16_t i = 0; i < m_basic_para.num_tx_queues; i++) {
		debug("initializing tx queue %d", i);

		// setup descriptor ring, see section 7.1.9
		uint32_t ring_size_bytes = m_num_tx_bufs * sizeof(union ixgbe_adv_tx_desc);
		DMAMemoryPair mem = DMAMemoryAllocator::getInstance().allocDMAMemory(ring_size_bytes, this->m_fds.container_fd);
		memset(mem.virt, -1, ring_size_bytes);
		// tell the device where it can write to (its iova, so its view)
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDBAL(i), (uint32_t) (mem.phy & 0xFFFFFFFFull));
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDBAH(i), (uint32_t) (mem.phy >> 32));
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDLEN(i), ring_size_bytes);
		debug("tx ring %d phy addr:  0x%012lX", i, mem.phy);
		debug("tx ring %d virt addr: 0x%012lX", i, (uintptr_t) mem.virt);

		// descriptor writeback magic values, important to get good performance and low PCIe overhead
		// see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
		// we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
		uint32_t txdctl = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TXDCTL(i));
		// there are no defines for this in ixgbe_type.h for some reason
		// pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
		txdctl &= ~(0x7F | (0x7F << 8) | (0x7F << 16)); // clear bits
		txdctl |= (36 | (8 << 8) | (4 << 16)); // from DPDK
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TXDCTL(i), txdctl);

        p_tx_ring_buffers[i]->linkDescriptor2DMAMemory(mem);
        p_tx_ring_buffers[i]->preparePktBuffer();
	}
	// final step: enable DMA
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_DMATXCTL, IXGBE_DMATXCTL_TE);
	return true;

}
bool VFIODev::_prepare_rx_queue(){
	for (uint16_t queue_id = 0; queue_id < m_basic_para.num_rx_queues; queue_id++){
		// enable queue and wait if necessary
		set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_RXDCTL(queue_id), IXGBE_RXDCTL_ENABLE);
		wait_set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RXDCTL(queue_id), IXGBE_RXDCTL_ENABLE);
		// rx queue starts out full
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDH(queue_id), 0);
		// was set to 0 before in the init function
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDT(queue_id), m_num_rx_bufs - 1);
		// Implementation of RX queue preparation
	}

	return true;
}

bool VFIODev::_prepare_tx_queue(){
	for (uint16_t queue_id = 0; queue_id < m_basic_para.num_tx_queues; queue_id++){
		debug("starting tx queue %d", queue_id);
		// tx queue starts out empty
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDH(queue_id), 0);
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDT(queue_id), 0);
		// enable queue and wait if necessary
		set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
		wait_set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
		// Implementation of TX queue preparation
	}
		return true;
}
void VFIODev::_enable_msi_interrupt(uint16_t queue_id){
	// Step 1: The software driver associates between Tx and Rx interrupt causes and the EICR
	// register by setting the IVAR[n] registers.
	set_ivar(m_basic_para.p_bar_addr[0], 0, queue_id, 0);

	// Step 2: Program SRRCTL[n].RDMTS (per receive queue) if software uses the receive
	// descriptor minimum threshold interrupt
	// We don't use the minimum threshold interrupt

	// Step 3: All interrupts should be set to 0b (no auto clear in the EIAC register). Following an
	// interrupt, software might read the EICR register to check for the interrupt causes.
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EIAC, 0x00000000);

	// Step 4: Set the auto mask in the EIAM register according to the preferred mode of operation.
	// In our case we prefer not auto-masking the interrupts

	// Step 5: Set the interrupt throttling in EITR[n] and GPIE according to the preferred mode of operation.
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EITR(queue_id), m_interrupt_para.itr_rate);

	// Step 6: Software clears EICR by writing all ones to clear old interrupt causes
	_dev_clear_interrupts();

	// Step 7: Software enables the required interrupt causes by setting the EIMS register
	u32 mask = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EIMS);
	mask |= (1 << queue_id);
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EIMS, mask);
	debug("Using MSI interrupts");
}

void VFIODev::_enable_msix_interrupt(uint16_t queue_id){
	// Step 1: The software driver associates between interrupt causes and MSI-X vectors and the
	// throttling timers EITR[n] by programming the IVAR[n] and IVAR_MISC registers.
	uint32_t gpie = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GPIE);
	gpie |= IXGBE_GPIE_MSIX_MODE | IXGBE_GPIE_PBA_SUPPORT | IXGBE_GPIE_EIAME;
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GPIE, gpie);
	set_ivar(m_basic_para.p_bar_addr[0], 0, queue_id, queue_id);

	// Step 2: Program SRRCTL[n].RDMTS (per receive queue) if software uses the receive
	// descriptor minimum threshold interrupt
	// We don't use the minimum threshold interrupt

	// Step 3: The EIAC[n] registers should be set to auto clear for transmit and receive interrupt
	// causes (for best performance). The EIAC bits that control the other and TCP timer
	// interrupt causes should be set to 0b (no auto clear).
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EIAC, IXGBE_EIMS_RTX_QUEUE);

	// Step 4: Set the auto mask in the EIAM register according to the preferred mode of operation.
	// In our case we prefer to not auto-mask the interrupts

	// Step 5: Set the interrupt throttling in EITR[n] and GPIE according to the preferred mode of operation.
	// 0x000 (0us) => ... INT/s
	// 0x008 (2us) => 488200 INT/s
	// 0x010 (4us) => 244000 INT/s
	// 0x028 (10us) => 97600 INT/s
	// 0x0C8 (50us) => 20000 INT/s
	// 0x190 (100us) => 9766 INT/s
	// 0x320 (200us) => 4880 INT/s
	// 0x4B0 (300us) => 3255 INT/s
	// 0x640 (400us) => 2441 INT/s
	// 0x7D0 (500us) => 2000 INT/s
	// 0x960 (600us) => 1630 INT/s
	// 0xAF0 (700us) => 1400 INT/s
	// 0xC80 (800us) => 1220 INT/s
	// 0xE10 (900us) => 1080 INT/s
	// 0xFA7 (1000us) => 980 INT/s
	// 0xFFF (1024us) => 950 INT/s
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EITR(queue_id), m_interrupt_para.itr_rate);

	// Step 6: Software enables the required interrupt causes by setting the EIMS register
	u32 mask = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EIMS);
	mask |= (1 << queue_id);
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_EIMS, mask);
	debug("Using MSIX interrupts");
}

bool VFIODev::enable_interrupt(){
	for (uint16_t queue_id = 0; queue_id < m_basic_para.num_rx_queues; queue_id++)
	{
		if (!m_interrupt_para.interrupt_queues[queue_id].interrupt_enabled) {
		return false;
	}
	switch (m_interrupt_para.interrupt_type) {
		case VFIO_PCI_MSIX_IRQ_INDEX:
			_enable_msix_interrupt(queue_id);
			break;
		case VFIO_PCI_MSI_IRQ_INDEX:
			_enable_msi_interrupt(queue_id);
			break;
		default:
			warn("Interrupt type not supported: %d", m_interrupt_para.interrupt_type);
			return false;
	}
	}
	return true;
}

bool VFIODev::set_promisc(bool enable){
	if (enable) {
		info("enabling promisc mode");
		set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
	} else {
		info("disabling promisc mode");
		clear_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
	}
	return true;
}


bool VFIODev::_initialize_interrupt(){
    debug("entered VFIODev::_initialize_interrupt");
	return
	this->_host_setup_IRQ_type()			&&
	this->_host_alloc_IRQ_queues()			&&
	this->_host_setup_IRQ_queues()		;
}

bool VFIODev::_host_setup_IRQ_type(){
    debug("entered VFIODev::_host_setup_IRQ_type");
	if (!m_fds.device_fd) {
		return false;
	}
	info("Setup VFIO Interrupts");

	for (int i = VFIO_PCI_MSIX_IRQ_INDEX; i >= 0; i--) {
		struct vfio_irq_info irq = {};
        irq.argsz = sizeof(irq);
        irq.index = i;
		ioctl(m_fds.device_fd, VFIO_DEVICE_GET_IRQ_INFO, &irq);
		/* if this vector cannot be used with eventfd continue with next*/
		if ((irq.flags & VFIO_IRQ_INFO_EVENTFD) == 0) {
			debug("IRQ doesn't support Event FD");
			continue;
		}
		this->m_interrupt_para.interrupt_type = i;
        debug("Using IRQ type %d with %d vectors", i, irq.count);
        return true;
	}
    return false;
}
bool VFIODev::_host_alloc_IRQ_queues(){
    debug("entered VFIODev::_host_alloc_IRQ_queues");
	this->m_interrupt_para.interrupt_queues.resize(m_basic_para.num_rx_queues);
	return true;
}
int VFIODev::_vfio_enable_msi(){
	debug("Enable MSI Interrupts");
	char irq_set_buf[IRQ_SET_BUF_LEN];
	struct vfio_irq_set* irq_set;
	int* fd_ptr;

	// setup event fd
	int event_fd = eventfd(0, 0);

	irq_set = reinterpret_cast<struct vfio_irq_set*>(irq_set_buf);
	irq_set->argsz = sizeof(irq_set_buf);
	irq_set->count = 1;
	irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSI_IRQ_INDEX;
	irq_set->start = 0;
	fd_ptr = reinterpret_cast<int*>(&irq_set->data);
	*fd_ptr = event_fd;
	int ret = ioctl(m_fds.device_fd, VFIO_DEVICE_SET_IRQS, irq_set);
	if (ret < 0 )
	{
		error("Failed to set MSIX IRQS");
		return -1;
	}
	return event_fd;
}

int VFIODev::_vfio_enable_msix(int index){
	info("Enable MSIX Interrupts");
	char irq_set_buf[MSIX_IRQ_SET_BUF_LEN];
	struct vfio_irq_set* irq_set;
	int* fd_ptr;

	// setup event fd
	int event_fd = eventfd(0, 0);

	irq_set = reinterpret_cast<struct vfio_irq_set*>(irq_set_buf);
	irq_set->argsz = sizeof(irq_set_buf);

	if (!index) {
		index = 1;
	} else if (index > MAX_INTERRUPT_VECTORS)
		index = MAX_INTERRUPT_VECTORS + 1;

	irq_set->count = index;
	irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSIX_IRQ_INDEX;
	irq_set->start = 0;
	fd_ptr = reinterpret_cast<int*>(&irq_set->data);
	*fd_ptr = event_fd;

	int ret = ioctl(m_fds.device_fd, VFIO_DEVICE_SET_IRQS, irq_set);
	if (ret < 0) {
		error("Failed to set MSIX IRQS");
		return -1;
	}
	return event_fd;
}

int VFIODev::_vfio_epoll_ctl(int event_fd){
	struct epoll_event event;
	event.events = EPOLLIN;
	event.data.fd = event_fd;

	int epoll_fd = (int) check_err(epoll_create1(0), "to created epoll");

	int ret = epoll_ctl(epoll_fd, EPOLL_CTL_ADD, event_fd, &event);
	if (ret < 0) {
		error("Failed to add event fd to epoll instance");
		return -1;
	}
	return epoll_fd;
}

bool VFIODev::_host_setup_IRQ_queues(){
	switch (m_interrupt_para.interrupt_type) {	
		case VFIO_PCI_MSIX_IRQ_INDEX: {
			for (uint32_t rx_queue = 0; rx_queue < m_basic_para.num_rx_queues; rx_queue++) {
				int vfio_event_fd = _vfio_enable_msix(rx_queue);
				int vfio_epoll_fd = _vfio_epoll_ctl(vfio_event_fd);
				m_interrupt_para.interrupt_queues[rx_queue].vfio_event_fd = vfio_event_fd;
				m_interrupt_para.interrupt_queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				m_interrupt_para.interrupt_queues[rx_queue].moving_avg.length = 0;
				m_interrupt_para.interrupt_queues[rx_queue].moving_avg.index = 0;
				m_interrupt_para.interrupt_queues[rx_queue].interval = INTERRUPT_INITIAL_INTERVAL;
			}
			break;
		}
		case VFIO_PCI_MSI_IRQ_INDEX: {
			int vfio_event_fd = _vfio_enable_msi();
			int vfio_epoll_fd = _vfio_epoll_ctl(vfio_event_fd);
			for (uint32_t rx_queue = 0; rx_queue < m_basic_para.num_rx_queues; rx_queue++) {
				m_interrupt_para.interrupt_queues[rx_queue].vfio_event_fd = vfio_event_fd;
				m_interrupt_para.interrupt_queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				m_interrupt_para.interrupt_queues[rx_queue].moving_avg.length = 0;
				m_interrupt_para.interrupt_queues[rx_queue].moving_avg.index = 0;
				m_interrupt_para.interrupt_queues[rx_queue].interval = INTERRUPT_INITIAL_INTERVAL;
			}
			break;
		}
		default:
			warn("Interrupt type not supported: %d", m_interrupt_para.interrupt_type);
			return false;
	}
	return true;
}


uint32_t VFIODev::_get_link_speed(){
	uint32_t links = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_LINKS);
	if (!(links & IXGBE_LINKS_UP)) {
		return 0;
	}
	switch (links & IXGBE_LINKS_SPEED_82599) {
		case IXGBE_LINKS_SPEED_100_82599:
			return 100;
		case IXGBE_LINKS_SPEED_1G_82599:
			return 1000;
		case IXGBE_LINKS_SPEED_10G_82599:
			return 10000;
		default:
			return 0;
	}
}

bool VFIODev::wait_for_link(){
	info("Waiting for link...");
	int32_t max_wait = 10000000; // 10 seconds in us
	uint32_t poll_interval = 100000; // 10 ms in us
	while (!(_get_link_speed()) && max_wait > 0) {
		usleep(poll_interval);
		max_wait -= poll_interval;
	}
	info("Link speed is %d Mbit/s", _get_link_speed());
	return true;
}