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
#define BATCH_SIZE 32 // the number of pkt to be sent per time
#define TX_CLEAN_BATCH 32 // the number of tx descriptors to clean in one batch
#define wrap_ring(index, ring_size) (uint16_t) ((index + 1) & (ring_size - 1))
#include <string>


static const char pkt_data[] = {
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
	// rest of the payload is zero-filled because mempools guarantee empty bufs_with_data
};

VFIODev::VFIODev(std::string pci_addr, uint8_t max_bar_index) :
BasicDev(pci_addr,max_bar_index)
{
	 		_getFD()     				&&
			_mapBAR (max_bar_index)     &&
			_enableDMA()                ;
}

VFIODev::~VFIODev(){
};

bool VFIODev::_getFD() {
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
    success("VFIO group id for device %s is %d", this->m_basic_para.pci_addr.c_str(), group_id);
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
    success("VFIO container fd acquired: %d", cfd);
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
    success("VFIO group fd acquired: %d", gfd);
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
    success("VFIO group %d added to container %d", this->m_fds.group_fd, this->m_fds.container_fd);
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
    success("VFIO device fd acquired: %d", dfd);
    return true;
}

bool VFIODev::_mapBAR (uint8_t bar_index) {
    m_basic_para.max_bar_index = bar_index;
    if (m_basic_para.max_bar_index > VFIO_PCI_BAR5_REGION_INDEX){
        warn("BAR index %d is out of range", m_basic_para.max_bar_index);
        return false;
    }
    if (this->m_fds.device_fd == -1) {
        warn("Device fd is invalid");
        return false;
    }
    for (int i = 0; i <= m_basic_para.max_bar_index; i++) {
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
		success("BAR %d mapped at address %p, size: %llu bytes", i, (void*)temp_addr, region_info.size);
 
    }
    return true;         
};


bool VFIODev::_enableDMA() {
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
	success("DMA enabled on device %s", this->m_basic_para.pci_addr.c_str());
    return true;
}





bool VFIODev::initHardware(const int interrupt_interval) {
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
	(void)this->_readStatus();
    this->_initialize_interrupt(interrupt_interval);
	success("Hardware initialized");
    return true;
};

bool VFIODev::setDescriptorRings() {
	if (m_num_rx_bufs == 0 || m_buf_rx_size == 0 ||
		m_num_tx_bufs == 0 || m_buf_tx_size == 0) {
		warn("RX or TX buffer parameters not set");
		return false;
	}
	info("Setting up descriptor rings...");
	this->_setRxDescriptorRing();
	// section 4.6.8 - init tx
	this->_setTxDescriptorRing();
	info("Descriptor rings set up");
    return true;
}

bool VFIODev::enableDevQueues() {
    debug("entered VFIODev::enableDevQueues");
	this->_enableDevRxQueue();
	this->_enableDevTxQueue();
    return true;
}


bool VFIODev::setRxRingBuffers(uint16_t num_rx_queues,uint32_t num_buf, uint32_t buf_size){
	info("settingRxRingBuffers");
    m_basic_para.num_rx_queues = num_rx_queues;
    m_num_rx_bufs = num_buf;
    m_buf_rx_size = buf_size;
    for (uint16_t i = 0; i < m_basic_para.num_rx_queues; i++) {
		// p_mempool.push_back(new MemoryPool(num_buf, buf_size, m_fds.container_fd));
        p_rx_ring_buffers.push_back(new IXGBE_RxRingBuffer);
        p_rx_ring_buffers[i]->linkMemoryPool(new MemoryPool(num_buf, buf_size, m_fds.container_fd));
		success("Linked memory pool to RX ring buffer %d", i);
    }
    return true;
}

bool VFIODev::setTxRingBuffers(uint16_t num_tx_queues,uint32_t num_buf, uint32_t buf_size){
    m_basic_para.num_tx_queues = num_tx_queues;
    m_num_tx_bufs = num_buf;
    m_buf_tx_size = buf_size;
    for (uint16_t i = 0; i < m_basic_para.num_tx_queues; i++) {
        p_tx_ring_buffers.push_back(new IXGBE_TxRingBuffer);
		p_tx_ring_buffers[i]->linkMemoryPool(new MemoryPool(num_buf, buf_size, m_fds.container_fd));
    }
    return true;
}

DevStatus VFIODev::_readStatus(){
	uint32_t rx_pkts = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GPRC);
	uint32_t tx_pkts = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GPTC);
	uint64_t rx_bytes = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GORCL) + (((uint64_t) get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GORCH)) << 32);
	uint64_t tx_bytes = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GOTCL) + (((uint64_t) get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_GOTCH)) << 32);

	m_dev_stats.rx_pkts  += rx_pkts;
	m_dev_stats.tx_pkts  += tx_pkts;
	m_dev_stats.rx_bytes += rx_bytes;
	m_dev_stats.tx_bytes += tx_bytes;
	return m_dev_stats;
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


bool VFIODev::sendOnQueue(uint8_t* p_data, size_t size, uint16_t queue_id){ 
	(void)p_data;
	(void)size;
	(void)queue_id;
	return true; }
// 	if (queue_id >= m_basic_para.num_tx_queues) {
// 		warn("Invalid TX queue id %d", queue_id);
// 		return false;
// 	}
// 	struct pkt_buf* buf = p_tx_ring_buffers[queue_id]->getMemPool()->takeOutPktBuf();
// 	if (!buf) return false;
// 	buf->size = size;
// 	memcpy(buf->data, p_data, size);
// 	p_tx_ring_buffers[queue_id]->getMemPool()->pushBackPktBuf(buf);

// 	for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
// 		struct pkt_buf* buf = p_tx_mempool->takeOutPktBuf();
// 		buf->size = PKT_SIZE;
// 		memcpy(buf->data, pkt_data, sizeof(pkt_data));
// 		*(uint16_t*) (buf->data + 24) = _calc_ip_checksum(buf->data + 14, 20);
// 		bufs_with_data[buf_id] = buf;
// 	}
// 	// return them all to the mempool, all future allocations will return bufs_with_data with the data set above
// 	for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
// 		p_tx_mempool->pushBackPktBuf(bufs_with_data[buf_id]);
// 	}
// 	return true;
// }

bool VFIODev::fillTxMemPool(uint32_t num_buf){
	m_used_tx_buf_num = num_buf;
	struct MemoryPool* mempool = p_tx_ring_buffers[0]->getMemPool();
	// pre-fill all our packet buffers with some templates that can be modified later
	// we have to do it like this because sending is async in the hardware; we cannot re-use a buffer immediately
	
	struct pkt_buf** bufs_with_data = new struct pkt_buf*[num_buf];
	for (uint32_t buf_id = 0; buf_id < num_buf; buf_id++) {
		struct pkt_buf* buf = mempool->takeOutPktBuf();
		buf->size = PKT_SIZE;
		memcpy(buf->data, pkt_data, sizeof(pkt_data));
		*(uint16_t*) (buf->data + 24) = _calc_ip_checksum(buf->data + 14, 20);
		bufs_with_data[buf_id] = buf;
	}
	// return them all to the mempool, all future allocations will return bufs_with_data with the data set above
	for (uint32_t buf_id = 0; buf_id < num_buf; buf_id++) {
		mempool->pushBackPktBuf(bufs_with_data[buf_id]);
	}
	delete [] bufs_with_data;
	return true;
}

void VFIODev::send(){
    IXGBE_TxRingBuffer *tx_ring = p_tx_ring_buffers[0];
	
	uint64_t last_stats_printed = BasicDev::_monotonic_time();
	uint64_t counter = 0;
	struct DevStatus stats_old, stats;
	_initStatus(&stats);
	_initStatus(&stats_old);

	// array of bufs_with_data sent out in a batch
	struct pkt_buf* bufs_with_data[BATCH_SIZE];
	// tx loop
	for (;;) {
		// we cannot immediately recycle packets, we need to allocate new packets every time
		// the old packets might still be used by the NIC: tx is async
		(void)tx_ring->getMemPool()->takePktBuf(bufs_with_data, BATCH_SIZE);
		// track stats
		

	// the descriptor is explained in section 7.2.3.2.4
	// we just use a struct copy & pasted from intel, but it basically has two formats (hence a union):
	// 1. the write-back format which is written by the NIC once sending it is finished this is used in step 1
	// 2. the read format which is read by the NIC and written by us, this is used in step 2

	uint16_t clean_index = tx_ring->getCleanIndex(); // next descriptor to clean up

	// step 1: clean up descriptors that were sent out by the hardware and return them to the mempool
	// start by reading step 2 which is done first for each packet
	// cleaning up must be done in batches for performance reasons, so this is unfortunately somewhat complicated
	while (true) {
		// figure out how many descriptors can be cleaned up
		int32_t cleanable = tx_ring->getTxIndex() - clean_index; // tx_index is always ahead of clean (invariant of our queue)
		if (cleanable < 0) { // handle wrap-around
			cleanable = m_used_tx_buf_num + cleanable;
		}
		if (cleanable < TX_CLEAN_BATCH) {
			break;
		}
		// calculcate the index of the last transcriptor in the clean batch
		// Only clean when the cleanable number is more than TX_CLEAN_BATCH.
		int32_t cleanup_to = clean_index + TX_CLEAN_BATCH - 1;
		if ((uint32_t)cleanup_to >= m_used_tx_buf_num) {
			cleanup_to -= m_used_tx_buf_num;
		}
		volatile union ixgbe_adv_tx_desc* txd = tx_ring->getDescriptors() + cleanup_to;
		uint32_t status = txd->wb.status;
		// hardware sets this flag as soon as it's sent out, we can give back all bufs_with_data in the batch back to the mempool
		if (status & IXGBE_ADVTXD_STAT_DD) {
			int32_t i = clean_index;
			while (true) {
				struct pkt_buf* buf = (struct pkt_buf*) tx_ring->getMemPool()->getUsedBufAddr(i);
				tx_ring->getMemPool()->pushBackPktBuf(buf);
				if (i == cleanup_to) {
					break;
				}
				i = wrap_ring(i, m_used_tx_buf_num);
			}
			// next descriptor to be cleaned up is one after the one we just cleaned
			clean_index = wrap_ring(cleanup_to, m_used_tx_buf_num);
		} else {
			// clean the whole batch or nothing; yes, this leaves some packets in
			// the queue forever if you stop transmitting, but that's not a real concern
			break;
		}
	}
	tx_ring->setCleanIndex(clean_index);
	// step 2: send out as many of our packets as possible
	uint32_t sent;
	for (sent = 0; sent < BATCH_SIZE; sent++) {
		uint32_t next_index = wrap_ring(tx_ring->getTxIndex(), tx_ring->getMemPool()->getNumOfBufs());
		// we are full if the next index is the one we are trying to reclaim
		if (clean_index == next_index) {
			break;
		}
		struct pkt_buf* buf = bufs_with_data[sent];
		// remember virtual address to clean it up later
		tx_ring->getMemPool()->setUsedBufAddr(tx_ring->getTxIndex(), (void*) buf);
		volatile union ixgbe_adv_tx_desc* txd = tx_ring->getDescriptors() + tx_ring->getTxIndex();
		tx_ring->setTxIndex(next_index);
		// NIC reads from here
		uintptr_t data_offset = (uintptr_t)(buf->data - (uint8_t*)buf);
		txd->read.buffer_addr = buf->iova + data_offset;
		// always the same flags: one buffer (EOP), advanced data descriptor, CRC offload, data length
		txd->read.cmd_type_len =
			IXGBE_ADVTXD_DCMD_EOP | IXGBE_ADVTXD_DCMD_RS | IXGBE_ADVTXD_DCMD_IFCS | IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_DATA | buf->size;
		// no fancy offloading stuff - only the total payload length
		// implement offloading flags here:
		// 	* ip checksum offloading is trivial: just set the offset
		// 	* tcp/udp checksum offloading is more annoying, you have to precalculate the pseudo-header checksum
		txd->read.olinfo_status = buf->size << IXGBE_ADVTXD_PAYLEN_SHIFT;
	}
	// send out by advancing tail, i.e., pass control of the bufs_with_data to the nic
	// this seems like a textbook case for a release memory order, but Intel's driver doesn't even use a compiler barrier here
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDT(0), tx_ring->getTxIndex());

	if ((counter++ & 0xFFF) == 0) {
		uint64_t time = BasicDev::_monotonic_time();
		if (time - last_stats_printed > 1000 * 1000 * 1000) {
			// every second
			if (bufs_with_data[0]) {
				printf("bufs_with_data[0] (%u bytes): ", bufs_with_data[0]->size);
				for (uint32_t i = 0; i < bufs_with_data[0]->size; i++) {
					printf("%02x ", bufs_with_data[0]->data[i]);
				}
				printf("\n");
			}
			stats = this->_readStatus();
			_print_stats_diff(&stats, &stats_old, time - last_stats_printed);
			stats_old = stats;
			last_stats_printed = time;
		}
	}
}


}

void VFIODev::_initStatus(DevStatus* stats){
	stats->rx_bytes = 0;
	stats->rx_pkts = 0;
	stats->tx_bytes = 0;
	stats->tx_pkts = 0;
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


bool VFIODev::_setRxDescriptorRing(){
	info("initializing RX descriptor rings");
	if (m_num_rx_bufs == 0 || m_buf_rx_size == 0) {
		warn("RX buffer parameters not set");
		return false;
	}
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
		DMAMemoryPair DMA_mem_pair = DMAMemoryAllocator::getInstance().allocDMAMemory(ring_size_bytes, this->m_fds.container_fd);
		// neat trick from Snabb: initialize to 0xFF to prevent rogue memory accesses on premature DMA activation
		memset(DMA_mem_pair.virt, -1, ring_size_bytes);
		// tell the device where it can write to (its iova, so its view)
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDBAL(i), (uint32_t) (DMA_mem_pair.iova & 0xFFFFFFFFull));
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDBAH(i), (uint32_t) (DMA_mem_pair.iova >> 32));
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDLEN(i), ring_size_bytes);
		// set ring to empty at start
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDH(i), 0);
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_RDT(i), 0);

		// private data for the driver, 0-initialized
        if (p_rx_ring_buffers[i]== nullptr){
            error("RX ring buffer %d is nullptr", i);
        }
        p_rx_ring_buffers[i]->allocDMAMem2DescRing(DMA_mem_pair);
		p_rx_ring_buffers[i]->linkDescWithPKTBuf();

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
	success("finished RX initialization");
	return true;
}

bool VFIODev::_setTxDescriptorRing(){
	if (m_num_tx_bufs == 0 || m_buf_tx_size == 0) {
		warn("TX buffer parameters not set");
		return false;
	}
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
		DMAMemoryPair DMA_mem_pair = DMAMemoryAllocator::getInstance().allocDMAMemory(ring_size_bytes, this->m_fds.container_fd);
		memset(DMA_mem_pair.virt, -1, ring_size_bytes);
		// tell the device where it can write to (its iova, so its view)
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDBAL(i), (uint32_t) (DMA_mem_pair.iova & 0xFFFFFFFFull));
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDBAH(i), (uint32_t) (DMA_mem_pair.iova >> 32));
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDLEN(i), ring_size_bytes);
		debug("tx ring %d phy addr:  0x%012lX", i, DMA_mem_pair.iova);
		debug("tx ring %d virt addr: 0x%012lX", i, (uintptr_t) DMA_mem_pair.virt);

		// descriptor writeback magic values, important to get good performance and low PCIe overhead
		// see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
		// we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
		uint32_t txdctl = get_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TXDCTL(i));
		// there are no defines for this in ixgbe_type.h for some reason
		// pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
		txdctl &= ~(0x7F | (0x7F << 8) | (0x7F << 16)); // clear bits
		txdctl |= (36 | (8 << 8) | (4 << 16)); // from DPDK
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TXDCTL(i), txdctl);
        p_tx_ring_buffers[i]->allocDMAMem2DescRing(DMA_mem_pair);
	}
	// final step: enable DMA
	set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_DMATXCTL, IXGBE_DMATXCTL_TE);
	return true;

}
bool VFIODev::_enableDevRxQueue(){
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

bool VFIODev::_enableDevTxQueue(){
	for (uint16_t queue_id = 0; queue_id < m_basic_para.num_tx_queues; queue_id++){
		debug("starting tx queue %d", queue_id);
		// tx queue starts out empty
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDH(queue_id), 0);
		set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TDT(queue_id), 0);
		// enable queue and wait if necessary
		set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
		wait_set_bar_reg32(m_basic_para.p_bar_addr[0], IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
		// Implementation of TX queue preparation
        debug("finished tx queue %d", queue_id);
	}
		return true;
}
void VFIODev::_enableDevMSIInterrupt(uint16_t queue_id){
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

void VFIODev::_enableDevMSIxInterrupt(uint16_t queue_id){
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

bool VFIODev::enableDevInterrupt(){
    debug("entered VFIODev::enableDevInterrupt");
    if (m_interrupt_para.interrupt_queues.size() != m_basic_para.num_rx_queues) {
        error("Interrupt queues size %d does not match number of rx queues %d", 
            (int)m_interrupt_para.interrupt_queues.size(), 
            m_basic_para.num_rx_queues);
        return false;
    }
	for (uint16_t queue_id = 0; queue_id < m_basic_para.num_rx_queues; queue_id++)
	{
		if (!m_interrupt_para.interrupt_queues[queue_id].interrupt_enabled) {
            warn("Interrupt queue %d not properly initialized", queue_id);
		return false;
	}
	switch (m_interrupt_para.interrupt_type) {
		case VFIO_PCI_MSIX_IRQ_INDEX:
			_enableDevMSIxInterrupt(queue_id);
			break;
		case VFIO_PCI_MSI_IRQ_INDEX:
			_enableDevMSIInterrupt(queue_id);
			break;
		default:
			warn("Interrupt type not supported: %d", m_interrupt_para.interrupt_type);
			return false;
	}
	}
    debug("finished enabling interrupts");
	return true;
}

bool VFIODev::setPromisc(bool enable){
	if (enable) {
		info("enabling promisc mode");
		set_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
	} else {
		info("disabling promisc mode");
		clear_bar_flags32(m_basic_para.p_bar_addr[0], IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
	}
	return true;
}


bool VFIODev::_initialize_interrupt(const int &interrupt_interval){
    debug("entered VFIODev::_initialize_interrupt");
	return
	this->_getDevIRQType()				&&
	this->_allocIRQQueues()				&&
	this->_setupIRQQueues(interrupt_interval);
}

bool VFIODev::_getDevIRQType(){
    debug("entered VFIODev::_getDevIRQType");
	if (m_fds.device_fd<=0) {
		error("Device fd is invalid");
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
bool VFIODev::_allocIRQQueues(){
    debug("entered VFIODev::_allocIRQQueues");
	this->m_interrupt_para.interrupt_queues.resize(m_basic_para.num_rx_queues);
	return true;
}
int VFIODev::_injectEventFdToVFIODev_msi(){
	debug("Enable MSI Interrupts");
	char irq_set_buf[IRQ_SET_BUF_LEN];
	struct vfio_irq_set* irq_set;
	int* fd_ptr;

	// get a fresh event fd
	int event_fd = eventfd(0, 0);

	irq_set = reinterpret_cast<struct vfio_irq_set*>(irq_set_buf);
	irq_set->argsz = sizeof(irq_set_buf);
	irq_set->count = 1;
	irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSI_IRQ_INDEX;
	irq_set->start = 0;
	// inject the event fd into the data portion
	fd_ptr = reinterpret_cast<int*>(&irq_set->data);
	*fd_ptr = event_fd;
	// inject the pipe into the vfio device
	int ret = ioctl(m_fds.device_fd, VFIO_DEVICE_SET_IRQS, irq_set);
	if (ret < 0 )
	{
		error("Failed to set MSIX IRQS");
		return -1;
	}
	// return the injected event fd
	return event_fd;
}

int VFIODev::_injectEventFdToVFIODev_msix(int index){
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

bool VFIODev::_setupIRQQueues(const int &interrupt_interval){
	switch (m_interrupt_para.interrupt_type) {	
		case VFIO_PCI_MSIX_IRQ_INDEX: {
			for (uint32_t rx_queue = 0; rx_queue < m_basic_para.num_rx_queues; rx_queue++) {
				int vfio_event_fd = _injectEventFdToVFIODev_msix(rx_queue);
				int vfio_epoll_fd = _vfio_epoll_ctl(vfio_event_fd);
				m_interrupt_para.interrupt_queues[rx_queue].vfio_event_fd = vfio_event_fd;
				m_interrupt_para.interrupt_queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				m_interrupt_para.interrupt_queues[rx_queue].moving_avg.length = 0;
				m_interrupt_para.interrupt_queues[rx_queue].moving_avg.index = 0;
				m_interrupt_para.interrupt_queues[rx_queue].interval = interrupt_interval;
			}
			break;
		}
		case VFIO_PCI_MSI_IRQ_INDEX: {
			int vfio_event_fd = _injectEventFdToVFIODev_msi();
			int vfio_epoll_fd = _vfio_epoll_ctl(vfio_event_fd);
			for (uint32_t rx_queue = 0; rx_queue < m_basic_para.num_rx_queues; rx_queue++) {
				m_interrupt_para.interrupt_queues[rx_queue].vfio_event_fd = vfio_event_fd;
				m_interrupt_para.interrupt_queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				m_interrupt_para.interrupt_queues[rx_queue].moving_avg.length = 0;
				m_interrupt_para.interrupt_queues[rx_queue].moving_avg.index = 0;
				m_interrupt_para.interrupt_queues[rx_queue].interval = interrupt_interval;
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

bool VFIODev::wait4Link(){
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
