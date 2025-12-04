#include "interrupts.h"
#include "log.h"
#include <sys/ioctl.h>
#include <linux/vfio.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include "device.h"
#include "ixgbe_type.h"


const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;

interrupt::interrupt(basic_para_type& basic_para, vfio_fd_type& fds
):
m_para(
	basic_para,
	fds,
	0x28,
	nullptr,
	0
)

{
	_initialize();
}

interrupt::~interrupt(){

}
bool interrupt::_initialize(){
	return
	this->_host_setup_IRQ_type()			&&
	this->_host_alloc_IRQ_queues()			&&
	this->_host_setup_IRQ_queues()		;
}

bool interrupt::_host_setup_IRQ_type(){
	if (!m_para.fds.device_fd) {
		return false;
	}
	info("Setup VFIO Interrupts");

	for (int i = VFIO_PCI_MSIX_IRQ_INDEX; i >= 0; i--) {
		struct vfio_irq_info irq = {};
        irq.argsz = sizeof(irq);
        irq.index = i;
		ioctl(m_para.fds.device_fd, VFIO_DEVICE_GET_IRQ_INFO, &irq);
		/* if this vector cannot be used with eventfd continue with next*/
		if ((irq.flags & VFIO_IRQ_INFO_EVENTFD) == 0) {
			debug("IRQ doesn't support Event FD");
			continue;
		}
		this->m_para.interrupt_type = i;
        debug("Using IRQ type %d with %d vectors", i, irq.count);
        return true;
	}
    return false;
}
bool interrupt::_host_alloc_IRQ_queues(){
	this->m_para.interrupt_queues = std::make_unique<interrupt_queues[]>(this->m_para.basic.num_rx_queues);
	return true;
}
int interrupt::_vfio_enable_msi(){
	info("Enable MSI Interrupts");
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
	int ret = ioctl(m_para.fds.device_fd, VFIO_DEVICE_SET_IRQS, irq_set);
	if (ret < 0 )
	{
		error("Failed to set MSIX IRQS");
		return -1;
	}
	return event_fd;
}

int interrupt::_vfio_enable_msix(int index){
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

	int ret = ioctl(m_para.fds.device_fd, VFIO_DEVICE_SET_IRQS, irq_set);
	if (ret < 0) {
		error("Failed to set MSIX IRQS");
		return -1;
	}
	return event_fd;
}

int interrupt::_vfio_epoll_ctl(int event_fd){
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


bool interrupt::_host_setup_IRQ_queues(){
	switch (m_para.interrupt_type) {	
		case VFIO_PCI_MSIX_IRQ_INDEX: {
			for (uint32_t rx_queue = 0; rx_queue < m_para.basic.num_rx_queues; rx_queue++) {
				int vfio_event_fd = _vfio_enable_msix(rx_queue);
				int vfio_epoll_fd = _vfio_epoll_ctl(vfio_event_fd);
				m_para.interrupt_queues[rx_queue].vfio_event_fd = vfio_event_fd;
				m_para.interrupt_queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				m_para.interrupt_queues[rx_queue].moving_avg.length = 0;
				m_para.interrupt_queues[rx_queue].moving_avg.index = 0;
				m_para.interrupt_queues[rx_queue].interval = INTERRUPT_INITIAL_INTERVAL;
			}
			break;
		}
		case VFIO_PCI_MSI_IRQ_INDEX: {
			int vfio_event_fd = _vfio_enable_msi();
			int vfio_epoll_fd = _vfio_epoll_ctl(vfio_event_fd);
			for (uint32_t rx_queue = 0; rx_queue < m_para.basic.num_rx_queues; rx_queue++) {
				m_para.interrupt_queues[rx_queue].vfio_event_fd = vfio_event_fd;
				m_para.interrupt_queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				m_para.interrupt_queues[rx_queue].moving_avg.length = 0;
				m_para.interrupt_queues[rx_queue].moving_avg.index = 0;
				m_para.interrupt_queues[rx_queue].interval = INTERRUPT_INITIAL_INTERVAL;
			}
			break;
		}
		default:
			warn("Interrupt type not supported: %d", m_para.interrupt_type);
			return false;
	}
	return true;
}
