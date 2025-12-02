#include "interrupts.h"
#include "log.h"
#include <sys/ioctl.h>
#include <linux/vfio.h>


interrupt::interrupt(int device_fd,int interrupt_timeout_ms):
interrupt_timeout_ms(interrupt_timeout_ms)
{
	this->_set_interrupt_type(device_fd);
}

interrupt::~interrupt(){

}

bool interrupt::_set_interrupt_type(int device_fd){
	if (!device_fd) {
		return false;
	}
	info("Setup VFIO Interrupts");

	for (int i = VFIO_PCI_MSIX_IRQ_INDEX; i >= 0; i--) {
		struct vfio_irq_info irq = {};
        irq.argsz = sizeof(irq);
        irq.index = i;
		ioctl(device_fd, VFIO_DEVICE_GET_IRQ_INFO, &irq);
		/* if this vector cannot be used with eventfd continue with next*/
		if ((irq.flags & VFIO_IRQ_INFO_EVENTFD) == 0) {
			debug("IRQ doesn't support Event FD");
			continue;
		}
		this->interrupt_type = i;
        debug("Using IRQ type %d with %d vectors", i, irq.count);
        return true;
	}
    return false;
}