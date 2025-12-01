#include "interrupts.h"
#include "lib_vfio.h"
#include "log.h"
#include <sys/ioctl.h>


interrupt_handler::interrupt_handler(class vfio_device& device){
if (device.is_interrupt_enabled()) {
    this->_set_interrupt_type(device);
}
}

interrupt_handler::~interrupt_handler(){

}



bool interrupt_handler::_set_interrupt_type(class vfio_device& device){
	info("Setup VFIO Interrupts");

	for (int i = VFIO_PCI_MSIX_IRQ_INDEX; i >= 0; i--) {
		struct vfio_irq_info irq = {};
        irq.argsz = sizeof(irq);
        irq.index = i;

		check_err(ioctl(device.get_device_fd(), VFIO_DEVICE_GET_IRQ_INFO, &irq), "get IRQ Info");

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