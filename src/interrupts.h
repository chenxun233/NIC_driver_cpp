#ifndef INTERRUPTS_H
#define INTERRUPTS_H
#include <cstdint>

class interrupt_handler {
    public:
        interrupt_handler(class vfio_device& device);
        ~interrupt_handler();

    private:
        bool _set_interrupt_type(class vfio_device& device);
        uint8_t interrupt_type;
        int interrupt_fd;
};
#endif // INTERRUPTS_H