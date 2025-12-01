#ifndef INTERRUPTS_H
#define INTERRUPTS_H
#include <cstdint>

class interrupt {
    public:
        interrupt(class vfio_device& device);
        ~interrupt();

    private:
        bool _set_interrupt_type(class vfio_device& device);
        uint8_t interrupt_type;
        int interrupt_fd;
};
#endif // INTERRUPTS_H