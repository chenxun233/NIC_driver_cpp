#ifndef INTERRUPTS_H
#define INTERRUPTS_H
#include <cstdint>

class interrupt {
    public:
        interrupt(int device_fd,int interrupt_timeout_ms);
        ~interrupt();

    private:
        bool _set_interrupt_type(int device_fd);
        int interrupt_timeout_ms;
        uint8_t interrupt_type;
        int interrupt_fd;
};
#endif // INTERRUPTS_H