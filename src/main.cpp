#include "implementation.h"
#include "log.h"

int main() {
    basic_para_type params{
        .pci_addr = "0000:04:00.0",
        .bar_index = 0,
        .num_rx_queues = 4,
        .num_tx_queues = 4,
        .interrupt_timeout_ms = 1000
    };
    dev_ptr_type device = create_device(params);
    if (!device) {
        warn("Failed to create and initialize device");
        return -1;
    }
    return 0;
}