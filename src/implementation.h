#ifndef IMPLEMENTATION_H
#define IMPLEMENTATION_H
#include "vfio_dev.h"
#include "non_vfio_dev.h"


std::unique_ptr<BasicDev> return_p_device(
                            std::string pci_addr,
                            uint8_t     bar_index_max,
                            uint16_t    num_rx_queues,
                            uint16_t    num_tx_queues,
                            uint16_t    interrupt_timeout_ms
);


#endif // IMPLEMENTATION_H