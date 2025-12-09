#ifndef IMPLEMENTATION_H
#define IMPLEMENTATION_H
#include "vfio_dev.h"
#include "non_vfio_dev.h"

typedef std::unique_ptr<BasicDev> dev_ptr_type;

dev_ptr_type create_device(
                            std::string pci_addr,
                            uint8_t     bar_index_max,
                            uint16_t    num_rx_queues,
                            uint16_t    num_tx_queues,
                            uint16_t    interrupt_timeout_ms
);


#endif // IMPLEMENTATION_H