#include <memory>
#include "vfio_dev.h"
#include <thread>
#include <pthread.h>





std::unique_ptr<BasicDev> createDevice(const std::string pci_addr, 
                                        const uint8_t max_bar_index, 
                                        const uint8_t num_of_quueue, 
                                        const uint16_t num_of_buf, 
                                        const uint32_t buf_size, 
                                        const uint64_t INTERRUPT_INITIAL_INTERVAL, 
                                        const uint32_t timeout_ms);
