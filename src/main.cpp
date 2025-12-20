#include <memory>
#include "vfio_dev.h"
#include "log.h"

#define HUGE_PAGE_BITS 21
#define HUGE_PAGE_SIZE (1 << HUGE_PAGE_BITS) // 2_097_152 = 2MiB
#define SIZE_PKT_BUF_HEADROOM 40
#define MIN_NUM_OF_BUF 4096
#define PKT_BUF_SIZE 2048
#define PKT_SIZE 60

const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;
#define NUM_OF_BUF_RX_QUEUE 512
#define NUM_OF_BUF_TX_QUEUE 512

int main() {
    std::unique_ptr<BasicDev> device = std::make_unique<VFIODev>("0000:04:00.0",0);
           
    device->setRxRingBuffers(1,NUM_OF_BUF_RX_QUEUE, PKT_BUF_SIZE);
    device->setTxRingBuffers(1,NUM_OF_BUF_TX_QUEUE, PKT_BUF_SIZE);
    device->initHardware(INTERRUPT_INITIAL_INTERVAL)                ;
    device->setDescriptorRings()    ;
    device->enableDevQueues()       ;
    device->enableDevInterrupt()    ;
    device->setPromisc(true)        ;
    device->wait4Link()             ;            
    device->initTxDataMemPool()     ;
    return 0;
}