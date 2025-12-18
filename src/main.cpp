#include <memory>
#include "vfio_dev.h"
#include "log.h"

int main() {
    std::unique_ptr<BasicDev> device = std::make_unique<VfioDev>(
                                        "0000:04:00.0",
                                        1,
                                        1
    );
    device->getFD()         ;
    device->enableDMA()     ;             
    device->mapBAR(0)       ;             
    device->initHardware()  ;
    device->initRingBuffer();
    device->enable_interrupt() ;
    device->set_promisc(true)  ;
    device->wait_for_link()     ;            
    device->initMemPool()   ;
    return 0;
}