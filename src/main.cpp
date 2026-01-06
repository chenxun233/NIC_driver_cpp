#include <memory>
#include "vfio_dev.h"
#include "log.h"
#include <thread>
#include <pthread.h>


#define PKT_BUF_SIZE 2048
#define PKT_SIZE 60

const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;
uint64_t interrupt_interval = 100;
#define NUM_OF_RX_BUF 512
#define NUM_OF_TX_BUF 512
#define NUM_OF_QUEUE 1


void thread1(){
    std::unique_ptr<BasicDev> device1 = std::make_unique<Intel82599Dev>("0000:04:00.0",0);
    device1->initHardware();
    device1->setRxRingBuffers(NUM_OF_QUEUE,NUM_OF_RX_BUF, PKT_BUF_SIZE);
    device1->setTxRingBuffers(NUM_OF_QUEUE,NUM_OF_TX_BUF, PKT_BUF_SIZE);
    device1->initializeInterrupt(INTERRUPT_INITIAL_INTERVAL);
    device1->enableDevQueues()       ;
    device1->enableDevInterrupt()    ;
    device1->setPromisc(true)        ;
    device1->wait4Link()             ;            
    device1->fillTxMemPool(NUM_OF_TX_BUF);
    device1->send();
}

void thread2(){
    std::unique_ptr<BasicDev> device2 = std::make_unique<Intel82599Dev>("0000:05:00.0",0);
    device2->initHardware();
    device2->setRxRingBuffers(NUM_OF_QUEUE,NUM_OF_RX_BUF, PKT_BUF_SIZE);
    device2->setTxRingBuffers(NUM_OF_QUEUE,NUM_OF_TX_BUF, PKT_BUF_SIZE);
    device2->initializeInterrupt(INTERRUPT_INITIAL_INTERVAL);
    device2->enableDevQueues()       ;
    device2->enableDevInterrupt()    ;
    device2->setPromisc(true)        ;
    device2->wait4Link()             ;            
    device2->fillTxMemPool(NUM_OF_TX_BUF);
    device2->send();
}


int main() {
    std::thread t1(thread1);
    std::thread t2(thread2);

    t1.join();
    t2.join();
    return 0;
}