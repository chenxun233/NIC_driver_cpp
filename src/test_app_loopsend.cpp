#include <memory>
#include "vfio_dev.h"
#include <thread>
#include <pthread.h>
#include "factory.h"


#define PKT_BUF_SIZE 2048
#define PKT_SIZE 60

const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;
uint64_t interrupt_interval = 100;
#define NUM_OF_RX_BUF 2048
#define NUM_OF_TX_BUF 2048
#define NUM_OF_QUEUE 1


std::unique_ptr<BasicDev> device1 = createDevice("0000:04:00.0",0,NUM_OF_QUEUE,NUM_OF_RX_BUF, PKT_BUF_SIZE, INTERRUPT_INITIAL_INTERVAL, 100);
// std::unique_ptr<BasicDev> device2 = createDevice("0000:05:00.0",0,NUM_OF_QUEUE,NUM_OF_RX_BUF, PKT_BUF_SIZE, INTERRUPT_INITIAL_INTERVAL, 100);

void thread1(){
    static_cast<Intel82599Dev*>(device1.get())->loopSendTest(64);
}

// void thread2(){
//     static_cast<Intel82599Dev*>(device2.get())->loopSendTest(64);
// }

int main() {
    std::thread t1(thread1);
    // std::thread t2(thread2);
    t1.join();
    // t2.join();
    return 0;
}