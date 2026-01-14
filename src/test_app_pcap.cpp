#include <memory>
#include <pthread.h>
#include "factory.h"
#include <string>


#define PKT_BUF_SIZE 2048
#define PKT_SIZE 60

const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;
uint64_t interrupt_interval = 100;
#define NUM_OF_RX_BUF 2048
#define NUM_OF_TX_BUF 2048
#define NUM_OF_QUEUE 1

std::unique_ptr<BasicDev> device1 = createDevice("0000:05:00.0",0,NUM_OF_QUEUE,NUM_OF_RX_BUF, PKT_BUF_SIZE, INTERRUPT_INITIAL_INTERVAL, 100);

int main(int argc, char* argv[]) {
	if (argc != 2) {
		printf("Usage: %s <output file>\n", argv[0]);
		return 1;
	}
    std::string file_name = argv[1];
    static_cast<Intel82599Dev*>(device1.get())->capturePackets( 64,1000, file_name);
    return 0;
}