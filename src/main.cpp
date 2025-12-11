#include "implementation.h"
#include "log.h"

int main() {
    std::unique_ptr<BasicDev> device = return_p_device(
                                        "0000:04:00.0",
                                        0,
                                        1,
                                        1,
                                        1000
    );
    if (!device) {
        warn("Failed to create and initialize device");
        return -1;
    }
    return 0;
}