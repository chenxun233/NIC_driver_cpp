#include "implementation.h"
#include "log.h"

int main() {
    dev_ptr_type device = create_device(
                                        "0000:04:00.0",
                                        0,
                                        4,
                                        4,
                                        1000
    );
    if (!device) {
        warn("Failed to create and initialize device");
        return -1;
    }
    return 0;
}