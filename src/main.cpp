#include "lib_vfio.h"
#include "interrupts.h"

int main() {
    vfio_device device("0000:04:00.0", 0,10);
    return 0;
}