#ifndef IXY_DEVICE_H
#define IXY_DEVICE_H

#include <stdint.h>
#include <unistd.h>
#include <stddef.h> // add this for offsetof

#include "log.h"
#include "ixgbe_type.h"

#define MAX_QUEUES 64


// Forward declare struct to prevent cyclic include with stats.h

struct device_info {
	uint16_t vendor_id;
	uint16_t device_id;
	uint32_t class_id;
};


struct device_info get_device_info(const char* pci_addr);

struct __attribute__((__packed__)) mac_address {
	uint8_t	addr[6];
};

/**
 * container_of - cast a member of a structure out to the containing structure
 * Adapted from the Linux kernel.
 * This allows us to expose the same struct for all drivers to the user's
 * application and cast it to a driver-specific struct in the driver.
 * A simple cast would be sufficient if we always store it at the same offset.
 * This macro looks more complicated than it is, a good explanation can be
 * found at http://www.kroah.com/log/linux/container_of.html
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({\
	const __typeof__(((type*)0)->member)* __mptr = (ptr);\
	(type*)((char*)__mptr - offsetof(type, member));\
})





// getters/setters for PCIe memory mapped registers
// this code looks like it's in need of some memory barrier intrinsics, but that's apparently not needed on x86
// dpdk has release/acquire memory order calls before/after the memory accesses, but they are defined as
// simple compiler barriers (i.e., the same empty asm with dependency on memory as here) on x86
// dpdk also defines an additional relaxed load/store for the registers that only uses a volatile access,  we skip that for simplicity

static inline void set_bar_reg32(uint8_t* addr, int reg, uint32_t value) {
	__asm__ volatile ("" : : : "memory");
	*((volatile uint32_t*) (addr + reg)) = value;
}

static inline uint32_t get_bar_reg32(const uint8_t* addr, int reg) {
	__asm__ volatile ("" : : : "memory");
	return *((volatile uint32_t*) (addr + reg));
}

static inline void set_bar_flags32(uint8_t* addr, int reg, uint32_t flags) {
	set_bar_reg32(addr, reg, get_bar_reg32(addr, reg) | flags);
}

static inline void clear_bar_flags32(uint8_t* addr, int reg, uint32_t flags) {
	set_bar_reg32(addr, reg, get_bar_reg32(addr, reg) & ~flags);
}

static inline void wait_clear_bar_reg32(const uint8_t* addr, int reg, uint32_t mask) {
	__asm__ volatile ("" : : : "memory");
	uint32_t cur = 0;
	while (cur = *((volatile uint32_t*) (addr + reg)), (cur & mask) != 0) {
		debug("waiting for flags 0x%08X in register 0x%05X to clear, current value 0x%08X", mask, reg, cur);
		usleep(10000);
		__asm__ volatile ("" : : : "memory");
	}
}

static inline void wait_set_bar_reg32(const uint8_t* addr, int reg, uint32_t mask) {
	__asm__ volatile ("" : : : "memory");
	uint32_t cur = 0;
	while (cur = *((volatile uint32_t*) (addr + reg)), (cur & mask) != mask) {
		debug("waiting for flags 0x%08X in register 0x%05X, current value 0x%08X", mask, reg, cur);
		usleep(10000);
		__asm__ volatile ("" : : : "memory");
	}
}

// getters/setters for pci io port resources

static inline void write_io32(int fd, uint32_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline void write_io16(int fd, uint16_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline void write_io8(int fd, uint8_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline uint32_t read_io32(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint32_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		error("pread io resource");
	return temp;
}

static inline uint16_t read_io16(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint16_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		error("pread io resource");
	return temp;
}

static inline uint8_t read_io8(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint8_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		error("pread io resource");
	return temp;
}

inline void set_ivar(uint8_t* addr, int8_t direction, int8_t queue, int8_t msix_vector) {
	u32 ivar, index;
	msix_vector |= IXGBE_IVAR_ALLOC_VAL;
	index = ((16 * (queue & 1)) + (8 * direction));
	ivar = get_bar_reg32(addr, IXGBE_IVAR(queue >> 1));
	ivar &= ~(0xFF << index);
	ivar |= (msix_vector << index);
	set_bar_reg32(addr, IXGBE_IVAR(queue >> 1), ivar);
}

#endif // IXY_DEVICE_H
