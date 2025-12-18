#ifndef IXY_LOG_H
#define IXY_LOG_H

#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <assert.h>

#ifndef NDEBUG
#define debug(fmt, ...) do {\
	fprintf(stderr, "[DEBUG  ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)
#else
#define debug(fmt, ...) do {} while(0)
#undef assert
#define assert(expr) (void) (expr)
#endif
#define success(fmt, ...) do {\
	fprintf(stdout, "[SUCCESS] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define info(fmt, ...) do {\
	fprintf(stdout, "[INFO   ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define warn(fmt, ...) do {\
	fprintf(stderr, "[WARN   ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define error(fmt, ...) do {\
    fprintf(stderr, "[ERROR  ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
    abort();\
} while(0)

namespace log_detail {
template <typename T>
inline T check_err_impl(T result, const char* op, const char* file, int line, const char* func) {
    if (result == static_cast<T>(-1)) {
        int err = errno;
        char buf[512];
#if defined(__GLIBC__) && defined(_GNU_SOURCE)
        char* msg = strerror_r(err, buf, sizeof(buf));
        if (!msg) {
            snprintf(buf, sizeof(buf), "Errno %d", err);
            msg = buf;
        }
#else
        if (strerror_r(err, buf, sizeof(buf)) != 0) {
            snprintf(buf, sizeof(buf), "Errno %d", err);
        }
        const char* msg = buf;
#endif
        fprintf(stderr, "[ERROR] %s:%d %s(): Failed to %s: %s\n", file, line, func, op, msg);
        exit(err);
    }
    return result;
}
} // namespace log_detail

#define check_err(expr, op) \
    log_detail::check_err_impl((expr), (op), __FILE__, __LINE__, __func__)

[[maybe_unused]] static void hexdump(void* void_ptr, size_t len) {
	uint8_t* ptr = (uint8_t*) void_ptr;
	char ascii[17];
	for (uint32_t i = 0; i < len; i += 16) {
		printf("%06x: ", i);
		int j = 0;
		for (; j < 16 && i + j < len; j++) {
			printf("%02x", ptr[i + j]);
			if (j % 2) {
				printf(" ");
			}
			ascii[j] = isprint(ptr[i + j]) ? ptr[i + j] : '.';
		}
		ascii[j] = '\0';
		if (j < 16) {
			for (; j < 16; j++) {
				printf("  ");
				if (j % 2) {
					printf(" ");
				}
			}
		}
		printf("  %s\n", ascii);
	}
}

#endif //IXY_LOG_H
