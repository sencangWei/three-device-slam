#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

#include "extunit.h"

int ylx_open(const char *path) { return open(path, O_RDWR); }

int ylx_read_imu27(int fd, uint8_t out[27]) {
    unsigned long size = 0;
    int rc = xu_get_len(fd, 3, 1, &size);
    if (rc != 0) return rc;
    if (size != 27) return -27;
    return xu_get_cur(fd, 3, 1, size, out);
}

int ylx_close(int fd) { return close(fd); }
