#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

#include "extunit.h"

int ylx_open(const char *path) {
    int fd = open(path, O_RDWR);
    if (fd < 0) return fd;

    unsigned long size = 0;
    int rc = xu_get_len(fd, 3, 1, &size);
    if (rc != 0) {
        close(fd);
        return rc < 0 ? rc : -rc;
    }
    if (size != 27) {
        close(fd);
        return -27;
    }
    return fd;
}

int ylx_read_imu27(int fd, uint8_t out[27]) {
    return xu_get_cur(fd, 3, 1, 27, out);
}

int ylx_close(int fd) { return close(fd); }
