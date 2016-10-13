#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

#define BUFFER_SIZE     128

int read_value_from_file(const char* filename, const char* format, ...)
{
    int fd = 0;
    char buf[BUFFER_SIZE];
    va_list argptr;
    int len;

    fd = open(filename, O_RDONLY);

    if (fd < 0) {
        fprintf(stderr, "Can not open the file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        return -errno;
    }

    len = read(fd, buf, sizeof(buf));

    if (len <= 0) {
        fprintf(stderr, "Can not read data from file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        close(fd);
        return -errno;
    }

    // Set the '\0' to the end of the buffer.
    if (len == BUFFER_SIZE)
        len = len - 1;

    buf[len] = '\0';

    va_start(argptr, format);

    len = vsscanf(buf, format, argptr);

    va_end(argptr);

    if (len <= 0) {
        fprintf(stderr, "Can not parse the value from %s in format: %s\n",
                buf, format);
        close(fd);
        return -EINVAL;
    }

    close(fd);
    return 0;
}

int write_value_to_file(const char* filename, const char* format, ...)
{
    int fd = 0;
    char buf[BUFFER_SIZE];
    va_list argptr;
    int len, ret;

    va_start(argptr, format);

    len = vsnprintf(buf, sizeof(buf), format, argptr);

    va_end(argptr);

    if (len < 0 || len >= BUFFER_SIZE) {
        fprintf(stderr, "Can not create the strings, ret: %d\n",
                len);
        return -EINVAL;
    }

    // The NULL pointer '\0' will be added to buff by snprintf function.

    fd = open(filename, O_WRONLY);

    if (fd < 0) {
        fprintf(stderr, "Can not open the file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        return -errno;
    }

    ret = write(fd, buf, len + 1);

    if (ret != len + 1) {
        fprintf(stderr, "Can not write whole data from file: %s, error: %d(%s)\n",
                buf, errno, strerror(errno));
        close(fd);
        return -errno;
    }

    close(fd);
    return 0;
}
