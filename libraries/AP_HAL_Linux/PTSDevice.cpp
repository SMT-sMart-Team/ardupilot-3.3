#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#define _XOPEN_SOURCE 600 
#include <stdlib.h> 
#include <fcntl.h> 
#include <errno.h> 
#include <unistd.h> 
#include <stdio.h> 
#define __USE_BSD 
#include <termios.h>

#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "PTSDevice.h"

#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

PTSDevice::PTSDevice(const char *device_path): 
    _device_path(device_path)
{
}

PTSDevice::PTSDevice(int _fd_pts): 
    _fd(_fd_pts)
{
}

PTSDevice::~PTSDevice()
{

}

bool PTSDevice::close()
{
    if (_fd != -1) {
        if (::close(_fd) < 0) {
            return false;
        }
    }

    _fd = -1;

    return true;
}

bool PTSDevice::open()
{
    int fdm, rc; 
    FILE* fp;

    fdm = posix_openpt(O_RDWR); 
    if (fdm < 0) 
    { 
        ::fprintf(stderr, "Error %d on posix_openpt()\n", errno); 
        return 1; 
    } 

    rc = grantpt(fdm); 
    if (rc != 0) 
    { 
        ::fprintf(stderr, "Error %d on grantpt()\n", errno); 
        return 1; 
    } 

    rc = unlockpt(fdm); 
    if (rc != 0) 
    { 
        ::fprintf(stderr, "Error %d on unlockpt()\n", errno); 
        return 1; 
    } 

    this->_fd = fdm;
    memcpy(this->pts_slave_path, ptsname(fdm), strlen(ptsname(fdm)));///dev/pts/0

    // notify GPSD to get this slave pts
    //
    fp = fopen("/root/GPS_PTS", "w+");
    if(fp)
    {
      ::fprintf(fp, "%s", this->pts_slave_path);
      ::fclose(fp);
    }
    else
    {
        return false;
    }

    ::fprintf(stderr, "Create PTY %s successfully!\n", this->pts_slave_path); 

    return true;
}

ssize_t PTSDevice::read(uint8_t *buf, uint16_t n)
{
    return ::read(_fd, buf, n);
}

ssize_t PTSDevice::write(const uint8_t *buf, uint16_t n)
{
    struct pollfd fds;
    fds.fd = this->_fd;
    fds.events = POLLOUT;
    fds.revents = 0;

    int ret = 0;

    if (poll(&fds, 1, 0) == 1) {
        ret = ::write(this->_fd, buf, n);
    }

    return ret;
}

void PTSDevice::set_blocking(bool blocking)
{
    int flags = fcntl(_fd, F_GETFL, 0);
    
    if (blocking) {
        flags = flags & ~O_NONBLOCK;
    } else {
        flags = flags | O_NONBLOCK;
    }

    if (fcntl(_fd, F_SETFL, flags) < 0) {
        ::fprintf(stderr, "Failed to make UART nonblocking %s - %s\n",
                  _device_path, strerror(errno));
    }

}

void PTSDevice::_disable_crlf()
{
    struct termios t;
    memset(&t, 0, sizeof(t));

    tcgetattr(_fd, &t);

    // disable LF -> CR/LF
    t.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL | IXON | IXOFF);
    t.c_oflag &= ~(OPOST | ONLCR);
    t.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
    t.c_cc[VMIN] = 0;

    tcsetattr(_fd, TCSANOW, &t);
}

void PTSDevice::set_speed(uint32_t baudrate)
{
    struct termios t;
    memset(&t, 0, sizeof(t));

    tcgetattr(_fd, &t);
    cfsetspeed(&t, baudrate);
    tcsetattr(_fd, TCSANOW, &t);
}

#endif
