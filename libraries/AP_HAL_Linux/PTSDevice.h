#ifndef __AP_HAL_LINUX_PTSDEVICE_H__
#define __AP_HAL_LINUX_PTSDEVICE_H__

#include "SerialDevice.h"
#include <AP_HAL/utility/Socket.h>

class PTSDevice: public SerialDevice {
public:
    PTSDevice() {};
    PTSDevice(const char *device_path);
    PTSDevice(int _fd_pts);
    virtual ~PTSDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) override;
    virtual ssize_t read(uint8_t *buf, uint16_t n) override;
    virtual void set_blocking(bool blocking) override;
    virtual void set_speed(uint32_t speed) override;

private:
    void _disable_crlf();

    int _fd = -1;
    const char* _device_path;
#define MAX_PTS_PATH 128
    char pts_slave_path[MAX_PTS_PATH];
};

#endif
