/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HMC5983_H
#define AP_Compass_HMC5983_H

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#include "Compass.h"
#include "AP_Compass_Backend.h"

#define RT_TIMER 0 // add _timer(read_raw data) to rt_timer funcs

class AP_HMC5983_SerialBus
{
public:
    struct PACKED raw_value {
        int16_t val[3];
        uint8_t st2;
    };

    virtual bool register_read(uint8_t address, uint8_t *value) = 0;
    uint8_t register_read(uint8_t address) {
        uint8_t reg;
        register_read(address, &reg);
        return reg;
    }
    virtual bool register_write(uint8_t address, uint8_t value) = 0;
    virtual void setHighSpeed(uint8_t speed) = 0;
    virtual bool burst_read(uint8_t *buf) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
};

class AP_HMC5983_SPI: public AP_HMC5983_SerialBus
{
public:
    AP_HMC5983_SPI();
    bool register_read(uint8_t address, uint8_t *value);
    bool register_write(uint8_t address, uint8_t value);
    void setHighSpeed(uint8_t speed);
    bool burst_read(uint8_t *buf);
    AP_HAL::Semaphore* get_semaphore();
private:
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
};

class AP_Compass_HMC5983 : public AP_Compass_Backend
{
private:
    float               calibration[3];
    bool                _initialised;
    bool                read_raw(void);
    uint8_t             _base_config;
    bool                re_initialise(void);
    bool                read_register(uint8_t address, uint8_t *value);
    bool                write_register(uint8_t address, uint8_t value);

    bool _sem_take_blocking();
    bool _sem_take_nonblocking();
    bool _sem_give();

    AP_HMC5983_SerialBus *_bus;
    AP_HAL::Semaphore *_bus_sem;

    uint32_t            _retry_time; // when unhealthy the millis() value to retry at

    float			    _mag_x;
    float			    _mag_y;
    float			    _mag_z;
    float               _mag_x_accum;
    float               _mag_y_accum;
    float               _mag_z_accum;
    uint16_t			_accum_count;
    uint32_t            _last_accum_time;

    uint8_t             _compass_instance;
    uint8_t             _product_id;

public:
    AP_Compass_HMC5983(Compass &compass, AP_HMC5983_SerialBus *bus);
    bool        init(void);
    void        read(void);
    void        _timer(void);
    void        accumulate(void);
    Vector3f    _median_filter(Vector3f _mag_in);
    Vector3f    _user_filter(Vector3f _mag_in, uint8_t _uf);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

};
#endif
