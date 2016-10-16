/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AnalogIn_IIO.h"

// add by ZhaoYJ @2016-05-12
#ifdef SMT_NEW_SENSORS_BOARD
// #define IIO_DEBUG
// #define  IIO_DEBUG_ADIS
float AIScales[IIO_ANALOG_IN_COUNT] = {
    0.00131836,
    0.00131836,
    0.00131836,
    0.00131836,
    0.00131836,
    0.00629883,
    0.00087891,
    0.00087891
    };
#endif

extern const AP_HAL::HAL& hal;

const char* AnalogSource_IIO::analog_sources[IIO_ANALOG_IN_COUNT] = {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
    "in_voltage0_raw",
    "in_voltage1_raw",
    "in_voltage2_raw",
    "in_voltage3_raw",
    "in_voltage4_raw",
    "in_voltage5_raw",
    "in_voltage6_raw",
    "in_voltage7_raw",
#else
    "in_voltage0_raw",
#endif
};

AnalogSource_IIO::AnalogSource_IIO(int16_t pin, float initial_value) :
    _pin(pin),
    _value(initial_value),
    _sum_value(0),
    _sum_count(0),
    _pin_fd(-1)
{
    init_pins();
    select_pin();
#ifdef SMT_INS_ADIS16365_IIO
    if(!init_ins_iio())
        hal.util->prt("error: init adis16365 iio failed!");
#endif
}

void AnalogSource_IIO::init_pins(void)
{
    char buf[100];
    for (int i=0; i < IIO_ANALOG_IN_COUNT; i++) {
        // Construct the path by appending strings
        strncpy(buf, IIO_ANALOG_IN_DIR, sizeof(buf));
        strncat(buf, AnalogSource_IIO::analog_sources[i], sizeof(buf) - strlen(buf) -1);

        fd_analog_sources[i] = open(buf, O_RDONLY | O_NONBLOCK);
        if (fd_analog_sources[i] == -1) {
            ::printf("Failed to open analog pin %s\n", buf);
        }
    }
}

/*
  selects a diferent file descriptor among in the fd_analog_sources array
 */
void AnalogSource_IIO::select_pin(void)
{
    if(_pin > IIO_ANALOG_IN_COUNT)
        return;
    _pin_fd = fd_analog_sources[_pin];
}

/*
  reopens an analog source (by closing and opening it again) and selects it
 */
void AnalogSource_IIO::reopen_pin(void)
{
    char buf[100];

    if (fd_analog_sources[_pin] != -1) {
        close(fd_analog_sources[_pin]);
        fd_analog_sources[_pin] = -1;
        _pin_fd = -1;
    }

    if (_pin < 0) {
        return;
    }

    if (_pin > IIO_ANALOG_IN_COUNT) {
        // invalid pin
        return;
    }

    // Construct the path by appending strings
    strncpy(buf, IIO_ANALOG_IN_DIR, sizeof(buf));
    strncat(buf, AnalogSource_IIO::analog_sources[_pin], sizeof(buf) - strlen(buf) - 1);

    fd_analog_sources[_pin] = open(buf, O_RDONLY | O_NONBLOCK);
    if (fd_analog_sources[_pin] == -1) {
        ::printf("Failed to open analog pin %s\n", buf);
    }
    _pin_fd = fd_analog_sources[_pin];
}

float AnalogSource_IIO::read_average()
{
    read_latest();
    if (_sum_count == 0) {
        return _value;
    }
    hal.scheduler->suspend_timer_procs();
    _value = _sum_value / _sum_count;
    // _value_ratiometric = _sum_ratiometric / _sum_count;
    _sum_value = 0;
    // _sum_ratiometric = 0;
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();
    return _value;
}

float AnalogSource_IIO::read_latest()
{
    char sbuf[10];

    if (_pin_fd == -1) {
        _latest = 0;
        return 0;
    }

    if(_pin > IIO_ANALOG_IN_COUNT)
        return 0;

    memset(sbuf, 0, sizeof(sbuf));
    pread(_pin_fd, sbuf, sizeof(sbuf)-1, 0);
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#ifdef SMT_NEW_SENSORS_BOARD
    _latest = atoi(sbuf) * AIScales[_pin];
#else
    _latest = atoi(sbuf) * BBB_VOLTAGE_SCALING;
#endif
#else
    _latest = atoi(sbuf);
#endif

#ifdef IIO_DEBUG
    printf(" pin: %d, orig: %d, scale: %f, latest: %f\n", _pin, atoi(sbuf), AIScales[_pin], _latest);
#endif
    _sum_value += _latest;
    _sum_count++;

    return _latest;
}

// output is in volts
float AnalogSource_IIO::voltage_average()
{
    return read_average();
}

float AnalogSource_IIO::voltage_latest()
{
    read_latest();
    return _latest;
}

void AnalogSource_IIO::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }

    if(_pin > IIO_ANALOG_IN_COUNT)
        return;

    hal.scheduler->suspend_timer_procs();
    _pin = pin;
    _sum_value = 0;
    // _sum_ratiometric = 0;
    _sum_count = 0;
    _latest = 0;
    _value = 0;
    select_pin();
    // _value_ratiometric = 0;
    hal.scheduler->resume_timer_procs();
}

void AnalogSource_IIO::set_stop_pin(uint8_t p)
{}

void AnalogSource_IIO::set_settle_time(uint16_t settle_time_ms)
{}


#ifdef SMT_INS_ADIS16365_IIO
// open dev; get scales
bool AnalogSource_IIO::init_ins_iio()
{

    const char* ins_node[INS_IIO_RAW_NUM] = {
        "in_voltage0_supply_scale",
        "in_accel_scale",
        "in_anglvel_scale",
        "in_voltage0_supply_raw",
        "in_accel_x_raw",
        "in_accel_y_raw",
        "in_accel_z_raw",
        "in_anglvel_x_raw",
        "in_anglvel_y_raw",
        "in_anglvel_z_raw"
    };

    // open nodes
    char buf[100];
    for (int i=0; i < INS_IIO_RAW_NUM; i++) {
        strncpy(buf, INS_IIO_ANALOG_IN_DIR, sizeof(buf));
        strncat(buf, ins_node[i], sizeof(buf) - strlen(buf) -1);

        _ins_fd[i] = open(buf, O_RDONLY | O_NONBLOCK);
        if (_ins_fd[i] == -1) {
            printf("Failed to open analog pin %s\n", buf);
            return false;
        }
    }

    // get scale first
    hal.scheduler->suspend_timer_procs();
    char sbuf[10];
    memset(sbuf, 0, sizeof(sbuf));
    uint8_t try_num = 0;
    uint8_t idx = 0;

    while(try_num++ < 3)
    {
        if(-1 == pread(_ins_fd[idx++], sbuf, sizeof(sbuf)-1, 0))
        {
            printf("Failed to read analog pin %d\n", idx);
        }
        else
            break;
    }
    _ins_volt_scale = atof(sbuf);
#ifdef IIO_DEBUG_ADIS
    printf(" volt-scale: %f\n", _ins_volt_scale);
#endif

    while(try_num++ < 3)
    {
        if(-1 == pread(_ins_fd[idx++], sbuf, sizeof(sbuf)-1, 0))
        {
            printf("Failed to read analog pin %d\n", idx);
        }
        else
            break;
    }
    _ins_accl_scale = atof(sbuf);
#ifdef IIO_DEBUG_ADIS
    printf(" accl-scale: %f\n", _ins_accl_scale);
#endif

    while(try_num++ < 3)
    {
        if(-1 == pread(_ins_fd[idx++], sbuf, sizeof(sbuf)-1, 0))
        {
            printf("Failed to read analog pin %d\n", idx);
        }
        else
            break;
    }
    _ins_gyro_scale = atof(sbuf);
#ifdef IIO_DEBUG_ADIS
    printf(" gyro-scale: %f\n", _ins_gyro_scale);
#endif

    hal.scheduler->resume_timer_procs();

    return true;

}

uint16_t AnalogSource_IIO::read_imu_voltage()
{
    char sbuf[10];
    memset(sbuf, 0, sizeof(sbuf));


    if(-1 == pread(_ins_fd[ins_voltage_idx], sbuf, sizeof(sbuf)-1, 0))
    {
        printf("Failed to read analog pin %d\n", ins_voltage_idx);
    }

#ifdef IIO_DEBUG_ADIS
    printf(" voltage[%d]: %d\n", ins_voltage_idx, atoi(sbuf));
#endif

    return atoi(sbuf);

}

#define DATA_LEN 10
bool AnalogSource_IIO::read_imu_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    char sbuf[INS_IIO_RAW_NUM][DATA_LEN];
    memset(sbuf, 0, sizeof(sbuf));
    bool ret = false;
    uint8_t idx = ins_accl_x_idx;

#if 1
    for(; idx < ins_data_all; idx++) 
    {
        if((_ins_fd[idx] != NULL) && (sbuf[idx] != NULL))
        {
            if(-1 == pread(_ins_fd[idx], sbuf[idx], DATA_LEN - 1, 0))
            {
                printf("Failed to read analog pin %d\n", idx);
                return false;
            }
        }
        else
        {
            printf("ins_fd or sbuf NULL\n");
            return false;
        }
    }
#endif

    // get data
    idx = ins_accl_x_idx;
    *ax = atof(sbuf[idx++]) * _ins_accl_scale;
    *ay = atof(sbuf[idx++]) * _ins_accl_scale;
    *az = atof(sbuf[idx++]) * _ins_accl_scale;
    *gx = atof(sbuf[idx++]) * _ins_gyro_scale;
    *gy = atof(sbuf[idx++]) * _ins_gyro_scale;
    *gz = atof(sbuf[idx++]) * _ins_gyro_scale;

    return true;
}

#endif

AnalogIn_IIO::AnalogIn_IIO()
{}

void AnalogIn_IIO::init(void *)
{}


AP_HAL::AnalogSource* AnalogIn_IIO::channel(int16_t pin) {
    return new AnalogSource_IIO(pin, 0);
}

#ifdef SMT_NEW_SENSORS_BOARD
float AnalogIn_IIO::board_voltage(void)
{
    static char first = 1;
    if(first)
    {
        // init board voltage pin
        _board_volt_source = channel(BOARD_VOLT_PIN);
        first = 0;
    }

    _board_volt_source->set_pin(BOARD_VOLT_PIN);
#ifdef IIO_DEBUG
    printf(" board volt: %f\n", _board_volt_source->voltage_average());
#endif

    return _board_volt_source->voltage_average();

}
#endif


#endif // CONFIG_HAL_BOARD
