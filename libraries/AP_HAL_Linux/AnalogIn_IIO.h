#pragma once

#ifndef ANALOGIN_IIO_H
#define ANALOGIN_IIO_H
#include "AP_HAL_Linux.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

// AB ZhaoYJ@2016-10-15 for adis16365 iio

#ifdef SMT_INS_ADIS16365_IIO
#define INS_IIO_RAW_NUM 10
#define INS_IIO_ANALOG_IN_DIR "/sys/bus/iio/devices/iio:device1/"
#endif

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF

// add by ZhaoYJ @2016-05-12
#define BOARD_VOLT_PIN 7

#define BBB_VOLTAGE_SCALING 0.00142602816

#define IIO_ANALOG_IN_COUNT 8
// Note that echo BB-ADC cape should be loaded
#define IIO_ANALOG_IN_DIR "/sys/bus/iio/devices/iio:device0/"

#else
#define IIO_ANALOG_IN_COUNT 8
#define IIO_ANALOG_IN_DIR "/sys/bus/iio/devices/iio:device0/"
#endif

class AnalogSource_IIO : public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_IIO;
    AnalogSource_IIO(int16_t pin, float v);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }

    // AB ZhaoYJ@2016-10-15 for adis16365 iio
    //
#ifdef SMT_INS_ADIS16365_IIO
    enum {
        ins_voltage_idx = 3,
        ins_accl_x_idx,
        ins_accl_y_idx,
        ins_accl_z_idx,
        ins_gyro_x_idx,
        ins_gyro_y_idx,
        ins_gyro_z_idx,
        ins_data_all
    };
    uint16_t read_imu_voltage();
    bool read_imu_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
#endif

private:
    float       _value;
    float       _latest;
    float       _sum_value;
    // float       _value_ratiometric;
    uint8_t     _sum_count;
    int16_t     _pin;
    int         _pin_fd;
    int         fd_analog_sources[IIO_ANALOG_IN_COUNT];

    void reopen_pin(void);
    void init_pins(void);
    void select_pin(void);

    // AB ZhaoYJ@2016-10-15 for adis16365 iio
    //
#ifdef SMT_INS_ADIS16365_IIO

    int _ins_fd[INS_IIO_RAW_NUM];
    float _ins_volt_scale;
    float _ins_accl_scale;
    float _ins_gyro_scale;
    bool init_ins_iio();
#endif

    static const char *analog_sources[IIO_ANALOG_IN_COUNT];
};

class AnalogIn_IIO : public AP_HAL::AnalogIn {
public:

    // add by ZhaoYJ for board voltage monitoring @2016-05-12
    AP_HAL::AnalogSource* _board_volt_source;

    AnalogIn_IIO();
    void init(void *);
    AP_HAL::AnalogSource* channel(int16_t n);

    // we don't yet know how to get the board voltage
#ifdef SMT_NEW_SENSORS_BOARD
    float board_voltage(void);
#else
    float board_voltage(void) { return 0.0f; }
#endif

};
#endif
