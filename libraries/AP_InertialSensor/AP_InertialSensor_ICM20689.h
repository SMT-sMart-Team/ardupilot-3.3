/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_ICM20689_H__
#define __AP_INERTIAL_SENSOR_ICM20689_H__

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Progmem/AP_Progmem.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define ICM20689_DEBUG 0

// enable FIFO
#define MPU_SAMPLE_SIZE 14
#define MPU_FIFO_DOWNSAMPLE_COUNT 8
#define MPU_FIFO_DOWNSAMPLE_COUNT_ACC 4
#define MPU_FIFO_BUFFER_LEN 64 


// AB ZhaoYJ@2016-11-30 for user-defined 4 order chebyI filter
#define N_ORDER 4

typedef uint16_t dump_type;

typedef struct {
    Vector3i acc;
    Vector3i gyro;
    int16_t temp;
}dump_half_sec_type;

class AP_InertialSensor_ICM20689 : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_ICM20689(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return _have_sample_available; }
    bool accel_sample_available(void) { return _have_sample_available; }

    /* Put the ICM20689 in a known state so it can be
     * used both for the InertialSensor and as for backend of other drivers.
     *
     * The SPI semaphore must be taken and timer_procs suspended.
     *
     * This method puts the bus in low speed. If the initialization is
     * successful the bus is left on low speed so the caller can finish the
     * initialization of its driver.
     */
    static bool initialize_driver_state();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:

    static uint8_t _register_read(AP_HAL::SPIDeviceDriver *spi, uint8_t reg);
    static void _register_write(AP_HAL::SPIDeviceDriver *spi, uint8_t reg,
                                uint8_t val);

    bool                 _init_sensor(void);
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    bool                 _hardware_init(void);
    bool                 _sample_available();


    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    AP_HAL::Semaphore *_sem;

    // support for updating filter at runtime
    int16_t _last_gyro_filter_hz;
    int16_t _last_accel_filter_hz;

    // change the filter frequency
    void _set_accel_filter(uint8_t filter_hz);
    void _set_gyro_filter(uint8_t filter_hz);

#if USER_FILTER 
    Vector3f _accel_user_filter(Vector3f _accl_in, uint8_t _uf);
    Vector3f _gyro_user_filter(Vector3f _gyro_in, uint8_t _uf);
    Vector3f _accel_median_filter(Vector3f _accl_in);
    Vector3f _gyro_median_filter(Vector3f _gyro_in);
#endif

    void dump_data(dump_type data);


    void dump_data_half_sec(dump_half_sec_type data);

    // This structure is used to pass data from the timer which reads
    // the sensor to the main thread. The _shared_data_idx is used to
    // prevent race conditions by ensuring the data is fully updated
    // before being used by the consumer
    struct {
        Vector3f _accel_filtered;
        Vector3f _gyro_filtered;
    } _shared_data[2];
    volatile uint8_t _shared_data_idx;
#ifdef SMT_CAPTURE_IMU_RAW
    struct {
        Vector3f _accel_raw;
        Vector3f _gyro_raw;
    } _shared_data_raw[2];
#endif

    float _temp_filtered;
    AverageFilterFloat_Size5 _temp_filter;

    // Low Pass filters for gyro and accel 
    LowPassFilter2pVector3f _accel_filter;
    LowPassFilter2pVector3f _gyro_filter;

    // do we currently have a sample pending?
    bool _have_sample_available;

    // gyro and accel instances
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    // The default rotation for the IMU, its value depends on how the IMU is
    // placed by default on the system
    enum Rotation _default_rotation;

#if IMU_8KHZ

    // buffer for fifo read
    uint8_t _fifo_buffer[MPU_SAMPLE_SIZE*MPU_FIFO_BUFFER_LEN];

    /*
      accumulators for fast sampling
      See description in _accumulate_fast_sampling()
    */
    struct {
        Vector3f accel;
        Vector3f gyro;
        uint8_t count;
    } _accum;

    UserFilterDouble_Size3 *_accel_uf;
    UserFilterDouble_Size3 *_gyro_uf;

    void _read_fifo();
    void _fifo_reset();
    bool _block_read(uint8_t reg, uint8_t *buf,                                         uint32_t size);
    void _set_filter_register(void);
    void _filter_imu_1KHz(Vector3f imu_acc, Vector3f imu_gyro);
    bool _accumulate_fast_sampling(uint8_t *samples, uint8_t n_samples);
    bool _check_raw_temp(int16_t t2);
    int16_t _raw_temp;
    bool _bypass_acc_uf;
    bool _bypass_gyro_uf;
#endif

#if ICM20689_DEBUG
    static void _dump_registers(AP_HAL::SPIDeviceDriver *spi);
#endif
};

#endif // __AP_INERTIAL_SENSOR_ICM20689_H__
