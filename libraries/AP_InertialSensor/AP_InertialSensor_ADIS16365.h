/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_ADIS16365_H__
#define __AP_INERTIAL_SENSOR_ADIS16365_H__

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Progmem/AP_Progmem.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define ADIS16365_DEBUG 0
#define DUMP_DATA 0
#define FAST_BOOT 0
#define BURST_READ 1


// register memory map
#define BIT(n) (1 << n)

#define AP_PRODUCT_ID_ADIS16365 16365

#define ADIS16400_STARTUP_DELAY 290 /* ms */
#define ADIS16400_MTEST_DELAY 90 /* ms */

#define T_CS 50 // 48.8 ns
#define T_STALL 10 // 9 us
#define T_READRATE 40 // 40 us

#define ADIS16400_FLASH_CNT  0x00 /* Flash memory write count */
#define ADIS16400_SUPPLY_OUT 0x02 /* Power supply measurement */
#define ADIS16400_XGYRO_OUT 0x04 /* X-axis gyroscope output */
#define ADIS16400_YGYRO_OUT 0x06 /* Y-axis gyroscope output */
#define ADIS16400_ZGYRO_OUT 0x08 /* Z-axis gyroscope output */
#define ADIS16400_XACCL_OUT 0x0A /* X-axis accelerometer output */
#define ADIS16400_YACCL_OUT 0x0C /* Y-axis accelerometer output */
#define ADIS16400_ZACCL_OUT 0x0E /* Z-axis accelerometer output */
#define ADIS16400_XMAGN_OUT 0x10 /* X-axis magnetometer measurement */
#define ADIS16400_YMAGN_OUT 0x12 /* Y-axis magnetometer measurement */
#define ADIS16400_ZMAGN_OUT 0x14 /* Z-axis magnetometer measurement */
#define ADIS16400_TEMP_OUT  0x16 /* Temperature output */
#define ADIS16400_AUX_ADC   0x18 /* Auxiliary ADC measurement */

#define ADIS16350_XTEMP_OUT 0x10 /* X-axis gyroscope temperature measurement */
#define ADIS16350_YTEMP_OUT 0x12 /* Y-axis gyroscope temperature measurement */
#define ADIS16350_ZTEMP_OUT 0x14 /* Z-axis gyroscope temperature measurement */

#define ADIS16300_PITCH_OUT 0x12 /* X axis inclinometer output measurement */
#define ADIS16300_ROLL_OUT  0x14 /* Y axis inclinometer output measurement */
#define ADIS16300_AUX_ADC   0x16 /* Auxiliary ADC measurement */

#define ADIS16448_BARO_OUT0x16 /* Barometric pressure output */
#define ADIS16448_TEMP_OUT    0x18 /* Temperature output */

/* Calibration parameters */
#define ADIS16400_XGYRO_OFF 0x1A /* X-axis gyroscope bias offset factor */
#define ADIS16400_YGYRO_OFF 0x1C /* Y-axis gyroscope bias offset factor */
#define ADIS16400_ZGYRO_OFF 0x1E /* Z-axis gyroscope bias offset factor */
#define ADIS16400_XACCL_OFF 0x20 /* X-axis acceleration bias offset factor */
#define ADIS16400_YACCL_OFF 0x22 /* Y-axis acceleration bias offset factor */
#define ADIS16400_ZACCL_OFF 0x24 /* Z-axis acceleration bias offset factor */
#define ADIS16400_XMAGN_HIF 0x26 /* X-axis magnetometer, hard-iron factor */
#define ADIS16400_YMAGN_HIF 0x28 /* Y-axis magnetometer, hard-iron factor */
#define ADIS16400_ZMAGN_HIF 0x2A /* Z-axis magnetometer, hard-iron factor */
#define ADIS16400_XMAGN_SIF 0x2C /* X-axis magnetometer, soft-iron factor */
#define ADIS16400_YMAGN_SIF 0x2E /* Y-axis magnetometer, soft-iron factor */
#define ADIS16400_ZMAGN_SIF 0x30 /* Z-axis magnetometer, soft-iron factor */

#define ADIS16400_GPIO_CTRL 0x32 /* Auxiliary digital input/output control */
#define ADIS16400_MSC_CTRL  0x34 /* Miscellaneous control */
#define ADIS16400_SMPL_PRD  0x36 /* Internal sample period (rate) control */
#define ADIS16400_SENS_AVG  0x38 /* Dynamic range and digital filter control */
#define ADIS16400_SLP_CNT   0x3A /* Sleep mode control */
#define ADIS16400_DIAG_STAT 0x3C /* System status */

/* Alarm functions */
#define ADIS16400_GLOB_CMD  0x3E /* System command */
#define ADIS16400_ALM_MAG1  0x40 /* Alarm 1 amplitude threshold */
#define ADIS16400_ALM_MAG2  0x42 /* Alarm 2 amplitude threshold */
#define ADIS16400_ALM_SMPL1 0x44 /* Alarm 1 sample size */
#define ADIS16400_ALM_SMPL2 0x46 /* Alarm 2 sample size */
#define ADIS16400_ALM_CTRL  0x48 /* Alarm control */
#define ADIS16400_AUX_DAC   0x4A /* Auxiliary DAC data */

#define ADIS16334_LOT_ID1   0x52 /* Lot identification code 1 */
#define ADIS16334_LOT_ID2   0x54 /* Lot identification code 2 */
#define ADIS16400_PRODUCT_ID 0x56 /* Product identifier */
#define ADIS16334_SERIAL_NUMBER 0x58 /* Serial number, lot specific */

#define ADIS16400_ERROR_ACTIVE BIT(14)
#define ADIS16400_NEW_DATA     BIT(14)

/* MSC_CTRL */
#define ADIS16400_MSC_CTRL_MEM_TEST      BIT(11)
#define ADIS16400_MSC_CTRL_INT_SELF_TEST BIT(10)
#define ADIS16400_MSC_CTRL_NEG_SELF_TEST BIT(9)
#define ADIS16400_MSC_CTRL_POS_SELF_TEST BIT(8)
#define ADIS16400_MSC_CTRL_GYRO_BIAS     BIT(7)
#define ADIS16400_MSC_CTRL_ACCL_ALIGN    BIT(6)
#define ADIS16400_MSC_CTRL_DATA_RDY_EN   BIT(2)
#define ADIS16400_MSC_CTRL_DATA_RDY_POL_HIGH BIT(1)
#define ADIS16400_MSC_CTRL_DATA_RDY_DIO2        BIT(0)

/* SMPL_PRD */
#define ADIS16400_SMPL_PRD_TIME_BASE BIT(7)
#define ADIS16400_SMPL_PRD_DIV_MASK 0x7F

/* DIAG_STAT */
#define ADIS16400_DIAG_STAT_ZACCL_FAIL BIT(15)
#define ADIS16400_DIAG_STAT_YACCL_FAIL BIT(14)
#define ADIS16400_DIAG_STAT_XACCL_FAIL BIT(13)
#define ADIS16400_DIAG_STAT_XGYRO_FAIL BIT(12)
#define ADIS16400_DIAG_STAT_YGYRO_FAIL BIT(11)
#define ADIS16400_DIAG_STAT_ZGYRO_FAIL BIT(10)
#define ADIS16400_DIAG_STAT_ALARM2     BIT(9)
#define ADIS16400_DIAG_STAT_ALARM1     BIT(8)
#define ADIS16400_DIAG_STAT_FLASH_CHK  BIT(6)
#define ADIS16400_DIAG_STAT_SELF_TEST  BIT(5)
#define ADIS16400_DIAG_STAT_OVERFLOW   BIT(4)
#define ADIS16400_DIAG_STAT_SPI_FAIL   BIT(3)
#define ADIS16400_DIAG_STAT_FLASH_UPT  BIT(2)
#define ADIS16400_DIAG_STAT_POWER_HIGH BIT(1)
#define ADIS16400_DIAG_STAT_POWER_LOW  BIT(0)

/* GLOB_CMD */
#define ADIS16400_GLOB_CMD_SW_RESET     BIT(7)
#define ADIS16400_GLOB_CMD_P_AUTO_NULL  BIT(4)
#define ADIS16400_GLOB_CMD_FLASH_UPD    BIT(3)
#define ADIS16400_GLOB_CMD_DAC_LATCH    BIT(2)
#define ADIS16400_GLOB_CMD_FAC_CALIB    BIT(1)
#define ADIS16400_GLOB_CMD_AUTO_NULL    BIT(0)

/* SLP_CNT */
#define ADIS16400_SLP_CNT_POWER_OFF     BIT(8)

#define ADIS16334_RATE_DIV_SHIFT 8
#define ADIS16334_RATE_INT_CLK BIT(0)

#define ADIS16400_SPI_SLOW (300 * 1000)
#define ADIS16400_SPI_BURST (1000 * 1000)
#define ADIS16400_SPI_FAST (2000 * 1000)

#define ADIS16400_HAS_PROD_ID       BIT(0)
#define ADIS16400_NO_BURST          BIT(1)
#define ADIS16400_HAS_SLOW_MODE     BIT(2)
#define ADIS16400_HAS_SERIAL_NUMBER BIT(3)
#define ADIS16400_BURST_DIAG_STAT   BIT(4)
        



class AP_InertialSensor_ADIS16365 : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_ADIS16365(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return _have_sample_available; }
    bool accel_sample_available(void) { return _have_sample_available; }

    /* Put the ADIS16365 in a known state so it can be
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

    static uint16_t _register_read_16(AP_HAL::SPIDeviceDriver *spi, uint8_t reg);
    static void _register_write_16(AP_HAL::SPIDeviceDriver *spi, uint8_t reg,
                                uint16_t val);

    bool                 _init_sensor(void);
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    uint16_t             _register_read_16( uint8_t reg );
    void                 _register_write_16( uint8_t reg, uint16_t val );
    bool                 _hardware_init(void);
    bool                 _sample_available();

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    // support for updating filter at runtime
    int16_t _last_gyro_filter_hz;
    int16_t _last_accel_filter_hz;

    // change the filter frequency
    void _set_accel_filter(uint8_t filter_hz);
    void _set_gyro_filter(uint8_t filter_hz);


    static int16_t _check_status(AP_HAL::SPIDeviceDriver *spi);

    bool _burst_read(Vector3f *pAccl, Vector3f *pGyro);

    // This structure is used to pass data from the timer which reads
    // the sensor to the main thread. The _shared_data_idx is used to
    // prevent race conditions by ensuring the data is fully updated
    // before being used by the consumer
    struct {
        Vector3f _accel_filtered;
        Vector3f _gyro_filtered;
    } _shared_data[2];
    volatile uint8_t _shared_data_idx;

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

#if DUMP_DATA 
    static void _dump_registers(AP_HAL::SPIDeviceDriver *spi);
#endif
};

#endif // __AP_INERTIAL_SENSOR_ADIS16365_H__
