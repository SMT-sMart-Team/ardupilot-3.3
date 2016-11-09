/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

    -- Coded by Victor Mayoral Vilches --
*/

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_InertialSensor_ICM20689.h"
#include <AP_HAL_Linux/GPIO.h>

extern const AP_HAL::HAL& hal;

extern bool start_cali;


#define DUMP 0
#if DUMP
#include <stdio.h>
#include <stdlib.h>

#define DUMP_LEN 0x10000
static float dump[DUMP_LEN];
static uint32_t dump_cnt = 0;
#endif

#define DEBUG_FLOW 0


#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05

// ICM20689 registers
#define MPUREG_XA_OFFS_H                                0x77    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x78    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x7A    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x7B    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x7D    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x7E    // Z axis accelerometer offset (low byte)

// MPU6000 & ICM20689 registers
// not sure if present in ICM20689
// #define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                             0x00
#       define MPUREG_SMPLRT_500HZ                              0x01
#       define MPUREG_SMPLRT_250HZ                              0x03
#       define MPUREG_SMPLRT_200HZ                              0x04
#       define MPUREG_SMPLRT_100HZ                              0x09
#       define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG                                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                              0x00
#       define BITS_GYRO_FS_500DPS                              0x08
#       define BITS_GYRO_FS_1000DPS                             0x10
#       define BITS_GYRO_FS_2000DPS                             0x18
#       define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#       define BITS_GYRO_YGYRO_SELFTEST                 0x40
#       define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG                             0x1C
#define MPUREG_ACCEL_CONFIG2                            0x1D
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                                  0x23
#define MPUREG_INT_PIN_CFG                              0x37
#       define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                                 0x20    // latch data ready pin
#define MPUREG_INT_ENABLE                               0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS                               0x3A // some diff from MPU9250
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                                  0x01
#       define BIT_DMP_INT                                              0x02
#       define BIT_UNKNOWN_INT                                  0x04
#       define BIT_I2C_MST_INT                                  0x08
#       define BIT_FIFO_OFLOW_INT                               0x10
#       define BIT_ZMOT_INT                                             0x20
#       define BIT_MOT_INT                                              0x40
#       define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A // litte diff from 9250
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_AUTO                  0x01            //  auto mode
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // little diff from 9250, allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_FIFO_COUNTH                              0x72
#define MPUREG_FIFO_COUNTL                              0x73
#define MPUREG_FIFO_R_W                                 0x74
#define MPUREG_WHOAMI                                   0x75
#define MPUREG_WHOAMI_ICM20689                          0x98


// Configuration bits MPU 3000, MPU 6000 and ICM20689
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07

// AB ZhaoYJ@2016-11-01 for debugging LPF
// according to GYRO (also for ACCEL)
#define GYRO_SCALE_250DPS
// #define EN_LPF
#ifdef EN_LPF
#define BITS_DLPF_CFG_HZ BITS_DLPF_CFG_20HZ
// #define BITS_DLPF_CFG_HZ BITS_DLPF_CFG_42HZ                              
// #define BITS_DLPF_CFG_HZ BITS_DLPF_CFG_98HZ                              
#endif

#define ACCEL_SCALE_4G
#ifdef ACCEL_SCALE_4G
#define ICM20689_ACCEL_SCALE_1G    (GRAVITY_MSS / 8192.0f) // 4g
#else
// ICM20689 accelerometer scaling for 16g range: 2^15LSB/16g = 2048LSB/g, 1g = 9.8/2048 
#define ICM20689_ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f) // 16g
#endif

/*
 *  PS-MPU-9250A-00.pdf, page 8, lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */

#ifdef GYRO_SCALE_250DPS
#define GYRO_SCALE (0.0174532f / 131.0f) // radis to degree per LSB
#else
#define GYRO_SCALE (0.0174532f / 16.4f) // radis to degree per LSB
#endif

/*
 *  PS-MPU-9250A-00.pdf, page 9, lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPUXk
 *  variants however
 */

AP_InertialSensor_ICM20689::AP_InertialSensor_ICM20689(AP_InertialSensor &imu) :
	AP_InertialSensor_Backend(imu),
    _last_accel_filter_hz(-1),
    _last_gyro_filter_hz(-1),
    _shared_data_idx(0),
    _accel_filter(1000, 15),
    _gyro_filter(1000, 15),
    _have_sample_available(false),
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#ifdef SMT_NEW_BOARD
    _default_rotation(ROTATION_NONE)
#else
    _default_rotation(ROTATION_ROLL_180_YAW_270)
#endif
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
    /* no rotation needed */
    _default_rotation(ROTATION_NONE)
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    _default_rotation(ROTATION_NONE)
#else /* rotate for bbone default (and other boards) */
    _default_rotation(ROTATION_ROLL_180_YAW_90)
#endif
{
    // // AB ZhaoYJ@2016-11-06 for adding sem to avoid _timer start before update
    _sem = hal.util->new_semaphore();
    if (_sem == NULL) {
        hal.scheduler->panic(PSTR("ICM20689: failed to create semaphore!"));
    }
}


/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_ICM20689::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_ICM20689 *sensor = new AP_InertialSensor_ICM20689(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }

    hal.util->prt("[OK] ICM20689 detected done");

    return sensor;
}

bool AP_InertialSensor_ICM20689::initialize_driver_state() {
    AP_HAL::SPIDeviceDriver *spi = hal.spi->device(AP_HAL::SPIDevice_ICM20689);
    if (!spi)
        return false;

    AP_HAL::SPIDeviceDriver::State state = spi->get_state();
    if (state == AP_HAL::SPIDeviceDriver::State::FAILED)
        return false;
    if (state == AP_HAL::SPIDeviceDriver::State::RUNNING)
        return true;

    /* First time trying the initialization: if it fails from now on we will
     * set the device state to State::Failed so we don't try again if another
     * driver asks it */
    spi->set_state(AP_HAL::SPIDeviceDriver::State::FAILED);

    uint8_t whoami = _register_read(spi, MPUREG_WHOAMI);
    if (whoami != MPUREG_WHOAMI_ICM20689) {
        hal.console->printf("ICM20689: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
        goto fail_whoami;
    }

    // hal.util->prt("ICM20689: whoami - 0x%x", whoami);

    // initially run the bus at low speed
    spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {


        // reset device
        _register_write(spi, MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        // disable I2C as recommended by the datasheet
        _register_write(spi, MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);

        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(spi, MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_AUTO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(spi, MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_AUTO) {
            break;
        }

        hal.scheduler->delay(10);
        uint8_t status = _register_read(spi, MPUREG_INT_STATUS);
        if ((status & BIT_RAW_RDY_INT) != 0) {
            break;
        }
#if ICM20689_DEBUG
        _dump_registers(_spi);
#endif
    }

    if (tries == 5) {
        hal.console->println_P(PSTR("Failed to boot ICM20689 5 times"));
        goto fail_tries;
    }

    spi->set_state(AP_HAL::SPIDeviceDriver::State::RUNNING);

    return true;

fail_tries:
fail_whoami:
    spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    return false;
}

/*
  initialise the sensor
 */
bool AP_InertialSensor_ICM20689::_init_sensor(void)
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_ICM20689);
    _spi_sem = _spi->get_semaphore();

    if (!_hardware_init())
        return false;

    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    _product_id = 689; // ICM20689

    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ICM20689::_poll_data, void));

#if ICM20689_DEBUG
    _dump_registers(_spi);
#endif
    return true;
}

/*
  update the accel and gyro vectors
 */
bool AP_InertialSensor_ICM20689::update( void )
{
    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
#if DEBUG_FLOW 
        static uint16_t cnt = 0;
        if((0 == (cnt%300)) || (1 == (cnt%300)))
        {
            // hal.util->prt("[ %d us] MS5803 update %d", hal.scheduler->micros(), cnt);
            hal.util->prt("[ %d us] ICM20689 update %d ", hal.scheduler->micros(), cnt);
        }
        cnt++;
#endif
    }


    // pull the data from the timer shared data buffer
    uint8_t idx = _shared_data_idx;
    Vector3f gyro = _shared_data[idx]._gyro_filtered;
    Vector3f accel = _shared_data[idx]._accel_filtered;

    _have_sample_available = false;

    _sem->give();

    // accel: g
    // gyro: degree/s
    accel *= ICM20689_ACCEL_SCALE_1G;
    gyro *= GYRO_SCALE;

    accel.rotate(_default_rotation);
    gyro.rotate(_default_rotation);

    _publish_gyro(_gyro_instance, gyro);
    _publish_accel(_accel_instance, accel);

    if (_last_accel_filter_hz != _accel_filter_cutoff()) {
        _set_accel_filter(_accel_filter_cutoff());
        _last_accel_filter_hz = _accel_filter_cutoff();
    }

    if (_last_gyro_filter_hz != _gyro_filter_cutoff()) {
        _set_gyro_filter(_gyro_filter_cutoff());
        _last_gyro_filter_hz = _gyro_filter_cutoff();
    }

    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */

/**
 * Timer process to poll for new data from the ICM20689.
 */
void AP_InertialSensor_ICM20689::_poll_data(void)
{
    if (!_spi_sem->take_nonblocking()) {
        /*
          the semaphore being busy is an expected condition when the
          mainline code is calling wait_for_sample() which will
          grab the semaphore. We return now and rely on the mainline
          code grabbing the latest sample.
        */
        return;
    }
    _read_data_transaction();
    _spi_sem->give();
}


/*
  read from the data registers and update filtered data
 */
void AP_InertialSensor_ICM20689::_read_data_transaction() 
{
    /* one resister address followed by seven 2-byte registers */
    struct PACKED {
        uint8_t cmd;
        uint8_t int_status;
        uint8_t v[14];
    } rx, tx = { cmd : MPUREG_INT_STATUS | 0x80, };

#if DEBUG_FLOW 
    static uint16_t cnt = 0;
    if((0 == (cnt%3000)) || (1 == (cnt%3000)))
    {
        hal.util->prt("[ %d us] ICM20689 timer %d", hal.scheduler->micros(), cnt);
    }
    cnt++;
#endif

    _spi->transaction((const uint8_t *)&tx, (uint8_t *)&rx, sizeof(rx));

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

    Vector3f _accel_filtered = _accel_filter.apply(Vector3f(int16_val(rx.v, 1),
                                                   int16_val(rx.v, 0),
                                                   -int16_val(rx.v, 2)));

    Vector3f _gyro_filtered = _gyro_filter.apply(Vector3f(int16_val(rx.v, 5),
                                                 int16_val(rx.v, 4),
                                                 -int16_val(rx.v, 6)));

    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        // update the shared buffer
        uint8_t idx = _shared_data_idx ^ 1;
        _shared_data[idx]._accel_filtered = _accel_filtered;
        _shared_data[idx]._gyro_filtered = _gyro_filtered;
        _shared_data_idx = idx;

        _have_sample_available = true;
        _sem->give();
#if DUMP
        if(start_cali)
        {
        if((0 == (dump_cnt%(DUMP_LEN >> 3))) || (1 == (dump_cnt%(DUMP_LEN >> 3))))
        {
            hal.util->prt("[ %d us] ICM20689 dumpcnt %d (%s)", hal.scheduler->micros(), dump_cnt, start_cali?"cali":"no cali");
        }
        if(dump_cnt < DUMP_LEN)
        {
            // dump[dump_cnt++] = _accel_filtered.z;
            dump[dump_cnt++] = _gyro_filtered.z;
        }
        else if(DUMP_LEN == dump_cnt)
        {
            FILE *fd = fopen("/root/test/dump.log", "w");
            if(fd)
            {
                for(uint32_t ii = 0; ii < DUMP_LEN; ii++)
                {
                    fprintf(fd, "%f\n", dump[ii]);
                }
                fclose(fd);
                hal.util->prt("[OK] dump log done");
                exit(1);
            }
            else
            {
                hal.util->prt("[Err] failed to open dump log");
            }
        }
        }
#endif
    }
}

/*
  read an 8 bit register
 */
uint8_t AP_InertialSensor_ICM20689::_register_read(AP_HAL::SPIDeviceDriver *spi,
                                                  uint8_t reg)
{
    uint8_t addr = reg | 0x80; // Set most significant bit
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    spi->transaction(tx, rx, 2);

    return rx[1];
}

/*
  write an 8 bit register
 */
void AP_InertialSensor_ICM20689::_register_write(AP_HAL::SPIDeviceDriver *spi,
                                                uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    spi->transaction(tx, rx, 2);
}

inline uint8_t AP_InertialSensor_ICM20689::_register_read(uint8_t reg)
{
    return _register_read(_spi, reg);
}

inline void AP_InertialSensor_ICM20689::_register_write(uint8_t reg, uint8_t val)
{
    _register_write(_spi, reg, val);
}

/*
  set the accel filter frequency
 */
void AP_InertialSensor_ICM20689::_set_accel_filter(uint8_t filter_hz)
{
    _accel_filter.set_cutoff_frequency(1000, filter_hz);
}

/*
  set the gyro filter frequency
 */
void AP_InertialSensor_ICM20689::_set_gyro_filter(uint8_t filter_hz)
{
    _gyro_filter.set_cutoff_frequency(1000, filter_hz);
}


/*
  initialise the sensor configuration registers
 */
bool AP_InertialSensor_ICM20689::_hardware_init(void)
{
    // we need to suspend timers to prevent other SPI drivers grabbing
    // the bus while we do the long initialisation
    hal.scheduler->suspend_timer_procs();

    if (!_spi_sem->take(100)) {
        hal.console->printf("ICM20689: Unable to get semaphore");
        return false;
    }

    if (!initialize_driver_state())
        return false;

    // need test: ZhaoYJ@2016-10-19
    _register_write(MPUREG_PWR_MGMT_2, 0x00);            // only used for wake-up in accelerometer only low power mode

    // used no filter of 256Hz on the sensor, then filter using
    // the 2-pole software filter
#ifdef  EN_LPF
#define ACCEL_LPF_EN 0x8
#define ACCEL_AVERAGE_SAMPLE 0x30 // 32 samples
    _register_write(MPUREG_CONFIG, BITS_DLPF_CFG_HZ);
    _register_write(MPUREG_ACCEL_CONFIG2, (ACCEL_LPF_EN | BITS_DLPF_CFG_HZ));
    // _register_write(MPUREG_ACCEL_CONFIG2, (ACCEL_AVERAGE_SAMPLE | ACCEL_LPF_EN | BITS_DLPF_CFG_HZ));
#else
    _register_write(MPUREG_CONFIG, BITS_DLPF_CFG_256HZ_NOLPF2);
#endif

    // set sample rate to 1kHz, and use the 2 pole filter to give the
    // desired rate
    _register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_1000HZ);
#ifdef GYRO_SCALE_250DPS
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_250DPS);  // Gyro scale 250º/s
#else
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s
#endif

#ifdef ACCEL_SCALE_4G
    // RM-MPU-9250A-00.pdf, pg. 15, select accel full scale 4g
    _register_write(MPUREG_ACCEL_CONFIG,1<<3);
#else
    // RM-MPU-9250A-00.pdf, pg. 15, select accel full scale 16g
    _register_write(MPUREG_ACCEL_CONFIG,3<<3);
#endif

    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);

    // now that we have initialised, we set the SPI bus speed to high
    // (8MHz on APM2)
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    _spi_sem->give();

    hal.scheduler->resume_timer_procs();

    return true;
}

#if ICM20689_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_ICM20689::_dump_registers(AP_HAL::SPIDeviceDriver *spi)
{
    hal.console->println_P(PSTR("ICM20689 registers"));
    for (uint8_t reg=0; reg<=126; reg++) {
        uint8_t v = _register_read(spi, reg);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}
#endif


#endif // CONFIG_HAL_BOARD
