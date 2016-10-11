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

#include "AP_InertialSensor_ADIS16365.h"
#include <AP_HAL_Linux/GPIO.h>

extern const AP_HAL::HAL& hal;


/*
 *  
 *  
 */

AP_InertialSensor_ADIS16365::AP_InertialSensor_ADIS16365(AP_InertialSensor &imu) :
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
}


/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_ADIS16365::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_ADIS16365 *sensor = new AP_InertialSensor_ADIS16365(_imu);
    if (sensor == NULL) {
        hal.util->prt("sensor NULL");
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        hal.util->prt("sensor init error");
        delete sensor;
        return NULL;
    }

    return sensor;
}

bool AP_InertialSensor_ADIS16365::initialize_driver_state() {
    AP_HAL::SPIDeviceDriver *spi = hal.spi->device(AP_HAL::SPIDevice_ADIS16365);
    if (!spi)
    {
        hal.util->prt("error: spi device NULL");
        return false;
    }

    AP_HAL::SPIDeviceDriver::State state = spi->get_state();
    if (state == AP_HAL::SPIDeviceDriver::State::FAILED)
    {
        hal.util->prt("error: spi state FAILED");
        return false;
    }
    if (state == AP_HAL::SPIDeviceDriver::State::RUNNING)
        return true;

    /* First time trying the initialization: if it fails from now on we will
     * set the device state to State::Failed so we don't try again if another
     * driver asks it */
    spi->set_state(AP_HAL::SPIDeviceDriver::State::FAILED);

    uint16_t id = _register_read_16(spi, ADIS16400_PRODUCT_ID);
    if (id != AP_PRODUCT_ID_ADIS16365) {
        hal.console->printf("ADIS16365: unexpected PROD_ID 0x%x\n", (unsigned)id);
        hal.util->prt("ADIS16365: unexpected PROD_ID 0x%x\n", (unsigned)id);
        goto fail_id;
    }

#if ADIS16365_DEBUG 
    hal.util->prt("ADIS16365: PROD_ID %d\n", id);
#endif
    
    // initially run the bus at low speed
    spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);

    // Chip reset
    //
    _register_write_16(spi, ADIS16400_GLOB_CMD, ADIS16400_GLOB_CMD_SW_RESET);
    hal.scheduler->delay(100);
    //
    // Chip self-test
    _register_write_16(spi, ADIS16400_MSC_CTRL, ADIS16400_MSC_CTRL_MEM_TEST |
            ADIS16400_MSC_CTRL_INT_SELF_TEST | ADIS16400_MSC_CTRL_NEG_SELF_TEST |
            ADIS16400_MSC_CTRL_POS_SELF_TEST);

    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {

        hal.scheduler->delay(10);
        uint16_t status = _check_status(spi);
        if (status == 0) {
            break;
        }
    }

    if (tries == 5) {
        hal.util->prt("ADIS16365 self-test failed!");
        goto fail_tries;
    }

    spi->set_state(AP_HAL::SPIDeviceDriver::State::RUNNING);

    return true;

fail_tries:
fail_id:
    spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    return false;
}

/*
  initialise the sensor
 */
bool AP_InertialSensor_ADIS16365::_init_sensor(void)
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_ADIS16365);
    _spi_sem = _spi->get_semaphore();

    if (!_hardware_init())
    {
        hal.util->prt("hw init failed");
        return false;
    }

    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    _product_id = 0;

    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS16365::_poll_data, void));

#if ADIS16365_DEBUG
    _dump_registers(_spi);
#endif
    return true;
}

/*
  update the accel and gyro vectors
 */
bool AP_InertialSensor_ADIS16365::update( void )
{
#if ADIS16365_DEBUG
    static uint16_t cnt = 0;
    cnt++;
    if(!(cnt%100))
    {
        _dump_registers(_spi);
    }
#endif
#if 0
    // pull the data from the timer shared data buffer
    uint8_t idx = _shared_data_idx;
    Vector3f gyro = _shared_data[idx]._gyro_filtered;
    Vector3f accel = _shared_data[idx]._accel_filtered;

    _have_sample_available = false;

    accel *= ADIS16365_ACCEL_SCALE_1G;
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
#endif

    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */

/**
 * Timer process to poll for new data from the ADIS16365.
 */
void AP_InertialSensor_ADIS16365::_poll_data(void)
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
void AP_InertialSensor_ADIS16365::_read_data_transaction() 
{
#if 0
    /* one resister address followed by seven 2-byte registers */
    struct PACKED {
        uint8_t cmd;
        uint8_t int_status;
        uint8_t v[14];
    } rx, tx = { cmd : MPUREG_INT_STATUS | 0x80, };

    _spi->transaction((const uint8_t *)&tx, (uint8_t *)&rx, sizeof(rx));

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))

    Vector3f _accel_filtered = _accel_filter.apply(Vector3f(int16_val(rx.v, 1),
                                                   int16_val(rx.v, 0),
                                                   -int16_val(rx.v, 2)));

    Vector3f _gyro_filtered = _gyro_filter.apply(Vector3f(int16_val(rx.v, 5),
                                                 int16_val(rx.v, 4),
                                                 -int16_val(rx.v, 6)));
    // update the shared buffer
    uint8_t idx = _shared_data_idx ^ 1;
    _shared_data[idx]._accel_filtered = _accel_filtered;
    _shared_data[idx]._gyro_filtered = _gyro_filtered;
    _shared_data_idx = idx;

    _have_sample_available = true;
#endif
}

/*
  read an 8 bit register
 */
uint8_t AP_InertialSensor_ADIS16365::_register_read(AP_HAL::SPIDeviceDriver *spi,
                                                  uint8_t reg)
{
    uint8_t addr = reg; // Set most significant bit
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
void AP_InertialSensor_ADIS16365::_register_write(AP_HAL::SPIDeviceDriver *spi,
                                                uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg  | 0x80;
    tx[1] = val;
    spi->transaction(tx, rx, 2);
}

inline uint8_t AP_InertialSensor_ADIS16365::_register_read(uint8_t reg)
{
    return _register_read(_spi, reg);
}


inline void AP_InertialSensor_ADIS16365::_register_write(uint8_t reg, uint8_t val)
{
    _register_write(_spi, reg, val);
}


/*
  read an 16 bit register
 */
uint16_t AP_InertialSensor_ADIS16365::_register_read_16(AP_HAL::SPIDeviceDriver *spi,
                                                  uint8_t reg)
{
    uint8_t tx[2] = {reg, 0};
    uint8_t rx[2];

    spi->transaction(tx, rx, 2);


    tx[0] = 0;
    tx[1] = 0;

    spi->transaction(tx, rx, 2);


    return (rx[0] << 8 | rx[1]);
}

/*
  write an 16 bit register
 */
void AP_InertialSensor_ADIS16365::_register_write_16(AP_HAL::SPIDeviceDriver *spi,
                                                uint8_t reg, uint16_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | 0x80;
    tx[1] = (uint8_t) (val & 0xFF);

    spi->transaction(tx, rx, 2);

    tx[0] = (uint8_t)((reg + 1) | 0x80);
    tx[1] = (uint8_t) (val >> 8);

    spi->transaction(tx, rx, 2);
}

// WR 16bit reg
inline void AP_InertialSensor_ADIS16365::_register_write_16(uint8_t reg, uint16_t val)
{
    _register_write(_spi, reg, val);
}

// RD 16bit reg
inline uint16_t AP_InertialSensor_ADIS16365::_register_read_16(uint8_t reg)
{

    return _register_read_16(_spi, reg);
}

/*
  set the accel filter frequency
 */
void AP_InertialSensor_ADIS16365::_set_accel_filter(uint8_t filter_hz)
{
    _accel_filter.set_cutoff_frequency(1000, filter_hz);
}

/*
  set the gyro filter frequency
 */
void AP_InertialSensor_ADIS16365::_set_gyro_filter(uint8_t filter_hz)
{
    _gyro_filter.set_cutoff_frequency(1000, filter_hz);
}


/*
  initialise the sensor configuration registers
 */
bool AP_InertialSensor_ADIS16365::_hardware_init(void)
{
    // we need to suspend timers to prevent other SPI drivers grabbing
    // the bus while we do the long initialisation
    hal.scheduler->suspend_timer_procs();

    if (!_spi_sem->take(100)) {
        hal.console->printf("ADIS16365: Unable to get semaphore");
        return false;
    }

    if (!initialize_driver_state())
        return false;

    // initialise ADIS16365 config

    // now that we have initialised, we set the SPI bus speed to high
    // (8MHz on APM2)
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    _spi_sem->give();

    hal.scheduler->resume_timer_procs();

    return true;
}


int16_t AP_InertialSensor_ADIS16365::_check_status(AP_HAL::SPIDeviceDriver *spi)
{
    return _register_read_16(spi, ADIS16400_DIAG_STAT);
}

#if ADIS16365_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_ADIS16365::_dump_registers(AP_HAL::SPIDeviceDriver *spi)
{
    hal.console->println_P(PSTR("ADIS16365 registers"));
    hal.util->prt("============================");
    hal.util->prt("Supply Voltage: %d\n", _register_read_16(spi, ADIS16400_SUPPLY_OUT ));
#if 0
    for (uint8_t reg=0; reg<=126; reg++) {
        uint8_t v = _register_read(spi, reg);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
            hal.console->println();
        }
    }
#endif
    hal.console->println();
}
#endif


#endif // CONFIG_HAL_BOARD
