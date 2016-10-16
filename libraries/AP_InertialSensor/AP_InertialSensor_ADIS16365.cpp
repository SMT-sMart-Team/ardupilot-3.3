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
#if ADIS16365_DEBUG
    hal.util->prt("[%d us]: detect done", hal.scheduler->micros());
#endif

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

    uint16_t id = 0xFFFF;

    // seems like need to warmup/wakeup ADIS
    id = _register_read_16(spi, ADIS16334_LOT_ID1);

    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        id = _register_read_16(spi, ADIS16400_PRODUCT_ID);
        if (id != AP_PRODUCT_ID_ADIS16365) {
            hal.console->printf("ADIS16365: unexpected PROD_ID 0x%x\n", (unsigned)id);
            hal.util->prt("ADIS16365: unexpected PROD_ID 0x%x\n", (unsigned)id);
            if(5 == tries)
                goto fail_id;
        }
    }

    hal.util->prt("ADIS16365: PROD_ID %d\n", id);
    
    // initially run the bus at low speed
    spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);

    // Chip reset
    //
#if !FAST_BOOT
    _register_write_16(spi, ADIS16400_GLOB_CMD, ADIS16400_GLOB_CMD_SW_RESET);
    hal.scheduler->delay(ADIS16400_STARTUP_DELAY);
    //
#endif
    // Chip self-test
    _register_write_16(spi, ADIS16400_MSC_CTRL, ADIS16400_MSC_CTRL_MEM_TEST |
            ADIS16400_MSC_CTRL_INT_SELF_TEST | ADIS16400_MSC_CTRL_NEG_SELF_TEST |
            ADIS16400_MSC_CTRL_POS_SELF_TEST);

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
    // hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS16365::_poll_data, void));

    return true;
}

/*
  update the accel and gyro vectors
 */
bool AP_InertialSensor_ADIS16365::update( void )
{
#if DUMP_DATA
    static uint16_t cnt = 0;
    cnt++;
    if(!(cnt%100))
    {
        _dump_registers(_spi);
    }
#endif

    _read_data_transaction();

    // pull the data from the timer shared data buffer
    uint8_t idx = _shared_data_idx;
    Vector3f gyro = _shared_data[idx]._gyro_filtered;
    Vector3f accel = _shared_data[idx]._accel_filtered;

    _have_sample_available = false;

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
    _spi_sem->give();
}


#define SIGNED_FLOAT(value, mask_bit)  \
    ((value & (1 << (mask_bit - 1)))?( -1.0 * (~(value - 1) & (0xFFFF >> (16 - mask_bit)))) : (1.0 * (value & (0xFFFF >> (16 - mask_bit)))))
#define SUPPLY_SCALE 2.418e-3 // V
#define GYRO_SCALE   0.05 // deg/sec
// #define ACCEL_SCALE  3.333e-3 // mg
#define ACCEL_SCALE  3.333e-2 // mg
/*
  read from the data registers and update filtered data
 */
void AP_InertialSensor_ADIS16365::_read_data_transaction() 
{

    Vector3f accel_sample, gyro_sample;


    // check if data ready
    //
#if !BURST_READ
    // read accel
    uint16_t ax, ay, az, gx, gy, gz;
    ax = _register_read_16(ADIS16400_XACCL_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    ay = _register_read_16(ADIS16400_YACCL_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    az = _register_read_16(ADIS16400_ZACCL_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    
    // read gyro
    gx = _register_read_16(ADIS16400_XGYRO_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    gy = _register_read_16(ADIS16400_YGYRO_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    gz = _register_read_16(ADIS16400_ZGYRO_OUT) & (0xFFFF >> 2); // 2 = 16 - 14

    
    // scale
    accel_sample.x = ACCEL_SCALE * SIGNED_FLOAT(ax, 14); 
    accel_sample.y = ACCEL_SCALE * SIGNED_FLOAT(ay, 14); 
    accel_sample.z = ACCEL_SCALE * SIGNED_FLOAT(az, 14); 

    gyro_sample.x = GYRO_SCALE * SIGNED_FLOAT(gx, 14); 
    gyro_sample.y = GYRO_SCALE * SIGNED_FLOAT(gy, 14);
    gyro_sample.z = GYRO_SCALE * SIGNED_FLOAT(gz, 14);

    uint16_t volt = _register_read_16(ADIS16400_SUPPLY_OUT) & (0xFFFF >> (16 - 12));
    static uint16_t err = 0;
    if((volt > 2481) || (volt < 1364))
    {
        err++;
        hal.util->prt("sample error : %d", err);
        return ;
    }
#else

    // TODO: burst read
    if(!_burst_read(&accel_sample, &gyro_sample))
    {
        hal.util->prt("Warning: adis16365 burst read failed");
        return; 
    }
#endif

    // filter and shared
    Vector3f _accel_filtered = _accel_filter.apply(accel_sample);
    Vector3f _gyro_filtered = _gyro_filter.apply(gyro_sample);

// #if ADIS16365_DEBUG
#if 0
    static uint16_t cnt = 0;
    cnt++;
    if((0 == (cnt%1000)) || (1 == (cnt%1000)))
    {
        hal.util->prt("[%d us]: ============================", hal.scheduler->micros());
        hal.util->prt("Supply Voltage: %f[%d]\n", SUPPLY_SCALE * (_register_read_16(ADIS16400_SUPPLY_OUT) & (0xFFFF >> (16 - 12))), _register_read_16(ADIS16400_SUPPLY_OUT) & (0xFFFF >> (16 - 12)));
        hal.util->prt("new sample: accel_x: %f[%d], accel_y: %f[%d], accel_z: %f[%d]\n",
            accel_sample.x, ax, accel_sample.y, ay, accel_sample.z, az);

        hal.util->prt("new sample: gyro_x: %f[%d], gyro_y: %f[%d], gyro_z: %f[%d]\n",
            gyro_sample.x, gx, gyro_sample.y, gy, gyro_sample.z, gz);

        hal.util->prt("filtered: accel_x: %f, accel_y: %f, accel_z: %f\n", 
            _accel_filtered.x, _accel_filtered.y, _accel_filtered.z);
        hal.util->prt("filtered: gyro_x: %f, gyro_y: %f, gyro_z: %f\n", 
            _gyro_filtered.x, _gyro_filtered.y, _gyro_filtered.z);
    }
#endif



    // update the shared buffer
    uint8_t idx = _shared_data_idx ^ 1;
    _shared_data[idx]._accel_filtered = _accel_filtered;
    _shared_data[idx]._gyro_filtered = _gyro_filtered;
    _shared_data_idx = idx;

    _have_sample_available = true;

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

    hal.scheduler->delay_microseconds(T_STALL);

    tx[0] = 0;
    tx[1] = 0;

    spi->transaction(tx, rx, 2);

    hal.scheduler->delay_microseconds(T_STALL);


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

    hal.scheduler->delay_microseconds(T_STALL);

    tx[0] = (uint8_t)((reg + 1) | 0x80);
    tx[1] = (uint8_t) (val >> 8);

    hal.scheduler->delay_microseconds(T_STALL);

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

#define ADIS_BURST 0
#if ADIS_BURST
#define BURST_TX_MSG_LEN 22 // 11 16bits = 22 bytes
#else
#define BURST_TX_MSG_LEN 16 // 7 + 1 16bits = 16 bytes
#endif
bool AP_InertialSensor_ADIS16365::_burst_read(Vector3f *pAccl, Vector3f *pGyro)
{
    uint8_t rx[BURST_TX_MSG_LEN];


    uint16_t ax, ay, az, gx, gy, gz;


#if ADIS_BURST
    uint8_t tx_burst[2] = {
        0x3E, 0x00
    };
    // send burst msg
    _spi->transaction(tx_burst, rx, 2);
    hal.scheduler->delay_microseconds(T_READRATE);

    uint8_t tx[BURST_TX_MSG_LEN] = {
        0x00, 0x00, // supply volt
        0x00, 0x00, // gx
        0x00, 0x00, // gy
        0x00, 0x00, // gz
        0x00, 0x00, // ax
        0x00, 0x00, // ay
        0x00, 0x00, // az
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00
    };

    // collect burst data
    memset(rx, 0x0, sizeof(rx));
    _spi->transaction(tx, rx, BURST_TX_MSG_LEN);

    uint8_t idx = 0; // start with 2nd DOUT

#else
    uint8_t tx[BURST_TX_MSG_LEN] = {
        ADIS16400_SUPPLY_OUT, 0x00,
        ADIS16400_XGYRO_OUT, 0x00,
        ADIS16400_YGYRO_OUT, 0x00,
        ADIS16400_ZGYRO_OUT, 0x00,
        ADIS16400_XACCL_OUT, 0x00,
        ADIS16400_YACCL_OUT, 0x00,
        ADIS16400_ZACCL_OUT, 0x00,
        0x00, 0x00
    };
    _spi->transaction(tx, rx, BURST_TX_MSG_LEN);

    uint8_t idx = 2; // start with 2nd DOUT

#endif

#if ADIS16365_DEBUG
    static uint16_t cnt = 0;
    cnt++;
    if((0 == (cnt%1000)) || (1 == (cnt%1000)))
    {
        hal.util->prt("[%d us]: ============================", hal.scheduler->micros());
        hal.util->prt("bv: %f[%d]\n", SUPPLY_SCALE * ((rx[idx] << 8 | rx[idx+1]) & (0xFFFF >> (16 - 12))), ((rx[idx] << 8 | rx[idx+1]) & (0xFFFF >> (16 - 12))));
    }
#endif

    // check if bus error
    uint16_t volt = (rx[idx] << 8 | rx[idx+1]) & (0xFFFF >> (16 - 12));
    static uint16_t err = 0;
    if((volt > 2481) || (volt < 1364))
    {
        err++;
        hal.util->prt("sample error : %d", err);
        return false;
    }

    idx += 2;
    gx = (rx[idx] << 8 | rx[idx+1]);
    idx += 2;
    gy = (rx[idx] << 8 | rx[idx+1]);
    idx += 2;
    gz = (rx[idx] << 8 | rx[idx+1]);
    idx += 2;
    ax = (rx[idx] << 8 | rx[idx+1]);
    idx += 2;
    ay = (rx[idx] << 8 | rx[idx+1]);
    idx += 2;
    az = (rx[idx] << 8 | rx[idx+1]);

    // scale
    pAccl->x = ACCEL_SCALE * SIGNED_FLOAT(ax, 14); 
    pAccl->y = ACCEL_SCALE * SIGNED_FLOAT(ay, 14); 
    pAccl->z = ACCEL_SCALE * SIGNED_FLOAT(az, 14); 

    pGyro->x = GYRO_SCALE * SIGNED_FLOAT(gx, 14); 
    pGyro->y = GYRO_SCALE * SIGNED_FLOAT(gy, 14);
    pGyro->z = GYRO_SCALE * SIGNED_FLOAT(gz, 14);

    return true;

}

#if DUMP_DATA 
// dump all config registers - used for debug
void AP_InertialSensor_ADIS16365::_dump_registers(AP_HAL::SPIDeviceDriver *spi)
{
    hal.console->println_P(PSTR("ADIS16365 registers"));
    hal.util->prt("============================");
    hal.util->prt("Supply Voltage: %f[%d]\n", SUPPLY_SCALE * _register_read_16(spi, ADIS16400_SUPPLY_OUT), _register_read_16(spi, ADIS16400_SUPPLY_OUT) & (0xFFFF >> (16 - 12)));

    Vector3f accel_sample, gyro_sample;

    uint16_t ax, ay, az, gx, gy, gz;

    // check if data ready
    //
    // read accel
    ax = _register_read_16(spi, ADIS16400_XACCL_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    ay = _register_read_16(spi, ADIS16400_YACCL_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    az = _register_read_16(spi, ADIS16400_ZACCL_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    accel_sample.x = ACCEL_SCALE * SIGNED_FLOAT(ax, 14); 
    accel_sample.y = ACCEL_SCALE * SIGNED_FLOAT(ay, 14); 
    accel_sample.z = ACCEL_SCALE * SIGNED_FLOAT(az, 14); 
    
    hal.util->prt("new sample: accel_x: %f[%d], accel_y: %f[%d], accel_z: %f[%d]\n",
            accel_sample.x, ax, accel_sample.y, ay, accel_sample.z, az);
    // read gyro
    gx = _register_read_16(spi, ADIS16400_XGYRO_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    gy = _register_read_16(spi, ADIS16400_YGYRO_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    gz = _register_read_16(spi, ADIS16400_ZGYRO_OUT) & (0xFFFF >> 2); // 2 = 16 - 14
    gyro_sample.x = GYRO_SCALE * SIGNED_FLOAT(gx, 14); 
    gyro_sample.y = GYRO_SCALE * SIGNED_FLOAT(gy, 14);
    gyro_sample.z = GYRO_SCALE * SIGNED_FLOAT(gz, 14);

    hal.util->prt("new sample: gyro_x: %f[%d], gyro_y: %f[%d], gyro_z: %f[%d]\n",
            gyro_sample.x, gx, gyro_sample.y, gy, gyro_sample.z, gz);
}
#endif


#endif // CONFIG_HAL_BOARD
