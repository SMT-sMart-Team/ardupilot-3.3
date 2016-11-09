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
 */

/*
  originally written by Jose Julio, Pat Hickey and Jordi Muñoz

  Heavily modified by Andrew Tridgell
*/

#include <AP_HAL/AP_HAL.h>
#include "AP_Baro.h"

#define DUMP 0
#if DUMP
#include <stdio.h>
#include <stdlib.h>
#define DUMP_D1 0
#define DUMP_D1_COUNT 0

#define DUMP_LEN 0x4000
static uint32_t dump[DUMP_LEN];
static uint32_t dump_cnt = 0;
#endif

#define DEBUG_FLOW 0

// 
#define AVERAGE_PRESS 1
#if AVERAGE_PRESS
#define AVERAGE_WIN 16
#endif


extern const AP_HAL::HAL& hal;

extern bool start_cali_baro;

#define CMD_MS5803_RESET 0x1E
#define CMD_MS5803_PROM_Setup 0xA0
#define CMD_MS5803_PROM_C1 0xA2
#define CMD_MS5803_PROM_C2 0xA4
#define CMD_MS5803_PROM_C3 0xA6
#define CMD_MS5803_PROM_C4 0xA8
#define CMD_MS5803_PROM_C5 0xAA
#define CMD_MS5803_PROM_C6 0xAC
#define CMD_MS5803_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximum resolution (oversampling)

// AB ZhaoYJ@2016-10-30 for merge AC3.4
/* write to one of these addresses to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR256  0x40
#define ADDR_CMD_CONVERT_D1_OSR512  0x42
#define ADDR_CMD_CONVERT_D1_OSR1024 0x44
#define ADDR_CMD_CONVERT_D1_OSR2048 0x46
#define ADDR_CMD_CONVERT_D1_OSR4096 0x48

/* write to one of these addresses to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR256  0x50
#define ADDR_CMD_CONVERT_D2_OSR512  0x52
#define ADDR_CMD_CONVERT_D2_OSR1024 0x54
#define ADDR_CMD_CONVERT_D2_OSR2048 0x56
#define ADDR_CMD_CONVERT_D2_OSR4096 0x58

#define USEC_PER_MSEC 1000ULL

#define CONVERSION_TIME_OSR_4096   9.04 * USEC_PER_MSEC
#define CONVERSION_TIME_OSR_2048   4.54 * USEC_PER_MSEC
#define CONVERSION_TIME_OSR_1024   2.28 * USEC_PER_MSEC
#define CONVERSION_TIME_OSR_0512   1.17 * USEC_PER_MSEC
#define CONVERSION_TIME_OSR_0256   0.60 * USEC_PER_MSEC

/*
  use an OSR of 1024 to reduce the self-heating effect of the
sensor. Information from MS tells us that some individual sensors
are quite sensitive to this effect and that reducing the OSR can
make a big difference
*/
static const uint8_t ADDR_CMD_CONVERT_PRESSURE = ADDR_CMD_CONVERT_D1_OSR1024;
static const uint8_t ADDR_CMD_CONVERT_TEMPERATURE = ADDR_CMD_CONVERT_D2_OSR1024;
static const uint32_t CONVERSION_TIME = CONVERSION_TIME_OSR_1024;


#define MS5803_TEMP_SCALE                                       (float)(100.0)
#define VALUE_OF_20C                                            ((float)(20.0) * MS5803_TEMP_SCALE)
#define VALUE_OF_UNDER_15C                                      ((float)(-15.0) * MS5803_TEMP_SCALE)
#define VALUE_OF_45C                                            ((float)(45.0) * MS5803_TEMP_SCALE)
#define MS5803_MIN_TEMP                                         ((float)(-40.0) * MS5803_TEMP_SCALE)
#define MS5803_MAX_TEMP                                         ((float)(85.0) * MS5803_TEMP_SCALE)
// SPI Device //////////////////////////////////////////////////////////////////

AP_SerialBus_SPI_MS5803::AP_SerialBus_SPI_MS5803(enum AP_HAL::SPIDevice device, enum AP_HAL::SPIDeviceDriver::bus_speed speed) :
    _device(device),
    _speed(speed),
    _spi(NULL),
    _spi_sem(NULL)
{
}

void AP_SerialBus_SPI_MS5803::init()
{

    _spi = hal.spi->device(_device);
    if (_spi == NULL) {
        hal.scheduler->panic(PSTR("did not get valid SPI device driver!"));
    }
    _spi_sem = _spi->get_semaphore();
    if (_spi_sem == NULL) {
        hal.scheduler->panic(PSTR("AP_SerialBus_SPI did not get valid SPI semaphroe!"));
    }
    _spi->set_bus_speed(_speed);
}

uint16_t AP_SerialBus_SPI_MS5803::read_16bits(uint8_t reg)
{
    uint8_t tx[3] = { reg, 0, 0 };
    uint8_t rx[3];
    _spi->transaction(tx, rx, 3);
    return ((uint16_t) rx[1] << 8 ) | ( rx[2] );
}

uint32_t AP_SerialBus_SPI_MS5803::read_24bits(uint8_t reg)
{
    uint8_t tx[4] = { reg, 0, 0, 0 };
    uint8_t rx[4];
    _spi->transaction(tx, rx, 4);
    return (((uint32_t)rx[1])<<16) | (((uint32_t)rx[2])<<8) | ((uint32_t)rx[3]);
}

void AP_SerialBus_SPI_MS5803::write(uint8_t reg)
{
    uint8_t tx[1] = { reg };
    _spi->transaction(tx, NULL, 1);
}

bool AP_SerialBus_SPI_MS5803::sem_take_blocking() 
{
    return _spi_sem->take(10);
}

bool AP_SerialBus_SPI_MS5803::sem_take_nonblocking()
{
    return _spi_sem->take_nonblocking();
}

void AP_SerialBus_SPI_MS5803::sem_give()
{
    _spi_sem->give();
}


/// I2C SerialBus
AP_SerialBus_I2C_MS5803::AP_SerialBus_I2C_MS5803(AP_HAL::I2CDriver *i2c, uint8_t addr) :
    _i2c(i2c),
    _addr(addr),
    _i2c_sem(NULL) 
{
}

void AP_SerialBus_I2C_MS5803::init()
{
    _i2c_sem = _i2c->get_semaphore();
    if (_i2c_sem == NULL) {
        hal.scheduler->panic(PSTR("AP_SerialBus_I2C did not get valid I2C semaphore!"));
    }
}

uint16_t AP_SerialBus_I2C_MS5803::read_16bits(uint8_t reg)
{
    uint8_t buf[2];
    if (_i2c->readRegisters(_addr, reg, sizeof(buf), buf) == 0) {
        return (((uint16_t)(buf[0]) << 8) | buf[1]);
    }
    return 0;
}

uint32_t AP_SerialBus_I2C_MS5803::read_24bits(uint8_t reg)
{
    uint8_t buf[3];
    if (_i2c->readRegisters(_addr, reg, sizeof(buf), buf) == 0) {
        return (((uint32_t)buf[0]) << 16) | (((uint32_t)buf[1]) << 8) | buf[2];
    }
    return 0;
}

void AP_SerialBus_I2C_MS5803::write(uint8_t reg)
{
    _i2c->write(_addr, 1, &reg);
}

bool AP_SerialBus_I2C_MS5803::sem_take_blocking() 
{
    return _i2c_sem->take(10);
}

bool AP_SerialBus_I2C_MS5803::sem_take_nonblocking()
{
    return _i2c_sem->take_nonblocking();
}

void AP_SerialBus_I2C_MS5803::sem_give()
{
    _i2c_sem->give();
}

/*
  constructor
 */
AP_Baro_MS58XX::AP_Baro_MS58XX(AP_Baro &baro, AP_SerialBus *serial, bool use_timer) :
    AP_Baro_Backend(baro),
    _serial(serial),
    _updated(false),
    _state(0),
    _last_timer(0),
    _use_timer(use_timer),
    _D1(0.0f),
    _D2(0.0f)
{
    // // AB ZhaoYJ@2016-11-06 for adding sem to avoid _timer start before update
    _sem = hal.util->new_semaphore();
    if (_sem == NULL) {
        hal.scheduler->panic(PSTR("AP_Baro_MS5803: failed to create semaphore!"));
    }

    _instance = _frontend.register_sensor();
    _serial->init();
    if (!_serial->sem_take_blocking()){
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_MS58XX: failed to take serial semaphore for init"));
    }

    _serial->write(CMD_MS5803_RESET);
    hal.scheduler->delay(4);

    // We read the factory calibration
    // The on-chip CRC is not used
    _C1 = _serial->read_16bits(CMD_MS5803_PROM_C1);
    _C2 = _serial->read_16bits(CMD_MS5803_PROM_C2);
    _C3 = _serial->read_16bits(CMD_MS5803_PROM_C3);
    _C4 = _serial->read_16bits(CMD_MS5803_PROM_C4);
    _C5 = _serial->read_16bits(CMD_MS5803_PROM_C5);
    _C6 = _serial->read_16bits(CMD_MS5803_PROM_C6);

    if (!_check_crc()) {
        hal.scheduler->panic(PSTR("Bad CRC on MS5803"));
    }

    hal.util->prt("[OK] MS5803 detected done");

    // Send a command to read Temp first
    _serial->write(ADDR_CMD_CONVERT_TEMPERATURE);
    _last_timer = hal.scheduler->micros();
    _state = 0;

    _s_D1 = 0;
    _s_D2 = 0;
    _d1_count = 0;
    _d2_count = 0;

    _serial->sem_give();

    if (_use_timer) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Baro_MS58XX::_timer, void));
    }
}

/**
 * MS5803 crc4 method based on PX4Firmware code
 */
bool AP_Baro_MS58XX::_check_crc(void)
{
    int16_t cnt;
    uint16_t n_rem;
    uint16_t crc_read;
    uint8_t n_bit;
    uint16_t n_prom[8] = { _serial->read_16bits(CMD_MS5803_PROM_Setup),
                           _C1, _C2, _C3, _C4, _C5, _C6,
                           _serial->read_16bits(CMD_MS5803_PROM_CRC) };
    n_rem = 0x00;

    /* save the read crc */
    crc_read = n_prom[7];

    /* remove CRC byte */
    n_prom[7] = (0xFF00 & (n_prom[7]));

    for (cnt = 0; cnt < 16; cnt++) {
        /* uneven bytes */
        if (cnt & 1) {
            n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

        } else {
            n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;

            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    /* final 4 bit remainder is CRC value */
    n_rem = (0x000F & (n_rem >> 12));
    n_prom[7] = crc_read;

    /* return true if CRCs match */
    return (0x000F & crc_read) == (n_rem ^ 0x00);
}


/*
  Read the sensor. This is a state machine
  We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
  temperature does not change so quickly...
*/
void AP_Baro_MS58XX::_timer(void)
{
#if DEBUG_FLOW 
    static uint16_t cnt = 0;
    if((0 == (cnt%10000)) || (1 == (cnt%10000)))
    {
        hal.util->prt("[ %d us] timer %d", hal.scheduler->micros(), cnt);
    }
    cnt++;
#endif
    // Throttle read rate to 100hz maximum.
    // if (hal.scheduler->micros() - _last_timer < CONVERSION_TIME) {
    // Throttle read rate to 100hz maximum.
    // if (hal.scheduler->micros() - _last_timer < 10000) {
    if (hal.scheduler->micros() - _last_timer < CONVERSION_TIME) {
        return;
    }

    if (!_serial->sem_take_nonblocking()) {
        return;
    }

    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        // hal.util->prt("[ %d us] timer take sem", hal.scheduler->micros());
        if (_state == 0) {
            // On state 0 we read temp
            uint32_t d2 = _serial->read_24bits(0);
            if (d2 != 0) {
                _s_D2 += d2;
                _d2_count++;
                if (_d2_count == 32) {
                    // we have summed 32 values. This only happens
                    // when we stop reading the barometer for a long time
                    // (more than 1.2 seconds)
                    _s_D2 >>= 1;
                    _d2_count = 16;
                }
            }
            _state++;
            _serial->write(ADDR_CMD_CONVERT_PRESSURE);      // Command to read pressure
        } else {
            uint32_t d1 = _serial->read_24bits(0);
            // AB ZhaoYJ@2016-11-07 for SPI error
            if ((d1 != 0) && (0xFFFFFF != d1)) 
            {
#if AVERAGE_PRESS
                static uint32_t sample[AVERAGE_WIN];
                static uint16_t sample_idx = 0;
                static bool first = true;
                uint32_t d1_orig = d1;
                sample[(sample_idx++)%AVERAGE_WIN] = d1;
                if(!first)
                {
                    uint32_t sum = 0;
                    for(uint16_t ii = 0; ii < AVERAGE_WIN; ii++)
                    {
                        sum += sample[ii];
                    }
                    d1 = sum/AVERAGE_WIN;
                }
                else
                {
                    if(AVERAGE_WIN == sample_idx)
                    {
                        first = false;
                    }
                }
#endif
                // occasional zero values have been seen on the PXF
                // board. These may be SPI errors, but safest to ignore
                _s_D1 += d1;
                _d1_count++;
#if DUMP_D1
                if(start_cali_baro)
                {
                    if((0 == (dump_cnt%(DUMP_LEN >> 3))) || (1 == (dump_cnt%(DUMP_LEN >> 3))))
                    {
                        // hal.util->prt("[ %d us]: MS5803 dumpcnt %d(%s) d1(%d)->average_d1(%d)", hal.scheduler->micros(), dump_cnt, start_cali_baro?"cali":"uncali", d1_orig, d1);
                        hal.util->prt("[ %d us]: MS5803 dumpcnt %d(%s)", hal.scheduler->micros(), dump_cnt, start_cali_baro?"cali":"uncali");
                    }
                    if(dump_cnt < DUMP_LEN)
                    {
#if DUMP_D1_COUNT    
                        dump[dump_cnt++] = _d1_count;
#elif DUMP_D1
                        dump[dump_cnt++] = d1;
#endif
                    }
                    else if(DUMP_LEN == dump_cnt)
                    {
                        FILE *fd = fopen("/root/test/dump.log", "w");
                        if(fd)
                        {
                            for(uint32_t ii = 0; ii < DUMP_LEN; ii++)
                            {
                                fprintf(fd, "%d\n", dump[ii]);
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
                // hal.util->prt("[ %d us]: MS5803 timer dumpcnt-%d, _d1_count-%d", hal.scheduler->micros(), dump_cnt, _d1_count);
                if (_d1_count == 128) {
                    // we have summed 128 values. This only happens
                    // when we stop reading the barometer for a long time
                    // (more than 1.2 seconds)
                    _s_D1 >>= 1;
                    _d1_count = 64;
                }
                // Now a new reading exists
                _updated = true;
            }
            else
            {
                // AB ZhaoYJ@2016-11-07
                // validate value for SPI error
                hal.util->prt("[Err] bad Baro pressure data");
            }
            _state++;
            if (_state == 5) {
                _serial->write(ADDR_CMD_CONVERT_TEMPERATURE); // Command to read temperature
                _state = 0;
            } else {
                _serial->write(ADDR_CMD_CONVERT_PRESSURE); // Command to read pressure
            }
        }

        _last_timer = hal.scheduler->micros();
        // hal.util->prt("[ %d us] timer give sem", hal.scheduler->micros());
    }

    _sem->give();

    _serial->sem_give();
}

void AP_Baro_MS58XX::update()
{

    if (!_use_timer) {
        // if we're not using the timer then accumulate one more time
        // to cope with the calibration loop and minimise lag
        accumulate();
    }

    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
#if DEBUG_FLOW 
        static uint16_t cnt = 0;
        if((0 == (cnt%100)) || (1 == (cnt%100)))
        {
            // hal.util->prt("[ %d us] MS5803 update %d", hal.scheduler->micros(), cnt);
            hal.util->prt("[ %d us] update take sem", hal.scheduler->micros());
        }
        cnt++;
#endif
    }

    if (!_updated || !_d1_count) {
        _sem->give();
        return;
    }
    uint32_t sD1, sD2;
    uint8_t d1count, d2count;

    // Suspend timer procs because these variables are written to
    // in "_update".
    // hal.scheduler->suspend_timer_procs();
    sD1 = _s_D1; _s_D1 = 0;
    sD2 = _s_D2; _s_D2 = 0;
    d1count = _d1_count; _d1_count = 0;
    d2count = _d2_count; _d2_count = 0;
    _updated = false;
    // hal.scheduler->resume_timer_procs();

    _sem->give();

#if DUMP_D1_COUNT
    if((0 == (dump_cnt%(DUMP_LEN >> 3))) || (1 == (dump_cnt%(DUMP_LEN >> 3))))
    {
        hal.util->prt("[ %d us]: MS5803 dumpcnt %d", hal.scheduler->micros(), dump_cnt);
    }
    if(dump_cnt < DUMP_LEN)
    {
        dump[dump_cnt++] = d1count;
    }
    else if(DUMP_LEN == dump_cnt)
    {
        FILE *fd = fopen("/root/test/dump.log", "w");
        if(fd)
        {
            for(uint32_t ii = 0; ii < DUMP_LEN; ii++)
            {
                fprintf(fd, "%d\n", dump[ii]);
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
#endif
    
    if (d1count != 0) {
        _D1 = ((float)sD1) / d1count;
    }
    if (d2count != 0) {
        _D2 = ((float)sD2) / d2count;
    }
    _calculate();
}

/* MS5803 class */
AP_Baro_MS5803::AP_Baro_MS5803(AP_Baro &baro, AP_SerialBus *serial, bool use_timer)
    :AP_Baro_MS58XX(baro, serial, use_timer)
{}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5803::_calculate()
{
	float dT;
	float TEMP;
	float OFF;
	float SENS;

	float T2 = 0;
	float Aux = 0;
	float OFF2 = 0;
	float SENS2 = 0;
	// Formulas from manufacturer datasheet
	// sub -20c temperature compensation is not included

	// we do the calculations using floating point
	// as this is much faster on an AVR2560, and also allows
	// us to take advantage of the averaging of D1 and D1 over
	// multiple samples, giving us more precision
	dT = _D2 - (((uint32_t) _C5) << 8);
	TEMP = 2000 + (dT * _C6) / 8388608;
	OFF = _C2 * 65536.0f + (_C4 * dT) / 128;
	SENS = _C1 * 32768.0f + (_C3 * dT) / 256;

	if (TEMP < VALUE_OF_20C) {
		// second order temperature compensation when under 20 degrees C
		T2 = (dT * dT) / 0x80000000;
		Aux = (TEMP - 2000) * (TEMP - 2000);
		OFF2 = 3.0f * Aux;
		SENS2 = (7.0f * Aux) / 8;
		if (TEMP < VALUE_OF_UNDER_15C) {
			SENS2 = SENS2 + 2 * ((TEMP + 1500) * (TEMP + 1500));
		}

	} else {
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
		if (TEMP > VALUE_OF_45C)
			SENS2 = SENS2 - (((TEMP - 4500) * (TEMP - 4500)) / 8);
	}
	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	float pressure = (_D1 * SENS / 2097152 - OFF) / 32768;
	float temperature = TEMP * 0.01f;
	_copy_to_frontend(_instance, pressure, temperature);
}

/*
  Read the sensor from main code. This is only used for I2C MS5803 to
  avoid conflicts on the semaphore from calling it in a timer, which
  conflicts with the compass driver use of I2C
*/
void AP_Baro_MS58XX::accumulate(void)
{
    if (!_use_timer) {
        // the timer isn't being called as a timer, so we need to call
        // it in accumulate()
        _timer();
    }
}
