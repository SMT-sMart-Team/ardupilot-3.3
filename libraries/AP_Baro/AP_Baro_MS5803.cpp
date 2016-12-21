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
#define AVERAGE_WIN 64


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
static const uint8_t ADDR_CMD_CONVERT_PRESSURE = ADDR_CMD_CONVERT_D1_OSR1024; // ADDR_CMD_CONVERT_D1_OSR4096
static const uint8_t ADDR_CMD_CONVERT_TEMPERATURE = ADDR_CMD_CONVERT_D2_OSR1024; // ADDR_CMD_CONVERT_D2_OSR4096;
static const uint32_t CONVERSION_TIME = CONVERSION_TIME_OSR_1024; //CONVERSION_TIME_OSR_4096;


#define MS5803_TEMP_SCALE                                       (float)(100.0)
#define VALUE_OF_20C                                            ((float)(20.0) * MS5803_TEMP_SCALE)
#define VALUE_OF_UNDER_15C                                      ((float)(-15.0) * MS5803_TEMP_SCALE)
#define VALUE_OF_45C                                            ((float)(45.0) * MS5803_TEMP_SCALE)
#define MS5803_MIN_TEMP                                         ((float)(-40.0) * MS5803_TEMP_SCALE)
#define MS5803_MAX_TEMP                                         ((float)(85.0) * MS5803_TEMP_SCALE)
// SPI Device //////////////////////////////////////////////////////////////////
//
#define FORMER(curr, n, array_size) ((curr >= n)?(curr - n):(curr + array_size - n)) 
#define N_ORDER 4


// AB ZhaoYJ@2016-12-11 for user-defined 4 order chebyII filter
#define FILTER_TYPE 7 // 7 filters, 4 order with b & a
// fs: 200Hz
const double baro_ba[FILTER_TYPE][(N_ORDER+1)*2] = {
    // 0: fc=10Hz
    {0.001066578484441,-0.003520583754742, 0.004979264107821,-0.003520583754742,
    0.001066578484441,
    1,    -3.75490235187,    5.294313666885,    -3.32185631444,
    0.7825162529919},
    // 1: 20Hz
    {0.001504023202492,-0.002833704229474, 0.003768968353254,-0.002833704229474,
    0.001504023202492,
    1,   -3.498597652843,    4.617828546747,     -2.7230591526,
    0.604937864995},
    // 2: fc=30Hz
    {0.002440006636488,-0.001243697363317, 0.003428681549348,-0.001243697363317,
    0.002440006636488,
    1,   -3.217868090935,    3.945654810763,   -2.177266033495,
    0.4553006137634},
    // 3: fc=40Hz
    {0.004259816772569, 0.002810421933046, 0.006247310625167, 0.002810421933046,
    0.004259816772569,
    1,   -2.894721965841,    3.256463919134,   -1.668134292247,
    0.3267801269904},
    // 4: 5Hz-20Hz
    {0.001504023202492,-0.002833704229474, 0.003768968353254,-0.002833704229474,
    0.001504023202492,
    1,   -3.498597652843,    4.617828546747,     -2.7230591526,
    0.604937864995},
    // 5: 2Hz
    {0.0009836753866896,-0.003903796945646, 0.005840364964036,-0.003903796945646,
    0.0009836753866896,
    1,   -3.951327305119,    5.855163083587,   -3.856327448311,
    0.9524917916895},
    // 6: 5Hz
    {0.0009877867510385,-0.003762348901931, 0.005553744695291,-0.003762348901931,
    0.0009877867510385,
    1,   -3.878129734999,    5.641762572816,   -3.648875955419,
    0.8852477379956}
};

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
    // for average
    static float sample[AVERAGE_WIN];
    static uint16_t sample_idx = 0;
    static bool first = true;

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
            // avoid pulse value
            if ((d1 != 0) && (d1 < 10600000)) 
            {
                uint8_t baro_user_ft = _frontend.get_user_filter();
                float baro_filtered = float(d1)/100.0f; // zoom in 100 for val is too big

#define CHK_FT_TAP 0
#if CHK_FT_TAP 
                static uint32_t baro_cnt = 0;
#endif          

                if(baro_user_ft < 0xF) // ChebyII
                {
                    baro_filtered = _user_filter(baro_filtered, baro_user_ft);
                }
                else if(baro_user_ft == 0xF) // median
                {
                    baro_filtered = _median_filter(baro_filtered);
                }
                else if(baro_user_ft & 0x10) // median + ChebyII( &0xF Hz)
                {
                    baro_filtered = _median_filter(baro_filtered);
                    baro_filtered = _user_filter(baro_filtered, baro_user_ft & 0xF);
                }
                else if(baro_user_ft & 0x20) // ChebyII(Hz) + median 
                {
#if CHK_FT_TAP 
                    if((0 == (baro_cnt %4000)) || (1 == (baro_cnt%4000)))
                    {
                        hal.util->prt("[%d us] baro ft: %d (cheby %d), med_tap: %d", hal.scheduler->micros(), baro_user_ft, baro_user_ft&0xF, _frontend.get_med_tap());
                    }
#endif          
                    baro_filtered = _user_filter(baro_filtered, baro_user_ft & 0xF);
                    baro_filtered = _median_filter(baro_filtered);
                }

                if(_frontend.is_average())
                {
                    sample[(sample_idx)%AVERAGE_WIN] = baro_filtered;
                    uint16_t average_len = _frontend.get_average_len();
                    float sum = 0;
                    if(!first)
                    {
                        for(uint16_t ii = 0; ii < average_len; ii++)
                        {
                            sum += sample[FORMER(sample_idx, ii, AVERAGE_WIN)];
#if CHK_FT_TAP 
                            if((0 == (baro_cnt %4000)) || (1 == (baro_cnt%4000)))
                            {
                                hal.util->prt("[%d us] baro sum val: %f", hal.scheduler->micros(), 
                                        sample[FORMER(sample_idx, ii, AVERAGE_WIN)]);
                            }
#endif
                        }

#if CHK_FT_TAP 
                    if((0 == (baro_cnt %4000)) || (1 == (baro_cnt%4000)))
                    {
                        hal.util->prt("[%d us] baro aver before", hal.scheduler->micros());
                    }
#endif
                        baro_filtered = (sum/average_len);
#if CHK_FT_TAP 
                    if((0 == (baro_cnt %4000)) || (1 == (baro_cnt%4000)))
                    {
                        hal.util->prt("[%d us] baro aver after", hal.scheduler->micros());
                    }
#endif
                    }
                    else
                    {
                        if(average_len == sample_idx)
                        {
                            first = false;
                        }
                    }
                    sample_idx++;
                    sample_idx &= AVERAGE_WIN - 1;
#if CHK_FT_TAP 
                    if((0 == (baro_cnt %4000)) || (1 == (baro_cnt%4000)))
                    {
                        hal.util->prt("[%d us] baro sum: total %f (len: %d), aver %f", hal.scheduler->micros(), sum, average_len, baro_filtered);
                    }
#endif
                }

#if CHK_FT_TAP 
                if((0 == (baro_cnt %4000)) || (1 == (baro_cnt%4000)))
                {
                    hal.util->prt("[%d us] baro val: %d -> %d (%f)", hal.scheduler->micros(), d1, (uint32_t)(baro_filtered*100.0f), baro_filtered);
                }
                baro_cnt++;
#endif          
                // occasional zero values have been seen on the PXF
                // board. These may be SPI errors, but safest to ignore
                _s_D1 += (uint32_t)(baro_filtered*100.0f);
                _d1_count++;

                // record raw pressure

#ifdef SMT_CAPTURE_BARO_RAW
                // if(_frontend.is_log_raw())
                {
	                // float dT = _D2 - (((uint32_t) _C5) << 8);
	                // // TEMP = 2000 + (dT * _C6) / 8388608;
	                // float OFF = (_C2 << 16) + (_C4 * dT) / 128;
	                // float SENS = (_C1 << 15) + (_C3 * dT) / 256;
                    // float pressure = (_D1*SENS/2097152 - OFF)/32768;
                    // float temperature = (TEMP + 2000) * 0.01f;
#if 0
                    static uint32_t log_raw_cnt = 0;
                    if((0 == (log_raw_cnt%1000)) || (1 == (log_raw_cnt%1000))) 
                    {
                        hal.util->prt("[%d us]: baro capture raw", hal.scheduler->micros());
                    }
                    log_raw_cnt++;
#endif
	                _copy_to_frontend_raw(_instance, d1, 0x88);
                }
#endif

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
	TEMP = 2000.0f + (dT * (float)_C6) / 8388608.0f; // * (1.1920929e-7); //  / 8388608;
	OFF = _C2 * 65536.0f + (_C4 * dT) * 0.0078125f; // / 128;
	SENS = _C1 * 32768.0f + (_C3 * dT) * 0.00390625f; // / 256;

	if (TEMP < VALUE_OF_20C) {
		// second order temperature compensation when under 20 degrees C
		T2 = (dT * dT) *  0.000465661f * 0.000001f; // / 0x80000000;
		Aux = (TEMP - 2000.0f) * (TEMP - 2000.0f);
		OFF2 = 3.0f * Aux;
		SENS2 = (7.0f * Aux) * 0.125f; // / 8;
		if (TEMP < VALUE_OF_UNDER_15C) {
			SENS2 = SENS2 + 2.0f * ((TEMP + 1500.0f) * (TEMP + 1500.0f));
		}

	} else {
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
		if (TEMP > VALUE_OF_45C)
			SENS2 = SENS2 - (((TEMP - 4500.0f) * (TEMP - 4500.0f)) * 0.125f);
	}
	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	double pressure = (_D1 * SENS * 0.00476837f * 0.0001f /* / 2097152*/ - OFF) * 0.000030517578125f; // / 32768;
	float temperature = TEMP * 0.01f;
	_copy_to_frontend(_instance, (float)pressure, temperature);
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

float AP_Baro_MS58XX::_user_filter(float _in, uint8_t _uf)
{
    //  for ChebyII
#define FILTER_MAX_TAP 8
    static double filter_state[FILTER_MAX_TAP]; 
    static double filter_out[FILTER_MAX_TAP]; 
    static uint8_t curr_idx = 0;
    static bool first = true;
    float ret = 0.0f;
    // Chebyshev II
    const double *b;
    const double *a;

        if(_uf >= FILTER_TYPE) 
        {
            hal.util->prt("[Err] baro filter type wrong: %d", _uf);
            return ret;
        }

        b = &baro_ba[0][0] + (N_ORDER+1)*2*_uf;
        a = b + (N_ORDER+1);

#if TEST_FILTER 
    static uint32_t incr = 0;
    // if((0 == incr%4000) || (1 == incr%4000) || (2 == incr%4000))
    {
#if 0
        hal.util->prt("acc filter: %d", _imu.get_accl_user_filter());
        hal.util->prt("gyro filter: %d", _imu.get_gyro_user_filter());
        hal.util->prt("mean filter former: %d", _imu.get_mean_filter_former());
        hal.util->prt("mean filter latter: %d", _imu.get_mean_filter_latter());
        hal.util->prt("sizeof ba: %d", sizeof(ba));
        hal.util->prt("filter b: %.19f, %.19f, %.19f, %.19f, %.19f", 
                b[0], b[1], b[2], b[3], b[4]);
        hal.util->prt("filter a: %.19f, %.19f, %.19f, %.19f, %.19f", 
                a[0], a[1], a[2], a[3], a[4]);
        hal.util->prt("ChebyII filter idx: curr_idx %d, %d, %d, %d, %d", 
                curr_idx,
                FORMER(curr_idx, 1, FILTER_MAX_TAP), 
                FORMER(curr_idx, 2, FILTER_MAX_TAP), 
                FORMER(curr_idx, 3, FILTER_MAX_TAP), 
                FORMER(curr_idx, 4, FILTER_MAX_TAP)); 
        uint8_t med_f_len = _imu.get_mean_filter_former() + _imu.get_mean_filter_latter();
        hal.util->prt("Median filter idx: len: %d, curr_idx %d", med_f_len, curr_idx);
            for(uint8_t med_idx = 0; med_idx < med_f_len; med_idx++)
            {
                uint8_t jj = (med_f_len - med_idx);
                hal.util->prt("in idx: <%d>", FORMER(curr_idx, jj, MED_TAP));
            }
#endif

    }
    incr++;
#endif
        if(!first)
        {
            // update state
            filter_state[curr_idx] = _in;
            // filter x: 
            filter_out[curr_idx] = b[0]*filter_state[curr_idx] 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)] 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)] 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)] 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)] 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)] 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)]
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)]
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)];

            if (isnan(filter_out[curr_idx]) || isinf(filter_out[curr_idx])) {

                filter_out[curr_idx] = filter_state[curr_idx]; 
            }

            ret = filter_out[curr_idx];

            // update filter postion
            curr_idx++;
            curr_idx &= FILTER_MAX_TAP - 1;
            
        }
        else
        {
            filter_state[curr_idx]  = _in; 
            filter_out[curr_idx]  = _in; 
            curr_idx++;
            if(curr_idx == FILTER_MAX_TAP)
            {
                first = false;
                curr_idx = 0;
            }
            ret = _in;
        }

    // hal.util->prt("[ %d us] mag filter end", hal.scheduler->micros()); 
    return ret;
}

static double median_filter(double *pimu_in, uint8_t median_len)
{
    int i,j;
    double ret = 0.0f;  
    double bTemp;  

      
    for (j = 0; j < (median_len - 1); j++)  
    {  
        for (i = 0; i < (median_len - j - 1); i++)  
        {  
            if (pimu_in[i] > pimu_in[i + 1])  
            {  
                bTemp = pimu_in[i];  
                pimu_in[i] = pimu_in[i + 1];  
                pimu_in[i + 1] = bTemp;  
            }  
        }  
    }  

    // 计算中值  
    if ((median_len & 1) > 0)  
    {  
        // 数组有奇数个元素，返回中间一个元素  
        ret = pimu_in[median_len / 2];  
    }  
    else  
    {  
        // 数组有偶数个元素，返回中间两个元素平均值  
        ret = (pimu_in[median_len / 2 - 1] + pimu_in[median_len / 2]) / 2;  
    }  
  
    return ret;  

}

float AP_Baro_MS58XX::_median_filter(float _in)
{
    // for median filter: circular buff 16
#define MED_TAP 64
    static double med_filter_in[MED_TAP];
    static uint8_t curr_idx = 0;
    static bool first = true;
    float ret = 0.0f;

    if(!first)
    {
        uint8_t med_len = _frontend.get_med_tap() + 1; // include current in
        if(med_len > 1)
        {
            double med_in[MED_TAP]; 
            med_filter_in[curr_idx] = _in;
            for(uint8_t med_idx = 0; med_idx < med_len; med_idx++)
            {
                uint8_t dist = med_len - 1 - med_idx;
                med_in[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)];
            }
            ret = median_filter(med_in, med_len);  
            curr_idx++;
            curr_idx &= MED_TAP - 1;
        }
        else
        {
            // hal.util->prt("[Err] mag mean filter param wrong: ");
            return _in;
        }
    }
    else
    {
        med_filter_in[curr_idx] = _in; 
        curr_idx++;
        if(curr_idx == MED_TAP)
        {
            first = false;
            curr_idx = 0;
        }
        ret = _in;
    }


    // hal.util->prt("[ %d us] mag filter end", hal.scheduler->micros()); 
    return ret;
}
