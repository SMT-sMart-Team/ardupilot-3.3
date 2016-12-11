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
 *       AP_Compass_HMC5983.cpp - Arduino Library for HMC5983 I2C magnetometer
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

// AVR LibC Includes
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Compass_HMC5983.h"

extern const AP_HAL::HAL& hal;

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00 // for 5983, bit7 is temp-comp
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06
#define DataOutputRate_220HZ  0x07 // AB ZhaoYJ@2016-10-19 for HMC5983 

// AB ZhaoYJ@2016-12-11 for average compass value
#define AVERAGE_MAG 1
#define AVERAGE_WIN 64

#define FORMER(curr, n, array_size) ((curr >= n)?(curr - n):(curr + array_size - n)) 
#define N_ORDER 4


// AB ZhaoYJ@2016-12-11 for user-defined 4 order chebyII filter
#define FILTER_TYPE 7 // 7 filters, 4 order with b & a
const double compass_ba[FILTER_TYPE][5*2] = {
    // 0: fc=10Hz
    {0.0009877867510385,-0.003762348901931, 0.005553744695291,-0.003762348901931,
    0.0009877867510385,
    1,   -3.878129734999,    5.641762572816,   -3.648875955419,
    0.8852477379956},
    // 1: fc=20Hz
    {0.001066578484441,-0.003520583754742, 0.004979264107821,-0.003520583754742,
    0.001066578484441,
    1,    -3.75490235187,    5.294313666885,    -3.32185631444,
    0.7825162529919},
    // 2: fc=30Hz
    { 0.001235431141961,-0.003233484708145, 0.004349236721367,-0.003233484708145,
    0.001235431141961,
    1,   -3.628903305608,    4.954076395023,   -3.014454340388,
    0.6896343805619},
    // 3: 40Hz
    {0.001504023202492,-0.002833704229474, 0.003768968353254,-0.002833704229474,
    0.001504023202492,
    1,   -3.498597652843,    4.617828546747,     -2.7230591526,
    0.604937864995},
    // 4: 1Hz-20Hz
    {0.001035118616347, -0.003591819842621, 0.005152284049452, -0.003591819842620,     0.001035118616347,
    1.000000000000000, -3.790224337093825, 5.392406406613549, -3.412836705803140, 0.810693517880321},
    // 5: 2Hz
    {0.0009898590037292,-0.003951630671045, 0.005923551035664,-0.003951630671045,
    0.0009898590037292,
    1,-3.975669032622,    5.927302681588,   -3.927596156134,
    0.9759625148687},
    // 6: 5Hz
    {0.0009820297557845,-0.003880052678305, 0.005796341733976,-0.003880052678305,
    0.0009820297557845,
    1,-3.939149044189,    5.819291999087,   -3.821104108943,
    0.9409614499347}
};

// AB ZhaoYJ@2016-10-19 for HMC5983
#define TEMP_COMPENSTATE_EN   0x80

AP_HMC5983_SPI::AP_HMC5983_SPI()
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_HMC5983);

    if (_spi == NULL) {
        hal.console->printf("Cannot get SPIDevice_HMC5983\n");
        return;
    }
}

void AP_HMC5983_SPI::setHighSpeed(uint8_t speed)
{
}

AP_HAL::Semaphore * AP_HMC5983_SPI::get_semaphore()
{
    return _spi->get_semaphore();
}

#define BURST_CMD 0xC3
#define BURST_LEN 0x6
bool AP_HMC5983_SPI::burst_read(uint8_t *out)
{
    uint8_t tx[BURST_LEN + 1] = {BURST_CMD, };
    uint8_t rx[BURST_LEN + 1];

    _spi->transaction(tx, rx, BURST_LEN + 1);
    memcpy(out, &rx[1], BURST_LEN);
    return true;
}

bool AP_HMC5983_SPI::register_write(uint8_t address, uint8_t value)
{
    uint8_t tx[2] = {address, value};

    _spi->transaction(tx, NULL, 2);
    return true;
}

bool AP_HMC5983_SPI::register_read(uint8_t address, uint8_t *read_out)
{
    address |= 0x80;
    uint8_t tx[2] = { address, };
    uint8_t rx[2];

    _spi->transaction(tx, rx, 2);
    *read_out = rx[1];
    return true;
}



// constructor
AP_Compass_HMC5983::AP_Compass_HMC5983(Compass &compass, AP_HMC5983_SerialBus *bus):
    AP_Compass_Backend(compass),
    _bus(bus),
    _retry_time(0),
    _bus_sem(NULL),
    _mag_x(0),
    _mag_y(0),
    _mag_z(0),
    _mag_x_accum(0),
    _mag_y_accum(0),
    _mag_z_accum(0),
    _accum_count(0),
    _last_accum_time(0),
    _compass_instance(0),
    _product_id(0)
{}

// detect the sensor
AP_Compass_Backend *AP_Compass_HMC5983::detect(Compass &compass)
{
    AP_Compass_HMC5983 *sensor = new AP_Compass_HMC5983(compass, new AP_HMC5983_SPI());
    if (sensor == NULL) {
        hal.util->prt("HMC5983: sensor NULL");
        return NULL;
    }
    if (!sensor->init()) {
        hal.util->prt("HMC5983: delete sensor");
        delete sensor;
        return NULL;
    }

    hal.util->prt("[OK] HMC5983 detected done");

    return sensor;
}

// read_register - read a register value
bool AP_Compass_HMC5983::read_register(uint8_t address, uint8_t *value)
{
    if (!_bus->register_read(address, value)) {
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }
    return true;
}

// write_register - update a register value
bool AP_Compass_HMC5983::write_register(uint8_t address, uint8_t value)
{
    if (!_bus->register_write(address, value)) {
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }
    return true;
}

// Read Sensor data
bool AP_Compass_HMC5983::read_raw()
{
    uint8_t buff[BURST_LEN];

    if (!_bus->burst_read(buff)) {
        _bus->setHighSpeed(false);
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }

    int16_t rx, ry, rz;
    rx = (((int16_t)buff[0]) << 8) | buff[1];
    if ((_product_id == AP_COMPASS_TYPE_HMC5883L) || (_product_id == AP_COMPASS_TYPE_HMC5983)) {
        rz = (((int16_t)buff[2]) << 8) | buff[3];
        ry = (((int16_t)buff[4]) << 8) | buff[5];
    } else {
        ry = (((int16_t)buff[2]) << 8) | buff[3];
        rz = (((int16_t)buff[4]) << 8) | buff[5];
    }
    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

#ifdef SMT_CAPTURE_COMPASS_RAW
#if 0
    static uint32_t log_raw_cnt = 0;
    if((0 == (log_raw_cnt%1000)) || (1 == (log_raw_cnt%1000))) 
    {
        hal.util->prt("[%d us]: compass capture raw", hal.scheduler->micros());
    }
    log_raw_cnt++;
#endif
    publish_field_raw(Vector3f(_mag_x, _mag_y, _mag_z), _compass_instance);
#endif

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_HMC5983::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
#if !RT_TIMER
    _timer();
#endif
}

// reading from the magnetometer
void AP_Compass_HMC5983::_timer(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
   uint32_t tnow = hal.scheduler->micros();
   // if (_accum_count != 0 && (tnow - _last_accum_time) < 13333) {
   // the compass gets new data 75Hz 
   if (_accum_count != 0 && (tnow - _last_accum_time) < 5000) {
   // if (_accum_count != 0 && (tnow - _last_accum_time) < 6250) {
	  // the compass gets new data at 200Hz 
	  return;
   }

   if (!_bus_sem->take(1)) {
       // the bus is busy - try again later
       return;
   }
   bool result = read_raw();
   _bus_sem->give();

   if (result) {
	  // the _mag_N values are in the range -2048 to 2047, so we can
	  // accumulate up to 15 of them in an int16_t. Let's make it 14
	  // for ease of calculation. We expect to do reads at 10Hz, and
	  // we get new data at most 75Hz, so we don't expect to
	  // accumulate more than 8 before a read
      //
      // ChebyII filter & median filter
      //
      uint8_t mag_user_ft = _compass.get_user_filter();
      Vector3f mag_filtered = Vector3f(_mag_x, _mag_y, _mag_z);

#define CHK_FT_TAP 1
#if CHK_FT_TAP 
    static uint32_t mag_cnt = 0;
#endif

      if(mag_user_ft < 0xF) // ChebyII
      {
          mag_filtered = _user_filter(mag_filtered, mag_user_ft);
      }
      else if(mag_user_ft == 0xF) // median
      {
          mag_filtered = _median_filter(mag_filtered);
      }
      else if(mag_user_ft == 0x1F) // median + ChebyII(20Hz)
      {
          mag_filtered = _median_filter(mag_filtered);
          mag_filtered = _user_filter(mag_filtered, 1);
      }
      else if(mag_user_ft == 0x2F) // ChebyII(20Hz) + median 
      {
#if CHK_FT_TAP 
          if((0 == (mag_cnt %4000)) || (1 == (mag_cnt%4000)))
          {
              hal.util->prt("[%d us] mag ft: %d, med_tap: %d", hal.scheduler->micros(), mag_user_ft, _compass.get_med_tap());
          }
#endif
          mag_filtered = _user_filter(mag_filtered, 1);
          mag_filtered = _median_filter(mag_filtered);
      }
#if CHK_FT_TAP 
      mag_cnt++;
#endif

      if(_compass.is_average())
      {
            static Vector3f sample[AVERAGE_WIN];
            static uint16_t sample_idx = 0;
            static bool first = true;
            sample[(sample_idx)] = mag_filtered;
            uint16_t average_len = _compass.get_average_len();
            Vector3f sum;
            if(!first)
            {
                for(uint16_t ii = 0; ii < average_len ; ii++)
                {
                    sum.x += sample[FORMER(sample_idx, ii, AVERAGE_WIN)].x;
                    sum.y += sample[FORMER(sample_idx, ii, AVERAGE_WIN)].y;
                    sum.z += sample[FORMER(sample_idx, ii, AVERAGE_WIN)].z;
                }
            }
            else
            {
                if(average_len == sample_idx)
                {
                    first = false;
                }
            }
            sum = sum/average_len;

	        _mag_x_accum = sum.x;
	        _mag_y_accum = sum.y;
	        _mag_z_accum = sum.z;
	        _accum_count = 1;

#define DEBUG_AVERAGE 0
#if DEBUG_AVERAGE
            static uint16_t cnt = 0;
            if((0 == (cnt%3000)) || (1 == (cnt%3000)))
            {
                // hal.util->prt("[ %d us] MS5803 update %d", hal.scheduler->micros(), cnt);
                hal.util->prt("[%d us] HMC5983 _timer %d ", hal.scheduler->micros(), cnt);
                hal.util->prt(" HMC5983 _timer aver_len %d ", average_len);
                hal.util->prt(" HMC5983 _timer X(aver: %f) val:  ", sum.x);
                for(uint16_t ii = 0; ii < average_len ; ii++)
                {
                    hal.util->prt(" --- %f  ", sample[FORMER(sample_idx, ii, AVERAGE_WIN)].x);
                }

                hal.util->prt("[%d us] HMC5983 _timer Y(aver: %f) val:  ", average_len, sum.y);
                for(uint16_t ii = 0; ii < average_len ; ii++)
                {
                    hal.util->prt(" --- %f  ", sample[FORMER(sample_idx, ii, AVERAGE_WIN)].y);
                }

                hal.util->prt("[%d us] HMC5983 _timer Z(aver: %f) val:  ", average_len, sum.z);
                for(uint16_t ii = 0; ii < average_len ; ii++)
                {
                    hal.util->prt(" --- %f  ", sample[FORMER(sample_idx, ii, AVERAGE_WIN)].z);
                }
            }
            cnt++;
#endif
            sample_idx++;
            sample_idx &= AVERAGE_WIN - 1;

      }
      else
      {
	        _mag_x_accum += mag_filtered.x;
	        _mag_y_accum += mag_filtered.y;
	        _mag_z_accum += mag_filtered.z;
	        _accum_count++;
      }

	  // if (_accum_count == 14) {
	  //    _mag_x_accum /= 2;
	  //    _mag_y_accum /= 2;
	  //    _mag_z_accum /= 2;
	  //    _accum_count = 7;
	  // }
	  _last_accum_time = tnow;
   }
}

/*
 *  re-initialise after a IO error
 */
bool AP_Compass_HMC5983::re_initialise()
{
    if (!write_register(ConfigRegA, _base_config) ||
        !write_register(ConfigRegB, magGain) ||
        !write_register(ModeRegister, ContinuousConversion))
        // !write_register(ModeRegister, SingleConversion))
        return false;
    return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_HMC5983::init()
{
    int numAttempts = 0, good_count = 0;
    bool success = false;
    uint8_t calibration_gain = 0x20;
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    float gain_multiple = 1.0;

    hal.scheduler->suspend_timer_procs();
    hal.scheduler->delay(10);

    _bus_sem = _bus->get_semaphore();
    if (!_bus_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get HMC5983 semaphore"));
    }

    // determine if we are using 5983 or 5883L
    read_register(ConfigRegA, &_base_config);
    // hal.util->prt("HMC5983: configA: 0x%x, pre-wr 0x%x", _base_config, TEMP_COMPENSTATE_EN | SampleAveraging_8<<5 | DataOutputRate_220HZ<<2 | NormalOperation);
    _base_config = 0;
    if (!write_register(ConfigRegA, TEMP_COMPENSTATE_EN | SampleAveraging_8<<5 | DataOutputRate_220HZ<<2 | NormalOperation) ||
        !read_register(ConfigRegA, &_base_config)) {
        _bus_sem->give();
        hal.scheduler->resume_timer_procs();
        hal.util->prt("HMC5983 init wr config reg error");
        return false;
    }


    if ( _base_config == (TEMP_COMPENSTATE_EN | SampleAveraging_8<<5 | DataOutputRate_220HZ<<2 | NormalOperation)) {
        // hal.util->prt("HMC5983 detect");
        // a 5883L supports the sample averaging config
        _product_id = AP_COMPASS_TYPE_HMC5983;
        calibration_gain = 0x60;
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
        gain_multiple = 660.0f / 1090;  // adjustment for runtime vs calibration gain
    } else if ( _base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
        hal.util->prt("HMC5883L detect");
       // a 5883L supports the sample averaging config
        _product_id = AP_COMPASS_TYPE_HMC5883L;
        calibration_gain = 0x60;
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
        gain_multiple = 660.0f / 1090;  // adjustment for runtime vs calibration gain
    } else if (_base_config == (NormalOperation | DataOutputRate_75HZ<<2)) {
        hal.util->prt("HMC5843 detect");
        _product_id = AP_COMPASS_TYPE_HMC5843;
    } else {
        hal.util->prt("HMC5983: error detect-0x%x", _base_config);
        // not behaving like either supported compass type
        _bus_sem->give();
        hal.scheduler->resume_timer_procs();
        return false;
    }

    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;

    while ( success == 0 && numAttempts < 25 && good_count < 5)
    {
        // record number of attempts at initialisation
        numAttempts++;

        // force positiveBias (compass should return 715 for all channels)
        if (!write_register(ConfigRegA, PositiveBiasConfig))
        {
            hal.util->prt("wr configA error");
            continue;      // compass not responding on the bus
        }
        hal.scheduler->delay(50);

        // set gains
        if (!write_register(ConfigRegB, calibration_gain) ||
            // !write_register(ModeRegister, SingleConversion))
            !write_register(ModeRegister, ContinuousConversion))
        {
            hal.util->prt("wr configB & mode error");
            continue;
        }

        // read values from the compass
        hal.scheduler->delay(50);
        if (!read_raw())
            continue;      // we didn't read valid values

        hal.scheduler->delay(10);

        float cal[3];

        // hal.console->printf_P(PSTR("mag %d %d %d\n"), _mag_x, _mag_y, _mag_z);
        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

        // hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f\n"), cal[0], cal[1], cal[2]);

        // we throw away the first two samples as the compass may
        // still be changing its state from the application of the
        // strap excitation. After that we accept values in a
        // reasonable range
        if (numAttempts > 2 &&
            cal[0] > 0.7f && cal[0] < 1.35f &&
            cal[1] > 0.7f && cal[1] < 1.35f &&
            cal[2] > 0.7f && cal[2] < 1.35f) {
            // hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f good\n"), cal[0], cal[1], cal[2]);
            good_count++;
            calibration[0] += cal[0];
            calibration[1] += cal[1];
            calibration[2] += cal[2];
        }

#if 0
        /* useful for debugging */
        // hal.console->printf_P(PSTR("MagX: %d MagY: %d MagZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
        // hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), cal[0], cal[1], cal[2]);
        hal.util->prt("MagX: %d MagY: %d MagZ: %d\n", (int)_mag_x, (int)_mag_y, (int)_mag_z);
        hal.util->prt("CalX: %.2f CalY: %.2f CalZ: %.2f\n", cal[0], cal[1], cal[2]);
#endif
    }

    if (good_count >= 5) {
        /*
          The use of gain_multiple below is incorrect, as the gain
          difference between 2.5Ga mode and 1Ga mode is already taken
          into account by the expected_x and expected_yz values.  We
          are not going to fix it however as it would mean all
          APM1/APM2 users redoing their compass calibration. The
          impact is that the values we report on APM1/APM2 are lower
          than they should be (by a multiple of about 0.6). This
          doesn't have any impact other than the learned compass
          offsets
         */
        calibration[0] = calibration[0] * gain_multiple / good_count;
        calibration[1] = calibration[1] * gain_multiple / good_count;
        calibration[2] = calibration[2] * gain_multiple / good_count;
        success = true;
    } else {
        /* best guess */
        calibration[0] = 1.0;
        calibration[1] = 1.0;
        calibration[2] = 1.0;
    }

    // leave test mode
    if (!re_initialise()) {
        _bus_sem->give();
        hal.scheduler->resume_timer_procs();
        return false;
    }

    _bus_sem->give();
    hal.scheduler->resume_timer_procs();
    _initialised = true;

	// perform an initial read
	read();

#if 0
    hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), 
                          calibration[0], calibration[1], calibration[2]);
#endif

    if (success) {
        // register the compass instance in the frontend
        _compass_instance = register_compass();
        set_dev_id(_compass_instance, _product_id);

#if RT_TIMER
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_HMC5983::_timer, void));
#endif
    }

    return success;
}

// Read Sensor data
void AP_Compass_HMC5983::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
    if (_retry_time != 0) {
        if (hal.scheduler->millis() < _retry_time) {
            return;
        }
        if (!re_initialise()) {
            _retry_time = hal.scheduler->millis() + 1000;
			_bus->setHighSpeed(false);
            return;
        }
    }

	if (_accum_count == 0) {
	   accumulate();
       if (_retry_time != 0) {
		  _bus->setHighSpeed(false);
		  return;
	   }
	}

    Vector3f field(_mag_x_accum * calibration[0],
                   _mag_y_accum * calibration[1],
                   _mag_z_accum * calibration[2]);
    field /= _accum_count;

	_accum_count = 0;
	_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    // rotate to the desired orientation
    // 
    field.rotate(ROTATION_YAW_270);

    publish_field(field, _compass_instance);
    _retry_time = 0;
}

#define TEST_FILTER 0

#define FORMER(curr, n, array_size) ((curr >= n)?(curr - n):(curr + array_size - n)) 

static double median_filter(double *pimu_in, uint8_t median_len)
{
    int i,j;
    double ret;  
    double bTemp;  

      
    for (j = 0; j < median_len; j ++)  
    {  
        for (i = 0; i < median_len - j; i ++)  
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


Vector3f AP_Compass_HMC5983::_user_filter(Vector3f _mag_in, uint8_t _uf)
{
    //  for ChebyII
#define FILTER_MAX_TAP 8
    static Vector3d filter_state[FILTER_MAX_TAP]; 
    static Vector3d filter_out[FILTER_MAX_TAP]; 
    static uint8_t curr_idx = 0;
    static bool first = false;
    Vector3f ret;
    uint8_t ii = 0;
    // Chebyshev II
    const double *b;
    const double *a;


    {
        if(_uf >= FILTER_TYPE) 
        {
            hal.util->prt("[Err] mag filter type wrong: %d", _uf);
            return ret;
        }

        b = &compass_ba[0][0] + (N_ORDER+1)*2*_uf;
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
            filter_state[curr_idx].x = _mag_in.x;
            filter_state[curr_idx].y = _mag_in.y;
            filter_state[curr_idx].z = _mag_in.z;
            // filter x: 
            filter_out[curr_idx].x = b[0]*filter_state[curr_idx].x 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].x 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].x 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].x 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].x 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].x 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].x
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].x
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].x;

            if (isnan(filter_out[curr_idx].x) || isinf(filter_out[curr_idx].x)) {

                filter_out[curr_idx].x = filter_state[curr_idx].x; 
            }

            // filter y
            filter_out[curr_idx].y = b[0]*filter_state[curr_idx].y 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].y 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].y 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].y 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].y 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].y 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].y
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].y
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].y;

            if (isnan(filter_out[curr_idx].y) || isinf(filter_out[curr_idx].y)) {

                filter_out[curr_idx].y = filter_state[curr_idx].y; 
            }

            // filter z
            filter_out[curr_idx].z = b[0]*filter_state[curr_idx].z 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].z 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].z 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].z 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].z 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].z 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].z
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].z
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].z;

            if (isnan(filter_out[curr_idx].z) || isinf(filter_out[curr_idx].z)) {

                filter_out[curr_idx].z = filter_state[curr_idx].z; 
            }

            ret.x = filter_out[curr_idx].x;
            ret.y = filter_out[curr_idx].y;
            ret.z = filter_out[curr_idx].z;

            // update filter postion
            curr_idx++;
            curr_idx &= FILTER_MAX_TAP - 1;
            
        }
    }

    // hal.util->prt("[ %d us] mag filter end", hal.scheduler->micros()); 
    return ret;
}

Vector3f AP_Compass_HMC5983::_median_filter(Vector3f _mag_in)
{
    // for median filter: circular buff 16
#define MED_TAP 64
    static Vector3d med_filter_in[MED_TAP];
    static uint8_t curr_idx = 0;
    static bool first = false;
    Vector3f ret;
    uint8_t ii = 0;

    if(!first)
    {
        uint8_t med_len = _compass.get_med_tap() + 1; // include current in
        if(med_len > 1)
        {
            double med_in_x[MED_TAP]; 
            double med_in_y[MED_TAP];
            double med_in_z[MED_TAP];
            med_filter_in[curr_idx].x = _mag_in.x;
            med_filter_in[curr_idx].y = _mag_in.y;
            med_filter_in[curr_idx].z = _mag_in.z;
            for(uint8_t med_idx = 0; med_idx < med_len; med_idx++)
            {
                uint8_t dist = med_len - 1 - med_idx;
                med_in_x[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].x;
                med_in_y[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].y;
                med_in_z[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].z;
            }
            ret.x = median_filter(med_in_x, med_len);  
            ret.y = median_filter(med_in_y, med_len);  
            ret.z = median_filter(med_in_z, med_len);  
            curr_idx++;
            curr_idx &= MED_TAP - 1;
        }
        else
        {
            // hal.util->prt("[Err] mag mean filter param wrong: ");
            return _mag_in;
        }
    }
    else
    {
        first = false;
        for(uint8_t idx = 0; idx < MED_TAP; idx++)
        {
            med_filter_in[idx].x = 0.0d;
            med_filter_in[idx].y = 0.0d;
            med_filter_in[idx].z = 0.0d;
        }
    }


    // hal.util->prt("[ %d us] mag filter end", hal.scheduler->micros()); 
    return ret;
}
