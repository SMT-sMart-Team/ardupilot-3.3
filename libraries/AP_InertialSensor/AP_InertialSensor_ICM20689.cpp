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

// AB ZhaoYJ@2016-11-30 for user-defined 4 order chebyII filter
#define FILTER_TYPE 15 // 7 filters, 4 order with b & a

#define FORMER(curr, n, array_size) ((curr >= n)?(curr - n):(curr + array_size - n)) 

#define ELLIPTIC_80DB 1
#if !IMU_FAST_SAMPLE 
#if 0
const double ba[FILTER_TYPE][5*2] = {
    // 0: fc=10Hz
    {  9.941241125902e-05,-0.0003945263576618,0.0005902402068445,-0.0003945263576618,
          9.941241125902e-05,
            1,   -3.972458622135,    5.917754504397,   -3.918130095139,
                  0.972834225191},
    // 1: fc=20Hz
    {  0.0001003734958431,-0.0003890744437156,0.0005775969901159,-0.0003890744437156,
          0.0001003734958431,
            1,   -3.944864674435,     5.83610895405,   -3.837599574185,
                 0.9463554896637},
    // 2: fc=30Hz
    {   0.000102862236555,-0.0003835252095693, 0.000562306474394,-0.0003835252095693,
           0.000102862236555,
              1,   -3.917165192517,    5.754909352487,   -3.758241150432,
                   0.9204979709902},
    // 3: 40Hz
    {  0.0001068876168914,-0.0003776890948126,0.0005446874763283,-0.0003776890948126,
          0.0001068876168914,
            1,     -3.8893064155,    5.674005151862,   -3.679896540922,
                 0.8952008890803},
    // 4: 50Hz
    {  0.0001124883762028,-0.0003713038989948,0.0005251459150399,-0.0003713038989948,
          0.0001124883762028,
            1,   -3.861233372095,    5.593248173458,   -3.602415197646,
                 0.8704079111519},
    // 5: 60Hz
    {  0.0001197336574328,-0.0003640298656109,0.0005041830349917,-0.0003640298656109,
          0.0001197336574328,
            1,   -3.832889447413,    5.512491568755,   -3.525653162673,
                 0.8460666319496},
    // 6: 100Hz
    {    0.00016772222904,-0.0003160025650004,0.0004202992163563,-0.0003160025650004,
            0.00016772222904,
                1,   -3.715588329731,     5.18652198639,   -3.223093823181,
                     0.7522839050674}, 
#if 0
    // 7: 140Hz
    {    0.03491867202533,    0.109221934686,     0.15231860983,    0.109221934686,
            0.03491867202533,
                1,   -1.283404870008,    1.021296455388,  -0.3523374727386,
                    0.05504571061107},
    // 8: 200Hz
    {    0.08893723310995,   0.3209252123518,   0.4657669085511,   0.3209252123518,
            0.08893723310995,
                1,  -0.2139521883892,   0.5177818092894, -0.03877694557932,
                     0.0204391241538},
#endif
    // elliptic
    // 7: 10Hz
    {  0.0001365293649764,-0.0003551533699655,0.0004773999437498,-0.0003551533699655,
          0.0001365293649764,
            1,    -3.81612017141,    5.468667637301,   -3.487687994098,
                 0.8351807202926},
    // 8: 15Hz 
    {  0.0001911802771726,-0.0003000902507431,0.0004122248786295,-0.0003000902507431,
          0.0001911802771726,
            1,   -3.721617458581,    5.210625006096,    -3.25211605352,
                 0.7633031053408},
    // 9: 20Hz
    {  0.0002759610582909,-0.0001858490563426,0.0004075153360141,-0.0001858490563426,
          0.0002759610582909,
            1,   -3.625620035462,    4.958311097943,   -3.029764714886,
                 0.6976619794845},
    // 10: 25Hz
    {  0.0003980885129365,2.576616211313e-05,0.0005251917953061,2.576616211313e-05,
          0.0003980885129365,
            1,    -3.52828560094,     4.71214368799,   -2.820210784204,
                 0.6377269711995},
    // 11: 30Hz
    {  0.0005663497151324,0.0003784008300013,0.0008349291084672,0.0003784008300013,
          0.0005663497151324,
            1,   -3.429768711272,    4.472488606159,   -2.623003094254,
                 0.5830103539969},
    // 12: 35Hz
    {  0.0007906376344732,0.0009193435980036, 0.001411473231841,0.0009193435980036,
          0.0007906376344732,
            1,   -3.330220028609,    4.239661627194,   -2.437669304545,
                 0.5330639730933},
#if ELLIPTIC_80DB 
    // 13: 10Hz 80db
    {0.0003980052194648,2.560038094574e-05,  0.00052507500309,2.560038094574e-05,
    0.0003980052194648,
    1,   -3.528358568107,    4.712330426217,   -2.820372195743,
    0.6377739974605},
    // 14: 6Hz 80db
    {0.0001911564530998,-0.0003001208385711,0.0004122437690402,-0.0003001208385711,
    0.0001911564530998,
    1,   -3.721661619608,    5.210745694536,   -3.252226509473,
    0.7633369440482},
#else
    // 13: 40Hz
    {0.01825647670023,  0.04629083875403,  0.06204752664347,  0.04629083875403,
    0.01825647670023,
    1,   -2.003795946524,    1.913402001585,  -0.8899384841482,
    0.1716659149431},
    // 14: 50Hz
    {0.03167074869554,  0.09500734129155,   0.1312278441234,  0.09500734129155,
    0.03167074869554,
    1,   -1.500899021154,    1.356646489988,  -0.5839458813532,
    0.1131673951732},
#endif
};
#endif
const double ba[FILTER_TYPE][5*2] = {
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
    {0.001235431141961,-0.003233484708145, 0.004349236721367,-0.003233484708145,
    0.001235431141961,
    1,   -3.628903305608,    4.954076395023,   -3.014454340388,
    0.6896343805619},
    // 3: 40Hz
    {0.001504023202492,-0.002833704229474, 0.003768968353254,-0.002833704229474,
    0.001504023202492,
    1,   -3.498597652843,    4.617828546747,     -2.7230591526,
    0.604937864995},
    // 4: 50Hz
    {0.001893594048567,-0.002220262954039, 0.003389066536478,-0.002220262954039,
    0.001893594048567,
    1,   -3.362256889209,    4.282608240118,   -2.444765517273,
    0.527149895089},
    // 5: 60Hz
    {0.002440006636488,-0.001243697363317, 0.003428681549348,-0.001243697363317,
    0.002440006636488,
    1,   -3.217868090935,    3.945654810763,   -2.177266033495,
    0.4553006137634},
    // 6: 100Hz
    {0.007904657035683,   0.0130194351176,  0.01766923337388,   0.0130194351176,
    0.007904657035683,
    1,    -2.50166997543,     2.53332880823,   -1.188409920757,
    0.2162685056381},
#if 0
    // 7: 140Hz
    {    0.03491867202533,    0.109221934686,     0.15231860983,    0.109221934686,
            0.03491867202533,
                1,   -1.283404870008,    1.021296455388,  -0.3523374727386,
                    0.05504571061107},
    // 8: 200Hz
    {    0.08893723310995,   0.3209252123518,   0.4657669085511,   0.3209252123518,
            0.08893723310995,
                1,  -0.2139521883892,   0.5177818092894, -0.03877694557932,
                     0.0204391241538},
#endif
    // elliptic
    // 7: 10Hz
    {0.001622516136187,-0.002828375511362, 0.003810675479761,-0.002828375511362,
    0.001622516136187,
    1,   -3.530382593166,    4.717748289788,   -2.825146415162,
    0.6391810755881},
    // 8: 15Hz 
    {0.00258873589755,-0.001233000776317, 0.003639805616231,-0.001233000776317,
    0.00258873589755,
    1,   -3.283002014681,    4.133347122059,   -2.355458472614,
    0.5114709985556},
    // 9: 20Hz
    {0.004107551662318, 0.002010215185723, 0.005793408195504, 0.002010215185723,
    0.004107551662318,
    1,   -3.030285807338,    3.593357746771,   -1.954731115222,
    0.4097061641801},
    // 10: 25Hz
    {0.006307797121228, 0.007695269382593,  0.01159372061391, 0.007695269382593,
    0.006307797121228,
    1,   -2.774294863632,     3.10056644764,   -1.615273296793,
    0.3286412048251},
    // 11: 30Hz
    {0.009321247485553,  0.01656082346783,  0.02224586730533,  0.01656082346783,
    0.009321247485553,
    1,    -2.51686389898,    2.656330329843,   -1.329451603217,
    0.2640692636513},
    // 12: 35Hz
    {0.01326931768189,  0.02924921742757,  0.03878281187879,  0.02924921742757,
    0.01326931768189,
    1,   -2.259583229684,    2.260845108374,   -1.089950945088,
    0.2126328889613},
#if ELLIPTIC_80DB 
    // 13: 10Hz 80db
    {0.0003980052194648,2.560038094574e-05,  0.00052507500309,2.560038094574e-05,
    0.0003980052194648,
    1,   -3.528358568107,    4.712330426217,   -2.820372195743,
    0.6377739974605},
    // 14: 6Hz 80db
    {0.0001911564530998,-0.0003001208385711,0.0004122437690402,-0.0003001208385711,
    0.0001911564530998,
    1,   -3.721661619608,    5.210745694536,   -3.252226509473,
    0.7633369440482},
#else
    // 13: 40Hz
    {0.01825647670023,  0.04629083875403,  0.06204752664347,  0.04629083875403,
    0.01825647670023,
    1,   -2.003795946524,    1.913402001585,  -0.8899384841482,
    0.1716659149431},
    // 14: 50Hz
    {0.03167074869554,  0.09500734129155,   0.1312278441234,  0.09500734129155,
    0.03167074869554,
    1,   -1.500899021154,    1.356646489988,  -0.5839458813532,
    0.1131673951732},
#endif
};
#endif

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

#define DUMP 0
#if DUMP
#include <stdio.h>
#include <stdlib.h>

#define DUMP_LEN 0x10000
static dump_type dump[DUMP_LEN];
static uint32_t dump_cnt = 0;
#endif

#define DUMP_HALF_SEC 0
#if DUMP_HALF_SEC 
#include <stdio.h>
#include <stdlib.h>


#define DUMP_HALF_SEC_LEN 4096 // 0.5s*8*1024
static dump_half_sec_type dump_half_sec[DUMP_HALF_SEC_LEN];
static uint32_t dump_half_sec_cnt = 0;
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
#define MPUREG_FIFO_COUNTH                          0x72
#define MPUREG_FIFO_COUNTL                          0x73
#define MPUREG_FIFO_R_W                             0x74
#define MPUREG_CONFIG_FIFO_MODE_STOP         0x40
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
#define MPUREG_FIFO_EN                          0x23
#       define BIT_TEMP_FIFO_EN                     0x80
#       define BIT_XG_FIFO_EN                       0x40
#       define BIT_YG_FIFO_EN                       0x20
#       define BIT_ZG_FIFO_EN                       0x10
#       define BIT_ACCEL_FIFO_EN                    0x08
#       define BIT_SLV2_FIFO_EN                     0x04
#       define BIT_SLV1_FIFO_EN                     0x02
#       define BIT_SLV0_FIFI_EN0                    0x01
#define MPUREG_I2C_MST_CTRL                     0x24
#       define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_AUTO                  0x01            //  auto mode
#       define BIT_PWR_MGMT_1_CLK_XGYRO             0x01 // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO             0x02 // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO             0x03 // PLL with Z axis gyroscope reference
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

// ICM20608 specific registers
#define ICMREG_ACCEL_CONFIG2          0x1D
#define ICM_ACC_DLPF_CFG_1046HZ_NOLPF 0x00
#define ICM_ACC_DLPF_CFG_218HZ        0x01
#define ICM_ACC_DLPF_CFG_99HZ         0x02
#define ICM_ACC_DLPF_CFG_44HZ         0x03
#define ICM_ACC_DLPF_CFG_21HZ         0x04
#define ICM_ACC_DLPF_CFG_10HZ         0x05
#define ICM_ACC_DLPF_CFG_5HZ          0x06
#define ICM_ACC_DLPF_CFG_420HZ        0x07
#define ICM_ACC_FCHOICE_B             0x08

/* this is an undocumented register which
   if set incorrectly results in getting a 2.7m/s/s offset
   on the Y axis of the accelerometer
*/
#define MPUREG_ICM_UNDOC1       0x11
#define MPUREG_ICM_UNDOC1_VALUE 0xc9


// AB ZhaoYJ@2016-11-01 for debugging LPF
// according to GYRO (also for ACCEL)
// #define GYRO_SCALE_250DPS
// #define ACCEL_SCALE_4G
// #define EN_LPF
#ifdef EN_LPF
// #define BITS_DLPF_CFG_HZ BITS_DLPF_CFG_20HZ
// #define BITS_DLPF_CFG_HZ BITS_DLPF_CFG_188HZ
#define BITS_DLPF_CFG_HZ BITS_DLPF_CFG_98HZ
// #define BITS_DLPF_CFG_HZ BITS_DLPF_CFG_42HZ                              
// #define BITS_DLPF_CFG_HZ BITS_DLPF_CFG_98HZ                              
#endif

#ifdef ACCEL_SCALE_4G
#define ICM20689_ACCEL_SCALE_1G    (GRAVITY_MSS / 8192.0f) // 4g
#else
// ICM20689 accelerometer scaling for 16g range: 2^15LSB/16g = 2048LSB/g, 1g = 9.8/2048 
#define ICM20689_ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f) // 16g
#endif

#if IMU_FAST_SAMPLE 
    // Last status from register user control
    static uint8_t _last_stat_user_ctrl;    
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
#if IMU_FAST_SAMPLE 
    _bypass_acc_uf(false),
    _bypass_gyro_uf(false),
#endif
    _accl_med_len(0),
    _gyro_med_len(0),
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

#if IMU_SAMPLE_RATE == 8
    hal.util->prt("[OK] ICM20689(8KHz) detected done");
#elif IMU_SAMPLE_RATE == 1
    hal.util->prt("[OK] ICM20689(1KHz) detected done");
#endif

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

#if IMU_FAST_SAMPLE 
        _last_stat_user_ctrl = _register_read(spi, MPUREG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
            _last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            _register_write(spi, MPUREG_USER_CTRL, _last_stat_user_ctrl);
            hal.scheduler->delay(10);
        }

        /* reset device */
        _register_write(spi, MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        /* bus-dependent initialization */
            /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
             * done just after the device is reset) */
        _last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
        _register_write(spi, MPUREG_USER_CTRL, _last_stat_user_ctrl);


        // Wake up device and select GyroZ clock. Note that the
        // ICM20689 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(spi, MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(spi, MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }

        hal.scheduler->delay(10);
        uint8_t status = _register_read(spi, MPUREG_INT_STATUS);
        if ((status & BIT_RAW_RDY_INT) != 0) {
            break;
        }
#else

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
#endif

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
    // if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) 
        {
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
#ifdef SMT_CAPTURE_IMU_RAW
    Vector3f gyro_raw = _shared_data_raw[idx]._gyro_raw;
    Vector3f accel_raw = _shared_data_raw[idx]._accel_raw;
#endif

    _have_sample_available = false;

    // _sem->give();

    // accel: g
    // gyro: radius/s
    accel *= ICM20689_ACCEL_SCALE_1G;
    gyro *= GYRO_SCALE;


    accel.rotate(_default_rotation);
    gyro.rotate(_default_rotation);

    _publish_gyro(_gyro_instance, gyro);
    _publish_accel(_accel_instance, accel);
#ifdef SMT_CAPTURE_IMU_RAW
    _publish_gyro_raw(_gyro_instance, gyro_raw);
    _publish_accel_raw(_accel_instance, accel_raw);
#endif

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
// AB ZhaoYJ@2016-12-20 for fix polling interval
#define FIX_PERIOD_POLL 0
#define PERIOD_POLL 2500 // 2500 us 

// average 1.2ms 
void AP_InertialSensor_ICM20689::_poll_data(void)
{
#if FIX_PERIOD_POLL
    static uint64_t last_poll = hal.scheduler->micros64();
    static uint8_t poll_control = 0; 
    // if ((hal.scheduler->micros64() - last_poll) < PERIOD_POLL) {
    if (poll_control++&0x1) {
        return;
    }
    // update poll time 
    // last_poll = hal.scheduler->micros64();

#if 0
    static uint16_t cnt2 = 0;
    if((0 == (cnt2%3000)) || (1 == (cnt2%3000)))
    {
        hal.util->prt("[ %llu us] ICM20689 timer %d", hal.scheduler->micros64(), cnt2);
    }
    cnt2++;
#endif
#endif
    if (!_spi_sem->take_nonblocking()) {
        /*
          the semaphore being busy is an expected condition when the
          mainline code is calling wait_for_sample() which will
          grab the semaphore. We return now and rely on the mainline
          code grabbing the latest sample.
        */
        return;
    }
#if IMU_FAST_SAMPLE 
    _read_fifo();
#else
    _read_data_transaction();
#endif
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
    static uint16_t cnt1 = 0;
    if((0 == (cnt1%3000)) || (1 == (cnt1%3000)))
    {
        hal.util->prt("[%d us] ICM20689 timer %d", hal.scheduler->micros(), cnt1);
    }
    cnt1++;
#endif

    _spi->transaction((const uint8_t *)&tx, (uint8_t *)&rx, sizeof(rx));


    Vector3f imu_acc;
    imu_acc.x = int16_val(rx.v, 1);
    imu_acc.y = int16_val(rx.v, 0);
    imu_acc.z = -int16_val(rx.v, 2);

    Vector3f imu_gyro;
    imu_gyro.x = int16_val(rx.v, 5);
    imu_gyro.y = int16_val(rx.v, 4);
    imu_gyro.z = -int16_val(rx.v, 6);

#if USER_FILTER

    uint8_t acc_user_ft = _imu.get_accl_user_filter();
    uint8_t gyro_user_ft = _imu.get_gyro_user_filter();

    Vector3f _accel_filtered;
    Vector3f _gyro_filtered;

#define CHK_FT_TAP 0
#if CHK_FT_TAP 
    static uint32_t acc_gyro_cnt = 0;
#endif

    if(acc_user_ft < 0xF) // ChebyII
    {
        _accel_filtered = _accel_user_filter(imu_acc, acc_user_ft);
    }
    else if(acc_user_ft == 0xF) // median
    {
        _accel_filtered = _accel_median_filter(imu_acc);
    }
    else if(acc_user_ft & 0x10) // median + ChebyII(Hz)
    {
        _accel_filtered = _accel_median_filter(imu_acc);
        _accel_filtered = _accel_user_filter(_accel_filtered, acc_user_ft & 0xF);
    }
    else if(acc_user_ft & 0x20) // ChebyII(Hz) + median 
    {
#if CHK_FT_TAP 
        if((0 == (acc_gyro_cnt%4000)) || (1 == (acc_gyro_cnt%4000)))
        {
            hal.util->prt("acc ft: %d, med_tap: %d", acc_user_ft, _imu.get_med_tap_acc());
        }
#endif
        _accel_filtered = _accel_user_filter(imu_acc, acc_user_ft & 0xF);
        _accel_filtered = _accel_median_filter(_accel_filtered);
    }

    if(gyro_user_ft < 0xF) // ChebyII
    {
        _gyro_filtered = _gyro_user_filter(imu_gyro, gyro_user_ft);
    }
    else if(gyro_user_ft == 0xF) // median
    {
        _gyro_filtered = _gyro_median_filter(imu_gyro);
    }
    else if(gyro_user_ft & 0x10) // median + ChebyII(Hz)
    {
        _gyro_filtered = _gyro_median_filter(imu_gyro);
        _gyro_filtered = _gyro_user_filter(_gyro_filtered, gyro_user_ft & 0xF);
    }
    else if(gyro_user_ft & 0x20) // ChebyII(20Hz) + median 
    {
#if CHK_FT_TAP 
        if((0 == (acc_gyro_cnt%4000)) || (1 == (acc_gyro_cnt%4000)))
        {
            hal.util->prt("gyro ft: %d, med_tap: %d", gyro_user_ft, _imu.get_med_tap_gyro());
        }
#endif
        _gyro_filtered = _gyro_user_filter(imu_gyro, gyro_user_ft & 0xF);
        _gyro_filtered = _gyro_median_filter(_gyro_filtered);
    }
#if CHK_FT_TAP 
    acc_gyro_cnt++;
#endif

#else
    Vector3f _accel_filtered = _accel_filter.apply(imu_acc);

    Vector3f _gyro_filtered = _gyro_filter.apply(imu_gyro);
#endif

    // if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) 
        {
        // update the shared buffer
        uint8_t idx = _shared_data_idx ^ 1;
        _shared_data[idx]._accel_filtered = _accel_filtered;
        _shared_data[idx]._gyro_filtered = _gyro_filtered;

#ifdef SMT_CAPTURE_IMU_RAW
        _shared_data_raw[idx]._accel_raw = imu_acc;
        _shared_data_raw[idx]._gyro_raw = imu_gyro; 
#endif
        _shared_data_idx = idx;


        _have_sample_available = true;
        // _sem->give();
#if DUMP
        if(start_cali)
        {
            dump_data(_gyro_filtered.z);
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


    // used no filter of 256Hz on the sensor, then filter using
    // the 2-pole software filter
#ifdef  EN_LPF

#define ACCEL_LPF_EN 0x8
#define ACCEL_AVERAGE_SAMPLE 0x30 // 32 samples
    _register_write(MPUREG_CONFIG, BITS_DLPF_CFG_HZ);
    hal.scheduler->delay(1);
    _register_write(MPUREG_ACCEL_CONFIG2, (ACCEL_LPF_EN | BITS_DLPF_CFG_HZ));
    // _register_write(MPUREG_ACCEL_CONFIG2, (ACCEL_AVERAGE_SAMPLE | ACCEL_LPF_EN | BITS_DLPF_CFG_HZ));
#else
    // 1KHz
#if IMU_SAMPLE_RATE == 1
    _register_write(MPUREG_CONFIG, BITS_DLPF_CFG_188HZ);
#endif

    hal.scheduler->delay(1);
    _register_write(MPUREG_ACCEL_CONFIG2, 0x0);
#endif
    hal.scheduler->delay(1);

    // set sample rate to 1kHz, and use the 2 pole filter to give the
    // desired rate
    _register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_1000HZ);
    hal.scheduler->delay(1);
#ifdef GYRO_SCALE_250DPS
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_250DPS);  // Gyro scale 250º/s
#else
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s
#endif
    hal.scheduler->delay(1);

#ifdef ACCEL_SCALE_4G
    // RM-MPU-9250A-00.pdf, pg. 15, select accel full scale 4g
    _register_write(MPUREG_ACCEL_CONFIG,1<<3);
#else
    // RM-MPU-9250A-00.pdf, pg. 15, select accel full scale 16g
    _register_write(MPUREG_ACCEL_CONFIG,3<<3);
#endif
    hal.scheduler->delay(1);

    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, _register_read(MPUREG_INT_PIN_CFG) | BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);
    hal.scheduler->delay(1);

    // only used for wake-up in accelerometer only low power mode
    _register_write(MPUREG_PWR_MGMT_2, 0x00);
    hal.scheduler->delay(1);

#if IMU_FAST_SAMPLE 

    // always use FIFO
    _fifo_reset();

    // setup ODR and on-sensor filtering
    // TODO will be redundant with above configs, clean up it
    _set_filter_register();

    // ref from 3.4, need validation
    // this avoids a sensor bug, see description above
	_register_write(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE);
    
#endif

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

#if USER_FILTER 
#define TEST_FILTER 0

Vector3f AP_InertialSensor_ICM20689::_accel_user_filter(Vector3f _accl_in, uint8_t _uf)
{
    //  for ChebyII
#define FILTER_MAX_TAP 8
    static Vector3d filter_state[FILTER_MAX_TAP]; 
    static Vector3d filter_out[FILTER_MAX_TAP]; 
    static uint8_t curr_idx = 0;
    static bool first = true;
    Vector3f ret;
    // Chebyshev II
    const double *b;
    const double *a;


    {
        if(_uf >= FILTER_TYPE) 
        {
            hal.util->prt("[Err] acc filter type wrong: %d", _uf);
            return ret;
        }

        b = &ba[0][0] + (N_ORDER+1)*2*_uf;
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
            filter_state[curr_idx].x = _accl_in.x;
            filter_state[curr_idx].y = _accl_in.y;
            filter_state[curr_idx].z = _accl_in.z;
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
        else
        {

            filter_state[curr_idx].x = _accl_in.x;
            filter_state[curr_idx].y = _accl_in.y;
            filter_state[curr_idx].z = _accl_in.z;
            filter_out[curr_idx].x = _accl_in.x;
            filter_out[curr_idx].y = _accl_in.y;
            filter_out[curr_idx].z = _accl_in.z;

            curr_idx++;
            if(curr_idx == FILTER_MAX_TAP)
            {
                first = false;
                curr_idx = 0;
            }
            ret = _accl_in;
        }
    }

    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}


Vector3f AP_InertialSensor_ICM20689::_gyro_user_filter(Vector3f _gyro_in, uint8_t _uf)
{
    //  for ChebyII
#define FILTER_MAX_TAP 8
    static Vector3d filter_state[FILTER_MAX_TAP]; 
    static Vector3d filter_out[FILTER_MAX_TAP]; 
    static uint8_t curr_idx = 0;
    static bool first = true;
    Vector3f ret;
    // Chebyshev II
    const double *b;
    const double *a;


    {
        if(_uf >= FILTER_TYPE) 
        {
            hal.util->prt("[Err] gyro filter type wrong: %d", _uf);
            return ret;
        }

        b = &ba[0][0] + (N_ORDER+1)*2*_uf;
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
            filter_state[curr_idx].x = _gyro_in.x;
            filter_state[curr_idx].y = _gyro_in.y;
            filter_state[curr_idx].z = _gyro_in.z;
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
        else
        {
            filter_state[curr_idx].x = _gyro_in.x;
            filter_state[curr_idx].y = _gyro_in.y;
            filter_state[curr_idx].z = _gyro_in.z;
            filter_out[curr_idx].x = _gyro_in.x;
            filter_out[curr_idx].y = _gyro_in.y;
            filter_out[curr_idx].z = _gyro_in.z;
            curr_idx++;
            if(curr_idx == FILTER_MAX_TAP)
            {
                first = false;
                curr_idx = 0;
            }
            ret = _gyro_in;
        }
    }

    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}


#endif

#if IMU_FAST_SAMPLE 
bool AP_InertialSensor_ICM20689::_accumulate_fast_sampling(uint8_t *samples, uint8_t n_samples)
{
    int32_t tsum = 0;
    bool ret = true;
    
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;

        // use temperatue to detect FIFO corruption
        static uint32_t fifo_corr_cnt = 0;
        int16_t t2 = int16_val(data, 3);
        if (!_check_raw_temp(t2)) {
            fifo_corr_cnt++; 
            // hal.util->prt("temp reset(%d) %d -> %d",fifo_corr_cnt, _raw_temp, t2);
            _fifo_reset();
            ret = false;
            break;
        }
        tsum += t2;

#if DUMP_HALF_SEC
        dump_half_sec_type tmp_data;
        tmp_data.temp = t2;
#endif

        // 8KHz just accumulate, no filter
#if IMU_SAMPLE_RATE == 8
        if ((_accum.count & 1) == 0) {
            // accel data is at 4kHz
            Vector3f a(int16_val(data, 1),
                       int16_val(data, 0),
                       -int16_val(data, 2));

#if DUMP_HALF_SEC
            tmp_data.acc(int16_val(data, 1),
                       int16_val(data, 0),
                       -int16_val(data, 2));
#endif
            _accum.accel += a;
        }
#elif IMU_SAMPLE_RATE == 1
        // accel data is at 1kHz
        Vector3f a(int16_val(data, 1),
                   int16_val(data, 0),
                   -int16_val(data, 2));
        if(!_bypass_acc_uf)
        {

            if(_accel_uf != nullptr)
            {

                _accum.accel += _accel_uf->apply3d(a);
            }
            else
            {
                hal.util->prt("[Err] accel UF is NULL");
                _accum.accel += a;
            }
        }
        else // bypass uf
        {
            _accum.accel += a;
        }
#endif

        Vector3f g(int16_val(data, 5),
                   int16_val(data, 4),
                   -int16_val(data, 6));
#if DUMP_HALF_SEC
            tmp_data.gyro(int16_val(data, 5),
                   int16_val(data, 4),
                   -int16_val(data, 6));

            dump_data_half_sec(tmp_data);
#endif

#if IMU_SAMPLE_RATE == 8
        _accum.gyro += g;
#elif IMU_SAMPLE_RATE == 1
        if(!_bypass_gyro_uf)
        {
            if(_gyro_uf != nullptr)
            {
                _accum.gyro += _gyro_uf->apply3d(g);
            }
            else
            {
                hal.util->prt("[Err] gyro UF is NULL");
                _accum.gyro += g;
            }
        }
        else
        {
            _accum.gyro += g;
        }
#endif

        // tmp2 = tmp2;
        _accum.count++;

        // make sure it's 1KHz to EKF
        // 8KHz: average to 1KHz then filter
        // 1KHz: directly report to frontend
        if (_accum.count == MPU_FIFO_DOWNSAMPLE_COUNT) {

            // 8KHz, filter in 1KHz
#if IMU_FAST_SAMPLE
#if IMU_SAMPLE_RATE == 8
            _accum.accel = _accum.accel/MPU_FIFO_DOWNSAMPLE_COUNT_ACC;
            _accum.gyro = _accum.gyro/MPU_FIFO_DOWNSAMPLE_COUNT;
            _accum.accel = _accel_uf->apply3d(_accum.accel);
            _accum.gyro = _gyro_uf->apply3d(_accum.gyro);
#endif
            _accum.accel = _accel_median_filter(_accum.accel);
            _accum.gyro = _gyro_median_filter(_accum.gyro);
#endif

            // update the shared buffer
            uint8_t idx = _shared_data_idx ^ 1;
            _shared_data[idx]._accel_filtered = _accum.accel;
            _shared_data[idx]._gyro_filtered = _accum.gyro;

#ifdef SMT_CAPTURE_IMU_RAW
            _shared_data_raw[idx]._accel_raw = imu_acc;
            _shared_data_raw[idx]._gyro_raw = imu_gyro; 
#endif
            _shared_data_idx = idx;

            _have_sample_available = true;

            _accum.accel.zero();
            _accum.gyro.zero();
            _accum.count = 0;
        }
    }


    if (ret) {
        float temp = (static_cast<float>(tsum)/n_samples - 25.0f)/326.8f + 25.0f;
        _temp_filtered = _temp_filter.apply(temp);
        _publish_temperature(_gyro_instance, _temp_filtered);
    }
    
    return ret;
}
void AP_InertialSensor_ICM20689::_read_fifo()
{
    uint8_t n_samples;
    uint16_t bytes_read;
    uint8_t *rx = _fifo_buffer;
    bool need_reset = false;

    static bool first = true;
    if(first)
    {
        first = false;
        uint16_t filter_info = _imu.get_accl_user_filter_8KHz(); 
        uint16_t ft = (filter_info%10)%5; // convert to filter_type: 0 - chebyI, 1, chebyII, 2 - elliptic 
        uint16_t cutoff = (uint16_t)(filter_info - ft);

        // then, for accel, max cutoff is 94Hz~100Hz
        // TODO: this logic is messed up, so clean it when time available
        if((filter_info >= 188) && (filter_info < 200))
        {
            cutoff = 188;
            ft = filter_info - 188;
        }
        else if((filter_info < 300) && (filter_info >= 200))
        {
            cutoff = 200;
            ft = filter_info - 200;
        }
        else if(filter_info >= 300)
        {
            _bypass_acc_uf = true;
        }

        hal.util->prt("[Info] InvSense: accel filter_info %d, ft: %d, cutoff: %d", filter_info, (uint8_t)ft, cutoff);

        if(!_bypass_acc_uf)
        {
            _accel_uf = new UserFilterDouble_Size5(UserFilterDouble_Size5::sample_rate_1KHz, (uint8_t)ft, cutoff); 
        }

        filter_info = _imu.get_gyro_user_filter_8KHz(); 
        ft = (filter_info%10)%5; // convert to filter_type: 0 - chebyI, 1, chebyII, 2 - elliptic 
        cutoff = (uint16_t)(filter_info - ft);
        if((filter_info >= 188) && (filter_info < 200))
        {
            cutoff = 188;
            ft = filter_info - 188;
        }
        else if((filter_info < 300) && (filter_info >= 200))
        {
            cutoff = 200;
            ft = filter_info - 200;
        }
        else if(filter_info >= 300)
        {
            _bypass_gyro_uf = true;
        }

        if(!_bypass_gyro_uf)
        {
            _gyro_uf = new UserFilterDouble_Size5(UserFilterDouble_Size5::sample_rate_1KHz, (uint8_t)ft, cutoff);
        }

        hal.util->prt("[Info] InvSense: gyro filter_info %d, ft: %d, cutoff: %d", filter_info, (uint8_t)ft, cutoff);
    }

    if (!_block_read(MPUREG_FIFO_COUNTH, rx, 2)) {
        goto check_registers;
    }

    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / MPU_SAMPLE_SIZE;

#if DUMP
    dump_data(bytes_read);
#endif

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        // hal.util->prt("[Err-%d ms]: ICM20689 n_samples(%d) = 0", hal.scheduler->millis(), n_samples);
        goto check_registers;
    }

    /*
      testing has shown that if we have more than 32 samples in the
      FIFO then some of those samples will be corrupt. It always is
      the ones at the end of the FIFO, so clear those with a reset
      once we've read the first 24. Reading 24 gives us the normal
      number of samples for fast sampling at 400Hz
     */
    // TODO need test for 689
    if (n_samples > 128) {
        hal.util->prt("[Err-%d ms]: ICM20689 n_samples(%d) > 128", hal.scheduler->millis(), n_samples);
        need_reset = true;
        n_samples = 32;
    }
    
    while (n_samples > 0) {
#define MIN(a, b) ((a<=b)?a:b)
        uint8_t n = MIN(n_samples, MPU_FIFO_BUFFER_LEN);
        if (!_block_read(MPUREG_FIFO_R_W, rx, n * MPU_SAMPLE_SIZE)) {
            hal.util->prt("[Err-%d ms]: ICM20689 blk_read", hal.scheduler->millis());
            goto check_registers;
        }
        if (!_accumulate_fast_sampling(rx, n)) {
            // hal.util->prt("stop at %u of %u", n_samples, bytes_read/MPU_SAMPLE_SIZE);
            break;
        }
        n_samples -= n;
    }

    if (need_reset) {
        hal.util->prt("fifo reset n_samples %u", bytes_read/MPU_SAMPLE_SIZE);
        _fifo_reset();
    }
    
check_registers:
    return;
    
}

/*
  fetch temperature in order to detect FIFO sync errors
*/
bool AP_InertialSensor_ICM20689::_check_raw_temp(int16_t t2)
{
    if (abs(t2 - _raw_temp) < 400) {
        // cached copy OK
        return true;
    }
    uint8_t trx[2];
    if (_block_read(MPUREG_TEMP_OUT_H, trx, 2)) {
        _raw_temp = int16_val(trx, 0);
    }
    return (abs(t2 - _raw_temp) < 400);
}

bool AP_InertialSensor_ICM20689::_block_read(uint8_t reg, uint8_t *buf,
                                            uint32_t size)
{
    uint8_t tx = reg | 0x80; // Set most significant bit

    return _spi->transfer(&tx, 1, buf, size);
    // return true;
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_ICM20689::_set_filter_register(void)
{
    uint8_t config;

#if INVENSENSE_EXT_SYNC_ENABLE
    // add in EXT_SYNC bit if enabled
    config = (MPUREG_CONFIG_EXT_SYNC_AZ << MPUREG_CONFIG_EXT_SYNC_SHIFT);
#else
    config = 0;
#endif

    // 8KHz
#if IMU_SAMPLE_RATE == 8
    
    // this gives us 8kHz sampling on gyros and 4kHz on accels
    config |= BITS_DLPF_CFG_256HZ_NOLPF2;

    config |= MPUREG_CONFIG_FIFO_MODE_STOP;
    _register_write(MPUREG_CONFIG, config);

    // setup for 4kHz accels
    _register_write(ICMREG_ACCEL_CONFIG2, ICM_ACC_FCHOICE_B);
#endif
}

void AP_InertialSensor_ICM20689::_fifo_reset()
{
    uint8_t user_ctrl = _last_stat_user_ctrl;
    user_ctrl &= ~(BIT_USER_CTRL_FIFO_RESET | BIT_USER_CTRL_FIFO_EN);
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
    _register_write(MPUREG_FIFO_EN, 0);
    _register_write(MPUREG_USER_CTRL, user_ctrl);
    _register_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_RESET);
    _register_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_EN);
    _register_write(MPUREG_FIFO_EN, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN);
    hal.scheduler->delay(1);
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
    _last_stat_user_ctrl = user_ctrl | BIT_USER_CTRL_FIFO_EN;
}

#endif

void AP_InertialSensor_ICM20689::dump_data(dump_type data)
{
#if DUMP
        if((0 == (dump_cnt%(DUMP_LEN >> 3))) || (1 == (dump_cnt%(DUMP_LEN >> 3))))
        {
            hal.util->prt("[ %d us] ICM20689 dumpcnt %d (%s)", hal.scheduler->micros(), dump_cnt, start_cali?"cali":"no cali");
        }
        if(dump_cnt < DUMP_LEN)
        {
            // dump[dump_cnt++] = _accel_filtered.z;
            dump[dump_cnt++] = data;
        }
        else if(DUMP_LEN == dump_cnt)
        {
            FILE *fd = fopen("/var/APM/dump.txt", "w");
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
}

void AP_InertialSensor_ICM20689::dump_data_half_sec(dump_half_sec_type data)
{
#if DUMP_HALF_SEC
    static FILE *fd = fopen("/var/APM/dump", "a");
    static uint32_t half_sec_cnt = 0;
    static uint32_t start_ms = hal.scheduler->millis();

#if 0
        if((0 == (dump_half_sec_cnt%(DUMP_HALF_SEC_LEN >> 3))) || (1 == (dump_half_sec_cnt%(DUMP_HALF_SEC_LEN >> 3))))
        {
            if(half_sec_cnt !=0)
                hal.util->prt("[ %d us] ICM20689 dumpcnt %d (%s), acc:[%d-%d-%d], gyro:[%d-%d-%d]", hal.scheduler->micros(), dump_half_sec_cnt, start_cali?"cali":"no cali", data.acc.x, data.acc.y, data.acc.z, data.gyro.x, data.gyro.y, data.gyro.z);
        }
#endif
        if(dump_half_sec_cnt < DUMP_HALF_SEC_LEN)
        {
            // dump[dump_half_sec_cnt++] = _accel_filtered.z;
            dump_half_sec[dump_half_sec_cnt++] = data;
        }
        else if(DUMP_HALF_SEC_LEN == dump_half_sec_cnt)
        {
            if(fd)
            {
                // for(uint32_t ii = 0; ii < DUMP_HALF_SEC_LEN; ii++)
                {
                    // fprintf(fd, "%d,%d,%d,%d,%d,%d,%d\n", dump_half_sec[ii].acc.x,
                    //         dump_half_sec[ii].acc.y,
                    //         dump_half_sec[ii].acc.z,
                    //         dump_half_sec[ii].gyro.x,
                    //         dump_half_sec[ii].gyro.y,
                    //         dump_half_sec[ii].gyro.z,
                    //         dump_half_sec[ii].temp);
                    fwrite(dump_half_sec, sizeof(dump_half_sec_type), DUMP_HALF_SEC_LEN, fd);
                }
                dump_half_sec_cnt = 0;
                dump_half_sec[dump_half_sec_cnt++] = data;
                // fclose(fd);
                half_sec_cnt++; 
                if(half_sec_cnt%10 == 0)
                {
                    hal.util->prt("[OK-%d ms] dump %ds IMU", hal.scheduler
                        ->millis(), half_sec_cnt/2, half_sec_cnt);
                }
                //  hours
                if(half_sec_cnt == (60*60*2))
                // if(half_sec_cnt == 8)
                {
                    hal.util->prt("[OK] dump done [%d -> %d ms]", start_ms, hal.scheduler->millis());
                    fclose(fd);
                    exit(1);
                }
            }
            else
            {
                hal.util->prt("[Err] failed to open dump log");
            }
        }
#endif
}

static double median_filter(double *pimu_in, uint8_t median_len)
{
    int i,j;
    double ret;  
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

Vector3f AP_InertialSensor_ICM20689::_accel_median_filter(Vector3f _accl_in)
{
    // for median filter: circular buff 16
#define MED_TAP 64
    static Vector3d med_filter_in[MED_TAP];
    static uint8_t curr_idx = 0;
    static bool first = true;
    Vector3f ret;

    if(!first)
    {
        uint8_t med_len = _accl_med_len + 1; // _imu.get_med_tap_acc() + 1; // include current in
        if((med_len > 1) && (_accl_med_len != 64)) // 64: disable median
        {
            double med_in_x[MED_TAP]; 
            double med_in_y[MED_TAP];
            double med_in_z[MED_TAP];
            med_filter_in[curr_idx].x = _accl_in.x;
            med_filter_in[curr_idx].y = _accl_in.y;
            med_filter_in[curr_idx].z = _accl_in.z;
            for(uint8_t med_idx = 0; med_idx < med_len; med_idx++)
            {
                uint8_t dist = med_len - 1 - med_idx;
                med_in_x[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].x;
                med_in_y[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].y;
                med_in_z[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].z;
            }
            ret.x = median_filter(med_in_x, med_len);  
#if 0
            static uint32_t cnt_xx = 0;
            if((0 == (cnt_xx%1000)) || (1 == (cnt_xx%1000)))
            {
                hal.util->prt("ffffmedian x: <%f>, med_len: %d", ret.x, med_len);
                hal.util->prt("median x: before <%f>, current <%f>, average <%f>", med_in_x[med_len/2 - 1], med_in_x[med_len/2], (med_in_x[med_len/2 - 1]+med_in_x[med_len/2])/2);
            }
            cnt_xx++;
#endif

            ret.y = median_filter(med_in_y, med_len);  
            ret.z = median_filter(med_in_z, med_len);  
            curr_idx++;
            curr_idx &= MED_TAP - 1;
        }
        else
        {
            // hal.util->prt("[Err] acc mean filter param wrong: ");
            return _accl_in;
        }
    }
    else
    {
        _accl_med_len = _imu.get_med_tap_acc();
        med_filter_in[curr_idx].x = _accl_in.x;
        med_filter_in[curr_idx].y = _accl_in.y;
        med_filter_in[curr_idx].z = _accl_in.z;

        curr_idx++;
        if(curr_idx == MED_TAP)
        {
            first = false;
            curr_idx = 0;
        }
        ret = _accl_in;
    }


    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}

Vector3f AP_InertialSensor_ICM20689::_gyro_median_filter(Vector3f _gyro_in)
{
    // for median filter: circular buff 16
#define MED_TAP 64
    static Vector3d gyro_med_filter_in[MED_TAP];
    static uint8_t curr_idx = 0;
    static bool first = true;
    Vector3f ret;
    if(!first)
    {
        uint8_t med_len = _gyro_med_len + 1; // _imu.get_med_tap_gyro() + 1; // include current in
        if((med_len > 1) && (_gyro_med_len != 64)) // 64: disable median
        {
            double med_in_x[MED_TAP]; 
            double med_in_y[MED_TAP];
            double med_in_z[MED_TAP];
            gyro_med_filter_in[curr_idx].x = _gyro_in.x;
            gyro_med_filter_in[curr_idx].y = _gyro_in.y;
            gyro_med_filter_in[curr_idx].z = _gyro_in.z;
            for(uint8_t med_idx = 0; med_idx < med_len; med_idx++)
            {
                uint8_t dist = med_len - 1 - med_idx;
                med_in_x[med_idx] = gyro_med_filter_in[FORMER(curr_idx, dist, MED_TAP)].x;
                med_in_y[med_idx] = gyro_med_filter_in[FORMER(curr_idx, dist, MED_TAP)].y;
                med_in_z[med_idx] = gyro_med_filter_in[FORMER(curr_idx, dist, MED_TAP)].z;
            }
            ret.x = median_filter(med_in_x, med_len);  
            ret.y = median_filter(med_in_y, med_len);  
            ret.z = median_filter(med_in_z, med_len);  
            curr_idx++;
            curr_idx &= MED_TAP - 1;
        }
        else
        {
            // hal.util->prt("[Err] gyro mean filter param wrong: ");
            return _gyro_in;
        }
    }
    else
    {
        _gyro_med_len = _imu.get_med_tap_gyro();
        gyro_med_filter_in[curr_idx].x = _gyro_in.x;
        gyro_med_filter_in[curr_idx].y = _gyro_in.y;
        gyro_med_filter_in[curr_idx].z = _gyro_in.z;

        curr_idx++;
        if(curr_idx == MED_TAP)
        {
            first = false;
            curr_idx = 0;
        }
        ret = _gyro_in;
    }


    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}

#endif // CONFIG_HAL_BOARD
