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

/// @file	User.cpp
/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
//
#include <inttypes.h>
#include <AP_Math/AP_Math.h>
#include "Filter.h"
#include "UserFilter.h"

extern const AP_HAL::HAL& hal;

#define N_ORDER 4
#define TEST_FILTER  0

// 8KHz
#if USER_SAMPLE_RATE == 8000
double ba_chebyI[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
double ba_chebyII[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
    // fc = 5Hz
    {9.991707904666e-05,-0.0003996559896646,0.0005994778214259,-0.0003996559896646,
    9.991707904666e-05,
    1,   -3.998279208542,    5.994839106037,     -3.9948405857,
    0.9982806882053},
    // fc = 10Hz
    {  9.984038427655e-05,-0.0003993122719033,0.0005989437782923,-0.0003993122719033,
          9.984038427655e-05,
            1,   -3.996558404271,    5.989681133888,   -3.989687048987,
                  0.996564319373},
    // fc = 15Hz
    {  9.976989980307e-05,-0.0003989688421457,0.0005983979000566,-0.0003989688421457,
          9.976989980307e-05,
            1,   -3.994837574377,    5.984526044339,   -3.984539345386,
                 0.9948508754394},
    // fc = 20Hz
    {  9.970561024007e-05,-0.0003986256946983,0.0005978402174577,-0.0003986256946983,
          9.970561024007e-05,
            1,   -3.993116706048,    5.979373798258,     -3.9793974306,
                 0.9931403384386},
    // fc = 25Hz
    {  9.964750069948e-05,-0.000398282822749,0.0005972707625114,-0.000398282822749,
          9.964750069948e-05,
            1,   -3.991395786471,      5.9742243566,   -3.974261260511,
                 0.9914326905011},
    // fc = 30Hz
    {  9.959555678904e-05,-0.000397940218371,0.0005966895685065,-0.000397940218371,
          9.959555678904e-05,
            1,   -3.989674802833,    5.969077680403,   -3.969130791178,
                 0.9897279138534},
    // fc = 35Hz
    {  9.954976461007e-05,-0.0003975978725271,0.0005960966700008,-0.0003975978725271,
          9.954976461007e-05,
            1,   -3.987953742317,    5.963933730786,   -3.964005978832,
                 0.9880259908172},
    // fc = 40Hz
    {  9.951011075526e-05,-0.0003972557750732,0.0005954921028174,-0.0003972557750732,
          9.951011075526e-05,
            1,   -3.986232592105,     5.95879246895,   -3.958886779879,
                 0.9863269038083},
    // fc= 45Hz
    {  9.947658230662e-05,-0.0003969139147621,0.0005948759040415,-0.0003969139147621,
          9.947658230662e-05,
            1,   -3.984511339377,    5.953653856178,   -3.953773150898,
                 0.9846306353365},
    // fc = 50Hz
    {  9.944916683338e-05,-0.0003965722792469,0.0005942481120172,-0.0003965722792469,
          9.944916683338e-05,
            1,   -3.982789971308,    5.948517853828,   -3.948665048637,
                 0.9829371680041},
    // fc = 55Hz
    {  9.942785239004e-05,-0.0003962308550842,0.0005936087663442,-0.0003962308550842,
          9.942785239004e-05,
            1,   -3.981068475071,    5.943384423338,   -3.943562430011,
                 0.9812464845057},
    // fc = 60Hz
    {   9.94126275144e-05,-0.0003958896277372,0.0005929579078759,-0.0003958896277372,
           9.94126275144e-05,
              1,   -3.979346837836,    5.938253526222,   -3.938465252106,
                   0.9795585676274},
    // fc = 188Hz
    {   0.000101070951471,-0.0003871514871685, 0.000572531893836,-0.0003871514871685,
           0.000101070951471,
              1,   -3.935184591309,    5.807646134889,   -3.809699078855,
                   0.9372379060986},
    // fc = 200Hz
    {  0.0001014270436174,-0.0003863221750837,0.0005702646900831,-0.0003863221750837,
          0.0001014270436174,
            1,   -3.931031461867,    5.795462886853,   -3.797783938345,
                 0.9333529877871}

   };
double ba_elliptic[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
    // fc = 5Hz
    {  9.961993062354e-05,-0.0003977528147556,0.0005962664346872,-0.0003977528147556,
          9.961993062354e-05,
            1,   -3.988725875902,    5.966256500147,    -3.96633505078,
                 0.9888044272025},
    // fc = 10Hz
    {  9.960205659531e-05,-0.0003955129525668,0.0005918323948986,-0.0003955129525668,
          9.960205659531e-05,
            1,   -3.977421160214,    5.932579122164,   -3.932892193072,
                 0.9777342417347},
    // fc = 15Hz
    {  9.994175727317e-05,-0.0003932744461525,0.0005867187542452,-0.0003932744461525,
          9.994175727317e-05,
            1,   -3.966086169311,    5.898969032667,   -3.899670894654,
                 0.9667880847282},
    // fc = 20Hz
    {  0.0001006354410406,-0.0003910275940524, 0.000580952056621,-0.0003910275940524,
          0.0001006354410406,
            1,    -3.95472121963,    5.865427384948,   -3.866670608394,
                 0.9559646109946},
    // fc = 25Hz
    {  0.0001016805015353,-0.0003887590033845,0.0005745642568211,-0.0003887590033845,
          0.0001016805015353,
            1,   -3.943326627632,    5.831955318961,   -3.833890772572,
                 0.9452624889036},
    // fc = 30Hz
    {  0.0001030752907124,-0.0003864516959751,0.0005675925621708,-0.0003864516959751,
          0.0001030752907124,
            1,   -3.931902709781,    5.798553961308,   -3.801330811197,
                 0.9346804002613},
    // fc = 35Hz
    {  0.0001048190923968,-0.0003840852125327,0.0005600792770886,-0.0003840852125327,
          0.0001048190923968,
            1,   -3.920449782506,    5.765224425229,   -3.768990134329,
                 0.9242170401906},
    // fc = 40Hz
    {  0.0001069120963178,-0.000381635714892,0.0005520716505446,-0.000381635714892,
          0.0001069120963178,
            1,    -3.90896816218,    5.731967810594,   -3.736868138387,
                 0.9138711170117},
    // fc = 45Hz
    {   0.0001093553726195,-0.0003790760863548,0.0005436217263738,-0.0003790760863548,
          0.0001093553726195,
            1,   -3.897458165084,      5.6987852039,    -3.70496420646,
                 0.9036413521235},
    // fc = 50Hz
    {  0.0001121508468404,-0.0003763760301532,0.0005347861964058,-0.0003763760301532,
          0.0001121508468404,
            1,   -3.885920107383,     5.66567767827,   -3.673277708608,
                 0.8935264798864},
    // fc = 55Hz
    {  0.0001153012753561,-0.0003735021660612,0.0005256262563717,-0.0003735021660612,
          0.0001153012753561,
            1,   -3.874354305093,    5.632646293453,   -3.641808002166,
                 0.8835252475059},
    // fc = 60hz
    {  0.0001188102212783,-0.0003704181251812,0.0005162074645494,-0.0003704181251812,
          0.0001188102212783,
            1,   -3.862761074054,    5.599692095827,   -3.610554432041,
                 0.8736364149166},
    // fc = 188Hz
    {  0.0003570312388466,-5.045434520191e-05,0.0004730126798606,-5.045434520191e-05,
          0.0003570312388466,
            1,   -3.557617052442,    4.785325775499,   -2.881759613257,
                 0.6551381428344},
    // fc = 200Hz
    {  0.0003980885129365,2.576616211313e-05,0.0005251917953061,2.576616211313e-05,
          0.0003980885129365,
            1,    -3.52828560094,     4.71214368799,   -2.820210784204,
                 0.6377269711995}
   };
#elif USER_SAMPLE_RATE == 1000
double ba_chebyI[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
double ba_chebyII[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
double ba_elliptic[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
#endif



template <class T,  uint8_t FILTER_SIZE>
void UserFilter<T,FILTER_SIZE>::update(T sample, uint32_t timestamp)
{

    _new_data = true;
}

#define FORMER(curr, n, array_size) ((curr >= n)?(curr - n):(curr + array_size - n)) 
template <class T,  uint8_t FILTER_SIZE>
T UserFilter<T,FILTER_SIZE>::apply(T sample)
{
    uint16_t curr_idx = _sample_idx;
    if(!_first)
    {
        // update state
        filter_state[_sample_idx++] = sample;
        // wrap index if necessary
        if( _sample_idx >= FILTER_SIZE )
            _sample_idx = 0;

        // filter  
        filter_out[curr_idx] = b[0]*filter_state[curr_idx] 
            + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_SIZE)] 
            - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_SIZE)] 
            + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_SIZE)] 
            - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_SIZE)] 
            + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_SIZE)] 
            - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_SIZE)]
            + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_SIZE)]
            - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_SIZE)];

        if (isnan(filter_out[curr_idx]) || isinf(filter_out[curr_idx])) {

            filter_out[curr_idx] = filter_state[curr_idx]; 
        }

    }
    else
    {

        filter_state[_sample_idx] = sample;
        filter_out[_sample_idx] = sample;

        _sample_idx++;
        if(_sample_idx == FILTER_SIZE)
        {
            _first = false;
            _sample_idx = 0;
        }
    }

    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return filter_out[curr_idx];
}

template <class T,  uint8_t FILTER_SIZE>
Vector3f UserFilter<T,FILTER_SIZE>::apply3d(Vector3f sample)
{
    uint16_t curr_idx = _sample_idx;
    Vector3f ret;
#if TEST_FILTER 
    static uint32_t incr = 0;
    if((0 == incr%10000) || (1 == incr%10000) || (2 == incr%10000))
    {
        hal.util->prt("filter b: %.19f, %.19f, %.19f, %.19f, %.19f", 
                b[0], b[1], b[2], b[3], b[4]);
        hal.util->prt("filter a: %.19f, %.19f, %.19f, %.19f, %.19f", 
                a[0], a[1], a[2], a[3], a[4]);
        hal.util->prt("ChebyII filter idx: curr_idx %d, %d, %d, %d, %d", 
                curr_idx,
                FORMER(curr_idx, 1, FILTER_SIZE), 
                FORMER(curr_idx, 2, FILTER_SIZE), 
                FORMER(curr_idx, 3, FILTER_SIZE), 
                FORMER(curr_idx, 4, FILTER_SIZE)); 
#if 0
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
    if(!_first)
    {
        // update state
        filter_state_3d[_sample_idx].x = sample.x;
        filter_state_3d[_sample_idx].y = sample.y;
        filter_state_3d[_sample_idx].z = sample.z;
        // wrap index if necessary
        _sample_idx++; 
        if( _sample_idx >= FILTER_SIZE )
            _sample_idx = 0;

        // filter x 
        filter_out_3d[curr_idx].x = b[0]*filter_state_3d[curr_idx].x 
            + b[1]*filter_state_3d[FORMER(curr_idx, 1, FILTER_SIZE)].x 
            - a[1]*filter_out_3d[FORMER(curr_idx, 1, FILTER_SIZE)].x 
            + b[2]*filter_state_3d[FORMER(curr_idx, 2, FILTER_SIZE)].x 
            - a[2]*filter_out_3d[FORMER(curr_idx, 2, FILTER_SIZE)].x 
            + b[3]*filter_state_3d[FORMER(curr_idx, 3, FILTER_SIZE)].x 
            - a[3]*filter_out_3d[FORMER(curr_idx, 3, FILTER_SIZE)].x
            + b[4]*filter_state_3d[FORMER(curr_idx, 4, FILTER_SIZE)].x
            - a[4]*filter_out_3d[FORMER(curr_idx, 4, FILTER_SIZE)].x;

        if (isnan(filter_out_3d[curr_idx].x) || isinf(filter_out_3d[curr_idx].x)) {

            filter_out_3d[curr_idx].x = filter_state_3d[curr_idx].x; 
        }

        // filter y 
        filter_out_3d[curr_idx].y = b[0]*filter_state_3d[curr_idx].y 
            + b[1]*filter_state_3d[FORMER(curr_idx, 1, FILTER_SIZE)].y 
            - a[1]*filter_out_3d[FORMER(curr_idx, 1, FILTER_SIZE)].y 
            + b[2]*filter_state_3d[FORMER(curr_idx, 2, FILTER_SIZE)].y 
            - a[2]*filter_out_3d[FORMER(curr_idx, 2, FILTER_SIZE)].y 
            + b[3]*filter_state_3d[FORMER(curr_idx, 3, FILTER_SIZE)].y 
            - a[3]*filter_out_3d[FORMER(curr_idx, 3, FILTER_SIZE)].y
            + b[4]*filter_state_3d[FORMER(curr_idx, 4, FILTER_SIZE)].y
            - a[4]*filter_out_3d[FORMER(curr_idx, 4, FILTER_SIZE)].y;

        if (isnan(filter_out_3d[curr_idx].y) || isinf(filter_out_3d[curr_idx].y)) {

            filter_out_3d[curr_idx].y = filter_state_3d[curr_idx].y; 
        }

        // filter z 
        filter_out_3d[curr_idx].z = b[0]*filter_state_3d[curr_idx].z 
            + b[1]*filter_state_3d[FORMER(curr_idx, 1, FILTER_SIZE)].z 
            - a[1]*filter_out_3d[FORMER(curr_idx, 1, FILTER_SIZE)].z 
            + b[2]*filter_state_3d[FORMER(curr_idx, 2, FILTER_SIZE)].z 
            - a[2]*filter_out_3d[FORMER(curr_idx, 2, FILTER_SIZE)].z 
            + b[3]*filter_state_3d[FORMER(curr_idx, 3, FILTER_SIZE)].z 
            - a[3]*filter_out_3d[FORMER(curr_idx, 3, FILTER_SIZE)].z
            + b[4]*filter_state_3d[FORMER(curr_idx, 4, FILTER_SIZE)].z
            - a[4]*filter_out_3d[FORMER(curr_idx, 4, FILTER_SIZE)].z;

        if (isnan(filter_out_3d[curr_idx].z) || isinf(filter_out_3d[curr_idx].z)) {

            filter_out_3d[curr_idx].z = filter_state_3d[curr_idx].z; 
        }

        ret.x = filter_out_3d[curr_idx].x;
        ret.y = filter_out_3d[curr_idx].y;
        ret.z = filter_out_3d[curr_idx].z;

    }
    else
    {


        filter_state_3d[curr_idx].x = sample.x;
        filter_state_3d[curr_idx].y = sample.y;
        filter_state_3d[curr_idx].z = sample.z;
        filter_out_3d[curr_idx].x = sample.x;
        filter_out_3d[curr_idx].y = sample.y;
        filter_out_3d[curr_idx].z = sample.z;

        _sample_idx++;
        if(_sample_idx == FILTER_SIZE)
        {
            _first = false;
            _sample_idx = 0;
        }
        ret = sample;
    }

    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}


template <class T,  uint8_t FILTER_SIZE>
void UserFilter<T,FILTER_SIZE>::init(filter_t ft, freq_t fr_t)
{
    hal.util->prt("UserFilter: type: %d, cutoff: %dHz", ft, (fr_t+1)*5);
    
    if(5 == FILTER_SIZE)
    {
        switch(ft)
        {
            case ft_chebyI:
                b = &ba_chebyI[0][0] + (N_ORDER+1)*2*fr_t;
                a = b + (N_ORDER+1);
                break;
            case ft_chebyII:
                b = &ba_chebyII[0][0] + (N_ORDER+1)*2*fr_t;
                a = b + (N_ORDER+1);
                break;
            case ft_elliptic:
                b = &ba_elliptic[0][0] + (N_ORDER+1)*2*fr_t;
                a = b + (N_ORDER+1);
                break;
            default:
                break;
        }
    }
}

// reset - clear all samples
template <class T, uint8_t FILTER_SIZE>
void UserFilter<T,FILTER_SIZE>::reset(void)
{
    // call parent's apply function to get the sample into the array
    // FilterWithBuffer<T,FILTER_SIZE>::reset();
}

// add new instances as needed here
template void UserFilter<float,3>::update(float sample, uint32_t timestamp);
template float UserFilter<float,3>::apply(float sample);
template void UserFilter<float,3>::reset(void);
template void UserFilter<float,4>::update(float sample, uint32_t timestamp);
template float UserFilter<float,4>::apply(float sample);
template void UserFilter<float,4>::reset(void);
template void UserFilter<float,5>::update(float sample, uint32_t timestamp);
template float UserFilter<float,5>::apply(float sample);
template void UserFilter<float,5>::reset(void);
template void UserFilter<float,6>::update(float sample, uint32_t timestamp);
template float UserFilter<float,6>::apply(float sample);
template void UserFilter<float,6>::reset(void);
template void UserFilter<float,7>::update(float sample, uint32_t timestamp);
template float UserFilter<float,7>::apply(float sample);
template void UserFilter<float,7>::reset(void);

template void UserFilter<double,3>::update(double sample, uint32_t timestamp);
template double UserFilter<double,3>::apply(double sample);
template void UserFilter<double,3>::reset(void);
template void UserFilter<double,4>::update(double sample, uint32_t timestamp);
template double UserFilter<double,4>::apply(double sample);
template void UserFilter<double,4>::reset(void);
template void UserFilter<double,5>::update(double sample, uint32_t timestamp);
template double UserFilter<double,5>::apply(double sample);
template Vector3f UserFilter<double,5>::apply3d(Vector3f sample);
template void UserFilter<double,5>::reset(void);
template void UserFilter<double,5>::init(filter_t ft, freq_t fr_t);
template void UserFilter<double,6>::update(double sample, uint32_t timestamp);
template double UserFilter<double,6>::apply(double sample);
template void UserFilter<double,6>::reset(void);
template void UserFilter<double,7>::update(double sample, uint32_t timestamp);
template double UserFilter<double,7>::apply(double sample);
template void UserFilter<double,7>::reset(void);

