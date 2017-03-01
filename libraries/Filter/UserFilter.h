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

//
/// @file	User.h
/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>


// 1st parameter <T> is the type of data being filtered.
// 2nd parameter <FILTER_SIZE> is the number of elements in the filter
template <class T, uint8_t FILTER_SIZE>
class UserFilter //  : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    enum sample_rate {
        sample_rate_400Hz = 0,
        sample_rate_1KHz,
        sample_rate_2KHz,
        sample_rate_4KHz,
        sample_rate_8KHz,
        sample_rate_num
    };

    enum filter_t{
        ft_chebyI = 0,
        ft_chebyII,
        ft_elliptic,
        ft_num
    };

    enum freq_t{
        freq_5Hz = 0,
        freq_10Hz,
        freq_15Hz,
        freq_20Hz,
        freq_25Hz,
        freq_30Hz,
        freq_35Hz,
        freq_40Hz,
        freq_45Hz,
        freq_50Hz,
        freq_55Hz,
        freq_60Hz,
        freq_188Hz,
        freq_200Hz,
        freq_num 
    };

    // constructor
    UserFilter() {}; // : FilterWithBuffer<T,FILTER_SIZE>(); 
    // TODO: fs will be needed for future
    UserFilter(sample_rate fs, uint8_t ft, uint16_t cutoff) {

        _first = true;
        reset();
        freq_t fr_t = (freq_t) (cutoff/(uint16_t)5 - 1);
        // check cutoff freq is valid
        if((0 != cutoff%5) || (cutoff < 5) || (cutoff >= 200))
        {
            // hal.util->prt("[Warn] UserFilter: invalid cutoff freq!");
            if(cutoff < 5)
            {
                fr_t = freq_5Hz;        
            }
            else if(cutoff == 188)
            {
                fr_t = freq_188Hz;
            }
            else if(cutoff >= 200)
            {
                fr_t = freq_200Hz;        
            }
        }
        init(FILTER_SIZE - 1, fs, (filter_t) ft, fr_t);
    }; // : FilterWithBuffer<T,FILTER_SIZE>(); 

    // update - Add a new raw value to the filter, but don't recalculate
    void update(T sample, uint32_t timestamp);

    T apply(T sample);

    Vector3f apply3d(Vector3f sample);


    // init coeffs
    void init(uint8_t tap, uint8_t fs, filter_t ft, freq_t fr_t);

    // reset - clear the filter
    virtual void        reset();

private:
    bool            _new_data;
    bool            _first;
    double          *b;
    double          *a;

    T               filter_state[FILTER_SIZE];
    T               filter_out[FILTER_SIZE];

    uint16_t        _sample_idx;

    Vector3d        filter_state_3d[FILTER_SIZE];
    Vector3d        filter_out_3d[FILTER_SIZE];

    // microsecond timestamps for samples. This is needed
    // to cope with non-uniform time spacing of the data
};

typedef UserFilter<float,3> UserFilterFloat_Size3;
typedef UserFilter<float,4> UserFilterFloat_Size4;
typedef UserFilter<float,5> UserFilterFloat_Size5;
typedef UserFilter<float,6> UserFilterFloat_Size6;
typedef UserFilter<float,7> UserFilterFloat_Size7;

typedef UserFilter<double,3> UserFilterDouble_Size3;
typedef UserFilter<double,4> UserFilterDouble_Size4;
typedef UserFilter<double,5> UserFilterDouble_Size5;
typedef UserFilter<double,6> UserFilterDouble_Size6;
typedef UserFilter<double,7> UserFilterDouble_Size7;
