/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_CropSprayer.h"
#include "AP_CropSprayer_Analog.h"

extern const AP_HAL::HAL& hal;

#define DEBUG_FLOW 0

/// Constructor
AP_CropSprayer_Analog::AP_CropSprayer_Analog(AP_CropSprayer &mon, AP_CropSprayer::CropSprayer_State &mon_state) :
    AP_CropSprayer_Backend(mon, mon_state)
{
    _pin_analog_source = hal.analogin->channel(mon._quantity_pin);

    // always healthy
    _state.healthy = true;
}

// read - read the voltage and current
void
AP_CropSprayer_Analog::read()
{
    // this copes with changing the pin at runtime
    _pin_analog_source->set_pin(_mon._quantity_pin);

    // get voltage
    _state.current_quantity = _pin_analog_source->read_average() * _mon._multiplier + _mon._offset;

#if DEBUG_FLOW 
        static uint16_t cnt = 0;
        if((0 == (cnt%100)) || (1 == (cnt%100)))
        {
            hal.util->prt("[ %d us] crop analog read: %f", hal.scheduler->micros(), _state.current_quantity);
        }
        cnt++;
#endif

}
