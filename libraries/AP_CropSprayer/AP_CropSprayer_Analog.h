/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_CROPSPRAYER_ANALOG_H
#define AP_CROPSPRAYER_ANALOG_H

#include <AP_ADC/AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include "AP_CropSprayer.h"
#include "AP_CropSprayer_Backend.h"

#define ENABLE 1
#define CROPSPRAYER_PIN_DEF 1
#define CROPSPRAYER_MULTI_DEF 1.0 
#define CROPSPRAYER_CAP_DEF 20.0

// 
#define HAL_GPIO_QUANTITY_DI 9 
#define HAVE_WATER 1 
#define NO_WATER 0 



class AP_CropSprayer_Analog : public AP_CropSprayer_Backend
{
public:

    /// Constructor
    AP_CropSprayer_Analog(AP_CropSprayer &mon, AP_CropSprayer::CropSprayer_State &mon_state);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

protected:

    AP_HAL::AnalogSource *_pin_analog_source;
};
#endif  // AP_CROPSPRAYER_ANALOG_H
