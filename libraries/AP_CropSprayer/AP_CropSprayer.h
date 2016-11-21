/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_CROPSPRAYER_H
#define AP_CROPSPRAYER_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>



#define AP_BATT_CAPACITY_DEFAULT            3300
#define AP_BATT_LOW_VOLT_TIMEOUT_MS         10000   // low voltage of 10 seconds will cause battery_exhausted to return true

// declare backend class
class AP_CropSprayer_Backend;
class AP_CropSprayer_Analog;

class AP_CropSprayer
{
    friend class AP_CropSprayer_Backend;
    friend class AP_CropSprayer_Analog;

public:

    /// Constructor
    AP_CropSprayer();


    // The CropSprayer_State structure is filled in by the backend driver
    struct CropSprayer_State {
        uint8_t     instance;           //
        bool        healthy;            // 
        float       raw;            //  raw data from AI
        float       current_quantity;       // current in amperes
        uint32_t    last_time_micros;   // 
    };


    // detect and initialise any available battery monitors
    void init();

    /// Read the battery voltage and current for all batteries.  Should be called at 10hz
    void read();

    // healthy - returns true if monitor is functioning
    bool healthy() const;

    float get_raw(uint8_t instance) const { return state.raw; }
    float quantity() const { return state.current_quantity; }
    // float quantity() const { return 0.0;  }


    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// parameters
    AP_Int8     _monitoring;
    AP_Int8     _quantity_pin;
    AP_Float    _multiplier;
    AP_Float    _offset;
    AP_Int32    _capacity;

private:
    CropSprayer_State state;
    AP_CropSprayer_Backend *drivers;
};
#endif  // AP_CROPSPRAYER_H
