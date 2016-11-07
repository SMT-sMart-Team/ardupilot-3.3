/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_CropSprayer.h"
#include "AP_CropSprayer_Analog.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_CropSprayer::var_info[] PROGMEM = {
    // @Param: _MONITOR
    // @DisplayName: Crop Spayer monitor
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Analog Voltage Only,4:Analog Voltage and Current,5:SMBus,6:Bebop
    // @User: Standard
    AP_GROUPINFO("_MONITOR", 0, AP_CropSprayer, _monitoring, ENABLE),

    // @Param: _VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:Pixhawk, 13:A13, 100:PX4
    // @User: Standard
    AP_GROUPINFO("_PIN", 1, AP_CropSprayer, _quantity_pin, CROPSPRAYER_PIN_DEF),


    // @Param: _VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("_MULT", 2, AP_CropSprayer, _multiplier, CROPSPRAYER_MULTI_DEF),


    // @Param: _AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: Volts
    // @User: Standard
    AP_GROUPINFO("_OFFSET", 3, AP_CropSprayer, _offset, 0),

    // @Param: _CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("_CAPACITY", 4, AP_CropSprayer, _capacity, CROPSPRAYER_CAP_DEF),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
#define DEBUG_FLOW 0
AP_CropSprayer::AP_CropSprayer(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    // _monitoring = ENABLE;
    // _quantity_pin = 5; // 1; 
    // _multiplier = 1.0;
    // _offset = 0.0;

}

// init - instantiate the battery monitors
void
AP_CropSprayer::init()
{
#if DEBUG_FLOW 
    hal.util->prt("[ %d us] crop init");
#endif
    if(_monitoring)
    {
        drivers = new AP_CropSprayer_Analog(*this, state);
    }

    // call init function for each backend
    if (drivers != NULL) {
        drivers->init();
#if DEBUG_FLOW 
        hal.util->prt("[ %d us] crop driver init");
#endif
    }
}

// read - read the voltage and current for all instances
void
AP_CropSprayer::read()
{
#if DEBUG_FLOW 
        static uint16_t cnt = 0;
        if((0 == (cnt%100)) || (1 == (cnt%100)))
        {
            hal.util->prt("[ %d us] crop read: %d", hal.scheduler->micros(), cnt);
        }
        cnt++;
#endif

    if(_monitoring)
    {
        drivers->read();
    }
}

// healthy - returns true if monitor is functioning
bool AP_CropSprayer::healthy() const {
    return state.healthy;
}




