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
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_AHRS.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_gyro();
    }
#if EKF_CALC_GYRO_USE
    return _gyro_ekf_calc;
#else
    return _gyro_estimate;
#endif
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_dcm_matrix();
    }
    return _dcm_matrix;
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_gyro_drift();
    }
    return _gyro_bias;
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
    // update DCM
    AP_AHRS_DCM::reset_gyro_drift();

    // reset the EKF gyro bias states
    EKF.resetGyroBias();
}

void AP_AHRS_NavEKF::update(void)
{
    // we need to restore the old DCM attitude values as these are
    // used internally in DCM to calculate error values for gyro drift
    // correction
    roll = _dcm_attitude.x;
    pitch = _dcm_attitude.y;
    yaw = _dcm_attitude.z;
    update_cd_values();

    AP_AHRS_DCM::update();

    // keep DCM attitude available for get_secondary_attitude()
    _dcm_attitude(roll, pitch, yaw);

    if (!ekf_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = hal.scheduler->millis();
        }
        if (hal.scheduler->millis() - start_time_ms > startup_delay_ms) {
            ekf_started = EKF.InitialiseFilterDynamic();
        }
    }
    if (ekf_started) {
        EKF.UpdateFilter();
        EKF.getRotationBodyToNED(_dcm_matrix);
        if (using_EKF()) {
            Vector3f eulers;
            EKF.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF.getGyroBias(_gyro_bias);
            _gyro_bias = -_gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            _gyro_estimate.zero();
            uint8_t healthy_count = 0;    
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i) && healthy_count < 2) {
                    _gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                _gyro_estimate /= healthy_count;
            }
            _gyro_estimate += _gyro_bias;

            float abias1, abias2;
            EKF.getAccelZBias(abias1, abias2);

            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i==0) {
                    accel.z -= abias1;
                } else if (i==1) {
                    accel.z -= abias2;
                }
                if (_ins.get_accel_health(i)) {
                    _accel_ef_ekf[i] = _dcm_matrix * accel;
                }
            }

            if(_ins.use_accel(0) && _ins.use_accel(1)) {
                float IMU1_weighting;
                EKF.getIMU1Weighting(IMU1_weighting);
                _accel_ef_ekf_blended = _accel_ef_ekf[0] * IMU1_weighting + _accel_ef_ekf[1] * (1.0f-IMU1_weighting);
            } else {
                _accel_ef_ekf_blended = _accel_ef_ekf[_ins.get_primary_accel()];
            }


            // AB ZhaoYJ@2017-03-20 for tdiff of angle and velocity
#if EKF_CALC_GYRO_ACCEL
#define TRY_DCM 0

#define TIME32_SUB(x, y) ((x >= y)?(x - y):(0xFFFFFFFF - y + x))

            static bool first = true;
            if(!first)
            {
                //  calc diff
                uint32_t now = hal.scheduler->micros();
                uint32_t delta_t = TIME32_SUB(now, _last_ekf_t);
                // constrain dt to at least 2.5ms)
                if(delta_t < 2500)
                {
                    return;
                }

                Vector3f cur_vel;
                EKF.getVelNED(cur_vel);
                Vector3d angle, vel;
#if TRY_DCM
                angle.x = (double) _dcm_attitude.x;
                angle.y = (double) _dcm_attitude.y;
                angle.z = (double) _dcm_attitude.z;
#else
                angle.x = (double)eulers.x;
                angle.y = (double)eulers.y;
                angle.z = (double)eulers.z;
#endif
                vel.x = (double)cur_vel.x;
                vel.y = (double)cur_vel.y;
                vel.z = (double)cur_vel.z;

                _angle_rate_EKF = (angle - _last_ekf_angle)*1000000.0d/(double)(delta_t); // rad/s 
                _accel_EKF = (vel - _last_ekf_vel)*1000000.0d/(double)delta_t; // m/s/s 
                _accel_EKF.z -= GRAVITY_MSS;

#if EKF_CALC_GYRO_ACCEL_LPF 

                Vector3f gyro_tmp, accl_tmp;
                gyro_tmp = _lpf_ekf_gyro.apply(Vector3f((float)_angle_rate_EKF.x, 
                            (float)_angle_rate_EKF.y,
                            (float)_angle_rate_EKF.z));
                accl_tmp = _lpf_ekf_accl.apply(Vector3f((float)_accel_EKF.x,
                            (float)_accel_EKF.y,
                            (float)_accel_EKF.z));
                _angle_rate_EKF = Vector3d((double)gyro_tmp.x, (double)gyro_tmp.y, (double)gyro_tmp.z);
                _accel_EKF = Vector3d((double)accl_tmp.x, (double)accl_tmp.y, (double)accl_tmp.z);
#endif

                // record gyro & accl ekf calculated
                _gyro_ekf_calc = Vector3f((float)_angle_rate_EKF.x, (float)_angle_rate_EKF.y, (float)_angle_rate_EKF.z);

                _accel_ef_ekf_calc = Vector3f((float)_accel_EKF.x, (float)_accel_EKF.y, (float)_accel_EKF.z);


#if 0
                if((fabs(_angle_rate_EKF.x) > 0.01f)
                   || (fabs(_angle_rate_EKF.y) > 0.01f)
                   || (fabs(_angle_rate_EKF.z) > 0.01f)
                   || (now == 0)
                   )
                {
                    hal.util->prt("[%d ms]EKF cal seems not good: gyroX-> %f, gyroY-> %f, gyroZ->%f", hal.scheduler->millis(),  
                            _angle_rate_EKF.x,
                            _angle_rate_EKF.y,
                            _angle_rate_EKF.z);
                    hal.util->prt("=== now(%u), _last_ekf_t(%u)", now, _last_ekf_t);
                    hal.util->prt("=== angle.x(%.19f), _last_ekf_angle.x(%.19f)", angle.x, _last_ekf_angle.x);
                    hal.util->prt("=== angle.y(%.19f), _last_ekf_angle.y(%.19f)", angle.y, _last_ekf_angle.y);
                    hal.util->prt("=== angle.z(%.19f), _last_ekf_angle.z(%.19f)", angle.z, _last_ekf_angle.z);
                }
#endif

                // update 
                _last_ekf_t = now;
                _last_ekf_angle = angle;
                _last_ekf_vel = vel;
                
            }
            else // init
            {
                first = false;
#if EKF_CALC_GYRO_ACCEL_LPF 
                _lpf_ekf_gyro.set_cutoff_frequency(1000, 20);
                _lpf_ekf_accl.set_cutoff_frequency(1000, 20);
#endif
                _last_ekf_t = hal.scheduler->micros();
                _last_ekf_angle.x = (double)eulers.x;
                _last_ekf_angle.y = (double)eulers.y;
                _last_ekf_angle.z = (double)eulers.z;
                Vector3f cur_vel;
                EKF.getVelNED(cur_vel);
                _last_ekf_vel.x = (double)cur_vel.x;
                _last_ekf_vel.y = (double)cur_vel.y;
                _last_ekf_vel.z = (double)cur_vel.z;
                hal.util->prt("[%d ms]EKF cal gyro & accel start, last_ekf_t: %lu", hal.scheduler->millis(),  _last_ekf_t);
            }
#endif
        }
    }
}

// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef(uint8_t i) const
{
    if(!using_EKF()) {
        return AP_AHRS_DCM::get_accel_ef(i);
    }
    return _accel_ef_ekf[i];
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended(void) const
{
    if(!using_EKF()) {
        return AP_AHRS_DCM::get_accel_ef_blended();
    }
#if EKF_CALC_ACCEL_USE
    return _accel_ef_ekf_calc;
#else
    return _accel_ef_ekf_blended;
#endif
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended_log(void) const
{
    if(!using_EKF()) {
        return AP_AHRS_DCM::get_accel_ef_blended();
    }
    return _accel_ef_ekf_blended;
}


void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
    if (ekf_started) {
        ekf_started = EKF.InitialiseFilterBootstrap();        
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    if (ekf_started) {
        ekf_started = EKF.InitialiseFilterBootstrap();        
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    Vector3f ned_pos;
    if (using_EKF() && EKF.getLLH(loc) && EKF.getPosNED(ned_pos)) {
        // fixup altitude using relative position from AHRS home, not
        // EKF origin
        loc.alt = get_home().alt - ned_pos.z*100;
        return true;
    }
    return AP_AHRS_DCM::get_position(loc);
}

// status reporting of estimated errors
float AP_AHRS_NavEKF::get_error_rp(void) const
{
    return AP_AHRS_DCM::get_error_rp();
}

float AP_AHRS_NavEKF::get_error_yaw(void) const
{
    return AP_AHRS_DCM::get_error_yaw();
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS_NavEKF::wind_estimate(void)
{
    if (!using_EKF()) {
        // EKF does not estimate wind speed when there is no airspeed
        // sensor active
        return AP_AHRS_DCM::wind_estimate();
    }
    Vector3f wind;
    EKF.getWind(wind);
    return wind;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float *airspeed_ret) const
{
    return AP_AHRS_DCM::airspeed_estimate(airspeed_ret);
}

// true if compass is being used
bool AP_AHRS_NavEKF::use_compass(void)
{
    if (using_EKF()) {
        return EKF.use_compass();
    }
    return AP_AHRS_DCM::use_compass();
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers)
{
    if (using_EKF()) {
        // return DCM attitude
        eulers = _dcm_attitude;
        return true;
    }
    if (ekf_started) {
        // EKF is secondary
        EKF.getEulerAngles(eulers);
        return true;
    }
    // no secondary available
    return false;
}

// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc)
{
    if (using_EKF()) {
        // return DCM position
        AP_AHRS_DCM::get_position(loc);
        return true;
    }    
    if (ekf_started) {
        // EKF is secondary
        EKF.getLLH(loc);
        return true;
    }
    // no secondary available
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::groundspeed_vector();
    }
    Vector3f vec;
    EKF.getVelNED(vec);
    return Vector2f(vec.x, vec.y);
}

void AP_AHRS_NavEKF::set_home(const Location &loc)
{
    AP_AHRS_DCM::set_home(loc);
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const 
{
    return using_EKF();
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    if (using_EKF()) {
        EKF.getVelNED(vec);
        return true;
    }
    return false;
}

// return a relative ground position in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_relative_position_NED(Vector3f &vec) const
{
    if (using_EKF()) {
        return EKF.getPosNED(vec);
    }
    return false;
}

bool AP_AHRS_NavEKF::using_EKF(void) const
{
    uint8_t ekf_faults;
    EKF.getFilterFaults(ekf_faults);
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    bool ret = ekf_started && ((_ekf_use == EKF_USE_WITH_FALLBACK && EKF.healthy()) || (_ekf_use == EKF_USE_WITHOUT_FALLBACK && ekf_faults == 0));
    if (!ret) {
        return false;
    }

    if (_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
        _vehicle_class == AHRS_VEHICLE_GROUND) {
        nav_filter_status filt_state;
        EKF.getFilterStatus(filt_state);
        if (hal.util->get_soft_armed() && !filt_state.flags.using_gps && _gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            // if the EKF is not fusing GPS and we have a 3D lock, then
            // plane and rover would prefer to use the GPS position from
            // DCM. This is a safety net while some issues with the EKF
            // get sorted out
            return false;
        }
        if (hal.util->get_soft_armed() && filt_state.flags.const_pos_mode) {
            return false;
        }
        if (!filt_state.flags.attitude ||
            !filt_state.flags.horiz_vel ||
            !filt_state.flags.vert_vel ||
            !filt_state.flags.horiz_pos_abs ||
            !filt_state.flags.vert_pos) {
            return false;
        }
    }
    return ret;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_NavEKF::healthy(void) const
{
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    if (_ekf_use != EKF_DO_NOT_USE) {
        bool ret = ekf_started && EKF.healthy();
        if (!ret) {
            return false;
        }
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
             _vehicle_class == AHRS_VEHICLE_GROUND) &&
            !using_EKF()) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return true;
    }
    return AP_AHRS_DCM::healthy();    
}

void AP_AHRS_NavEKF::set_ekf_use(bool setting)
{
#if !AHRS_EKF_USE_ALWAYS
    _ekf_use.set(setting);
#endif
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) const
{
    // initialisation complete 10sec after ekf has started
    return (ekf_started && (hal.scheduler->millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
};

// write optical flow data to EKF
void  AP_AHRS_NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
{
    EKF.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
}

// inhibit GPS useage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    return EKF.setInhibitGPS();
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler)
{
    EKF.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS_NavEKF::getMagOffsets(Vector3f &magOffsets)
{
    bool status = EKF.getMagOffsets(magOffsets);
    return status;
}

// report any reason for why the backend is refusing to initialise
const char *AP_AHRS_NavEKF::prearm_failure_reason(void) const
{
    if (_ekf_use != EKF_DO_NOT_USE) {
        return EKF.prearm_failure_reason();
    }
    return nullptr;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

