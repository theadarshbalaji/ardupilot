#pragma once
#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

class AP_RangeFinder_NRA24_CAN : public AP_RangeFinder_Backend_CAN {
public:
    AP_RangeFinder_NRA24_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        // Maintain the "nra24" registration string so Mission Planner recognizes the driver
        AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::RadarCAN, "nra24")
    {
    }

    void update(void) override;

    // handler for incoming frames
    bool handle_frame(AP_HAL::CANFrame &frame) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }
};

#endif  // AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED