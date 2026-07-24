#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED

#include "AP_RangeFinder_NRA24_CAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>

// update the state of the sensor
void AP_RangeFinder_NRA24_CAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (get_reading(state.distance_m)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        // No valid data received within the timeout period
        set_status(RangeFinder::Status::NoData);
    }
}

// handler for incoming frames
bool AP_RangeFinder_NRA24_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t id = frame.id;

    // Jiyi H30 terrain radars broadcast on 0x73C or 0x75C
    if (id != 0x073CU && id != 0x075CU) {
        return false;
    }

    // Guard against memory bounds errors by enforcing the 6-byte payload requirement
    if (frame.dlc != 6) {
        return false;
    }

    // Extract distance in mm using basic array indexing (Little Endian)
    // 256U ensures compiler safety during unsigned math operations
    uint16_t distance_mm = frame.data[1] + (frame.data[2] * 256U);
    
    // Convert to meters
    const float dist_m = distance_mm * 0.001f;

    // Push the distance reading to the ArduPilot frontend
    accumulate_distance_m(dist_m);

    return true;
}

#endif  // AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED