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
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_MR72_DRIVER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Proximity_MR72_CAN.h"

const AP_Param::GroupInfo AP_Proximity_MR72_CAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: CAN receive ID
    // @Description: The receive ID of the CAN frames. A value of zero means default Jiyi ID (0x00D6) is accepted.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("RECV_ID", 1, AP_Proximity_MR72_CAN, receive_id, 0),

    AP_GROUPEND
};

AP_Proximity_MR72_CAN::AP_Proximity_MR72_CAN(AP_Proximity &_frontend,
                                     AP_Proximity::Proximity_State &_state,
                                     AP_Proximity_Params& _params):
    AP_Proximity_Backend(_frontend, _state, _params)
{
    // Bound to the Jiyi radar but keeping the MR72 wrapper for build compatibility
    multican_MR72 = NEW_NOTHROW MultiCAN{FUNCTOR_BIND_MEMBER(&AP_Proximity_MR72_CAN::handle_frame, bool, AP_HAL::CANFrame &), AP_CAN::Protocol::RadarCAN, "Jiyi MultiCAN"};
    if (multican_MR72 == nullptr) {
        AP_BoardConfig::allocation_error("Failed to create proximity multican");
    }

    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

// update state
void AP_Proximity_MR72_CAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms > 500) {
        // no new data.
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// handler for incoming frames
bool AP_Proximity_MR72_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    const uint16_t id = frame.id;
    
    // Default Jiyi ID is 0x00D6
    uint16_t expected_id = (receive_id > 0) ? (uint16_t)receive_id : 0x00D6;

    if (id != expected_id) {
        return false;
    }

    // Process the distance payload directly
    if (parse_distance_message(frame)) {
        last_update_ms = AP_HAL::millis();
    }

    return true;
}

// parse a distance message from CAN frame
bool AP_Proximity_MR72_CAN::parse_distance_message(AP_HAL::CANFrame &frame)
{
    // The Jiyi R-21 radar sends 8 bytes
    if (frame.dlc != 8) {
        return false;
    }

    // Check header byte
    if (frame.data[0] != 0xEA) {
        return false;
    }

    // Extract distance in mm using basic array indexing (Little Endian)
    // Added 256U to ensure strict unsigned compiler math
    uint16_t distance_mm = frame.data[1] + (frame.data[2] * 256U);
    float objects_dist = distance_mm * 0.001f;

    // Jiyi obstacle radars are mounted forward-facing (0 degrees)
    const float yaw = 0.0f; 

    if (ignore_reading(yaw, objects_dist)) {
        // obstacle is out of range based on min/max parameters
        return false;
    }

    // Reset temporary boundary, add the new reading, and commit to the main boundary
    _temp_boundary.reset();
    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(yaw);
    _temp_boundary.add_distance(face, yaw, objects_dist);
    _temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
    
    database_push(yaw, objects_dist);
    return true;
}

#endif  // AP_PROXIMITY_MR72_DRIVER_ENABLED