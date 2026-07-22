#include "Copter.h"

#if AP_RANGEFINDER_ENABLED

void Copter::SurfaceTracking::update_surface_offset()
{
    const uint32_t now_ms = millis();
    const bool timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

    if (((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) ||
        ((surface == Surface::CEILING) && copter.rangefinder_up_ok() && (copter.rangefinder_up_state.glitch_count == 0))) {

        AP_SurfaceDistance &rf_state = (surface == Surface::GROUND) ? copter.rangefinder_state : copter.rangefinder_up_state;

        // Use a local variable for the position controller target
        float filtered_alt_m = rf_state.alt_m;

        if (surface == Surface::GROUND) {
            const float raw_dist = rf_state.alt_m;
            const bool is_auto_mode = (copter.flightmode->mode_number() == Mode::Number::AUTO);

            if (!is_auto_mode) {
                // Keep memory perfectly synced while flying manually
                last_valid_surface_dist = raw_dist;
                gap_start_time_ms = 0;
            } else {
                // Initialize if it somehow reads 0 (failsafe)
                if (last_valid_surface_dist <= 0.05f) {
                    last_valid_surface_dist = raw_dist;
                }

                // Gap detection: sudden drop of > 0.5 meters
                if (raw_dist > (last_valid_surface_dist + 0.5f)) {
                    if (gap_start_time_ms == 0) {
                        gap_start_time_ms = now_ms; 
                    }

                    if (now_ms - gap_start_time_ms < 2500) {
                        // Delay active: ignore gap, hold previous crop canopy height
                        filtered_alt_m = last_valid_surface_dist;
                    } else {
                        // Gap accepted as actual terrain after 2.5s
                        last_valid_surface_dist = raw_dist;
                    }
                } else {
                    // Normal terrain tracking (flat ground or rising canopy)
                    gap_start_time_ms = 0;
                    last_valid_surface_dist = raw_dist;
                }
            }
        }

        // Apply the filtered altitude to the drone's position controller
        const float terrain_u_m = filtered_alt_m * copter.ahrs.get_rotation_body_to_ned().z.z;

        copter.pos_control->set_pos_terrain_target_D_m(-terrain_u_m);
        last_update_ms = now_ms;
        valid_for_logging = true;

        if (timeout || reset_target || (last_glitch_cleared_ms != rf_state.glitch_cleared_ms)) {
            copter.pos_control->init_pos_terrain_D_m(-terrain_u_m);
            reset_target = false;
            last_glitch_cleared_ms = rf_state.glitch_cleared_ms;
        }

    } else {
        if (timeout && !reset_target) {
            copter.pos_control->init_pos_terrain_D_m(0);
            valid_for_logging = false;
            reset_target = true;
            
            // Failsafe: Wipe filter memory if sensor fails or tracking stops
            last_valid_surface_dist = 0.0f;
            gap_start_time_ms = 0;
        }
    }
}

void Copter::SurfaceTracking::external_init()
{
    if ((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) {
        last_update_ms = millis();
        reset_target = false;
    }
}

bool Copter::SurfaceTracking::get_target_dist_for_logging(float &target_dist_m) const
{
    if (!valid_for_logging || (surface == Surface::NONE)) {
        return false;
    }

    const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
    target_dist_m = dir * copter.pos_control->get_pos_desired_U_m();
    return true;
}

float Copter::SurfaceTracking::get_dist_for_logging() const
{
    return ((surface == Surface::CEILING) ? copter.rangefinder_up_state.alt_m : copter.rangefinder_state.alt_m);
}

void Copter::SurfaceTracking::set_surface(Surface new_surface)
{
    if (surface == new_surface) {
        return;
    }
    if ((new_surface == Surface::GROUND) && !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no downward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    if ((new_surface == Surface::CEILING) && !copter.rangefinder.has_orientation(ROTATION_PITCH_90)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no upward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    surface = new_surface;
    reset_target = true;
    
    // Wipe memory on surface flip
    last_valid_surface_dist = 0.0f;
    gap_start_time_ms = 0;
}

#endif  // AP_RANGEFINDER_ENABLED