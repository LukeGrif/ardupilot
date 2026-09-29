#include "Sub.h"

/*
 * DYNAMIC mode
 *
 * Accepts the same external commands as GUIDED (SET_POSITION_TARGET_LOCAL_NED,
 * SET_POSITION_TARGET_GLOBAL_INT, SET_ATTITUDE_TARGET, MAV_CMD_CONDITION_YAW,
 * MAV_CMD_DO_REPOSITION) with these differences:
 *
 * - The vehicle always holds position, depth and heading when no command is
 *   active. Velocity, climb rate and yaw rate commands stop after DYN_TIMEOUT
 *   seconds without an update and the vehicle holds where it stops.
 * - SET_ATTITUDE_TARGET sets roll and pitch targets that are held while the
 *   position controller keeps running, so the vehicle can hold position at a
 *   commanded attitude. The quaternion yaw is ignored (use CONDITION_YAW or the
 *   yaw fields of the position target messages); the body yaw rate is used if
 *   not masked out. Thrust other than 0.5 commands a climb rate.
 * - Horizontal and vertical thrust demands are rotated into the body frame so
 *   position control stays correct while the vehicle is rolled or pitched.
 * - With DYN_BT_ENABLE set, the downward rangefinder is used to hold range
 *   above the seafloor (bottom tracking) instead of a fixed depth.
 * - Joystick forward/lateral/throttle/yaw sticks move the vehicle while held
 *   and it holds position, depth and heading where they are released.
 * - MAV_CMD_MISSION_START flies the uploaded mission's waypoints (e.g. a
 *   lawnmower survey) with AC_WPNav, keeping the roll/pitch hold, then holds at
 *   the last waypoint. Any position/velocity command stops the path.
 */

// initialise dynamic mode
bool ModeDynamic::init(bool ignore_checks)
{
    if (!sub.position_ok() && !ignore_checks) {
        return false;
    }

    init_controllers();

    // start level
    roll_target_cd = 0.0f;
    pitch_target_cd = 0.0f;
    att_update_ms = AP_HAL::millis();

    // hold the current heading
    pilot_yawing = false;
    gcs_velocity_ms = 0;
    sub.yaw_rate_only = false;
    sub.yaw_look_at_heading = ahrs.yaw_sensor;
    sub.yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);

    return true;
}

// initialise the position controllers and hold at the stopping point
void ModeDynamic::init_controllers()
{
    path_running = false;

    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->NE_set_correction_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_D_cmss());
    position_control->D_set_correction_speed_accel_cm(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_D_cmss());

    position_control->NE_init_controller();
    // NE_init_controller seeds the velocity integrator from the lean angle
    // target, which in this mode is a deliberate attitude rather than thrust
    position_control->NE_get_vel_pid().reset_I();

    // this also clears the terrain offset used for bottom tracking
    position_control->D_init_controller();
    bt_active = false;

    Vector2p stopping_point_ne_m;
    position_control->get_stopping_point_NE_m(stopping_point_ne_m);
    set_horizontal_target(SubMode::POSITION, stopping_point_ne_m, Vector2f());

    postype_t stopping_point_d_m;
    position_control->get_stopping_point_D_m(stopping_point_d_m);
    set_vertical_target(SubMode::POSITION, stopping_point_d_m, 0.0f);
}

bool ModeDynamic::timed_out(uint32_t update_ms, uint32_t now_ms, uint32_t timeout_ms) const
{
    return (now_ms - update_ms) > timeout_ms;
}

// true while velocity commands from a ground station keep arriving
bool ModeDynamic::gcs_velocity_active() const
{
    return gcs_velocity_ms != 0 && (AP_HAL::millis() - gcs_velocity_ms) < GCS_VELOCITY_PRIORITY_MS;
}

void ModeDynamic::set_horizontal_target(SubMode submode, const Vector2p& pos_ne_m, const Vector2f& vel_ne_ms)
{
    if (path_running) {
        stop_path("new command");
    }
    horiz_submode = submode;
    pos_target_ne_m = pos_ne_m;
    vel_target_ne_ms = vel_ne_ms;
    horiz_update_ms = AP_HAL::millis();
}

// pos_d_m is relative to the bottom tracking offset (if any)
void ModeDynamic::set_vertical_target(SubMode submode, float pos_d_m, float vel_d_ms)
{
    if (path_running) {
        stop_path("new command");
    }
    vert_submode = submode;
    pos_target_d_m = pos_d_m;
    vel_target_d_ms = vel_d_ms;
    vert_update_ms = AP_HAL::millis();
    climb_from_thrust = false;
}

// returns false if the destination is outside the fence
bool ModeDynamic::check_destination(const Vector3f& destination) const
{
#if AP_FENCE_ENABLED
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        return false;
    }
#endif
    return true;
}

// set a destination (NEU, cm from the EKF origin). The vehicle holds there once it arrives.
bool ModeDynamic::dynamic_set_destination(const Vector3f& destination)
{
    if (!check_destination(destination)) {
        return false;
    }

    set_horizontal_target(SubMode::POSITION, Vector2p{destination.x, destination.y} * 0.01, Vector2f());
    set_vertical_target(SubMode::POSITION, -destination.z * 0.01 - position_control->get_pos_terrain_D_m(), 0.0f);

#if HAL_LOGGING_ENABLED
    sub.Log_Write_GuidedTarget(uint8_t(horiz_submode), destination, Vector3f());
#endif

    return true;
}

bool ModeDynamic::dynamic_set_destination(const Location& dest_loc)
{
#if AP_FENCE_ENABLED
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        return false;
    }
#endif

    Vector3f destination;
    if (!dest_loc.get_vector_from_origin_NEU_cm(destination)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        return false;
    }

    return dynamic_set_destination(destination);
}

bool ModeDynamic::dynamic_set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    if (!check_destination(destination)) {
        return false;
    }

    dynamic_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    return dynamic_set_destination(destination);
}

// set a velocity target (NEU, cm/s). The vehicle holds position when it times out.
void ModeDynamic::dynamic_set_velocity(const Vector3f& velocity)
{
    gcs_velocity_ms = AP_HAL::millis();
    set_horizontal_target(SubMode::VELOCITY, pos_target_ne_m, Vector2f{velocity.x, velocity.y} * 0.01);
    set_vertical_target(SubMode::VELOCITY, pos_target_d_m, -velocity.z * 0.01);
}

void ModeDynamic::dynamic_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    dynamic_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);
    dynamic_set_velocity(velocity);
}

// set a position target that moves at a velocity (NEU, cm and cm/s)
bool ModeDynamic::dynamic_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity)
{
    if (!check_destination(destination)) {
        return false;
    }

    set_horizontal_target(SubMode::POSVEL, Vector2p{destination.x, destination.y} * 0.01, Vector2f{velocity.x, velocity.y} * 0.01);
    set_vertical_target(SubMode::POSVEL, -destination.z * 0.01 - position_control->get_pos_terrain_D_m(), -velocity.z * 0.01);

#if HAL_LOGGING_ENABLED
    sub.Log_Write_GuidedTarget(uint8_t(horiz_submode), destination, velocity);
#endif

    return true;
}

bool ModeDynamic::dynamic_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    if (!check_destination(destination)) {
        return false;
    }

    dynamic_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    return dynamic_set_destination_posvel(destination, velocity);
}

// Set the yaw target. Unlike GUIDED, a command with neither yaw nor yaw rate
// leaves the current heading target unchanged, so heading set by
// MAV_CMD_CONDITION_YAW is kept while velocity commands are streamed.
void ModeDynamic::dynamic_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (!use_yaw && !use_yaw_rate) {
        return;
    }
    sub.mode_guided.guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_angle);
    yaw_rate_update_ms = AP_HAL::millis();
}

// set roll and pitch targets from a quaternion; yaw is ignored
void ModeDynamic::dynamic_set_angle(const Quaternion &q, bool use_attitude, bool use_climb_rate, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_cds)
{
    if (use_attitude) {
        float roll_rad, pitch_rad, yaw_rad;
        q.to_euler(roll_rad, pitch_rad, yaw_rad);
        roll_target_cd = degrees(roll_rad) * 100.0f;
        pitch_target_cd = degrees(pitch_rad) * 100.0f;
        att_update_ms = AP_HAL::millis();
    }

    if (use_climb_rate) {
        if (!is_zero(climb_rate_cms)) {
            set_vertical_target(SubMode::VELOCITY, pos_target_d_m, -climb_rate_cms * 0.01);
            climb_from_thrust = true;
        } else if (climb_from_thrust) {
            // thrust returned to neutral: stop climbing and hold depth
            set_vertical_target(SubMode::VELOCITY, pos_target_d_m, 0.0f);
        }
    }

    if (use_yaw_rate) {
        dynamic_set_yaw_state(false, 0.0f, true, yaw_rate_cds, false);
    }
}

// returns the range above the seafloor being held, or -1 if bottom tracking is not active
float ModeDynamic::get_bottom_track_target_cm() const
{
    if (!bt_active) {
        return -1.0f;
    }
    // with bottom tracking active the desired altitude is relative to the seafloor
    return position_control->get_pos_desired_U_cm();
}

// set the range above the seafloor to hold, keeping the horizontal target
bool ModeDynamic::set_bottom_track_target_cm(float range_cm)
{
#if AP_RANGEFINDER_ENABLED
    if (!bt_active) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Dynamic: bottom tracking not active");
        return false;
    }
    if (range_cm < sub.rangefinder_state.min * 100.0f || range_cm > sub.rangefinder_state.max * 100.0f) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Dynamic: range target out of rangefinder limits");
        return false;
    }
    set_vertical_target(SubMode::POSITION, -range_cm * 0.01, 0.0f);
    return true;
#else
    return false;
#endif
}

// set a horizontal destination (NE, cm from the EKF origin) and a range above the seafloor
bool ModeDynamic::dynamic_set_destination_NE_range(const Vector2f& destination_ne_cm, float range_cm)
{
    if (!check_destination(Vector3f{destination_ne_cm.x, destination_ne_cm.y, inertial_nav.get_position_z_up_cm()})) {
        return false;
    }
    if (!set_bottom_track_target_cm(range_cm)) {
        return false;
    }
    set_horizontal_target(SubMode::POSITION, Vector2p{destination_ne_cm.x, destination_ne_cm.y} * 0.01, Vector2f());
    return true;
}

// dynamic_run - runs the dynamic controller
// should be called at 100hz or more
void ModeDynamic::run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        init_controllers();
        sub.yaw_look_at_heading = ahrs.yaw_sensor;
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t timeout_ms = MAX(g2.dyn_timeout.get(), 0.1f) * 1000;

    // joystick forward/lateral/throttle (stops a running path when used)
    update_pilot_translation();

    if (path_running) {
        run_path();
    } else {
        update_horizontal(now_ms, timeout_ms);
        update_vertical(now_ms, timeout_ms);
    }

    // roll and pitch: commanded target plus pilot input
    if (is_positive(g2.dyn_att_timeout) && timed_out(att_update_ms, now_ms, g2.dyn_att_timeout * 1000)) {
        roll_target_cd = 0.0f;
        pitch_target_cd = 0.0f;
    }
    const float angle_max_cd = constrain_float(g2.dyn_angle_max, 0.0f, 85.0f) * 100.0f;
    const float roll_cd = constrain_float(roll_target_cd + channel_roll->get_control_in(), -angle_max_cd, angle_max_cd);
    const float pitch_cd = constrain_float(pitch_target_cd + channel_pitch->get_control_in(), -angle_max_cd, angle_max_cd);

    // yaw rate commands time out like velocity commands
    if (sub.auto_yaw_mode == AUTO_YAW_RATE && timed_out(yaw_rate_update_ms, now_ms, timeout_ms)) {
        sub.yaw_rate_only = false;
        sub.yaw_look_at_heading = ahrs.yaw_sensor;
        sub.yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
        sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }

    float target_yaw_rate_cds = 0.0f;
    update_yaw(target_yaw_rate_cds);

    // call attitude controller
    switch (sub.auto_yaw_mode) {
    case AUTO_YAW_HOLD:
        // yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, target_yaw_rate_cds);
        break;
    case AUTO_YAW_RATE:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, sub.yaw_look_at_heading_slew * 100.0f);
        break;
    case AUTO_YAW_LOOK_AT_HEADING:
        attitude_control->input_euler_angle_roll_pitch_slew_yaw_cd(roll_cd, pitch_cd, sub.mode_guided.get_auto_heading(), sub.yaw_look_at_heading_slew * 100.0f);
        break;
    default:
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(roll_cd, pitch_cd, sub.mode_guided.get_auto_heading(), true);
        break;
    }

    output_thrust();
}

/*
 * Joystick translation: forward/lateral sticks command a body-frame velocity
 * (full stick = WP_SPD) and the throttle stick a climb rate (PILOT_SPEED_UP/DN).
 * When a stick is released the target velocity goes to zero, so the vehicle
 * stops and holds the position/depth it stopped at, like any other velocity
 * command in this mode.
 */
void ModeDynamic::update_pilot_translation()
{
    if (sub.failsafe.pilot_input) {
        pilot_horizontal = false;
        pilot_vertical = false;
        return;
    }
    if (gcs_velocity_active()) {
        // a ground station is streaming velocities (e.g. a gamepad in the
        // control app): it has priority over the joystick sticks
        pilot_horizontal = false;
        pilot_vertical = false;
        return;
    }

    const float forward = channel_forward->norm_input_dz();
    const float lateral = channel_lateral->norm_input_dz();
    if (fabsf(forward) > 0.02f || fabsf(lateral) > 0.02f) {
        const float speed_ms = sub.wp_nav.get_default_speed_NE_ms();
        const float cos_yaw = ahrs.cos_yaw();
        const float sin_yaw = ahrs.sin_yaw();
        const Vector2f vel_ne_ms{(forward * cos_yaw - lateral * sin_yaw) * speed_ms,
                                 (forward * sin_yaw + lateral * cos_yaw) * speed_ms};
        set_horizontal_target(SubMode::VELOCITY, pos_target_ne_m, vel_ne_ms);
        pilot_horizontal = true;
    } else if (pilot_horizontal) {
        set_horizontal_target(SubMode::VELOCITY, pos_target_ne_m, Vector2f());
        pilot_horizontal = false;
    }

    float climb_rate_cms = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    climb_rate_cms = constrain_float(climb_rate_cms, -sub.get_pilot_speed_dn(), g.pilot_speed_up);
    if (fabsf(climb_rate_cms) > 1.0f) {
        set_vertical_target(SubMode::VELOCITY, pos_target_d_m, -climb_rate_cms * 0.01f);
        pilot_vertical = true;
    } else if (pilot_vertical) {
        set_vertical_target(SubMode::VELOCITY, pos_target_d_m, 0.0f);
        pilot_vertical = false;
    }
}

// pilot yaw input overrides the yaw target; the new heading is held when the pilot lets go
void ModeDynamic::update_yaw(float &target_yaw_rate_cds)
{
    if (sub.failsafe.pilot_input) {
        return;
    }
    if (gcs_velocity_active()) {
        if (pilot_yawing) {
            // hand over from the stick: hold the heading reached
            pilot_yawing = false;
            sub.yaw_rate_only = false;
            sub.yaw_look_at_heading = ahrs.yaw_sensor;
            sub.yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
            sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
        }
        return;
    }
    target_yaw_rate_cds = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate_cds)) {
        pilot_yawing = true;
        sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_HOLD);
    } else if (pilot_yawing) {
        pilot_yawing = false;
        sub.yaw_rate_only = false;
        sub.yaw_look_at_heading = ahrs.yaw_sensor;
        sub.yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
        sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }
}

void ModeDynamic::update_horizontal(uint32_t now_ms, uint32_t timeout_ms)
{
    switch (horiz_submode) {
    case SubMode::VELOCITY: {
        if (timed_out(horiz_update_ms, now_ms, timeout_ms)) {
            // the desired position stops where the vehicle comes to rest and is held there
            vel_target_ne_ms.zero();
        }
        Vector2f vel_ne_ms = vel_target_ne_ms;
        position_control->input_vel_accel_NE_m(vel_ne_ms, Vector2f());
        break;
    }
    case SubMode::POSVEL: {
        if (timed_out(horiz_update_ms, now_ms, timeout_ms)) {
            vel_target_ne_ms.zero();
        }
        const float dt = position_control->get_dt_s();
        pos_target_ne_m.x += vel_target_ne_ms.x * dt;
        pos_target_ne_m.y += vel_target_ne_ms.y * dt;
        Vector2p pos_ne_m = pos_target_ne_m;
        Vector2f vel_ne_ms = vel_target_ne_ms;
        position_control->input_pos_vel_accel_NE_m(pos_ne_m, vel_ne_ms, Vector2f());
        break;
    }
    case SubMode::POSITION: {
        Vector2p pos_ne_m = pos_target_ne_m;
        Vector2f vel_ne_ms;
        position_control->input_pos_vel_accel_NE_m(pos_ne_m, vel_ne_ms, Vector2f());
        break;
    }
    }

    position_control->NE_update_controller();
}

void ModeDynamic::update_vertical(uint32_t now_ms, uint32_t timeout_ms)
{
    // never target above SURFACE_DEPTH, and stop climbing once at the surface
    const float surface_d_m = -g.surface_depth * 0.01f - position_control->get_pos_terrain_D_m();
    pos_target_d_m = MAX(pos_target_d_m, surface_d_m);
    if (sub.ap.at_surface) {
        vel_target_d_ms = MAX(vel_target_d_ms, 0.0f);
    }

    switch (vert_submode) {
    case SubMode::VELOCITY: {
        if (timed_out(vert_update_ms, now_ms, timeout_ms)) {
            vel_target_d_ms = 0.0f;
            climb_from_thrust = false;
        }
        const float climb_rate_ms = constrain_float(-vel_target_d_ms, -sub.wp_nav.get_default_speed_down_ms(), sub.wp_nav.get_default_speed_up_ms());
        position_control->D_set_pos_target_from_climb_rate_ms(climb_rate_ms);
        break;
    }
    case SubMode::POSVEL: {
        if (timed_out(vert_update_ms, now_ms, timeout_ms)) {
            vel_target_d_ms = 0.0f;
        }
        pos_target_d_m += vel_target_d_ms * position_control->get_dt_s();
        float pos_d_m = pos_target_d_m;
        float vel_d_ms = vel_target_d_ms;
        position_control->input_pos_vel_accel_D_m(pos_d_m, vel_d_ms, 0.0f);
        break;
    }
    case SubMode::POSITION: {
        float pos_d_m = pos_target_d_m;
        float vel_d_ms = 0.0f;
        position_control->input_pos_vel_accel_D_m(pos_d_m, vel_d_ms, 0.0f);
        break;
    }
    }

    update_bottom_track();

    const float surface_desired_u_cm = g.surface_depth + position_control->get_pos_terrain_D_m() * 100.0f;
    if (position_control->get_pos_desired_U_cm() > surface_desired_u_cm) {
        position_control->set_pos_desired_U_cm(surface_desired_u_cm);
    }

    position_control->D_update_controller();
}

/*
 * Path following. Waypoints are read from the uploaded mission (the same
 * storage AUTO uses) and flown one leg at a time with AC_WPNav, stopping at
 * each waypoint so survey lines stay straight. Supported items:
 *   NAV_WAYPOINT / NAV_SPLINE_WAYPOINT / NAV_LOITER_TIME (param1 = hold seconds)
 *   NAV_LOITER_UNLIM (go there and finish), DO_CHANGE_SPEED, CONDITION_YAW.
 * Waypoints with an above-terrain altitude follow the seafloor using the
 * downward rangefinder (WPNAV_RFND_USE). The last waypoint is held afterwards.
 */
bool ModeDynamic::start_path(uint16_t first_index)
{
    if (!motors.armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Dynamic: arm before starting a path");
        return false;
    }
    const uint16_t num_cmds = sub.mission.num_commands();
    // index 0 is home
    first_index = MAX(first_index, 1);
    if (first_index >= num_cmds) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Dynamic: no path uploaded");
        return false;
    }

    // start from where we are, heading as set by WP_YAW_BEHAVIOR
    sub.wp_nav.wp_and_spline_init_m();
    bt_active = false;
    path_running = true;
    path_index = first_index;
    path_reached_ms = 0;

    switch (g.wp_yaw_behavior) {
    case WP_YAW_BEHAVIOR_NONE:
        break;
    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_LOOK_AHEAD);
        break;
    case WP_YAW_BEHAVIOR_CORRECT_XTRACK:
        sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_CORRECT_XTRACK);
        break;
    default:
        sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
        break;
    }

    if (!path_next_leg()) {
        return false;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Dynamic: path started at #%u of %u", unsigned(path_index), unsigned(num_cmds - 1));
    return true;
}

// stop following the path and hold where we are
void ModeDynamic::stop_path(const char *reason)
{
    if (!path_running) {
        return;
    }
    path_running = false;
    gcs().send_text(MAV_SEVERITY_INFO, "Dynamic: path stopped (%s)", reason);
    path_hold_here(false);
}

// hand over to the normal hold logic, at the final waypoint or where we stop
void ModeDynamic::path_hold_here(bool at_destination)
{
    Vector2p hold_ne_m;
    if (at_destination) {
        hold_ne_m = sub.wp_nav.get_wp_destination_NED_m().xy();
    } else {
        position_control->get_stopping_point_NE_m(hold_ne_m);
    }
    set_horizontal_target(SubMode::POSITION, hold_ne_m, Vector2f());
    set_vertical_target(SubMode::POSITION, -position_control->get_pos_desired_U_cm() * 0.01, 0.0f);

    // keep the current heading unless it was set explicitly
    if (sub.auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING && sub.auto_yaw_mode != AUTO_YAW_HOLD) {
        sub.yaw_rate_only = false;
        sub.yaw_look_at_heading = ahrs.yaw_sensor;
        sub.yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
        sub.mode_guided.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }
}

// process mission items from path_index until a waypoint leg is started.
// Returns false (and stops) if the path is finished or a leg can't be started.
bool ModeDynamic::path_next_leg()
{
    const uint16_t num_cmds = sub.mission.num_commands();
    while (path_index < num_cmds) {
        AP_Mission::Mission_Command cmd;
        if (!sub.mission.read_cmd_from_storage(path_index, cmd)) {
            stop_path("mission read failed");
            return false;
        }

        switch (cmd.id) {
        case MAV_CMD_NAV_WAYPOINT:
        case MAV_CMD_NAV_SPLINE_WAYPOINT:
        case MAV_CMD_NAV_LOITER_TIME:
        case MAV_CMD_NAV_LOITER_UNLIM: {
            Location loc(cmd.content.location);
            if (loc.lat == 0 && loc.lng == 0) {
                loc.lat = sub.current_loc.lat;
                loc.lng = sub.current_loc.lng;
            }
            if (!sub.wp_nav.set_wp_destination_loc(loc)) {
                stop_path("waypoint needs rangefinder/terrain data");
                return false;
            }
            path_hold_s = (cmd.id == MAV_CMD_NAV_LOITER_UNLIM) ? 0 : cmd.p1;
            path_reached_ms = 0;
            return true;
        }

        case MAV_CMD_DO_CHANGE_SPEED:
            if (cmd.content.speed.target_ms > 0) {
                sub.wp_nav.set_speed_NE_ms(cmd.content.speed.target_ms);
            }
            break;

        case MAV_CMD_CONDITION_YAW:
            sub.mode_auto.set_auto_yaw_look_at_heading(cmd.content.yaw.angle_deg, cmd.content.yaw.turn_rate_dps,
                                                       cmd.content.yaw.direction, cmd.content.yaw.relative_angle);
            break;

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "Dynamic: path skips item #%u (cmd %u)", unsigned(path_index), unsigned(cmd.id));
            break;
        }
        path_index++;
    }

    path_running = false;
    gcs().send_text(MAV_SEVERITY_INFO, "Dynamic: path complete");
    path_hold_here(true);
    return false;
}

void ModeDynamic::run_path()
{
    // AC_WPNav runs the horizontal controller and sets the vertical targets
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());
    position_control->D_update_controller();

    if (!sub.wp_nav.reached_wp_destination()) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (path_reached_ms == 0) {
        path_reached_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "Dynamic: reached #%u", unsigned(path_index));
    }
    AP_Mission::Mission_Command cmd;
    if (sub.mission.read_cmd_from_storage(path_index, cmd) && cmd.id == MAV_CMD_NAV_LOITER_UNLIM) {
        path_index = sub.mission.num_commands();
        path_next_leg();
        return;
    }
    if (now_ms - path_reached_ms >= path_hold_s * 1000U) {
        path_index++;
        path_next_leg();
    }
}

/*
 * Bottom tracking uses AC_PosControl's terrain offset: the offset follows the
 * seafloor altitude measured by the downward rangefinder, so the desired
 * altitude (and our vertical target) become the range above the seafloor.
 * If the rangefinder drops out the last seafloor estimate is kept, which holds
 * the last absolute depth rather than jumping to a new range.
 */
void ModeDynamic::update_bottom_track()
{
#if AP_RANGEFINDER_ENABLED
    if (g2.dyn_bt_enable <= 0) {
        if (bt_active) {
            bt_active = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Dynamic: bottom tracking off");
        }
        return;
    }

    if (!sub.rangefinder_alt_ok() || sub.rangefinder_state.inertial_alt_cm >= g.surftrak_depth) {
        return;
    }

    float seafloor_u_cm = sub.rangefinder_state.rangefinder_terrain_offset_cm;

    if (!bt_active) {
        // keep the current depth target; from now on it is relative to the seafloor
        const float prev_terrain_d_m = position_control->get_pos_terrain_D_m();
        position_control->init_pos_terrain_U_cm(seafloor_u_cm);
        pos_target_d_m -= position_control->get_pos_terrain_D_m() - prev_terrain_d_m;
        bt_active = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Dynamic: bottom tracking at %.2fm", get_bottom_track_target_cm() * 0.01f);
        return;
    }

    // don't let the seafloor drag the vehicle above SURFTRAK_DEPTH
    const float target_u_cm = position_control->get_pos_desired_U_cm() + seafloor_u_cm;
    if (target_u_cm > g.surftrak_depth) {
        seafloor_u_cm -= target_u_cm - g.surftrak_depth;
    }

    position_control->set_pos_terrain_target_U_cm(seafloor_u_cm);
#endif  // AP_RANGEFINDER_ENABLED
}

/*
 * The position controllers produce forward/lateral demands in the level,
 * heading-aligned frame and an up/down demand in the earth frame. Rotate them
 * into the body frame so they stay correct when the vehicle is rolled or
 * pitched. At zero roll and pitch this is the same output as GUIDED.
 */
void ModeDynamic::output_thrust()
{
    float lateral_out, forward_out;
    sub.translate_pos_control_rp(lateral_out, forward_out);

    // throttle_in is centred on 0.5 (neutral)
    const float up_out = (attitude_control->get_throttle_in() - 0.5f) * 2.0f;

    Matrix3f body_to_level;
    body_to_level.from_euler(ahrs.get_roll_rad(), ahrs.get_pitch_rad(), 0.0f);
    const Vector3f body_out = body_to_level.mul_transpose(Vector3f{forward_out, lateral_out, -up_out});

    motors.set_forward(constrain_float(body_out.x, -1.0f, 1.0f));
    motors.set_lateral(constrain_float(body_out.y, -1.0f, 1.0f));
    // the rotation already accounts for tilt so no angle boost
    attitude_control->set_throttle_out(constrain_float(0.5f - body_out.z * 0.5f, 0.0f, 1.0f), false, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);
}
