#include "Sub.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library

  It holds depth and position (when there is a position estimate) while the
  library twitches roll, then pitch, then yaw to find the rate and angle
  gains. Pilot stick input pauses the tune and flies on the original gains;
  it resumes once the sticks are centred.

  When the tune completes the original gains are flown again. Disarming
  while still in AUTOTUNE saves the tuned gains; leaving the mode first
  discards them.
 */

#if AUTOTUNE_ENABLED

bool AutoTune::init()
{
    // ensure we are armed and below the surface, with room to roll and pitch
    if (!sub.motors.armed()) {
        return false;
    }
    if (sub.ap.at_surface) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AutoTune: dive below the surface first");
        return false;
    }

    // tilting the vehicle to hold position (the copter approach) would
    // fight the tests, so position is held with the forward/lateral
    // thrusters by ModeAutotune instead
    return init_internals(false,
                          &sub.attitude_control,
                          &sub.pos_control,
                          &sub.ahrs_view);
}

void AutoTune::run()
{
    // run autotune mode
    AC_AutoTune::run();
}

/*
  get stick input climb rate
 */
float AutoTune::get_desired_climb_rate_ms(void) const
{
    float target_climb_rate_cms = sub.get_pilot_desired_climb_rate(sub.channel_throttle->get_control_in());
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -sub.get_pilot_speed_dn(), sub.g.pilot_speed_up);
    return target_climb_rate_cms * 0.01f;
}

/*
  get stick roll, pitch and yaw rate
 */
void AutoTune::get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads)
{
    float target_roll_cd, target_pitch_cd;
    sub.get_pilot_desired_lean_angles(sub.channel_roll->get_control_in(), sub.channel_pitch->get_control_in(),
                                      target_roll_cd, target_pitch_cd, sub.attitude_control.get_althold_lean_angle_max_cd());
    des_roll_rad = cd_to_rad(target_roll_cd);
    des_pitch_rad = cd_to_rad(target_pitch_cd);

    const float yaw_input = sub.channel_yaw->pwm_to_angle_dz_trim(sub.channel_yaw->get_dead_zone() * sub.gain, sub.channel_yaw->get_radio_trim());
    des_yaw_rate_rads = cd_to_rad(sub.get_pilot_desired_yaw_rate(yaw_input));
}

/*
  setup z controller velocity and accel limits
 */
void AutoTune::init_z_limits()
{
    sub.pos_control.D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), sub.g.pilot_speed_up, sub.g.pilot_accel_z);
    sub.pos_control.D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), sub.g.pilot_speed_up, sub.g.pilot_accel_z);
}

#if HAL_LOGGING_ENABLED
void AutoTune::log_pids()
{
    sub.logger.Write_PID(LOG_PIDR_MSG, sub.attitude_control.get_rate_roll_pid().get_pid_info());
    sub.logger.Write_PID(LOG_PIDP_MSG, sub.attitude_control.get_rate_pitch_pid().get_pid_info());
    sub.logger.Write_PID(LOG_PIDY_MSG, sub.attitude_control.get_rate_yaw_pid().get_pid_info());
}
#endif

/*
  check if we have a good position estimate
 */
bool AutoTune::position_ok()
{
    return sub.position_ok();
}

/*
  initialise autotune mode
*/
bool ModeAutotune::init(bool ignore_checks)
{
    if (!autotune.init()) {
        return false;
    }

    // hold the current position
    position_control->NE_set_max_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->NE_set_correction_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->NE_init_controller_stopping_point();

    return true;
}

void ModeAutotune::run()
{
    // When unarmed, disable motors and stabilization
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->NE_init_controller_stopping_point();
        position_control->D_relax_controller(motors.get_throttle_hover());
        return;
    }

    // the library runs the attitude and depth controllers
    autotune.run();

    control_horizontal();
}

void ModeAutotune::exit()
{
    autotune.stop();
}

// hold position with the forward/lateral thrusters, letting the pilot move
// the target with the forward/lateral sticks (as in POSHOLD)
void ModeAutotune::control_horizontal()
{
    float lateral_out = 0;
    float forward_out = 0;

    // get desired rates in the body frame
    Vector2f body_rates_cms = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    if (sub.position_ok()) {
        if (!position_control->NE_is_active()) {
            // the xy controller timed out, re-initialize
            position_control->NE_init_controller_stopping_point();
        }

        // convert to the earth frame and set target rates
        auto earth_rates_cms = ahrs.body_to_earth2D(body_rates_cms);
        position_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});

        // convert pos control roll and pitch angles back to lateral and forward efforts
        sub.translate_pos_control_rp(lateral_out, forward_out);

        // update the xy controller
        position_control->NE_update_controller();
    } else if (g.pilot_speed > 0) {
        // allow the pilot to reposition manually
        forward_out = body_rates_cms.x / (float)g.pilot_speed;
        lateral_out = body_rates_cms.y / (float)g.pilot_speed;
    }

    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}

#endif  // AUTOTUNE_ENABLED
