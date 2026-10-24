#include "Sub.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library

  The roll, pitch and yaw twitches are the same as Copter's. Unlike Copter,
  position is held with the forward and lateral thrusters instead of by
  leaning, and the forward/lateral/vertical demands are rotated into the body
  frame so the hold keeps working while the vehicle is tilted by a twitch.
 */

#if AUTOTUNE_ENABLED

bool AutoTune::init()
{
    // only allow AutoTune while armed
    if (!sub.motors.armed()) {
        return false;
    }

    // hold horizontal position when we know where we are
    sub.pos_control.NE_set_max_speed_accel_cm(sub.g.pilot_speed, sub.g.pilot_accel_z);
    sub.pos_control.NE_set_correction_speed_accel_cm(sub.g.pilot_speed, sub.g.pilot_accel_z);
    sub.pos_control.NE_init_controller_stopping_point();

    // lean-based position hold is not used, see control_horizontal()
    return init_internals(false,
                          &sub.attitude_control,
                          &sub.pos_control,
                          &sub.ahrs_view);
}

void AutoTune::run()
{
    // run autotune: attitude twitches and depth hold
    AC_AutoTune::run();

    control_horizontal();
}

/*
  hold position with the forward and lateral thrusters, and rotate the
  level-frame thrust demands into the body frame
 */
void AutoTune::control_horizontal()
{
    float lateral_out = 0;
    float forward_out = 0;

    // pilot forward/lateral input moves the hold point
    const Vector2f body_rates_cms {
        sub.get_pilot_desired_horizontal_rate(sub.channel_forward),
        sub.get_pilot_desired_horizontal_rate(sub.channel_lateral)
    };

    if (sub.position_ok()) {
        if (!pos_control->NE_is_active()) {
            // the xy controller timed out, re-initialize
            pos_control->NE_init_controller_stopping_point();
        }
        auto earth_rates_cms = sub.ahrs.body_to_earth2D(body_rates_cms);
        pos_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});

        // convert pos control roll and pitch angles back to lateral and forward efforts
        sub.translate_pos_control_rp(lateral_out, forward_out);

        pos_control->NE_update_controller();
    } else if (sub.g.pilot_speed > 0) {
        // allow the pilot to reposition manually
        forward_out = body_rates_cms.x / (float)sub.g.pilot_speed;
        lateral_out = body_rates_cms.y / (float)sub.g.pilot_speed;
    }

    // throttle_in is centred on 0.5 (neutral)
    const float up_out = (attitude_control->get_throttle_in() - 0.5f) * 2.0f;

    Matrix3f body_to_level;
    body_to_level.from_euler(sub.ahrs.get_roll_rad(), sub.ahrs.get_pitch_rad(), 0.0f);
    const Vector3f body_out = body_to_level.mul_transpose(Vector3f{forward_out, lateral_out, -up_out});

    sub.motors.set_forward(constrain_float(body_out.x, -1.0f, 1.0f));
    sub.motors.set_lateral(constrain_float(body_out.y, -1.0f, 1.0f));
    // the rotation already accounts for tilt so no angle boost
    attitude_control->set_throttle_out(constrain_float(0.5f - body_out.z * 0.5f, 0.0f, 1.0f), false, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);
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
    float roll_cd, pitch_cd;
    sub.get_pilot_desired_lean_angles(sub.channel_roll->get_control_in(), sub.channel_pitch->get_control_in(), roll_cd, pitch_cd, sub.attitude_control.lean_angle_max_cd());
    des_roll_rad = cd_to_rad(roll_cd);
    des_pitch_rad = cd_to_rad(pitch_cd);

    const float yaw_input = sub.channel_yaw->pwm_to_angle_dz_trim(sub.channel_yaw->get_dead_zone() * sub.gain, sub.channel_yaw->get_radio_trim());
    des_yaw_rate_rads = cd_to_rad(sub.get_pilot_desired_yaw_rate(yaw_input));
}

/*
  setup z controller velocity and accel limits
 */
void AutoTune::init_z_limits()
{
    pos_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), sub.g.pilot_speed_up, sub.g.pilot_accel_z);
    pos_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), sub.g.pilot_speed_up, sub.g.pilot_accel_z);
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
bool ModeAutoTune::init(bool ignore_checks)
{
    return autotune.init();
}

void ModeAutoTune::run()
{
    // if motors not enabled set throttle to neutral and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->D_relax_controller(0.5f);
        return;
    }

    autotune.run();
}

void ModeAutoTune::exit()
{
    autotune.stop();
}

#endif  // AUTOTUNE_ENABLED
