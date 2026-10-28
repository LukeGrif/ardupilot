#include "Sub.h"

/*
  AUTOTUNE mode tunes the roll, pitch and yaw rate and angle gains while
  holding depth and, when a position estimate is available, position.

  Two styles are available (ATUN_STYLE):

  Twitch (0) wraps the AC_AutoTune library and uses the same twitches as
  Copter.

  Gentle (1) moves one axis at a time through two blended slow sine waves
  while the existing gains hold the vehicle. It measures the frequency
  response from rate controller output u to body rate at the two
  frequencies, fits the model
      angle_ddot = a*u - b*rate - c*angle
  (a: thrust authority, b: water damping, c: buoyancy restoring moment,
  zero for yaw) to it by least squares, and calculates the gains from the
  fitted model. Measuring only at the test frequencies keeps the fit
  insensitive to noise, disturbances and the thrusters' nonlinear thrust.

  In both styles position is held with the forward and lateral thrusters
  instead of by leaning, and the forward/lateral/vertical demands are
  rotated into the body frame so the hold keeps working while the vehicle
  is tilted.
 */

#if AUTOTUNE_ENABLED

#define SUBTUNE_STILL_MS            2000    // time the vehicle must be still before each axis
#define SUBTUNE_SETTLE_WARN_MS      15000   // warn if the vehicle has not settled after this long
#define SUBTUNE_MODEL_HALVINGS      3       // most times the rate gains are halved if the vehicle will not settle
#define SUBTUNE_OVERRIDE_MS         2000    // time sticks must be centred before tuning resumes
#define SUBTUNE_FREQ_RATIO          3.0f    // second test frequency as a multiple of the first
#define SUBTUNE_MIN_FIT             0.7f    // minimum fraction of the response the model must explain
#define SUBTUNE_ACCEL_FRACTION      0.5f    // fraction of the measured authority used for the accel limit

const AP_Param::GroupInfo SubTune::var_info[] = {
    // @Param: STYLE
    // @DisplayName: AutoTune style
    // @Description: How AUTOTUNE mode tunes the vehicle. Gentle moves each axis through slow, small sine waves, fits a model of the vehicle's response in the water and calculates the gains from it. Model adds the sine waves straight to the thrusters while the vehicle holds level, sized so the vehicle turns at ATUN_RATE, and fits a model with quadratic water drag and thruster lag; use it with MOT_THST_LIN so thrust follows the thruster output. Twitch uses the Copter AutoTune twitches, configured with the AUTOTUNE_ parameters.
    // @Values: 0:Twitch,1:Gentle,2:Model
    // @User: Standard
    AP_GROUPINFO("STYLE", 1, SubTune, style, 1),

    // @Param: AXES
    // @DisplayName: Gentle AutoTune axes
    // @Description: Axes tuned by the gentle AutoTune style, one at a time in the order roll, pitch, yaw
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    AP_GROUPINFO("AXES", 2, SubTune, axes, 7),

    // @Param: AMP
    // @DisplayName: Gentle AutoTune amplitude
    // @Description: Largest angle the gentle AutoTune style moves the vehicle away from its starting attitude
    // @Units: deg
    // @Range: 2 30
    // @User: Standard
    AP_GROUPINFO("AMP", 3, SubTune, amplitude_deg, 10.0f),

    // @Param: PERIOD
    // @DisplayName: Gentle AutoTune period
    // @Description: Period of the slow sine wave used by the gentle AutoTune style. Longer periods move the vehicle more slowly, but the thrusters must work clear of their ESC deadband for the model to fit, so very slow tests may fail.
    // @Units: s
    // @Range: 2 20
    // @User: Standard
    AP_GROUPINFO("PERIOD", 4, SubTune, period_s, 3.0f),

    // @Param: CYCLES
    // @DisplayName: Gentle AutoTune cycles
    // @Description: Number of sine wave periods recorded for each axis by the gentle AutoTune style, after one period to fade the motion in
    // @Range: 2 10
    // @User: Standard
    AP_GROUPINFO("CYCLES", 5, SubTune, cycles, 3),

    // @Param: BW
    // @DisplayName: Gentle AutoTune response speed
    // @Description: Rate controller response speed the gentle AutoTune style designs the gains for. Higher values give faster, more aggressive gains. The test cannot measure thruster spin-up lag, so lower this if the vehicle oscillates on the tuned gains.
    // @Units: rad/s
    // @Range: 1 15
    // @User: Advanced
    AP_GROUPINFO("BW", 6, SubTune, bandwidth, 4.0f),

    // @Param: DIAG
    // @DisplayName: AutoTune diagnostic log
    // @Description: Log the rate controller outputs, body rates, attitude and thruster outputs at the loop rate (SUTD and SUTM messages) while armed in AUTOTUNE or MANUAL, for studying the tune offline. Uses a lot of log space.
    // @Values: 0:Off,1:On
    // @User: Advanced
    AP_GROUPINFO("DIAG", 7, SubTune, diag, 0),

    // @Param: RATE
    // @DisplayName: Model AutoTune test rate
    // @Description: Turn rate the Model style (ATUN_STYLE 2) sizes its test signal for, and the rate the water drag is linearised at for the gains. Larger rates fit better but move the vehicle more.
    // @Units: deg/s
    // @Range: 5 45
    // @User: Advanced
    AP_GROUPINFO("RATE", 8, SubTune, rate_dps, 15.0f),

    // @Param: INJ_MAX
    // @DisplayName: Model AutoTune largest test signal
    // @Description: Largest test signal the Model style (ATUN_STYLE 2) adds to the thruster output, as a fraction of full output
    // @Range: 0.05 0.8
    // @User: Advanced
    AP_GROUPINFO("INJ_MAX", 9, SubTune, inj_max, 0.4f),

    AP_GROUPEND
};

SubTune::SubTune()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  AC_AutoTune wrapper (twitch style)
 */

bool AutoTune::init()
{
    // only allow AutoTune while armed
    if (!sub.motors.armed()) {
        return false;
    }

    // lean-based position hold is not used, see ModeAutoTune::control_horizontal()
    return init_internals(false,
                          &sub.attitude_control,
                          &sub.pos_control,
                          &sub.ahrs_view);
}

void AutoTune::run()
{
    // run autotune: attitude twitches and depth hold
    AC_AutoTune::run();

    sub.mode_autotune.control_horizontal();
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
  after each twitch, before the gains change, send what it measured and the
  gains it was testing as NAMED_VALUE_FLOATs, ATN last so a GCS knows the
  set is complete:
    ATAxis  0 roll, 1 pitch, 2 yaw
    ATType  tune step (AC_AutoTune::TuneType: 0 rate D up, 1 rate D down,
            2 rate P up, 4 angle P down, 5 angle P up)
    ATTgt   target: rate in deg/s, or angle in deg for angle P steps
    ATMax   peak measured
    ATMin   lowest measured after the peak (the bounce-back)
    ATRP, ATRD, ATSP  rate P, rate D and angle P under test on this axis
 */
void AutoTune::do_post_test_gcs_announcements()
{
    const uint8_t ax = uint8_t(axis);
    float rp = 0, rd = 0, sp = 0;
    switch (axis) {
    case AxisType::ROLL:
        rp = tune_roll_rp; rd = tune_roll_rd; sp = tune_roll_sp;
        break;
    case AxisType::PITCH:
        rp = tune_pitch_rp; rd = tune_pitch_rd; sp = tune_pitch_sp;
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        rp = tune_yaw_rp; rd = tune_yaw_rd; sp = tune_yaw_sp;
        break;
    }
    test_count++;
    gcs().send_named_float("ATAxis", ax > 2 ? 2 : ax);
    gcs().send_named_float("ATType", float(uint8_t(tune_type)));
    gcs().send_named_float("ATTgt", test_target_cd() * 0.01f);
    gcs().send_named_float("ATMax", test_max_cd() * 0.01f);
    gcs().send_named_float("ATMin", test_min_cd() * 0.01f);
    gcs().send_named_float("ATRP", rp);
    gcs().send_named_float("ATRD", rd);
    gcs().send_named_float("ATSP", sp);
    gcs().send_named_float("ATN", test_count);
}

/*
  check if we have a good position estimate
 */
bool AutoTune::position_ok()
{
    return sub.position_ok();
}

/*
  loop-rate diagnostic log (ATUN_DIAG=1), armed in AUTOTUNE or MANUAL:
    SUTD: St (gentle step: 0 settle, 1 excite, 2 done, 9 other mode),
          Ax (axis), UR/UP/UY (rate controller outputs incl. FF),
          GX/GY/GZ (body rates), Rl/Pt/Yw (attitude), Lim (motor limit
          flags: 1 roll, 2 pitch, 4 yaw, 8 throttle)
    SUTM: thruster outputs M1..M8 (PWM)
 */
void SubTune::log_diag() const
{
#if HAL_LOGGING_ENABLED
    if (diag <= 0 || !sub.motors.armed()) {
        return;
    }
    const bool in_tune = sub.control_mode == Mode::Number::AUTOTUNE;
    if (!in_tune && sub.control_mode != Mode::Number::MANUAL) {
        return;
    }
    const uint64_t now_us = AP_HAL::micros64();
    const Vector3f gyro = sub.ahrs.get_gyro_latest();
    const AP_Motors6DOF &m = sub.motors;
    const uint8_t lim = (m.limit.roll ? 1 : 0) | (m.limit.pitch ? 2 : 0) |
                        (m.limit.yaw ? 4 : 0) | (m.limit.throttle_upper || m.limit.throttle_lower ? 8 : 0);
    AP::logger().WriteStreaming("SUTD", "TimeUS,St,Ax,UR,UP,UY,GX,GY,GZ,Rl,Pt,Yw,Lim",
                                "QBBfffffffffB",
                                now_us,
                                uint8_t(in_tune && use_sine_style() ? uint8_t(step) : 9),
                                uint8_t(axis),
                                m.get_roll() + m.get_roll_ff(),
                                m.get_pitch() + m.get_pitch_ff(),
                                m.get_yaw() + m.get_yaw_ff(),
                                gyro.x, gyro.y, gyro.z,
                                sub.ahrs.get_roll_rad(), sub.ahrs.get_pitch_rad(), sub.ahrs.get_yaw_rad(),
                                lim);
    uint16_t pwm[8];
    for (uint8_t i=0; i<8; i++) {
        pwm[i] = hal.rcout->read(i);
    }
    AP::logger().WriteStreaming("SUTM", "TimeUS,M1,M2,M3,M4,M5,M6,M7,M8", "QHHHHHHHH",
                                now_us, pwm[0], pwm[1], pwm[2], pwm[3], pwm[4], pwm[5], pwm[6], pwm[7]);
#endif
}

/*
  gentle style
 */

bool SubTune::init()
{
    if (!sub.motors.armed()) {
        return false;
    }

    sub.pos_control.D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), sub.g.pilot_speed_up, sub.g.pilot_accel_z);
    sub.pos_control.D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), sub.g.pilot_speed_up, sub.g.pilot_accel_z);
    sub.pos_control.D_init_controller();

    const uint32_t now_ms = AP_HAL::millis();
    hold_yaw_rad = sub.ahrs.get_yaw_rad();
    pilot_override = false;

    if (step == Step::DONE) {
        // returning after a completed tune: fly on the tuned gains so they can be checked
        load_gains(true);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: testing tuned gains");
        return true;
    }

    for (uint8_t i=0; i<3; i++) {
        orig[i] = get_gains(Axis(i));
        axis_done[i] = false;
        have_tuned[i] = false;
    }
    tuned_loaded = false;
    aux_used = false;
    settle_halvings = 0;

    if (!next_axis()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AutoTune: no axes selected in ATUN_AXES");
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Started (gentle)");
    start_settle(now_ms);
    return true;
}

void SubTune::run()
{
    const uint32_t now_ms = AP_HAL::millis();
    AC_AttitudeControl_Sub &attitude_control = sub.attitude_control;

    // pilot input
    float roll_cd, pitch_cd;
    sub.get_pilot_desired_lean_angles(sub.channel_roll->get_control_in(), sub.channel_pitch->get_control_in(), roll_cd, pitch_cd, attitude_control.lean_angle_max_cd());
    const float yaw_input = sub.channel_yaw->pwm_to_angle_dz_trim(sub.channel_yaw->get_dead_zone() * sub.gain, sub.channel_yaw->get_radio_trim());
    const float yaw_rate_cds = sub.get_pilot_desired_yaw_rate(yaw_input);
    float climb_rate_cms = sub.get_pilot_desired_climb_rate(sub.channel_throttle->get_control_in());
    climb_rate_cms = constrain_float(climb_rate_cms, -sub.get_pilot_speed_dn(), sub.g.pilot_speed_up);

    const bool pilot_input = !is_zero(roll_cd) || !is_zero(pitch_cd) || !is_zero(yaw_rate_cds) || !is_zero(climb_rate_cms);
    if (pilot_input) {
        override_ms = now_ms;
    }

    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (pilot_input || (pilot_override && now_ms - override_ms < SUBTUNE_OVERRIDE_MS)) {
        if (!pilot_override && step != Step::DONE) {
            // fly on the original gains while the pilot is in control
            restore_axis_feedback();
            pilot_override = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: pilot override, paused");
        }
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, yaw_rate_cds);
        hold_yaw_rad = sub.ahrs.get_yaw_rad();
    } else {
        if (pilot_override) {
            // restart the current axis
            pilot_override = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: resuming %s", axis_name(axis));
            start_settle(now_ms);
        }
        switch (step) {
        case Step::SETTLE:
            update_settle(now_ms);
            break;
        case Step::EXCITE:
            update_excite(now_ms);
            break;
        case Step::DONE:
            attitude_control.input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, hold_yaw_rad, true);
            break;
        }
    }

    // hold depth
    sub.pos_control.D_set_pos_target_from_climb_rate_cms(climb_rate_cms);
    sub.pos_control.D_update_controller();

    if (style == 2 && step == Step::EXCITE && axis == Axis::YAW && !pilot_override) {
        // the horizontal thrusters are also the yaw thrusters: leave them to
        // the yaw test, so each one's output is the yaw signal alone (with
        // the dead band, a mix doesn't add up); position hold resumes after
        sub.motors.set_forward(0.0f);
        sub.motors.set_lateral(0.0f);
    } else {
        sub.mode_autotune.control_horizontal();
    }
}

void SubTune::stop()
{
    restore_axis_feedback();
    restore_untuned();
    if (tuned_loaded) {
        load_gains(false);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Stopped");
}

// save the tuned gains if they are loaded and the pilot chose them
void SubTune::disarmed(bool in_autotune_mode)
{
    if (step != Step::DONE || !tuned_loaded) {
        return;
    }
    if (in_autotune_mode || aux_used) {
        save_gains();
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "AutoTune: Saved gains");
        // the next tune starts from scratch
        step = Step::SETTLE;
        tuned_loaded = false;
    } else {
        load_gains(false);
    }
}

// switch between original and tuned gains once the tune is complete
void SubTune::do_aux_function(RC_Channel::AuxSwitchPos ch_flag)
{
    if (step != Step::DONE) {
        if (ch_flag == RC_Channel::AuxSwitchPos::HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "AutoTune: must be complete to test gains");
        }
        return;
    }

    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::LOW:
        aux_used = true;
        load_gains(false);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: original gains");
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::HIGH:
        aux_used = true;
        load_gains(true);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: tuned gains");
        break;
    }
}

AC_PID &SubTune::rate_pid(Axis ax)
{
    switch (ax) {
    case Axis::ROLL:
        return sub.attitude_control.get_rate_roll_pid();
    case Axis::PITCH:
        return sub.attitude_control.get_rate_pitch_pid();
    case Axis::YAW:
        break;
    }
    return sub.attitude_control.get_rate_yaw_pid();
}

AC_P &SubTune::angle_p(Axis ax)
{
    switch (ax) {
    case Axis::ROLL:
        return sub.attitude_control.get_angle_roll_p();
    case Axis::PITCH:
        return sub.attitude_control.get_angle_pitch_p();
    case Axis::YAW:
        break;
    }
    return sub.attitude_control.get_angle_yaw_p();
}

const char *SubTune::axis_name(Axis ax) const
{
    switch (ax) {
    case Axis::ROLL:
        return "Roll";
    case Axis::PITCH:
        return "Pitch";
    case Axis::YAW:
        break;
    }
    return "Yaw";
}

SubTune::Gains SubTune::get_gains(Axis ax)
{
    AC_PID &pid = rate_pid(ax);
    Gains g;
    g.rate_p = pid.kP();
    g.rate_i = pid.kI();
    g.rate_d = pid.kD();
    g.rate_ff = pid.ff();
    g.angle_p = angle_p(ax).kP();
    switch (ax) {
    case Axis::ROLL:
        g.accel_radss = sub.attitude_control.get_accel_roll_max_radss();
        break;
    case Axis::PITCH:
        g.accel_radss = sub.attitude_control.get_accel_pitch_max_radss();
        break;
    case Axis::YAW:
        g.accel_radss = sub.attitude_control.get_accel_yaw_max_radss();
        break;
    }
    return g;
}

void SubTune::set_gains(Axis ax, const Gains &g)
{
    AC_PID &pid = rate_pid(ax);
    pid.set_kP(g.rate_p);
    pid.set_kI(g.rate_i);
    pid.set_kD(g.rate_d);
    pid.set_ff(g.rate_ff);
    angle_p(ax).set_kP(g.angle_p);
    switch (ax) {
    case Axis::ROLL:
        sub.attitude_control.set_accel_roll_max_radss(g.accel_radss);
        break;
    case Axis::PITCH:
        sub.attitude_control.set_accel_pitch_max_radss(g.accel_radss);
        break;
    case Axis::YAW:
        sub.attitude_control.set_accel_yaw_max_radss(g.accel_radss);
        break;
    }
}

void SubTune::load_gains(bool use_tuned)
{
    for (uint8_t i=0; i<3; i++) {
        if (have_tuned[i]) {
            set_gains(Axis(i), use_tuned ? tuned[i] : orig[i]);
        }
    }
    tuned_loaded = use_tuned;
}

void SubTune::save_gains()
{
    for (uint8_t i=0; i<3; i++) {
        if (!have_tuned[i]) {
            continue;
        }
        const Axis ax = Axis(i);
        set_gains(ax, tuned[i]);
        rate_pid(ax).save_gains();
        angle_p(ax).save_gains();
        switch (ax) {
        case Axis::ROLL:
            sub.attitude_control.save_accel_roll_max_radss(tuned[i].accel_radss);
            break;
        case Axis::PITCH:
            sub.attitude_control.save_accel_pitch_max_radss(tuned[i].accel_radss);
            break;
        case Axis::YAW:
            sub.attitude_control.save_accel_yaw_max_radss(tuned[i].accel_radss);
            break;
        }
        orig[i] = tuned[i];
    }
}

// select the next axis to tune, returns false when all are done
bool SubTune::next_axis()
{
    for (uint8_t i=0; i<3; i++) {
        if ((axes & (1U<<i)) && !axis_done[i]) {
            axis = Axis(i);
            return true;
        }
    }
    return false;
}

void SubTune::start_settle(uint32_t now_ms)
{
    restore_axis_feedback();
    step = Step::SETTLE;
    step_start_ms = now_ms;
    still_start_ms = now_ms;
    last_msg_ms = now_ms;
}

void SubTune::start_excite(uint32_t now_ms)
{
    step = Step::EXCITE;
    step_start_ms = now_ms;
    reset_sums();
    reset_model();
    if (style == 2) {
        weaken_axis_feedback();
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s sine test %.0fs", axis_name(axis), double((cycles + 1) * period_s));
}

void SubTune::reset_sums()
{
    for (uint8_t f=0; f<2; f++) {
        u_cos[f] = u_sin[f] = 0.0f;
        rate_cos[f] = rate_sin[f] = 0.0f;
    }
    record_s = 0.0f;
}

// hold level at the starting heading until the vehicle is still
void SubTune::update_settle(uint32_t now_ms)
{
    sub.attitude_control.input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, hold_yaw_rad, true);

    const AP_AHRS &ahrs = sub.ahrs;
    const bool still = fabsf(ahrs.get_roll_rad()) < radians(3) &&
                       fabsf(ahrs.get_pitch_rad()) < radians(3) &&
                       fabsf(wrap_PI(ahrs.get_yaw_rad() - hold_yaw_rad)) < radians(5) &&
                       ahrs.get_gyro().length() < radians(5);
    if (!still) {
        still_start_ms = now_ms;
    }
    if (now_ms - still_start_ms > SUBTUNE_STILL_MS) {
        start_excite(now_ms);
        return;
    }
    if (style == 2 && now_ms - step_start_ms > SUBTUNE_SETTLE_WARN_MS && settle_halvings < SUBTUNE_MODEL_HALVINGS) {
        // the starting gains may be oscillating: the model test does not need
        // them, so lower them for the rest of the tune
        halve_rate_gains();
        step_start_ms = now_ms;
        return;
    }
    if (now_ms - step_start_ms > SUBTUNE_SETTLE_WARN_MS && now_ms - last_msg_ms > 5000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: waiting for vehicle to settle");
        last_msg_ms = now_ms;
    }
}

// two blended slow sine waves, faded in over the first period; both are zero
// at the end of each period so the test ends back at the starting attitude
float SubTune::excitation_rad(float t_s) const
{
    const float period = MAX(period_s.get(), 1.0f);
    const float omega = M_2PI / period;
    const float envelope = constrain_float(t_s / period, 0.0f, 1.0f);
    return radians(amplitude_deg) * envelope * (0.6f * sinf(omega * t_s) + 0.4f * sinf(SUBTUNE_FREQ_RATIO * omega * t_s));
}

void SubTune::update_excite(uint32_t now_ms)
{
    if (style == 2) {
        update_excite_model(now_ms);
        return;
    }
    const float period = MAX(period_s.get(), 1.0f);
    const float t_s = (now_ms - step_start_ms) * 0.001f;
    const float offset_rad = excitation_rad(t_s);

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = hold_yaw_rad;
    switch (axis) {
    case Axis::ROLL:
        roll_rad = offset_rad;
        break;
    case Axis::PITCH:
        pitch_rad = offset_rad;
        break;
    case Axis::YAW:
        yaw_rad = wrap_PI(hold_yaw_rad + offset_rad);
        break;
    }
    sub.attitude_control.input_euler_angle_roll_pitch_yaw_rad(roll_rad, pitch_rad, yaw_rad, false);

    // record whole periods after the fade in
    const float dt = sub.G_Dt;
    if (t_s > period && is_positive(dt)) {
        const Vector3f gyro = sub.ahrs.get_gyro_latest();
        float u, rate;
        switch (axis) {
        case Axis::ROLL:
            u = sub.motors.get_roll() + sub.motors.get_roll_ff();
            rate = gyro.x;
            break;
        case Axis::PITCH:
            u = sub.motors.get_pitch() + sub.motors.get_pitch_ff();
            rate = gyro.y;
            break;
        case Axis::YAW:
        default:
            u = sub.motors.get_yaw() + sub.motors.get_yaw_ff();
            rate = gyro.z;
            break;
        }
        const float omega = M_2PI / period;
        for (uint8_t f=0; f<2; f++) {
            const float w = (f == 0) ? omega : SUBTUNE_FREQ_RATIO * omega;
            const float cos_wt = cosf(w * t_s);
            const float sin_wt = sinf(w * t_s);
            u_cos[f] += u * cos_wt * dt;
            u_sin[f] += u * sin_wt * dt;
            rate_cos[f] += rate * cos_wt * dt;
            rate_sin[f] += rate * sin_wt * dt;
        }
        record_s += dt;
    }

    if (t_s >= (cycles + 1) * period) {
        finish_axis();
    }
}

/*
  fit the model to the measured frequency response. With U and R the
  complex amplitudes of u and rate at frequency w, the model gives
      j*w*a*U - j*w*b*R - c*R = -w^2*R
  which is two real equations in a, b and c per frequency. Each pair is
  normalised so both frequencies count equally, then solved by least
  squares. Yaw has no restoring moment and is solved for a and b only.
 */
bool SubTune::fit_model(float &a, float &b, float &c, float &fit) const
{
    const float period = MAX(period_s.get(), 1.0f);
    if (record_s < period) {
        return false;
    }
    const bool solve_c = axis != Axis::YAW;

    Matrix3f ata;
    Vector3f atb;
    float rhs_sq = 0.0f;
    // rows and right hand sides, kept to calculate the residual
    Vector3f rows[4];
    float rhs[4];

    for (uint8_t f=0; f<2; f++) {
        const float w = M_2PI / period * ((f == 0) ? 1.0f : SUBTUNE_FREQ_RATIO);
        // complex amplitude X = sum(x*e^(-jwt)) = cos sum - j*sin sum
        const float ur = u_cos[f], ui = -u_sin[f];
        const float rr = rate_cos[f], ri = -rate_sin[f];
        const float scale = sqrtf(rr * rr + ri * ri) * w * w;
        if (!is_positive(scale)) {
            return false;
        }
        rows[2*f]   = Vector3f(-w * ui, w * ri, solve_c ? -rr : 0.0f) / scale;
        rhs[2*f]    = -w * w * rr / scale;
        rows[2*f+1] = Vector3f(w * ur, -w * rr, solve_c ? -ri : 0.0f) / scale;
        rhs[2*f+1]  = -w * w * ri / scale;
    }
    for (uint8_t r=0; r<4; r++) {
        ata.a += rows[r] * rows[r].x;
        ata.b += rows[r] * rows[r].y;
        ata.c += rows[r] * rows[r].z;
        atb += rows[r] * rhs[r];
        rhs_sq += rhs[r] * rhs[r];
    }
    if (!solve_c) {
        // keep the matrix invertible, c is fixed at zero
        ata.c.z = 1.0f;
    }
    if (!ata.invert()) {
        return false;
    }
    const Vector3f theta = ata * atb;
    a = theta.x;
    b = theta.y;
    c = theta.z;

    float residual_sq = 0.0f;
    for (uint8_t r=0; r<4; r++) {
        const float e = rows[r] * theta - rhs[r];
        residual_sq += e * e;
    }
    fit = is_positive(rhs_sq) ? 1.0f - residual_sq / rhs_sq : 0.0f;
    return true;
}

// fit the model and calculate the gains for the current axis
void SubTune::finish_axis()
{
    if (style == 2) {
        finish_axis_model();
        return;
    }
    const uint8_t i = uint8_t(axis);
    float a = 0.0f, b = 0.0f, c = 0.0f;
    float fit = 0.0f;
    const bool solved = fit_model(a, b, c, fit);

    tuned[i] = orig[i];
    if (!solved || !is_positive(a) || fit < SUBTUNE_MIN_FIT) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AutoTune: %s fit poor (%.0f%%), gains unchanged", axis_name(axis), double(fit * 100.0f));
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AutoTune: try a larger ATUN_AMP or shorter ATUN_PERIOD");
    } else {
        b = MAX(b, 0.0f);

        // FF supplies the output needed to overcome the water damping at
        // the target rate. P speeds the rate error decay up by omega and
        // I removes steady errors, e.g. from the restoring moment or the
        // tether, slowly enough that the error response does not overshoot:
        //   e_dot = -(a*P + b)*e - a*I*integral(e)
        const float omega = constrain_float(bandwidth, 1.0f, 15.0f);
        tuned[i].rate_ff = constrain_float(b / a, 0.0f, 2.0f);
        tuned[i].rate_p = constrain_float(omega / a, 0.01f, 2.0f);
        tuned[i].rate_i = constrain_float(0.25f * omega * tuned[i].rate_p, 0.0f, 2.0f);
        // keep the angle loop well inside the rate loop
        tuned[i].angle_p = constrain_float((omega + b) / 3.0f, 1.0f, 12.0f);
        // leave thrust in reserve for depth and position
        tuned[i].accel_radss = MAX(SUBTUNE_ACCEL_FRACTION * a, radians(10.0f));
        have_tuned[i] = true;

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s model a=%.2f b=%.2f c=%.2f fit=%.0f%%",
                      axis_name(axis), double(a), double(b), double(c), double(fit * 100.0f));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s P:%.3f I:%.3f FF:%.3f AngP:%.2f Accel:%.0f",
                      axis_name(axis), double(tuned[i].rate_p), double(tuned[i].rate_i), double(tuned[i].rate_ff),
                      double(tuned[i].angle_p), double(degrees(tuned[i].accel_radss)));
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: SUBT
    // @Description: Sub gentle AutoTune result for one axis
    // @Field: TimeUS: Time since system startup
    // @Field: Ax: axis, 0:roll, 1:pitch, 2:yaw
    // @Field: A: fitted thrust authority, angular acceleration per unit of rate controller output
    // @Field: B: fitted damping rate
    // @Field: C: fitted restoring moment, angular acceleration per unit of angle
    // @Field: Fit: fraction of the measured frequency response explained by the model
    // @Field: P: tuned rate P
    // @Field: I: tuned rate I
    // @Field: FF: tuned rate FF
    // @Field: AngP: tuned angle P
    AP::logger().Write("SUBT", "TimeUS,Ax,A,B,C,Fit,P,I,FF,AngP", "QBffffffff",
                       AP_HAL::micros64(), i, a, b, c, fit,
                       tuned[i].rate_p, tuned[i].rate_i, tuned[i].rate_ff, tuned[i].angle_p);
#endif

    axis_done[i] = true;
    const uint32_t now_ms = AP_HAL::millis();
    if (next_axis()) {
        start_settle(now_ms);
        return;
    }

    step = Step::DONE;
    restore_untuned();
    load_gains(true);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "AutoTune: Success");
}

/*
  model style (ATUN_STYLE 2)
 */

#define SUBTUNE_MODEL_FILT_HZ       5.0f    // low pass on every signal of the fit
#define SUBTUNE_MODEL_LAG_STEP_S    0.005f  // lags tried: 0, 5, ... ms (thrusters plus the gyro filter)
#define SUBTUNE_MODEL_WARMUP_S      0.5f    // let the filters settle before recording
#define SUBTUNE_MODEL_INJ_START     0.15f   // test signal size for the probe period
#define SUBTUNE_MODEL_FEEDBACK      0.2f    // tested axis's rate P, I and D during its test

void SubTune::reset_model()
{
    memset(ls_ata, 0, sizeof(ls_ata));
    memset(ls_atb, 0, sizeof(ls_atb));
    memset(ls_btb, 0, sizeof(ls_btb));
    ls_sum_y = 0.0f;
    ls_count = 0;
    memset(u_hist, 0, sizeof(u_hist));
    memset(sg_hist, 0, sizeof(sg_hist));
    u_hist_idx = 0;
    u_hist_len = 0;
    const float rate_hz = 1.0f / MAX(sub.G_Dt, 0.0005f);
    filt_u.set_cutoff_frequency(rate_hz, SUBTUNE_MODEL_FILT_HZ);
    filt_sg.set_cutoff_frequency(rate_hz, SUBTUNE_MODEL_FILT_HZ);
    filt_w.set_cutoff_frequency(rate_hz, SUBTUNE_MODEL_FILT_HZ);
    filt_q.set_cutoff_frequency(rate_hz, SUBTUNE_MODEL_FILT_HZ);
    filt_s.set_cutoff_frequency(rate_hz, SUBTUNE_MODEL_FILT_HZ);
    filt_u.reset();
    filt_sg.reset();
    filt_w.reset();
    filt_q.reset();
    filt_s.reset();
    prev_wf = 0.0f;
    inj_amp = MIN(SUBTUNE_MODEL_INJ_START, inj_max.get());
    inj_scaled = false;
    probe_w_sq = 0.0f;
    probe_n = 0;
    rate_sq = 0.0f;
    rate_n = 0;
}

/*
  hold level and add the test signal to the tested axis's output. The first
  period (faded in) is a probe: its response sizes the signal so the vehicle
  turns at about ATUN_RATE for the rest of the test, whatever the current
  gains. Every sample after the filters settle goes into the fit, except
  while the thrusters are saturated on this axis.
 */
void SubTune::update_excite_model(uint32_t now_ms)
{
    const float period = MAX(period_s.get(), 1.0f);
    const float t_s = (now_ms - step_start_ms) * 0.001f;
    const float omega = M_2PI / period;
    AC_AttitudeControl_Sub &attitude_control = sub.attitude_control;

    attitude_control.input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, hold_yaw_rad, true);

    // the test signal
    const float envelope = constrain_float(t_s / period, 0.0f, 1.0f);
    const float inj = inj_amp * envelope * (0.6f * sinf(omega * t_s) + 0.4f * sinf(SUBTUNE_FREQ_RATIO * omega * t_s));
    switch (axis) {
    case Axis::ROLL:
        attitude_control.actuator_roll_sysid(inj);
        break;
    case Axis::PITCH:
        attitude_control.actuator_pitch_sysid(inj);
        break;
    case Axis::YAW:
        attitude_control.actuator_yaw_sysid(inj);
        break;
    }

    // what the vehicle did: output (incl. the signal, applied last loop),
    // body rate and angle on this axis
    const Vector3f gyro = sub.ahrs.get_gyro_latest();
    float u, w, ang;
    bool limited;
    switch (axis) {
    case Axis::ROLL:
        u = sub.motors.get_roll() + sub.motors.get_roll_ff();
        w = gyro.x;
        ang = sub.ahrs.get_roll_rad();
        limited = sub.motors.limit.roll;
        break;
    case Axis::PITCH:
        u = sub.motors.get_pitch() + sub.motors.get_pitch_ff();
        w = gyro.y;
        ang = sub.ahrs.get_pitch_rad();
        limited = sub.motors.limit.pitch;
        break;
    case Axis::YAW:
    default:
        u = sub.motors.get_yaw() + sub.motors.get_yaw_ff();
        w = gyro.z;
        ang = 0.0f;
        limited = sub.motors.limit.yaw;
        break;
    }

    // probe: second half of the first period, then size the signal
    if (!inj_scaled) {
        if (t_s > 0.5f * period) {
            probe_w_sq += w * w;
            probe_n++;
        }
        if (t_s >= period) {
            // the probe ran at 50-100% envelope, about 75% on average, so
            // the full signal would turn the vehicle about 1/0.75 times faster
            const float probe_amp = sqrtf(2.0f * probe_w_sq / MAX(probe_n, 1U)) / 0.75f;
            const float want = radians(constrain_float(rate_dps, 5.0f, 45.0f));
            const float scale = want / MAX(probe_amp, radians(0.5f));
            inj_amp = constrain_float(inj_amp * scale, 0.03f, constrain_float(inj_max, 0.05f, 0.8f));
            inj_scaled = true;
        }
    }

    // the same filter on every signal, so it cancels out of the fit; the
    // nonlinear terms are formed before filtering
    const float dt = sub.G_Dt;
    const float uf = filt_u.apply(u);
    const float sgf = filt_sg.apply(is_zero(u) ? 0.0f : (u > 0 ? 1.0f : -1.0f));
    const float wf = filt_w.apply(w);
    const float qf = filt_q.apply(fabsf(w) * w);
    const float sf = filt_s.apply(sinf(ang));
    const float accf = is_positive(dt) ? (wf - prev_wf) / dt : 0.0f;
    prev_wf = wf;

    u_hist[u_hist_idx] = uf;
    sg_hist[u_hist_idx] = sgf;
    u_hist_idx = (u_hist_idx + 1) % U_HIST;
    if (u_hist_len < U_HIST) {
        u_hist_len++;
    }

    if (t_s > SUBTUNE_MODEL_WARMUP_S && !limited && is_positive(dt)) {
        rate_sq += w * w;
        rate_n++;
        for (uint8_t lag=0; lag<MODEL_LAGS; lag++) {
            const uint8_t back = MIN(uint8_t(roundf(lag * SUBTUNE_MODEL_LAG_STEP_S / dt)), uint8_t(U_HIST - 1));
            if (back >= u_hist_len) {
                continue;
            }
            const uint8_t k = (u_hist_idx + U_HIST - 1 - back) % U_HIST;
            const float phi[MODEL_N] { u_hist[k], sg_hist[k], -wf, -qf, -sf, 1.0f };
            for (uint8_t r=0; r<MODEL_N; r++) {
                for (uint8_t c=r; c<MODEL_N; c++) {
                    ls_ata[lag][pack(r, c)] += phi[r] * phi[c];
                }
                ls_atb[lag][r] += phi[r] * accf;
            }
            ls_btb[lag] += accf * accf;
        }
        ls_sum_y += accf;
        ls_count++;
    }

    if (t_s >= (cycles + 1) * period) {
        switch (axis) {
        case Axis::ROLL:
            attitude_control.actuator_roll_sysid(0.0f);
            break;
        case Axis::PITCH:
            attitude_control.actuator_pitch_sysid(0.0f);
            break;
        case Axis::YAW:
            attitude_control.actuator_yaw_sysid(0.0f);
            break;
        }
        finish_axis();
    }
}

/*
  least squares for one lag: theta = [a, e, b, b2, c, d]. Yaw has no
  righting moment, so c is left out. Returns false if the problem is singular.
 */
bool SubTune::solve_model(uint8_t lag, float theta[MODEL_N], float &sse) const
{
    uint8_t idx[MODEL_N];
    uint8_t n = 0;
    for (uint8_t k=0; k<MODEL_N; k++) {
        if (k == 4 && axis == Axis::YAW) {
            continue;
        }
        idx[n++] = k;
    }

    // augmented, symmetric normal equations
    float m[MODEL_N][MODEL_N + 1];
    for (uint8_t r=0; r<n; r++) {
        for (uint8_t c=0; c<n; c++) {
            m[r][c] = ls_ata[lag][pack(idx[r], idx[c])];
        }
        m[r][n] = ls_atb[lag][idx[r]];
    }

    // Gaussian elimination with partial pivoting
    for (uint8_t col=0; col<n; col++) {
        uint8_t piv = col;
        for (uint8_t r=col+1; r<n; r++) {
            if (fabsf(m[r][col]) > fabsf(m[piv][col])) {
                piv = r;
            }
        }
        if (fabsf(m[piv][col]) < 1e-9f) {
            return false;
        }
        if (piv != col) {
            for (uint8_t c=0; c<=n; c++) {
                const float tmp = m[col][c];
                m[col][c] = m[piv][c];
                m[piv][c] = tmp;
            }
        }
        for (uint8_t r=col+1; r<n; r++) {
            const float f = m[r][col] / m[col][col];
            for (uint8_t c=col; c<=n; c++) {
                m[r][c] -= f * m[col][c];
            }
        }
    }
    float x[MODEL_N];
    for (int8_t r=n-1; r>=0; r--) {
        float v = m[r][n];
        for (uint8_t c=r+1; c<n; c++) {
            v -= m[r][c] * x[c];
        }
        x[r] = v / m[r][r];
    }

    for (uint8_t k=0; k<MODEL_N; k++) {
        theta[k] = 0.0f;
    }
    for (uint8_t r=0; r<n; r++) {
        theta[idx[r]] = x[r];
    }

    // residual sum of squares: y'y - 2 theta'A'y + theta'A'A theta
    float quad = 0.0f;
    float lin = 0.0f;
    for (uint8_t i=0; i<MODEL_N; i++) {
        lin += theta[i] * ls_atb[lag][i];
        for (uint8_t j=0; j<MODEL_N; j++) {
            const float aij = ls_ata[lag][pack(i, j)];
            quad += theta[i] * aij * theta[j];
        }
    }
    sse = ls_btb[lag] - 2.0f * lin + quad;
    return true;
}

/*
  for the test the tested axis's rate loop is kept at SUBTUNE_MODEL_FEEDBACK
  of its gains, so the thrusters follow the test signal rather than fighting
  it (the attitude, depth and position holds keep working). The gains are put
  back after the test, when the pilot takes over and when the tune stops.
 */
void SubTune::weaken_axis_feedback()
{
    if (axis_weakened) {
        return;
    }
    pre_weaken = get_gains(axis);
    AC_PID &pid = rate_pid(axis);
    pid.set_kP(pre_weaken.rate_p * SUBTUNE_MODEL_FEEDBACK);
    pid.set_kI(pre_weaken.rate_i * SUBTUNE_MODEL_FEEDBACK);
    pid.set_kD(pre_weaken.rate_d * SUBTUNE_MODEL_FEEDBACK);
    axis_weakened = true;
}

// put back the gains of axes without a tune if halve_rate_gains lowered them
void SubTune::restore_untuned()
{
    if (settle_halvings == 0) {
        return;
    }
    for (uint8_t i=0; i<3; i++) {
        if (!have_tuned[i]) {
            set_gains(Axis(i), orig[i]);
        }
    }
    settle_halvings = 0;
}

// halve the rate gains of the axes not yet tuned; put back when the tune stops
void SubTune::halve_rate_gains()
{
    for (uint8_t i=0; i<3; i++) {
        if (have_tuned[i]) {
            continue;
        }
        AC_PID &pid = rate_pid(Axis(i));
        pid.set_kP(pid.kP() * 0.5f);
        pid.set_kI(pid.kI() * 0.5f);
        pid.set_kD(pid.kD() * 0.5f);
    }
    settle_halvings++;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: not settling, rate gains halved for the test");
}

void SubTune::restore_axis_feedback()
{
    if (!axis_weakened) {
        return;
    }
    set_gains(axis, pre_weaken);
    axis_weakened = false;
}

// pick the thruster lag that fits best and calculate the gains from that model
void SubTune::finish_axis_model()
{
    restore_axis_feedback();
    const uint8_t i = uint8_t(axis);
    // the lag that fits best, refined between the lags tried by a parabola
    // through the residuals: the authority moves several percent per 5 ms of
    // lag, because the output and the rate are tied together by the feedback
    float sse_lag[MODEL_LAGS];
    int8_t best_lag = -1;
    for (uint8_t lag=0; lag<MODEL_LAGS; lag++) {
        float theta[MODEL_N];
        sse_lag[lag] = -1.0f;
        if (ls_count > 100 && solve_model(lag, theta, sse_lag[lag]) && is_positive(theta[0])) {
            if (best_lag < 0 || sse_lag[lag] < sse_lag[best_lag]) {
                best_lag = lag;
            }
        } else {
            sse_lag[lag] = -1.0f;
        }
    }
    float best[MODEL_N] {};
    float best_sse = 0.0f;
    float lag_frac = 0.0f;
    if (best_lag >= 0) {
        solve_model(best_lag, best, best_sse);
        if (best_lag > 0 && best_lag < MODEL_LAGS - 1 &&
            sse_lag[best_lag - 1] >= 0.0f && sse_lag[best_lag + 1] >= 0.0f) {
            const float s0 = sse_lag[best_lag - 1], s1 = sse_lag[best_lag], s2 = sse_lag[best_lag + 1];
            const float curv = s0 - 2.0f * s1 + s2;
            if (is_positive(curv)) {
                lag_frac = constrain_float(0.5f * (s0 - s2) / curv, -0.5f, 0.5f);
                float other[MODEL_N];
                float other_sse;
                if (solve_model(best_lag + (lag_frac > 0 ? 1 : -1), other, other_sse)) {
                    const float w = fabsf(lag_frac);
                    for (uint8_t k=0; k<MODEL_N; k++) {
                        best[k] += w * (other[k] - best[k]);
                    }
                }
            }
        }
    }
    const float mean_y = ls_count > 0 ? ls_sum_y / ls_count : 0.0f;
    const float var_y = ls_count > 0 ? ls_btb[MAX(best_lag, int8_t(0))] - ls_count * mean_y * mean_y : 0.0f;
    const float fit = (best_lag >= 0 && is_positive(var_y)) ? 1.0f - best_sse / var_y : 0.0f;
    const float a = best[0], e = best[1], b = best[2], b2 = best[3], c = best[4], d = best[5];
    const float lag_s = MAX(best_lag + lag_frac, 0.0f) * SUBTUNE_MODEL_LAG_STEP_S;
    const float rate_amp = sqrtf(2.0f * rate_sq / MAX(rate_n, 1U));

    tuned[i] = orig[i];
    if (best_lag < 0 || fit < SUBTUNE_MIN_FIT) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AutoTune: %s fit poor (%.0f%%), gains unchanged", axis_name(axis), double(fit * 100.0f));
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AutoTune: try a larger ATUN_RATE or ATUN_INJ_MAX");
    } else {
        // damping as a straight line at the design rate: the quadratic term
        // b2*|w|*w has the same first harmonic as (8/(3*pi))*b2*W*w for a
        // sine of amplitude W
        const float w_ref = radians(constrain_float(rate_dps, 5.0f, 45.0f));
        const float b_eq = MAX(b + 8.0f / (3.0f * M_PI) * MAX(b2, 0.0f) * w_ref, 0.0f);
        const float omega = constrain_float(bandwidth, 1.0f, 15.0f);
        tuned[i].rate_ff = constrain_float(b_eq / a, 0.0f, 2.0f);
        tuned[i].rate_p = constrain_float(omega / a, 0.01f, 2.0f);
        tuned[i].rate_i = constrain_float(0.25f * omega * tuned[i].rate_p, 0.0f, 2.0f);
        tuned[i].angle_p = constrain_float((omega + b_eq) / 3.0f, 1.0f, 12.0f);
        tuned[i].accel_radss = MAX(SUBTUNE_ACCEL_FRACTION * a, radians(10.0f));
        have_tuned[i] = true;

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s model a=%.2f b=%.2f c=%.2f fit=%.0f%%",
                      axis_name(axis), double(a), double(b_eq), double(c), double(fit * 100.0f));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s drag b=%.2f b2=%.1f lag=%.0fms test=%.2f %.0fdps",
                      axis_name(axis), double(b), double(b2), double(lag_s * 1000.0f), double(inj_amp), double(degrees(rate_amp)));
        // the thrust step at zero output, as a fraction of full output:
        // positive means thrust starts below MOT_SPIN_MIN, negative above it
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s dead band step %.1f%% of full output",
                      axis_name(axis), double(100.0f * e / a));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s P:%.3f I:%.3f FF:%.3f AngP:%.2f Accel:%.0f",
                      axis_name(axis), double(tuned[i].rate_p), double(tuned[i].rate_i), double(tuned[i].rate_ff),
                      double(tuned[i].angle_p), double(degrees(tuned[i].accel_radss)));
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: SUBM
    // @Description: Sub model AutoTune (ATUN_STYLE 2) result for one axis
    // @Field: TimeUS: Time since system startup
    // @Field: Ax: axis, 0:roll, 1:pitch, 2:yaw
    // @Field: A: fitted thrust authority, angular acceleration per unit of output
    // @Field: B: fitted linear damping
    // @Field: B2: fitted quadratic damping
    // @Field: C: fitted restoring moment
    // @Field: E: fitted thrust step at zero output (dead band compensation error)
    // @Field: D: fitted constant angular acceleration (trim, tether)
    // @Field: Lag: thruster lag that fitted best
    // @Field: Fit: fraction of the measured angular acceleration explained by the model
    // @Field: Inj: size of the test signal
    // @Field: W: rate amplitude during the test
    // @Field: P: tuned rate P
    // @Field: I: tuned rate I
    // @Field: FF: tuned rate FF
    // @Field: AngP: tuned angle P
    AP::logger().Write("SUBM", "TimeUS,Ax,A,E,B,B2,C,D,Lag,Fit,Inj,W,P,I,FF,AngP", "QBffffffffffffff",
                       AP_HAL::micros64(), i, a, e, b, b2, c, d, lag_s, fit, inj_amp, rate_amp,
                       tuned[i].rate_p, tuned[i].rate_i, tuned[i].rate_ff, tuned[i].angle_p);
#endif

    axis_done[i] = true;
    const uint32_t now_ms = AP_HAL::millis();
    if (next_axis()) {
        start_settle(now_ms);
        return;
    }

    step = Step::DONE;
    restore_untuned();
    load_gains(true);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "AutoTune: Success");
}

/*
  mode
 */

bool ModeAutoTune::init(bool ignore_checks)
{
    if (!motors.armed()) {
        return false;
    }

    using_sine = subtune.use_sine_style();
    init_position_hold();

    if (using_sine) {
        return subtune.init();
    }
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

    if (using_sine) {
        subtune.run();
    } else {
        autotune.run();
    }
}

void ModeAutoTune::exit()
{
    if (using_sine) {
        subtune.stop();
    } else {
        autotune.stop();
    }
}

void ModeAutoTune::disarmed(bool in_autotune_mode)
{
    if (using_sine) {
        subtune.disarmed(in_autotune_mode);
    } else {
        autotune.disarmed(in_autotune_mode);
    }
}

void ModeAutoTune::do_aux_function(RC_Channel::AuxSwitchPos ch_flag)
{
    if (using_sine) {
        subtune.do_aux_function(ch_flag);
    } else {
        autotune.do_aux_function(ch_flag);
    }
}

void ModeAutoTune::init_position_hold()
{
    position_control->NE_set_max_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->NE_set_correction_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->NE_init_controller_stopping_point();
}

/*
  hold position with the forward and lateral thrusters, and rotate the
  level-frame thrust demands into the body frame
 */
void ModeAutoTune::control_horizontal()
{
    float lateral_out = 0;
    float forward_out = 0;

    // pilot forward/lateral input moves the hold point
    const Vector2f body_rates_cms {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    if (sub.position_ok()) {
        if (!position_control->NE_is_active()) {
            // the xy controller timed out, re-initialize
            position_control->NE_init_controller_stopping_point();
        }
        auto earth_rates_cms = ahrs.body_to_earth2D(body_rates_cms);
        position_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});

        // convert pos control roll and pitch angles back to lateral and forward efforts
        sub.translate_pos_control_rp(lateral_out, forward_out);

        position_control->NE_update_controller();
    } else if (g.pilot_speed > 0) {
        // allow the pilot to reposition manually
        forward_out = body_rates_cms.x / (float)g.pilot_speed;
        lateral_out = body_rates_cms.y / (float)g.pilot_speed;
    }

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

#endif  // AUTOTUNE_ENABLED
